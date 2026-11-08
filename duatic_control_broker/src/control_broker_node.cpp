/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <duatic_controller_msgs/msg/control_heartbeat.hpp>
#include <duatic_controller_msgs/msg/control_state.hpp>
#include <duatic_controller_msgs/srv/claim_control.hpp>
#include <duatic_controller_msgs/srv/release_control.hpp>
#include <rclcpp/rclcpp.hpp>

#include "duatic_control_broker/broker.hpp"

namespace duatic_control_broker
{

using ClaimControl = duatic_controller_msgs::srv::ClaimControl;
using ControlState = duatic_controller_msgs::msg::ControlState;
using ControlHeartbeat = duatic_controller_msgs::msg::ControlHeartbeat;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using ReleaseControl = duatic_controller_msgs::srv::ReleaseControl;
using SwitchController = controller_manager_msgs::srv::SwitchController;

constexpr auto kControllerManagerTimeout = std::chrono::seconds(2);

// Reads the groups from parameters: groups lists them, and each group names its default_mode, its modes,
// and the controllers of each mode under <group>.<mode>.
Config read_config(rclcpp::Node& node)
{
  Config config;
  config.heartbeat_timeout_s = node.declare_parameter("heartbeat_timeout", 1.0);
  const auto groups = node.declare_parameter("groups", std::vector<std::string>{});
  for (const auto& group : groups) {
    GroupConfig group_config;
    group_config.default_mode = node.declare_parameter(group + ".default_mode", std::string{});
    const auto modes = node.declare_parameter(group + ".modes", std::vector<std::string>{});
    for (const auto& mode : modes) {
      group_config.modes[mode] = node.declare_parameter(group + "." + mode, std::vector<std::string>{});
    }
    if (group_config.modes.count(group_config.default_mode) == 0) {
      throw std::invalid_argument("Group " + group + ": default_mode is not one of its modes");
    }
    config.groups[group] = group_config;
  }
  if (config.groups.empty()) {
    throw std::invalid_argument("No groups configured");
  }
  return config;
}

std::string join(const std::vector<std::string>& items)
{
  std::string out;
  for (const auto& item : items) {
    out += (out.empty() ? "" : ", ") + item;
  }
  return out;
}

// Claims, releases and both timers run one at a time in the `calls_` group, so the broker's state changes
// only there. mutex_ guards it against the heartbeat callbacks, and is never held while controller_manager
// is asked to switch, so a heartbeat is never kept waiting by a switch.
class ControlBrokerNode : public rclcpp::Node
{
public:
  explicit ControlBrokerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node("control_broker", options), broker_(read_config(*this))
  {
    const auto controller_manager = declare_parameter("controller_manager", std::string{ "controller_manager" });

    clients_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    calls_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    heartbeats_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    list_client_ =
        create_client<ListControllers>(controller_manager + "/list_controllers", rclcpp::ServicesQoS(), clients_);
    switch_client_ =
        create_client<SwitchController>(controller_manager + "/switch_controller", rclcpp::ServicesQoS(), clients_);

    state_pub_ = create_publisher<ControlState>("~/state", rclcpp::QoS(1).reliable().transient_local());
    claim_srv_ = create_service<ClaimControl>(
        "~/claim",
        [this](const ClaimControl::Request::SharedPtr req, ClaimControl::Response::SharedPtr res) {
          on_claim(*req, *res);
        },
        rclcpp::ServicesQoS(), calls_);
    release_srv_ = create_service<ReleaseControl>(
        "~/release",
        [this](const ReleaseControl::Request::SharedPtr req, ReleaseControl::Response::SharedPtr res) {
          on_release(*req, *res);
        },
        rclcpp::ServicesQoS(), calls_);

    rclcpp::SubscriptionOptions heartbeat_options;
    heartbeat_options.callback_group = heartbeats_;
    heartbeat_sub_ = create_subscription<ControlHeartbeat>(
        "~/heartbeat", rclcpp::QoS(10),
        [this](const ControlHeartbeat& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          broker_.heartbeat(msg.lease_id, now_s());
        },
        heartbeat_options);

    expiry_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() { release_expired(); }, calls_);
    defaults_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() { apply_defaults(); }, calls_);
    publish();
  }

private:
  double now_s()
  {
    return steady_.now().seconds();
  }

  void on_claim(const ClaimControl::Request& req, ClaimControl::Response& res)
  {
    Plan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan = broker_.plan_claim(req.owner, req.groups, req.modes);
    }
    if (plan.ok) {
      plan.message = apply(plan.change);
      plan.ok = plan.message.empty();
    }
    res.success = plan.ok;
    if (!plan.ok) {
      res.message = plan.message;
      RCLCPP_WARN(get_logger(), "%s could not claim %s: %s", req.owner.c_str(), join(req.groups).c_str(),
                  plan.message.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      res.lease_id = broker_.commit_claim(req.owner, plan, now_s());
    }
    res.message = "Claimed";
    RCLCPP_INFO(get_logger(), "%s claimed %s as %s", req.owner.c_str(), join(req.groups).c_str(), res.lease_id.c_str());
    publish();
  }

  void on_release(const ReleaseControl::Request& req, ReleaseControl::Response& res)
  {
    const auto problem = release(req.lease_id);
    res.success = problem.empty();
    res.message = problem.empty() ? "Released" : problem;
    RCLCPP_INFO(get_logger(), "%s: %s", req.lease_id.c_str(), res.message.c_str());
  }

  // Returns the groups of a lease to their default modes. An empty string on success, the reason otherwise.
  std::string release(const std::string& lease_id)
  {
    Plan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan = broker_.plan_release(lease_id);
    }
    if (!plan.ok) {
      return plan.message;
    }
    const auto problem = apply(plan.change);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      broker_.commit_release(lease_id, plan, problem.empty());
    }
    publish();
    return problem.empty() ? "" : "Released, but the switch back failed: " + problem;
  }

  void release_expired()
  {
    std::vector<std::string> expired;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      expired = broker_.expired(now_s());
    }
    for (const auto& lease_id : expired) {
      {
        // A heartbeat may have arrived while an earlier lease was being released.
        std::lock_guard<std::mutex> lock(mutex_);
        const auto still = broker_.expired(now_s());
        if (std::find(still.begin(), still.end(), lease_id) == still.end()) {
          continue;
        }
      }
      const auto problem = release(lease_id);
      RCLCPP_WARN(get_logger(), "%s: no heartbeat, released%s", lease_id.c_str(),
                  problem.empty() ? "" : (". " + problem).c_str());
    }
  }

  // Puts every group nobody holds in its default mode, retried until it works once.
  void apply_defaults()
  {
    if (!list_client_->service_is_ready()) {
      return;
    }
    Plan plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan = broker_.plan_defaults();
    }
    const auto problem = plan.ok ? apply(plan.change) : plan.message;
    if (!problem.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Could not put the free groups in their default mode yet: %s", problem.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      broker_.commit_modes(plan);
    }
    defaults_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Every free group in its default mode");
    publish();
  }

  // Runs a switch through controller_manager. Returns an empty string on success, the reason otherwise.
  std::string apply(const Switch& change)
  {
    if (change.activate.empty() && change.deactivate.empty()) {
      return "";
    }
    if (!list_client_->wait_for_service(kControllerManagerTimeout)) {
      return "controller_manager does not answer";
    }
    auto listed = list_client_->async_send_request(std::make_shared<ListControllers::Request>());
    if (listed.wait_for(kControllerManagerTimeout) != std::future_status::ready) {
      list_client_->remove_pending_request(listed);
      return "controller_manager did not list its controllers";
    }
    std::set<std::string> active;
    std::set<std::string> loaded;
    // Held here: a range-for over listed.get()->controller would outlive the only owner of the response.
    const auto controllers = listed.get();
    for (const auto& controller : controllers->controller) {
      loaded.insert(controller.name);
      if (controller.state == "active") {
        active.insert(controller.name);
      }
    }
    auto request = std::make_shared<SwitchController::Request>();
    for (const auto& name : change.activate) {
      if (loaded.count(name) == 0) {
        return "Controller " + name + " is not loaded";
      }
      if (active.count(name) == 0) {
        request->activate_controllers.push_back(name);
      }
    }
    for (const auto& name : change.deactivate) {
      if (active.count(name) != 0) {
        request->deactivate_controllers.push_back(name);
      }
    }
    if (request->activate_controllers.empty() && request->deactivate_controllers.empty()) {
      return "";
    }
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout = rclcpp::Duration(kControllerManagerTimeout);
    auto switched = switch_client_->async_send_request(request);
    if (switched.wait_for(kControllerManagerTimeout * 2) != std::future_status::ready) {
      switch_client_->remove_pending_request(switched);
      return "controller_manager did not answer the switch";
    }
    const auto response = switched.get();
    if (!response->ok) {
      return "controller_manager refused the switch: " + response->message;
    }
    return "";
  }

  void publish()
  {
    ControlState msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& group : broker_.groups()) {
        duatic_controller_msgs::msg::ControlGroupState state;
        state.group = group.group;
        state.mode = group.mode;
        state.owner = group.owner;
        state.lease_id = group.lease_id;
        msg.groups.push_back(state);
      }
    }
    state_pub_->publish(msg);
  }

  std::mutex mutex_;
  Broker broker_;
  rclcpp::Clock steady_{ RCL_STEADY_TIME };
  rclcpp::CallbackGroup::SharedPtr clients_;
  rclcpp::CallbackGroup::SharedPtr calls_;
  rclcpp::CallbackGroup::SharedPtr heartbeats_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<ControlState>::SharedPtr state_pub_;
  rclcpp::Service<ClaimControl>::SharedPtr claim_srv_;
  rclcpp::Service<ReleaseControl>::SharedPtr release_srv_;
  rclcpp::Subscription<ControlHeartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  rclcpp::TimerBase::SharedPtr defaults_timer_;
};

}  // namespace duatic_control_broker

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<duatic_control_broker::ControlBrokerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
