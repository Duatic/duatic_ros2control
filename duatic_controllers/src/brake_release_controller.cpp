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

#include <duatic_controllers/brake_release_controller.hpp>

// C++ system headers
#include <array>

// Other headers
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace duatic::controllers
{
namespace
{
/**
 * @brief whether the hardware reports the brake behind this state interface as released
 */
bool is_brake_released(const hardware_interface::LoanedStateInterface& interface)
{
  const auto value = duatic::controllers::compat::try_get_value(interface);
  return value && *value != 0.0;
}

// Fractions of max_torque tried in both directions before the next higher one, to load the pins as little as possible
constexpr std::array<double, 3> torque_fractions{ 0.25, 0.5, 1.0 };
// A probe that reaches this attempt has stopped searching
constexpr std::size_t max_attempts = 2 * torque_fractions.size();

/**
 * @brief direction a probe pushes the joint in during the given attempt
 */
double probe_direction(std::size_t attempt)
{
  return attempt % 2 == 0 ? 1.0 : -1.0;
}
}  // namespace

BrakeReleaseController::BrakeReleaseController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration BrakeReleaseController::command_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    // Leaving effort unclaimed lets a gravity compensation add its feedforward torque to the position control
    if (mode_ == Mode::Auto) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    config.names.emplace_back(joint + "/" + "target_brake_state");
  }

  return config;
}

controller_interface::InterfaceConfiguration BrakeReleaseController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    if (mode_ == Mode::Auto) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
    config.names.emplace_back(joint + "/" + "brake_released");
  }

  return config;
}

controller_interface::CallbackReturn BrakeReleaseController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<brake_release_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Release mode: " << params_.mode);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // check if joints are empty
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  try {
    // obtain the operation mode
    mode_ = mode_from_str(params_.mode);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during configuration: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // A controller node lives in the namespace of its controller manager
  const std::string switch_controller_service = "controller_manager/switch_controller";
  switch_controller_client_ =
      get_node()->create_client<controller_manager_msgs::srv::SwitchController>(switch_controller_service);
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial setup done");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // clear out vectors in case of restart (reactivation to be precise)
  joint_position_command_interfaces_.clear();
  joint_velocity_command_interfaces_.clear();
  brake_target_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_effort_state_interfaces_.clear();
  brake_released_state_interfaces_.clear();
  probes_.assign(params_.joints.size(), JointProbe{});
  last_state_change_time_ = std::nullopt;
  current_state_ = State::Init;

  // get the actual interface in an ordered way (same order as the joints parameter)
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "brake_released",
                                                    brake_released_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - brake_released");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, "target_brake_state",
                                                    brake_target_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - target_brake_state");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (mode_ == Mode::Auto) {
    if (!controller_interface::get_ordered_interfaces(
            state_interfaces_, params_.joints, hardware_interface::HW_IF_POSITION, joint_position_state_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - position");
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!controller_interface::get_ordered_interfaces(
            state_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT, joint_effort_state_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - effort");
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints,
                                                      hardware_interface::HW_IF_POSITION,
                                                      joint_position_command_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - position");
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints,
                                                      hardware_interface::HW_IF_VELOCITY,
                                                      joint_velocity_command_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - velocity");
      return controller_interface::CallbackReturn::FAILURE;
    }
    for (std::size_t i = 0; i < probes_.size(); i++) {
      const auto position = duatic::controllers::compat::try_get_value(joint_position_state_interfaces_.at(i).get());
      if (!position) {
        RCLCPP_WARN(get_node()->get_logger(), "No initial position for joint '%s'", params_.joints[i].c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      probes_[i].target = *position;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  hold_brakes();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BrakeReleaseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                 [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  // 1. In all mode at the beginning we need to excite the brake (high power try to pull the pin)
  if (current_state_ == State::Init && !last_state_change_time_) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Releasing brakes");
    for (auto& interface : brake_target_command_interfaces_) {
      // Excite the brakes
      if (!interface.get().set_value<int>(1)) {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                            "Failed to put brake into 'excite' state for: " << interface.get().get_name());
      }
    }
    current_state_ = State::Commanding;
    last_state_change_time_ = time;
    last_status_log_time_ = time;
  }

  bool finished = false;
  // Auto mode pushes each joint with a growing torque in alternating directions and escalating torque limits, until the
  // motor lifts the load off the pin and the brake releases
  if (mode_ == Mode::Auto && current_state_ == State::Commanding) {
    const double step = params_.probe_velocity * period.seconds();
    finished = true;
    for (std::size_t i = 0; i < params_.joints.size(); i++) {
      double position{};
      double effort{};
      try {
        position = duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());
        effort = duatic::controllers::compat::require_value(joint_effort_state_interfaces_.at(i).get());
      } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to read state of joint '%s': %s", params_.joints[i].c_str(),
                     e.what());
        return controller_interface::return_type::ERROR;
      }

      auto& probe = probes_[i];
      if (probe.attempt < max_attempts) {
        const double direction = probe_direction(probe.attempt);
        const double torque_limit = params_.max_torque * torque_fractions[probe.attempt / 2];
        // While the joint is blocked the growing position error raises the drive torque in the probing direction
        const bool blocked = effort * direction > torque_limit;
        if (is_brake_released(brake_released_state_interfaces_.at(i).get())) {
          RCLCPP_INFO_STREAM(get_node()->get_logger(), "Brake released, holding joint: " << params_.joints[i]);
          probe.target = position;
          probe.attempt = max_attempts;
        } else if (blocked) {
          // Restarting from the measured position takes the extra torque off the pin at once
          probe.target = position;
          if (++probe.attempt < max_attempts) {
            RCLCPP_INFO_STREAM(get_node()->get_logger(),
                               "No release up to " << torque_limit << " Nm, changing direction: " << params_.joints[i]);
          }
        }
      }
      if (probe.attempt < max_attempts) {
        probe.target += probe_direction(probe.attempt) * step;
        finished = false;
      }
      command_joint(i, probe.target);
    }
  }
  // Manual mode waits until all brakes are released or a user configurable timeout expires
  if (mode_ == Mode::Manual && current_state_ == State::Commanding) {
    finished = all_brakes_released() || time - last_state_change_time_.value() >
                                            rclcpp::Duration(std::chrono::duration<double>(params_.timeout));
  }

  if (finished) {
    hold_brakes();
    if (all_brakes_released()) {
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "All brakes released and put to hold state, deactivating");
      request_deactivation();
    } else {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Brakes put to hold state, not released: " << unreleased_joints());
    }
    current_state_ = State::Holding;
  }

  if (current_state_ == State::Commanding && time - last_status_log_time_ > rclcpp::Duration(3, 0)) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Brakes not released yet: " << unreleased_joints());
    last_status_log_time_ = time;
  }

  return controller_interface::return_type::OK;
}
bool BrakeReleaseController::all_brakes_released() const
{
  return std::all_of(brake_released_state_interfaces_.begin(), brake_released_state_interfaces_.end(),
                     [](const auto& interface) { return is_brake_released(interface.get()); });
}

std::string BrakeReleaseController::unreleased_joints() const
{
  std::string names;
  for (std::size_t i = 0; i < brake_released_state_interfaces_.size(); i++) {
    if (!is_brake_released(brake_released_state_interfaces_.at(i).get())) {
      names += (names.empty() ? "" : ", ") + params_.joints.at(i);
    }
  }
  return names;
}

void BrakeReleaseController::hold_brakes()
{
  for (auto& interface : brake_target_command_interfaces_) {
    if (!interface.get().set_value<int>(2)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Failed to put brake into 'holding' state for: " << interface.get().get_name());
    }
  }
}

void BrakeReleaseController::command_joint(std::size_t index, double position)
{
  // A velocity target would add its damping torque at once instead of letting the position error ramp it up
  if (!joint_position_command_interfaces_.at(index).get().set_value(position) ||
      !joint_velocity_command_interfaces_.at(index).get().set_value(0.0)) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for joint '%s'", params_.joints[index].c_str());
  }
}

void BrakeReleaseController::request_deactivation()
{
  if (!switch_controller_client_->service_is_ready()) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Cannot deactivate, service not available: " << switch_controller_client_->get_service_name());
    return;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers.push_back(get_node()->get_name());
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  switch_controller_client_->async_send_request(
      request, [logger = get_node()->get_logger()](
                   rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        if (!future.get()->ok) {
          RCLCPP_ERROR_STREAM(logger, "Failed to deactivate: " << future.get()->message);
        }
      });
}

controller_interface::CallbackReturn
BrakeReleaseController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace duatic::controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::BrakeReleaseController, controller_interface::ControllerInterface)
