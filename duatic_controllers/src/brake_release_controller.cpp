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
#include <cmath>
#include <limits>

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

// Torque offsets as fractions of max_torque the sweep moves through in turn, to load the pins as little as possible
constexpr std::array<double, 7> sweep_levels{ 0.25, -0.25, 0.5, -0.5, 1.0, -1.0, 0.0 };
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
    if (mode_ == Mode::Auto) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
      config.names.emplace_back(joint + "/" + "p_gain");
      config.names.emplace_back(joint + "/" + "i_gain");
    }
    config.names.emplace_back(joint + "/" + "target_brake_state");
  }
  // The torque offset is chained onto the gravity compensation, which keeps carrying the load
  if (mode_ == Mode::Auto) {
    for (const auto& name : reference_joint_names()) {
      config.names.emplace_back(name + "/" + hardware_interface::HW_IF_EFFORT);
    }
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
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
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

  const std::size_t joint_count = params_.joints.size();
  if (mode_ == Mode::Auto && !params_.coupling.empty() && params_.coupling.size() != joint_count * joint_count) {
    RCLCPP_ERROR(get_node()->get_logger(), "'coupling' has %zu entries, expected %zu for %zu joints",
                 params_.coupling.size(), joint_count * joint_count, joint_count);
    return controller_interface::CallbackReturn::FAILURE;
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
  joint_p_gain_command_interfaces_.clear();
  joint_i_gain_command_interfaces_.clear();
  effort_reference_interfaces_.clear();
  brake_target_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  brake_released_state_interfaces_.clear();
  probes_.assign(params_.joints.size(), JointProbe{});
  joint_velocities_.assign(params_.joints.size(), 0.0);
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
            state_interfaces_, params_.joints, hardware_interface::HW_IF_VELOCITY, joint_velocity_state_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - velocity");
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
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, "p_gain",
                                                      joint_p_gain_command_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - p_gain");
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, "i_gain",
                                                      joint_i_gain_command_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - i_gain");
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, reference_joint_names(),
                                                      hardware_interface::HW_IF_EFFORT, effort_reference_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - effort reference");
      return controller_interface::CallbackReturn::FAILURE;
    }
    for (std::size_t i = 0; i < probes_.size(); i++) {
      double position{};
      try {
        position = duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());
        probes_[i].p_gain = duatic::controllers::compat::require_value(joint_p_gain_command_interfaces_.at(i).get());
        probes_[i].i_gain = duatic::controllers::compat::require_value(joint_i_gain_command_interfaces_.at(i).get());
      } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to read the state of joint '%s': %s", params_.joints[i].c_str(),
                     e.what());
        return controller_interface::CallbackReturn::ERROR;
      }
      // A hand back before the first update then holds the joint where it is
      command_joint(i, position);
    }
    // Without the position loop the offset alone acts on the pin, while the drive's damping stays
    for (std::size_t i = 0; i < probes_.size(); i++) {
      if (!joint_p_gain_command_interfaces_.at(i).get().set_value(0.0) ||
          !joint_i_gain_command_interfaces_.at(i).get().set_value(0.0)) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to disable the position loop of joint '%s'",
                    params_.joints[i].c_str());
      }
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  if (mode_ == Mode::Auto) {
    for (std::size_t i = 0; i < probes_.size(); i++) {
      if (probes_[i].level < sweep_levels.size()) {
        hand_back(i);
      }
    }
    // NaN is the unset reference, so the gravity compensation adds nothing even if it stays chained to another
    // controller
    for (auto& interface : effort_reference_interfaces_) {
      (void)interface.get().set_value(std::numeric_limits<double>::quiet_NaN());
    }
  }
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
  if (current_state_ == State::Init) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Releasing brakes");
    for (auto& interface : brake_target_command_interfaces_) {
      // Excite the brakes
      if (!interface.get().set_value<int>(1)) {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                            "Failed to put brake into 'excite' state for: " << interface.get().get_name());
      }
    }
    current_state_ = State::Commanding;
    last_status_log_time_ = time;
  }

  bool finished = false;
  // Auto mode sweeps a torque offset per drive through alternating directions and growing levels, until the motor lifts
  // the load off the pin and the brake releases
  if (mode_ == Mode::Auto && current_state_ == State::Commanding) {
    const double step = params_.torque_rate * period.seconds();
    try {
      for (std::size_t i = 0; i < joint_velocities_.size(); i++) {
        joint_velocities_[i] = duatic::controllers::compat::require_value(joint_velocity_state_interfaces_.at(i).get());
      }
    } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read the joint velocities: %s", e.what());
      return controller_interface::return_type::ERROR;
    }
    finished = true;
    for (std::size_t i = 0; i < params_.joints.size(); i++) {
      auto& probe = probes_[i];
      if (probe.level >= sweep_levels.size()) {
        continue;
      }

      double position{};
      try {
        position = duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());
      } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to read the position of joint '%s': %s",
                     params_.joints[i].c_str(), e.what());
        return controller_interface::return_type::ERROR;
      }

      if (is_brake_released(brake_released_state_interfaces_.at(i).get())) {
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Brake released, holding joint: " << params_.joints[i]);
        probe.level = sweep_levels.size();
        hold_brake(i);
      } else {
        const double level = sweep_levels[probe.level] * params_.max_torque;
        probe.offset = std::clamp(level, probe.offset - step, probe.offset + step);
        if (probe.offset == level) {
          // A level is held while the drive moves in the pushed direction, so a free joint travels until its release is
          // detected, while a drive ringing on its pin does not hold it
          const double direction = (level > 0.0) - (level < 0.0);
          if (drive_velocity(i) * direction < params_.stop_velocity) {
            if (++probe.level < sweep_levels.size()) {
              RCLCPP_INFO_STREAM(get_node()->get_logger(), "No release up to "
                                                               << std::abs(level)
                                                               << " Nm, changing direction: " << params_.joints[i]);
            }
          }
        }
      }

      // The position target follows the joint, so the position loop takes over where the joint is, and a coupled drive
      // already handed back keeps holding its own joint
      command_joint(i, position);
      if (probe.level < sweep_levels.size()) {
        finished = false;
      } else {
        hand_back(i);
      }
    }
    command_offsets();
  }
  // Manual mode holds every brake as soon as it is released and waits until all are
  if (mode_ == Mode::Manual && current_state_ == State::Commanding) {
    for (std::size_t i = 0; i < brake_released_state_interfaces_.size(); i++) {
      if (is_brake_released(brake_released_state_interfaces_.at(i).get())) {
        hold_brake(i);
      }
    }
    finished = all_brakes_released();
  }

  if (finished) {
    // Released brakes are in hold already, the others stayed excited in case a coupled drive unloaded them late
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
  for (std::size_t i = 0; i < brake_target_command_interfaces_.size(); i++) {
    hold_brake(i);
  }
}

void BrakeReleaseController::hold_brake(std::size_t index)
{
  auto& interface = brake_target_command_interfaces_.at(index).get();
  if (!interface.set_value<int>(2)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Failed to put brake into 'holding' state for: " << interface.get_name());
  }
}

void BrakeReleaseController::command_joint(std::size_t index, double position)
{
  if (!joint_position_command_interfaces_.at(index).get().set_value(position) ||
      !joint_velocity_command_interfaces_.at(index).get().set_value(0.0)) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to set command for joint '%s'", params_.joints[index].c_str());
  }
}

void BrakeReleaseController::command_offsets()
{
  const std::size_t joint_count = probes_.size();
  for (std::size_t i = 0; i < joint_count; i++) {
    // Without a coupling every joint has its own drive
    double effort = probes_[i].offset;
    if (!params_.coupling.empty()) {
      effort = 0.0;
      for (std::size_t k = 0; k < joint_count; k++) {
        effort += params_.coupling[i * joint_count + k] * probes_[k].offset;
      }
    }
    if (!effort_reference_interfaces_.at(i).get().set_value(effort)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set the effort offset of joint '%s'", params_.joints[i].c_str());
    }
  }
}

double BrakeReleaseController::drive_velocity(std::size_t index) const
{
  // The drive velocities follow from the joint velocities through the transpose of the torque coupling
  if (params_.coupling.empty()) {
    return joint_velocities_[index];
  }
  const std::size_t joint_count = joint_velocities_.size();
  double velocity = 0.0;
  for (std::size_t i = 0; i < joint_count; i++) {
    velocity += params_.coupling[i * joint_count + index] * joint_velocities_[i];
  }
  return velocity;
}

void BrakeReleaseController::hand_back(std::size_t index)
{
  auto& probe = probes_.at(index);
  probe.offset = 0.0;
  if (!joint_p_gain_command_interfaces_.at(index).get().set_value(probe.p_gain) ||
      !joint_i_gain_command_interfaces_.at(index).get().set_value(probe.i_gain)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to hand back joint '%s'", params_.joints[index].c_str());
  }
}

std::vector<std::string> BrakeReleaseController::reference_joint_names() const
{
  std::vector<std::string> names;
  for (const auto& joint : params_.joints) {
    names.emplace_back(params_.gravity_compensation_controller + "/" + joint);
  }
  return names;
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
