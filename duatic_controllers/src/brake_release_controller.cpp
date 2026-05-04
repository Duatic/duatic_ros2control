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

#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include <duatic_controllers/brake_release_controller.hpp>

// C++ system headers
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

#include <pinocchio/collision/collision.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/check-data.hpp>

// Other headers
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace duatic::controllers
{

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
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
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
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.emplace_back(joint + "/" + "acceleration_commanded");
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
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Start IK worker thread
  // IK worker will be started in on_activate after Pinocchio model is built

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

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // clear out vectors in case of restart
  joint_position_command_interfaces_.clear();
  joint_velocity_command_interfaces_.clear();

  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  joint_acceleration_state_interfaces_.clear();

  // get the actual interface in an ordered way (same order as the joints parameter)
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
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "acceleration_commanded",
                                                    joint_acceleration_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - acceleration");
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
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BrakeReleaseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                 [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  return controller_interface::return_type::OK;
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
