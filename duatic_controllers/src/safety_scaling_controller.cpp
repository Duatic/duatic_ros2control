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

#include <duatic_controllers/safety_scaling_controller.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace duatic::controllers
{
SafetyScalingController::SafetyScalingController()
{
}
controller_interface::InterfaceConfiguration SafetyScalingController::command_interface_configuration() const
{
  // Claim the necessary command interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : params_.joints) {
    config.names.push_back(joint + "/" + max_torque_name);
    config.names.push_back(joint + "/" + max_velocity_name);
  }

  if (config.names.empty()) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "No joint passed - this controller cannot operate on this base");
  }

  return config;
}

controller_interface::InterfaceConfiguration SafetyScalingController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::CallbackReturn SafetyScalingController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<safety_scaling_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyScalingController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
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
SafetyScalingController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  max_torque_scaling_factor_command_interfaces_.clear();
  max_velocity_scaling_factor_command_interfaces_.clear();

  // Obtain a sorted list of the interfaces
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, max_torque_name,
                                                    max_torque_scaling_factor_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - scaling_factor_max_torque");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, max_velocity_name,
                                                    max_velocity_scaling_factor_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - scaling_factor_max_velocity");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // Expose it as ros2 parameters
  // We do this on purpose instead of a topic or a service call
  // as setting this parameter interrupts the realtime data communication
  auto set_param = [this](const std::string& name, const CommandInterfaceReference& interface) {
    const double current = duatic::controllers::compat::require_value(interface.get());

    if (!get_node()->has_parameter(name)) {
      get_node()->declare_parameter(name, current);
    } else {
      get_node()->set_parameter(rclcpp::Parameter(name, current));
    }
    return true;
  };

  for (std::size_t i = 0; i < params_.joints.size(); i++) {
    const std::string joint_name = params_.joints[i];

    const std::string param_name_base = joint_name + "/";
    const std::string scaling_factor_max_torque_param = param_name_base + max_torque_name;
    const std::string scaling_factor_max_velocity_param = param_name_base + max_velocity_name;

    try {
      set_param(scaling_factor_max_torque_param, max_torque_scaling_factor_command_interfaces_.at(i));
      set_param(scaling_factor_max_velocity_param, max_velocity_scaling_factor_command_interfaces_.at(i));

    } catch (const duatic::controllers::exceptions::MissingInterfaceValue& ex) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Failed to read command interface value while initializing scaling params for joint: "
                              << joint_name << "   " << ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyScalingController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SafetyScalingController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                  [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  auto&& node = get_node();
  for (std::size_t i = 0; i < params_.joints.size(); i++) {
    const std::string joint_name = params_.joints[i];

    const std::string param_name_base = joint_name + "/";
    const std::string scaling_factor_max_torque_param = param_name_base + max_torque_name;
    const std::string scaling_factor_max_velocity_param = param_name_base + max_velocity_name;

    // Read the parameter and clamp it to a sensible range (do not allow "overscaling")
    const auto max_torque_factor =
        std::clamp(node->get_parameter(scaling_factor_max_torque_param).as_double(), 0.0, 1.0);
    const auto max_velocity_factor =
        std::clamp(node->get_parameter(scaling_factor_max_velocity_param).as_double(), 0.0, 1.0);

    if (!max_torque_scaling_factor_command_interfaces_[i].get().set_value(max_torque_factor)) {
      // use .at(i) on purpose as this will throw an exception even if we had "luck" and [i] was successful
      RCLCPP_ERROR_STREAM(
          node->get_logger(),
          "Failed to set scaling factor for: " << max_torque_scaling_factor_command_interfaces_.at(i).get().get_name());
    }
    if (!max_velocity_scaling_factor_command_interfaces_[i].get().set_value(max_velocity_factor)) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Failed to set scaling factor for: "
                              << max_velocity_scaling_factor_command_interfaces_.at(i).get().get_name());
    }
  }

  return controller_interface::return_type::OK;
}
}  // namespace duatic::controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(duatic::controllers::SafetyScalingController, controller_interface::ControllerInterface)
