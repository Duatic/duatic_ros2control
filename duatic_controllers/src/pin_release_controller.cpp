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

#include <duatic_controllers/pin_release_controller.hpp>

#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_list_macros.hpp>

/**
 * @file pin_release_controller.cpp
 * @brief Implements the brake pin release controller lifecycle and update loop.
 */

namespace duatic::controllers
{

PinReleaseController::PinReleaseController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration PinReleaseController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint : params_.joints) {
    config.names.emplace_back(joint + "/target_brake_state");
  }

  return config;
}

controller_interface::InterfaceConfiguration PinReleaseController::state_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::NONE, {} };
}

controller_interface::CallbackReturn PinReleaseController::on_init()
{
  try {
    param_listener_ = std::make_unique<pin_release_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PinReleaseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PinReleaseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  brake_target_command_interfaces_.clear();
  current_state_ = State::Releasing;
  activate_time_ = std::nullopt;

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, "target_brake_state",
                                                    brake_target_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - target_brake_state");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PinReleaseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PinReleaseController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PinReleaseController::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PinReleaseController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PinReleaseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                               [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  if (!activate_time_) {
    activate_time_ = time;
    RCLCPP_INFO(get_node()->get_logger(), "Releasing brake pins");
  }

  if (current_state_ == State::Releasing) {
    for (auto& interface : brake_target_command_interfaces_) {
      interface.get().set_value<int>(1);
    }

    const double elapsed = (time - activate_time_.value()).seconds();
    if (elapsed >= params_.hold_timeout) {
      RCLCPP_INFO(get_node()->get_logger(), "Switching brake pins to holding state");
      current_state_ = State::Holding;
    }
  }

  if (current_state_ == State::Holding) {
    for (auto& interface : brake_target_command_interfaces_) {
      interface.get().set_value<int>(2);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace duatic::controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::PinReleaseController, controller_interface::ControllerInterface)
