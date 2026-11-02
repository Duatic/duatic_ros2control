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
 * products derived from this software without  hardware_interface::CallbackReturn activate();t specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "duatic_duadrive_interface/duadrive_interface_mock.hpp"

#include <algorithm>
#include <cmath>

namespace duatic::duadrive_interface
{
DuaDriveInterfaceMock::DuaDriveInterfaceMock(rclcpp::Logger logger)
  : DuaDriveInterfaceBase(logger), mock_acceleration_callback_([]() { return 0.0; })  // default: NO ACCELERATION
{
}

DuaDriveInterfaceMock::~DuaDriveInterfaceMock()
{
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::init(const DuaDriveInterfaceParameters& params)
{
  params_ = params;
  logger_ = rclcpp::get_logger("DuaDriveHardwareInterfaceMock_" + params.joint_name);
  generate_state_interface_descriptions();
  generate_command_interface_desriptions();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::activate()
{
  drive_info_.drive_name = "mock";
  drive_info_.drive_model = "DuaDrive_mock";
  drive_info_.drive_build_tag = "none";
  error_latched_ = false;

  // Same limits as configured on the real drive
  max_joint_velocity_ = params_.maximum_joint_velocity * hw_limits_gain;
  if (max_joint_velocity_ <= 0.0) {
    RCLCPP_FATAL_STREAM(logger_, "Implausible maximum joint velocity of " << max_joint_velocity_
                                                                          << " for drive: " << get_name());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO_STREAM(logger_, "Maximum joint velocity: " << params_.maximum_joint_velocity << " rad/s "
                                                         << " mock limit: " << max_joint_velocity_ << " rad/s");

  max_joint_torque_ = params_.maximum_joint_effort * hw_limits_gain;
  if (max_joint_torque_ <= 0.0) {
    RCLCPP_FATAL_STREAM(logger_,
                        "Implausible maximum joint torque of " << max_joint_torque_ << " for drive: " << get_name());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO_STREAM(logger_, "Maximum joint effort: " << params_.maximum_joint_effort << " Nm "
                                                       << " mock limit: " << max_joint_torque_ << " Nm");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::configure()
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::deactivate()
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DuaDriveInterfaceMock::read([[maybe_unused]] const rclcpp::Time& time,
                                                            [[maybe_unused]] const rclcpp::Duration& period)
{
  last_reading_update_ = std::chrono::system_clock::now();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DuaDriveInterfaceMock::write([[maybe_unused]] const rclcpp::Time& time,
                                                             const rclcpp::Duration& period)
{
  state_.joint_position_commanded = command_.joint_position;
  state_.joint_velocity_commanded = command_.joint_velocity;
  state_.joint_acceleration_commanded = command_.joint_acceleration;
  state_.joint_torque_commanded = command_.joint_torque;
  state_.joint_freeze_mode_commanded = command_.joint_freeze_mode;
  state_.current_drive_mode = active_mode_;
  state_.current_drive_state =
      error_latched_ ? rsl_drive_sdk::fsm::StateEnum::Error : rsl_drive_sdk::fsm::StateEnum::ControlOp;

  // Saturate the commanded torque as the real drive does
  const double joint_torque = get_saturated_joint_torque();

  if (error_latched_ || command_.joint_freeze_mode) {  // enforce freeze on error or if commanded
    state_.joint_velocity = 0.0;
    state_.joint_acceleration = 0.0;
  } else {
    const double dt = period.seconds();
    switch (active_mode_) {
      case rsl_drive_sdk::mode::ModeEnum::Freeze:
        state_.joint_velocity = 0.0;
        state_.joint_acceleration = 0.0;
        break;
      case rsl_drive_sdk::mode::ModeEnum::JointVelocity:
        // Especially support the joint velocity mode where we calculate the position from the last position and the
        // velocity
        state_.joint_velocity = command_.joint_velocity;
        state_.joint_position += command_.joint_velocity * dt;
        break;
      case rsl_drive_sdk::mode::ModeEnum::JointTorque:
      {
        state_.joint_velocity *=
            0.999;  // just always remove a tiny little bit of energy from the system before integrating
        // when there is no position/velocity commands given: integrate torque-induced acceleration
        const double mock_acceleration = mock_acceleration_callback_();
        state_.joint_position += ((0.5 * mock_acceleration * dt) + state_.joint_velocity) *
                                 dt;  // 0.5 a t^2 + v t  <- using PREVIOUS velocity
        state_.joint_velocity += mock_acceleration * dt;
        state_.joint_acceleration = mock_acceleration;
        state_.joint_torque = joint_torque;
      } break;
      default:
        state_.joint_torque = joint_torque;
        state_.joint_acceleration = command_.joint_acceleration;
        state_.joint_velocity = command_.joint_velocity;
        state_.joint_position = command_.joint_position;
    }
    // Mimic the real drive: freeze + error. The hardware interface then freezes the whole robot in the next cycle
    if (std::abs(state_.joint_velocity) > max_joint_velocity_) {
      RCLCPP_ERROR_STREAM(logger_, "Maximum velocity exceeded on joint: " << get_name() << " (" << state_.joint_velocity
                                                                          << " rad/s)");
      error_latched_ = true;
      state_.current_drive_state = rsl_drive_sdk::fsm::StateEnum::Error;
      state_.joint_velocity = 0.0;
      state_.joint_acceleration = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

void DuaDriveInterfaceMock::enforce_position(const double position)
{
  state_.joint_position = position;
  state_.joint_position_commanded = position;
  command_.joint_position = position;

  state_.joint_velocity = 0;
  state_.joint_velocity_commanded = 0;
  command_.joint_velocity = 0;
}

}  // namespace duatic::duadrive_interface
