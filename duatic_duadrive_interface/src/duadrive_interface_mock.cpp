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

namespace duatic::duadrive_interface
{
DuaDriveInterfaceMock::DuaDriveInterfaceMock(rclcpp::Logger logger) : DuaDriveInterfaceBase(logger)
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
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DuaDriveInterfaceMock::write([[maybe_unused]] const rclcpp::Time& time,
                                                             const rclcpp::Duration& period)
{
  state_.joint_position_commanded = command_.joint_position;
  state_.joint_velocity_commanded = command_.joint_velocity;
  state_.joint_acceleration_commanded = command_.joint_acceleration;
  state_.joint_torque_commanded = command_.joint_torque;

  if (active_mode_ == rsl_drive_sdk::mode::ModeEnum::Freeze) {
    return hardware_interface::return_type::OK;
  }

  if (active_mode_ == rsl_drive_sdk::mode::ModeEnum::JointVelocity) {
    // Especially support the joint velocity mode where we calculate the position from the last position and the
    // velocity
    state_.joint_velocity = command_.joint_velocity;
    state_.joint_position += command_.joint_velocity * period.to_chrono<std::chrono::duration<float>>().count();
  } else {
    state_.joint_torque = command_.joint_torque;
    state_.joint_acceleration = command_.joint_acceleration;
    state_.joint_velocity = command_.joint_velocity;
    state_.joint_position = command_.joint_position;
  }

  return hardware_interface::return_type::OK;
}
void DuaDriveInterfaceMock::enforce_position(const double position)
{
  state_.joint_position = position;
  state_.joint_position_commanded = position;
  command_.joint_position = position;
}
}  // namespace duatic::duadrive_interface
