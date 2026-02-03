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

#pragma once

// System
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <thread>

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"

// sdk
#include <ethercat_sdk_master/EthercatMasterSingleton.hpp>
#include <rsl_drive_sdk/Drive.hpp>

namespace duatic_ros2control_hardware
{

struct DuaDriveInterfaceParameters
{
  std::string ethercat_bus;
  std::string joint_name;
  std::string drive_parameter_file_path;
  std::string drive_default_parameter_file_path;
  int device_address;
};
struct DuaDriveInterfaceState
{
  // This is the current state read from the drive
  double joint_position_{};
  double joint_velocity_{};
  double joint_acceleration_{};
  double joint_torque_{};

  double temperature_system{};
  double temperature_coil_A{};
  double temperature_coil_B{};
  double temperature_coil_C{};
  double bus_voltage{};

  // This is what the drive tells us (as feedback) what we commanded
  double position_commanded_{};
  double velocity_commanded_{};
  double acceleration_commanded_{};
  double effort_commanded_{};
  double command_freeze_mode_{};
};
struct DuaDriveInterfaceCommands
{
  // This is what we shall command the drive
  double joint_position_command_{};
  double joint_velocity_command_{};
  double joint_acceleration_command_{};
  double joint_torque_command_{};

  double p_gain_{};
  double i_gain_{};
  double d_gain_{};
};

class DuaDriveInterface
{
public:
  DuaDriveInterface(rclcpp::Logger logger);
  hardware_interface::CallbackReturn on_init(const DuaDriveInterfaceParameters& params);
  std::vector<hardware_interface::StateInterface> export_state_interfaces() const;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() const;

  hardware_interface::CallbackReturn on_activate();
  hardware_interface::CallbackReturn on_configure();
  hardware_interface::CallbackReturn on_deactivate();

  hardware_interface::return_type read();
  hardware_interface::return_type write();

  hardware_interface::return_type
  prepare_command_mode_switch([[maybe_unused]] const std::vector<std::string>& start_interfaces,
                              [[maybe_unused]] const std::vector<std::string>& stop_interfaces);
  hardware_interface::return_type
  perform_command_mode_switch([[maybe_unused]] const std::vector<std::string>& start_interfaces,
                              [[maybe_unused]] const std::vector<std::string>& stop_interfaces);

private:
  rclcpp::Logger logger_;

  ecat_master::EthercatMasterSingleton::Handle ecat_master_handle_;
  rsl_drive_sdk::DriveEthercatDevice::SharedPtr drive_;

  DuaDriveInterfaceState state_;
  DuaDriveInterfaceCommands commands_;

  rsl_drive_sdk::mode::ModeEnum active_mode_{ rsl_drive_sdk::mode::ModeEnum::Freeze };
  rsl_drive_sdk::mode::ModeEnum previous_mode_{ rsl_drive_sdk::mode::ModeEnum::JointVelocity };
};
}  // namespace duatic_ros2control_hardware