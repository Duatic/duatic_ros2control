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

#pragma once

// System
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <unordered_map>

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"

// sdk

#include "duatic_duadrive_interface/duadrive_utils.hpp"

namespace duatic::duadrive_interface
{
struct DuaDriveInterfaceParameters
{
  std::string ethercat_bus;
  std::string joint_name;
  std::string drive_parameter_file_path;
  int device_address;
};
struct DuaDriveInterfaceInfo
{
  std::string drive_name;
  std::string drive_model;
  std::string drive_build_tag;
};
struct DuaDriveInterfaceState
{
  // This is the current state read from the drive
  double joint_position{};
  double joint_velocity{};
  double joint_acceleration{};
  double joint_torque{};

  double current_q{};
  double current_d{};
  double current_coil_A{};
  double current_coil_B{};
  double current_coil_C{};

  double voltage_coil_A{};
  double voltage_coil_B{};
  double voltage_coil_C{};

  double power_active{};
  double power_reactive{};

  double temperature_system{};
  double temperature_coil_A{};
  double temperature_coil_B{};
  double temperature_coil_C{};
  double bus_voltage{};

  // This is what the drive tells us (as feedback) what we commanded
  double joint_position_commanded{};
  double joint_velocity_commanded{};
  double joint_acceleration_commanded{};
  double joint_torque_commanded{};
  double joint_freeze_mode_commanded{};
  double current_q_commanded{};
};
struct DuaDriveInterfaceCommands
{
  // This is what we shall command the drive
  double joint_position{};
  double joint_velocity{};
  double joint_acceleration{};
  double joint_torque{};
  bool joint_freeze_mode{ true };  // Init with true to enforce start in freeze mode
  double p_gain{};
  double i_gain{};
  double d_gain{};
};

class DuaDriveInterfaceBase
{
public:
  explicit DuaDriveInterfaceBase(rclcpp::Logger logger);
  virtual ~DuaDriveInterfaceBase();

  // As it is inherently unsafe to copy this class delete the copy ctr
  DuaDriveInterfaceBase(const DuaDriveInterfaceBase&) = delete;
  DuaDriveInterfaceBase& operator=(const DuaDriveInterfaceBase&) = delete;
  /**
   * @brief perform initialization of the duadrive interface component
   */
  virtual hardware_interface::CallbackReturn init(const DuaDriveInterfaceParameters& params) = 0;
  /**
   * @brief obtain the configured joint name of this drive
   */
  const std::string& get_name() const;
  /**
   * @brief perform the activation procedure of this drive component
   * @note this will perform a read internally trying to obtain the current position values
   */
  virtual hardware_interface::CallbackReturn activate();
  /**
   * @brief perform the configuration procedure of this drive component
   * This will simply attach the drive representation to the ethercat bus
   */
  virtual hardware_interface::CallbackReturn configure();
  /**
   * @brief perform the deactivation procedure of this drive component
   * This will try to put the drive into freeze mode
   */
  virtual hardware_interface::CallbackReturn deactivate();
  /**
   * @brief perform a single read on the drive component
   */
  virtual hardware_interface::return_type read();
  /**
   * @brief perform a single write on the drive component
   */
  virtual hardware_interface::return_type write();

  void stage_command(const DuaDriveInterfaceCommands command)
  {
    command_ = command;
  }
  const DuaDriveInterfaceCommands& get_last_command() const
  {
    return command_;
  }
  const DuaDriveInterfaceState& get_last_state() const
  {
    return state_;
  }

  void configure_drive_mode(rsl_drive_sdk::mode::ModeEnum mode)
  {
    previous_mode_ = active_mode_;
    active_mode_ = mode;

    // Configure the current position as target position to avoid sudden jumps
    command_.joint_position = state_.joint_position;
  }

  const DuaDriveInterfaceInfo get_drive_info() const
  {
    return drive_info_;
  }

  const std::vector<hardware_interface::InterfaceDescription> get_state_interface_descriptions() const
  {
    return state_interface_descriptions_;
  }
  const std::vector<hardware_interface::InterfaceDescription> get_command_interface_descriptions() const
  {
    return command_interface_descriptions_;
  }
  auto& get_default_state_mapping()
  {
    return state_interface_mapping_;
  }
  auto& get_default_command_mapping()
  {
    return command_interface_mapping_;
  }

protected:
  rclcpp::Logger logger_;

  DuaDriveInterfaceState state_;
  DuaDriveInterfaceCommands command_;
  DuaDriveInterfaceParameters params_;
  DuaDriveInterfaceInfo drive_info_;

  rsl_drive_sdk::mode::ModeEnum active_mode_{ rsl_drive_sdk::mode::ModeEnum::Freeze };
  rsl_drive_sdk::mode::ModeEnum previous_mode_{ rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorquePidGains };

  std::vector<hardware_interface::InterfaceDescription> state_interface_descriptions_;
  std::vector<hardware_interface::InterfaceDescription> command_interface_descriptions_;

  std::unordered_map<std::string, SupportedVariant> state_interface_mapping_;
  std::unordered_map<std::string, SupportedVariant> command_interface_mapping_;

  void generate_state_interface_descriptions();
  void generate_command_interface_desriptions();
};

}  // namespace duatic::duadrive_interface
