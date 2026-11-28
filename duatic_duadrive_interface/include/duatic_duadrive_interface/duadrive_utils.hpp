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

/*stl*/
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <string>

/*drive sdk*/
#include <duatic_duadrive_sdk/v1/duadrive.hpp>

/*project*/
#include "duatic_duadrive_interface/interface_utils.hpp"

namespace duatic::duadrive_interface
{

/**
 * @brief helper function to sanitize command inputs
 * In case the command is nan or inf it is set to 0
 * @return false in case the input has been sanitized
 */
constexpr bool sanitize_command_input(double& cmd)
{
  if (std::isnan(cmd) || std::isinf(cmd)) {
    cmd = 0.0;
    return false;
  }
  return true;
}

/**
 * @brief true if at least one warning flag is set
 */
inline bool any_set(const duadrive_sdk::v1::Warnings& w)
{
  return w.over_temperature_bridge || w.over_temperature_stator || w.over_temperature_cpu || w.encoder_outlier_motor ||
         w.encoder_outlier_gear || w.encoder_outlier_joint || w.incomplete_calibration || w.encoder_crc_gear ||
         w.encoder_crc_joint;
}

/**
 * @brief true if at least one error flag is set
 */
inline bool any_set(const duadrive_sdk::v1::Errors& e)
{
  return e.invalid_joint_torque || e.pdo_timeout || e.direct_drive_elec_zero_offset_invalid ||
         e.max_velocity_exceeded || e.joint_position_limits_soft || e.joint_position_limits_hard;
}

/**
 * @brief true if at least one fatal flag is set
 */
inline bool any_set(const duadrive_sdk::v1::Fatals& f)
{
  return f.over_temperature_bridge || f.over_temperature_stator || f.over_temperature_cpu || f.motor_encoder ||
         f.current_sensor || f.over_voltage || f.under_voltage || f.joint_motor_encoder;
}

/**
 * @brief helper function which prints the warnings, errors and fatals raised or cleared between two status words via
 * ROS2 logging. Raised flags are logged with their severity, cleared flags as info
 */
inline void print_drive_status_changes(const std::string& drive_name, const duadrive_sdk::v1::StatusEvent& event,
                                       rclcpp::Logger& logger)
{
  if (any_set(event.get_warnings_raised())) {
    RCLCPP_WARN_STREAM(logger, "[" << drive_name << "]:Warning: " << event.get_warnings_raised());
  }
  if (any_set(event.get_errors_raised())) {
    RCLCPP_ERROR_STREAM(logger, "[" << drive_name << "]:Error: " << event.get_errors_raised());
  }
  if (any_set(event.get_fatals_raised())) {
    RCLCPP_FATAL_STREAM(logger, "[" << drive_name << "]:Fatal: " << event.get_fatals_raised());
  }
  if (any_set(event.get_warnings_cleared())) {
    RCLCPP_INFO_STREAM(logger, "[" << drive_name << "]:Warning disappeared: " << event.get_warnings_cleared());
  }
  if (any_set(event.get_errors_cleared())) {
    RCLCPP_INFO_STREAM(logger, "[" << drive_name << "]:Error disappeared: " << event.get_errors_cleared());
  }
  if (any_set(event.get_fatals_cleared())) {
    RCLCPP_INFO_STREAM(logger, "[" << drive_name << "]:Fatal disappeared: " << event.get_fatals_cleared());
  }
}

/**
 * @brief helper function which prints all warnings, errors and fatals of a status word via ROS2 logging
 */
inline void print_drive_status(const std::string& drive_name, const duadrive_sdk::v1::Statusword& status_word,
                               rclcpp::Logger& logger)
{
  if (status_word.has_warnings()) {
    RCLCPP_WARN_STREAM(logger, "[" << drive_name << "]:Warning: " << status_word.get_warnings());
  }
  if (status_word.has_errors()) {
    RCLCPP_ERROR_STREAM(logger, "[" << drive_name << "]:Error: " << status_word.get_errors());
  }
  if (status_word.has_fatals()) {
    RCLCPP_FATAL_STREAM(logger, "[" << drive_name << "]:Fatal: " << status_word.get_fatals());
  }
  if (!status_word.has_warnings() && !status_word.has_errors() && !status_word.has_fatals()) {
    RCLCPP_INFO_STREAM(logger, "[" << drive_name << "]:"
                                   << "Drive status: OK");
  }
}

inline const std::set<std::string> relevant_interface_types{ "position", "velocity", "effort", "freeze_mode" };
/**
 * @brief select the new drive mode from the selected (claimed) drive interfaces
 */
inline duadrive_sdk::v1::ControlMode select_mode(const std::set<std::string>& interfaces, rclcpp::Logger& logger_)
{
  // 1. extract the types of all selected new interfaces
  std::set<std::string> interface_types;
  for (const auto& interface : interfaces) {
    const auto interface_type = extract_interface_type(interface);

    // For the drive mode selection only certain interface types are relevant
    if (relevant_interface_types.contains(interface_type)) {
      interface_types.insert(interface_type);
    }
  }

  // The freeze mode interface always overrides all other modes
  // This means: WHEN the freeze_mode interface is claimend we go into freeze mode !
  if (interface_types.find("freeze_mode") != interface_types.end()) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: Freeze");
    return duadrive_sdk::v1::ControlMode::Freeze;
  }

  // 2. start to select mode depending on selection
  // Option 1: only one interface selected - choose between position, velocity, effort
  if (interface_types.find("position") != interface_types.end() && interface_types.size() == 1) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPosition");
    return duadrive_sdk::v1::ControlMode::JointPosition;
  }

  if (interface_types.find("velocity") != interface_types.end() && interface_types.size() == 1) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointVelocity");
    return duadrive_sdk::v1::ControlMode::JointVelocity;
  }

  if (interface_types.find("effort") != interface_types.end() && interface_types.size() == 1) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointTorque");
    return duadrive_sdk::v1::ControlMode::JointTorque;
  }

  // Option 2:  2-tuple Combinations of the modes above
  if (interface_types.find("position") != interface_types.end() &&
      interface_types.find("velocity") != interface_types.end() && interface_types.size() == 2) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPositionVelocity");
    return duadrive_sdk::v1::ControlMode::JointPositionVelocity;
  }

  if (interface_types.find("position") != interface_types.end() &&
      interface_types.find("effort") != interface_types.end() && interface_types.size() == 2) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPositionTorque");
    return duadrive_sdk::v1::ControlMode::JointPositionTorque;
  }
  // Option 3: 3-tuple combinations of the modes above
  if (interface_types.find("position") != interface_types.end() &&
      interface_types.find("velocity") != interface_types.end() &&
      interface_types.find("effort") != interface_types.end() && interface_types.size() == 3) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPositionVelocityTorquePID");
    return duadrive_sdk::v1::ControlMode::JointPositionVelocityTorquePidGains;
  }

  // This is our fallback mode (Freeze for safety reasons)
  RCLCPP_WARN_STREAM(logger_, "Fallback to: Select drive mode: Freeze (Note this is usually do to an invalid "
                              "combination of interfaces)");
  for (const auto& itype : interface_types) {
    RCLCPP_WARN_STREAM(logger_, itype);
  }
  return duadrive_sdk::v1::ControlMode::Freeze;
}
/**
 * @brief A list with all modes that do not use the commanded position (e.g. velocity control)
 * TODO(firesurfer) this is not constepxr friendly at the moment
 */
inline std::set<duadrive_sdk::v1::ControlMode> modes_without_position_control()
{
  using Modes = duadrive_sdk::v1::ControlMode;
  return std::set{ Modes::Freeze, Modes::Current, Modes::JointTorque, Modes::JointVelocity, Modes::MotorVelocity };
};

inline duadrive_sdk::v1::Command build_command(const duadrive_sdk::v1::ControlMode mode, const double joint_position,
                                               const double joint_velocity, const double joint_torque,
                                               const duadrive_sdk::v1::PidGains& gains)
{
  using CM = duadrive_sdk::v1::ControlMode;
  using namespace duadrive_sdk::v1;  // NOLINT(build/namespaces)
  switch (mode) {
    case CM::JointPosition:
      return JointPositionCommand(joint_position);
    case CM::JointPositionTorque:
      return JointPositionTorqueCommand(joint_position, static_cast<float>(joint_torque));
    case CM::JointVelocity:
      return JointVelocityCommand(static_cast<float>(joint_velocity));
    case CM::JointPositionVelocity:
      return JointPositionVelocityCommand(static_cast<float>(joint_position), static_cast<float>(joint_velocity));
    case CM::JointPositionVelocityTorque:
      return JointPositionVelocityTorqueCommand(joint_position, static_cast<float>(joint_velocity),
                                                static_cast<float>(joint_torque));
    case CM::JointPositionVelocityTorquePidGains:
      return JointPositionVelocityTorquePidGainsCommand(joint_position, static_cast<float>(joint_velocity),
                                                        static_cast<float>(joint_torque), gains);
    case CM::Freeze:
      return FreezeCommand();
    case CM::Disable:
      return DisableCommand();
    case CM::JointTorque:
      return JointTorqueCommand(static_cast<float>(joint_torque));
    default:
      logging::error() << "Unhandled command control mode" << mode << std::endl;
      return DisableCommand();
  }
}

}  // namespace duatic::duadrive_interface
