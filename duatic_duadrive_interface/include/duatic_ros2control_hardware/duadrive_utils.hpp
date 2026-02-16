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
#include <rsl_drive_sdk/Statusword.hpp>
#include <rsl_drive_sdk/mode/ModeEnum.hpp>

/*project*/
#include "duatic_ros2control_hardware/interface_utils.hpp"

namespace duatic_ros2control_hardware
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
 * @brief helper function which print the output of the sdk function via ROS2 logging
 */
inline void print_drive_status_changes(
    const std::string& drive_name, const rsl_drive_sdk::Statusword& current_status_word,
    rsl_drive_sdk::Statusword previous_status_word /*create copy because getmessagesDiff is not const declared*/,
    rclcpp::Logger& logger)
{
  std::vector<std::string> infos;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  std::vector<std::string> fatals;

  current_status_word.getMessagesDiff(previous_status_word, infos, warnings, errors, fatals);

  for (const auto& msg : infos) {
    RCLCPP_INFO_STREAM(logger, "[" << drive_name << "]:" << msg);
  }
  for (const auto& msg : warnings) {
    RCLCPP_WARN_STREAM(logger, "[" << drive_name << "]:" << msg);
  }
  for (const auto& msg : errors) {
    RCLCPP_ERROR_STREAM(logger, "[" << drive_name << "]:" << msg);
  }
  for (const auto& msg : fatals) {
    RCLCPP_FATAL_STREAM(logger, "[" << drive_name << "]:" << msg);
  }
}

inline rsl_drive_sdk::mode::ModeEnum select_mode(const std::vector<std::string>& start_interfaces,
                                                 [[maybe_unused]] const std::vector<std::string>& stop_interfaces,
                                                 rclcpp::Logger& logger_)
{
  // 1. extract the types of all selected new interfaces
  std::set<std::string> interface_types;
  for (const auto& interface : start_interfaces) {
    interface_types.insert(extract_interface_type(interface));
  }

  // 2. start to select mode depending on selection
  // Option 1: only one interface selected - choose between position, velocity, effort
  if (interface_types.find("position") != interface_types.end() && interface_types.size() == 1) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPosition");
    return rsl_drive_sdk::mode::ModeEnum::JointPosition;
  }

  if (interface_types.find("velocity") != interface_types.end() && interface_types.size() == 1) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointVelocity");
    return rsl_drive_sdk::mode::ModeEnum::JointVelocity;
  }

  if (interface_types.find("effort") != interface_types.end() && interface_types.size() == 1) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointTorque");
    return rsl_drive_sdk::mode::ModeEnum::JointTorque;
  }

  if (interface_types.find("freeze_mode") != interface_types.end() && interface_types.size() == 1) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: Freeze");
    return rsl_drive_sdk::mode::ModeEnum::Freeze;
  }

  // Option 2:  2-tuple Combinations of the modes above
  if (interface_types.find("position") != interface_types.end() &&
      interface_types.find("velocity") != interface_types.end() && interface_types.size() == 2) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPositionVelocity");
    return rsl_drive_sdk::mode::ModeEnum::JointPositionVelocity;
  }

  if (interface_types.find("velocity") != interface_types.end() &&
      interface_types.find("effort") != interface_types.end() && interface_types.size() == 2) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPositionTorque");
    return rsl_drive_sdk::mode::ModeEnum::JointPositionTorque;
  }
  // Option 3: 3-tuple combinations of the modes above
  if (interface_types.find("position") != interface_types.end() &&
      interface_types.find("velocity") != interface_types.end() &&
      interface_types.find("effort") != interface_types.end() && interface_types.size() == 3) {
    RCLCPP_DEBUG_STREAM(logger_, "Select drive mode: JointPositionVelocityTorquePID");
    return rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorquePidGains;
  }

  // This is our fallback mode (Freeze for safety reasons)
  RCLCPP_WARN_STREAM(logger_, "Fallback to: Select drive mode: Freeze (Note this is usually do to an invalid "
                              "combination of interfaces)");
  return rsl_drive_sdk::mode::ModeEnum::Freeze;
}

}  // namespace duatic_ros2control_hardware
