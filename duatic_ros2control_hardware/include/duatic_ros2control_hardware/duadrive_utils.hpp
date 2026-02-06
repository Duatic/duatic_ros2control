#pragma once
#include <cmath>
#include <vector>

#include <rsl_drive_sdk/Statusword.hpp>
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

}  // namespace duatic_ros2control_hardware
