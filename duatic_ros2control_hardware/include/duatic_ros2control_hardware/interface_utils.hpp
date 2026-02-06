#pragma once

#include <string>
#include <cmath>
namespace duatic_ros2control_hardware
{
inline std::string extract_interface_type(const std::string& interface)
{
  // Extract only the interface type
  return interface.substr(interface.find_last_of('/') + 1);
}

}  // namespace duatic_ros2control_hardware
