#pragma once

#include <string>
#include <cmath>
#include <variant>
#include <hardware_interface/handle.hpp>

namespace duatic_ros2control_hardware
{
inline std::string extract_interface_type(const std::string& interface)
{
  // Extract only the interface type
  return interface.substr(interface.find_last_of('/') + 1);
}


// Template magic to generate ros2control compatible type strings from specific types
template<typename T>
struct interface_type_to_string;

template<>
struct interface_type_to_string<double>{
  static constexpr std::string_view value = "double";
};

template<>
struct interface_type_to_string<bool>{
  static constexpr std::string_view value = "bool";
};

template<>
struct interface_type_to_string<int>{
  static constexpr std::string_view value = "int";
};

/**
 * @brief helpers function which allows to create a ros2control InterfaceDescription with less writing effort
 */
template< typename T>
hardware_interface::InterfaceDescription create_interface_description(
  const std::string& joint_name,
  const std::string& interface_type,
  const T& initial_value = {}
) {
  return hardware_interface::InterfaceDescription(
    joint_name, hardware_interface::InterfaceInfo{
      .name = interface_type,
       .initial_value = std::to_string(initial_value),
      .data_type = std::string( interface_type_to_string<T>::value),
      .size = 0, // Only to suppress warning
      .enable_limits = false
    }
  );
}




}  // namespace duatic_ros2control_hardware
