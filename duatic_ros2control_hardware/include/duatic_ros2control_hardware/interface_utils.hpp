#pragma once

#include <string>
#include <cmath>
#include <variant>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>

namespace duatic_ros2control_hardware
{
inline std::string extract_interface_type(const std::string& interface)
{
  // Extract only the interface type
  return interface.substr(interface.find_last_of('/') + 1);
}

// Template magic to generate ros2control compatible type strings from specific types
template <typename T>
struct interface_type_to_string;

template <>
struct interface_type_to_string<double>
{
  static constexpr std::string_view value = "double";
};

template <>
struct interface_type_to_string<bool>
{
  static constexpr std::string_view value = "bool";
};

template <>
struct interface_type_to_string<int>
{
  static constexpr std::string_view value = "int";
};

using SupportedVariant = std::variant<double*, int*, bool*>;
using StateInterfaceMapping = std::vector<std::pair<hardware_interface::StateInterface::SharedPtr, SupportedVariant>>;
using CommandInterfaceMapping =
    std::vector<std::pair<hardware_interface::CommandInterface::SharedPtr, SupportedVariant>>;

/**
 * @brief helpers function which allows to create a ros2control InterfaceDescription with less writing effort
 */
template <typename T>
hardware_interface::InterfaceDescription create_interface_description(const std::string& joint_name,
                                                                      const std::string& interface_type,
                                                                      const T& initial_value = {})
{
  return hardware_interface::InterfaceDescription(
      joint_name, hardware_interface::InterfaceInfo{ .name = interface_type,
                                                     .initial_value = std::to_string(initial_value),
                                                     .data_type = std::string(interface_type_to_string<T>::value),
                                                     .size = 0,  // Only to suppress warning
                                                     .enable_limits = false });
}

/**
 * @brief helper function for creating a more efficient mapping / list of ros2control state interface from the interface
 * name to the underlying pointer
 */
StateInterfaceMapping inline create_state_interface_mapping(
    const std::unordered_map<std::string, SupportedVariant>& mapping,
    const hardware_interface::SystemInterface& hw_interface)
{
  StateInterfaceMapping result;
  for (const auto& [name, variant] : mapping) {
    result.push_back({ hw_interface.get_state_interface_handle(name), variant });
  }
  return result;
}

CommandInterfaceMapping inline create_command_interface_mapping(
    const std::unordered_map<std::string, SupportedVariant>& mapping,
    const hardware_interface::SystemInterface& hw_interface)
{
  CommandInterfaceMapping result;
  for (const auto& [name, variant] : mapping) {
    result.push_back({ hw_interface.get_command_interface_handle(name), variant });
  }
  return result;
}

inline void update_state_interfaces(const StateInterfaceMapping& mapping,
                                    hardware_interface::SystemInterface& hw_interface)
{
  for (const auto& [handle, variant] : mapping) {
    std::visit([&](auto&& value) { hw_interface.set_state(handle, *value, false); }, variant);
  }
}

inline void update_command_interfaces(const CommandInterfaceMapping& mapping,
                                      hardware_interface::SystemInterface& hw_interface)
{
  for (const auto& [handle, variant] : mapping) {
    std::visit([&](auto&& value) { hw_interface.get_command(handle, *value, false); }, variant);
  }
}

}  // namespace duatic_ros2control_hardware
