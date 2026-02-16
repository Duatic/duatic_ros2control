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
#include <variant>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>

/*ros2control*/
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>

namespace duatic_ros2control_hardware
{
/**
 * @brief helper method to extra the interface from a ros2control interface string
 */
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

/**
 * @brief ros2control expects us for the "default_value" to use "true" and "false" instead of "1" and "0"
 */
template <typename T>
constexpr std::string compatible_to_string(const T& val)
{
  if constexpr (std::is_same_v<T, bool>) {
    return val ? "true" : "false";
  } else {
    return std::to_string(val);
  }
}

using SupportedVariant = std::variant<double*, int*, bool*>;
using StateInterfaceMapping = std::vector<std::pair<hardware_interface::StateInterface::SharedPtr, SupportedVariant>>;
using CommandInterfaceMapping =
    std::vector<std::pair<hardware_interface::CommandInterface::SharedPtr, SupportedVariant>>;

/**
 * @brief obtain the internal ros2control interface name
 */
inline std::string get_interface_name(const std::string& joint_name, const std::string& interface_type)
{
// We cannot really do anything about this warning. (Does not make sense to fill in dummy filler in there)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  // This is a bit nasty as we need to build the whole description in order to obtain the name
  return hardware_interface::InterfaceDescription(joint_name,
                                                  hardware_interface::InterfaceInfo{ .name = interface_type })
      .get_name();
#pragma GCC diagnostic pop
}
/**
 * @brief helpers function which allows to create a ros2control InterfaceDescription with less writing effort
 */
template <typename T>
hardware_interface::InterfaceDescription create_interface_description(const std::string& joint_name,
                                                                      const std::string& interface_type,
                                                                      const T& initial_value = {})
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  return hardware_interface::InterfaceDescription(
      joint_name, hardware_interface::InterfaceInfo{ .name = interface_type,
                                                     .initial_value = compatible_to_string<T>(initial_value),
                                                     .data_type = std::string(interface_type_to_string<T>::value),
                                                     .size = 0,  // Only to suppress warning
                                                     .enable_limits = false });

#pragma GCC diagnostic pop
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
