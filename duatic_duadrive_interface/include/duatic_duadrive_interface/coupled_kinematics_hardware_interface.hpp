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
#include <unordered_map>
#include <filesystem>
#include <set>

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ROS
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// sdk
#include <duatic_duadrive_interface/interface_utils.hpp>
#include <duatic_duadrive_interface/duadrive_interface_base.hpp>
#include <duatic_duadrive_interface/duadrive_interface_mock.hpp>
#include <duatic_duadrive_interface/coupled_kinematics_translation_traits.hpp>
#include <duatic_duadrive_interface/coupled_kinematics_types.hpp>
#include <duatic_duadrive_interface/coupled_kinematics_position_limiter.hpp>

namespace duatic::duadrive_interface
{
/**
 * @tparam DriveTypeT - type of the actuator interface to be used - allows to inject Mock or Real hardware
 * implementations without runtime overhead
 * @tparam kinematics_translator - translation class for translation serial to coupled kinematics and vice versa
 * @tparam enable_advanced_command_limit - Enable the advanced position command limiting algorithm
 */
template <typename DriveTypeT, kinematics::KinematicsTranslator kinematics_translator,
          bool enable_advanced_command_limit = false>
class CoupledKinematicsHardwareInterfaceBase : public hardware_interface::SystemInterface
{
public:
  CoupledKinematicsHardwareInterfaceBase() : logger_(rclcpp::get_logger("CoupledKinematicsHardwareInterfaceBase"))
  {
  }

  std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions() override
  {
    // This is rather simple - we just append all state interface descriptions of each drive and are done
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;

    for (const auto& drive : drives_) {
      RCLCPP_INFO_STREAM(logger_, "Setting up state interfaces for drive: " << drive->get_name());
      const auto drive_state_interfaces = drive->get_state_interface_descriptions();
      RCLCPP_INFO_STREAM(logger_, "size: " << drive_state_interfaces.size());
      state_interfaces.insert(state_interfaces.end(), drive_state_interfaces.begin(), drive_state_interfaces.end());
    }

    // More interesting is now the internal state data mapping
    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_.at(i);
      auto& state = state_serial_kinematics_.at(i);

      auto state_mapping = drive->get_default_state_mapping();
      // Now comes the magic - we replace the "position, velocity, acceleration, torque" fields (+ their commanded
      // counterparts) with our local ones that have the translated kinematics
      state_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_POSITION)] = &state.position;
      state_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_VELOCITY)] = &state.velocity;
      state_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_ACCELERATION)] =
          &state.acceleration;
      state_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_EFFORT)] = &state.torque;

      state_mapping[get_interface_name(drive->get_name(), "position_commanded")] = &state.position_commanded;
      state_mapping[get_interface_name(drive->get_name(), "velocity_commanded")] = &state.velocity_commanded;
      state_mapping[get_interface_name(drive->get_name(), "acceleration_commanded")] = &state.acceleration_commanded;
      state_mapping[get_interface_name(drive->get_name(), "effort_commanded")] = &state.torque_commanded;

      // Append to our interface global mapping (which is used later on in the configure step to create our final
      // mapping)
      state_interface_pre_mapping_.insert(state_mapping.begin(), state_mapping.end());
    }

    return state_interfaces;
  }
  std::vector<hardware_interface::InterfaceDescription> export_unlisted_command_interface_descriptions() override
  {
    std::vector<hardware_interface::InterfaceDescription> command_interfaces;

    for (const auto& drive : drives_) {
      RCLCPP_INFO_STREAM(logger_, "Setting up command interfaces for drive: " << drive->get_name());
      const auto drive_command_interfaces = drive->get_command_interface_descriptions();
      RCLCPP_INFO_STREAM(logger_, "size: " << drive_command_interfaces.size());
      command_interfaces.insert(command_interfaces.end(), drive_command_interfaces.begin(),
                                drive_command_interfaces.end());
    }
    // Append the arm wide freeze mode field
    command_interfaces.emplace_back(
        create_interface_description<bool>(this->get_hardware_info().name, "freeze_mode", true));

    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_[i];
      auto& cmd = commands_serial_kinematics_[i];

      auto cmd_mapping = drive->get_default_command_mapping();
      // Now comes the magic - we replace the "position, velocity, acceleration, torque" fields (+ their commanded
      // counterparts) with our local ones that have the translated kinematics
      cmd_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_POSITION)] = &cmd.position;
      cmd_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_VELOCITY)] = &cmd.velocity;
      cmd_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_ACCELERATION)] = &cmd.acceleration;
      cmd_mapping[get_interface_name(drive->get_name(), hardware_interface::HW_IF_EFFORT)] = &cmd.torque;

      // Append to our interface global mapping (which is used later on in the configure step to create our final
      // mapping)
      command_interface_pre_mapping_.insert(cmd_mapping.begin(), cmd_mapping.end());
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams& system_info) override
  {
    RCLCPP_INFO_STREAM(logger_, "on_init");
    const auto system_name = system_info.hardware_info.name;
    // The logger is now a child logger with a more descriptive name
    logger_ = rclcpp::get_logger(system_name + "_hardware_interface");

    RCLCPP_INFO_STREAM(logger_, "Start with drives");
    // We obtain information about configured joints and create DuaDriveInterface instances from them
    // and initialize them
    for (const auto& joint : system_info.hardware_info.joints) {
      const auto ethercat_address = std::stoi(joint.parameters.at("ethercat_address"));
      const auto ethercat_bus = joint.parameters.at("ethercat_bus");
      const auto joint_name = joint.name;
      const auto drive_parameter_file_path = joint.parameters.at("drive_parameter_file_path");

      RCLCPP_INFO_STREAM(logger_, "Setup drive instance for joint: " << joint_name
                                                                     << " on ethercat bus:" << ethercat_bus
                                                                     << " at address: " << ethercat_address);
      RCLCPP_INFO_STREAM(logger_, "Drive parameter file: " << drive_parameter_file_path);
      // We do not startup a drive without a parameter file on purpose
      // even though this is possible we want to make sure that we use the parameteritation from the file
      if (!std::filesystem::exists(drive_parameter_file_path)) {
        RCLCPP_ERROR_STREAM(logger_, "Drive parameter file does not exist!");
        return hardware_interface::CallbackReturn::FAILURE;
      }
      drives_.emplace_back(std::make_unique<DriveTypeT>(logger_));
      // As we need to apply the kinematic translation we need some place to store the corresponding data
      // TODO(firesurfer) we could ellide copies if we would directly use pointers on the state interface
      state_coupled_kinematics_.emplace_back(CoupledJointState{});
      state_serial_kinematics_.emplace_back(SerialJointState{});

      const auto position_limits = system_info.hardware_info.limits.at(joint_name);
      RCLCPP_INFO_STREAM(logger_, "Position limits: min: " << position_limits.min_position
                                                           << " max: " << position_limits.max_position);
      position_limiters_.emplace_back(
          AdvancedPositionCommandLimiter{ position_limits.min_position, position_limits.max_position });

      commands_coupled_kinematics_.emplace_back(SerialCommand{});
      commands_serial_kinematics_.emplace_back(CoupledCommand{});
      // Init doesn't really do anything apart from setting parameters
      drives_.back()->init(DuaDriveInterfaceParameters{ .ethercat_bus = ethercat_bus,
                                                        .joint_name = joint_name,
                                                        .drive_parameter_file_path = drive_parameter_file_path,
                                                        .device_address = ethercat_address });
    }

    // TODO(firesurfer) - this could probably be implemented in a nicer way (no constexpr if on a specific type !)
    // In case there are predefined positions for mock operation we enforce them
    // This is a pure compile time statement
    if constexpr (std::is_same_v<DriveTypeT, DuaDriveInterfaceMock>) {
      if (system_info.hardware_info.hardware_parameters.contains("initial_positions")) {
        const auto positions = parse_initial_positions(system_info.hardware_info.hardware_parameters.at("initial_"
                                                                                                        "positions"));
        if (positions.size() == drives_.size()) {
          for (std::size_t i = 0; i < drives_.size(); i++) {
            drives_[i]->enforce_position(positions[i]);
          }
        } else {
          RCLCPP_WARN_STREAM(logger_, "Initial positions vector size ("
                                          << positions.size() << ") does not match amount of drives (" << drives_.size()
                                          << ") We do not apply the initial positions therefore");
        }
      }
    }

    RCLCPP_INFO_STREAM(logger_, "Done setting up drives");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) override
  {
    for (auto& drive : drives_) {
      // Call configure for each drive and propagate errors if necessary
      // We currently treat every error that can happen in this stage as fatal
      if (drive->configure() != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL_STREAM(logger_, "Failed to 'configure' drive: " << drive->get_name() << ". Aborting startup!");
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }

    // And obtain the handle for the freeze mode interface
    freeze_mode_interface_ = get_command_interface_handle(get_hardware_info().name + "/freeze_mode");

    state_interface_mapping_ = create_state_interface_mapping(state_interface_pre_mapping_, *this);
    command_interface_mapping_ = create_command_interface_mapping(command_interface_pre_mapping_, *this);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state) override
  {
    for (auto& drive : drives_) {
      // Call activate for each drive and propagate errors if necessary
      // We currently treat every error that can happen in this stage as fatal
      if (drive->activate() != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL_STREAM(logger_, "Failed to 'activate' drive: " << drive->get_name() << ". Aborting startup!");
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }

    // Enforce an initial read now so that the internal fields are correctly filed
    if (read(rclcpp::Time(), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
      RCLCPP_FATAL_STREAM(logger_, "Failed to perform initial 'read'. Aborting startup!");
      return hardware_interface::CallbackReturn::FAILURE;
    }

    // Now the internal fields are updated an we can update the externally exposed commands
    update_write_command_interfaces(command_interface_mapping_, *this);
    // Explicitly handle the joint position case !
    // Otherwise the external interface will command 0
    for (std::size_t i = 0; i < drives_.size(); i++) {
      const auto& drive = drives_[i];

      this->set_command(get_interface_name(drive->get_name(), hardware_interface::HW_IF_POSITION),
                        state_serial_kinematics_[i].position);
      RCLCPP_INFO_STREAM(logger_, "Set actuator " << drive->get_name()
                                                  << " position command to:" << state_serial_kinematics_[i].position);
      position_limiters_[i].init_last_valid_position(state_serial_kinematics_[i].position);
      // TODO(firesurfer) - do the same for the velocity, acceleration and torque fields ?
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    for (auto& drive : drives_) {
      // Call deactivate for each drive and propagate errors if necessary
      if (drive->deactivate() != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_FATAL_STREAM(logger_, "Failed to 'deactivate' drive: " << drive->get_name() << ". Aborting startup!");
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read([[maybe_unused]] const rclcpp::Time& time,
                                       [[maybe_unused]] const rclcpp::Duration& period) override
  {
    // TODO(firesurfer) - replace with std::views::zip (or own implementation) when available
    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_[i];
      auto& state = state_coupled_kinematics_[i];
      // Try to read from each drive - in case of an error
      if (drive->read() != hardware_interface::return_type::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to 'read' from drive: " << drive->get_name());
        return hardware_interface::return_type::ERROR;
      }

      // Update the coupled state (one could call this motor readings)
      const auto& latest_reading = drive->get_last_state();
      state.position = latest_reading.joint_position;
      state.velocity = latest_reading.joint_velocity;
      state.acceleration = latest_reading.joint_acceleration;
      state.torque = latest_reading.joint_torque;

      state.position_commanded = latest_reading.joint_position_commanded;
      state.velocity_commanded = latest_reading.joint_velocity_commanded;
      state.acceleration_commanded = latest_reading.joint_acceleration_commanded;
      state.torque_commanded = latest_reading.joint_torque_commanded;
    }

    // Now comes the interesting part. We need to take the data from each drive and apply the serial linkage
    // NOTE: This assumes the joints are declared in the correct order (No clue how I could validate that)
    kinematics_translator::map_from_coupled_to_serial(state_coupled_kinematics_, state_serial_kinematics_);
    // Perform the update of the exposed state interface
    update_state_interfaces(state_interface_mapping_, *this);

    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write([[maybe_unused]] const rclcpp::Time& time,
                                        [[maybe_unused]] const rclcpp::Duration& period) override
  {
    // Get commands from the ros2control exposed command interfaces
    update_command_interfaces(command_interface_mapping_, *this);

    // self collision avoidance logic
    // this logic is actually a bit advanced as it allows to move out of positions limits if the drive is current inside
    // a position limit
    if constexpr (enable_advanced_command_limit) {
      for (std::size_t i = 0; i < drives_.size(); i++) {
        position_limiters_[i].limit(commands_serial_kinematics_[i], state_serial_kinematics_[i]);
      }
    }

    // Translated commands to coupled kinematics
    kinematics_translator::map_from_serial_to_coupled(commands_serial_kinematics_, commands_coupled_kinematics_);

    const bool enforced_freeze = freeze_mode_interface_->get_optional<bool>().value();
    // Stage all commands with the coupled
    for (std::size_t i = 0; i < drives_.size(); i++) {
      auto& drive = drives_[i];
      auto& cmd = commands_coupled_kinematics_[i];

      auto command = drive->get_last_command();
      command.joint_position = cmd.position;
      command.joint_velocity = cmd.velocity;
      command.joint_acceleration = cmd.acceleration;
      command.joint_torque = cmd.torque;
      // Forward the enforced freeze mode here
      command.joint_freeze_mode = enforced_freeze;

      drive->stage_command(command);
    }

    // Perform actual write action
    // Note this is still asynchronous to the bus communication
    for (auto& drive : drives_) {
      if (drive->write() != hardware_interface::return_type::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to 'write' to drive: " << drive->get_name());
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type
  prepare_command_mode_switch([[maybe_unused]] const std::vector<std::string>& start_interfaces,
                              [[maybe_unused]] const std::vector<std::string>& stop_interfaces) override
  {
    // Prepare command mode switch by selecting the write drive mode depending on the selected interfaces
    for (const auto& interface : start_interfaces) {
      currently_active_interfaces_.insert(interface);
    }

    for (const auto& interface : stop_interfaces) {
      currently_active_interfaces_.erase(interface);
    }
    // This is now staged and will be applied in the "perform_command_mode_switch" method
    current_active_drive_mode_ = select_mode(currently_active_interfaces_, logger_);
    RCLCPP_INFO_STREAM(logger_, "Staging new control mode: " << current_active_drive_mode_);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override
  {
    // This is run in the realtime context -> We now configure each drive to use the new mode
    // Will be applied in the next "write" run
    for (auto& drive : drives_) {
      drive->configure_drive_mode(current_active_drive_mode_);
    }
    RCLCPP_INFO_STREAM(logger_, "Configured new control mode: " << current_active_drive_mode_);
    return hardware_interface::return_type::OK;
  }

  virtual ~CoupledKinematicsHardwareInterfaceBase()
  {
    RCLCPP_INFO_STREAM(logger_, "Destructor of DynaArm Hardware Interface called");
  }

protected:
  std::vector<typename DriveTypeT::UniquePtr> drives_;
  std::vector<CoupledJointState> state_coupled_kinematics_;
  std::vector<SerialJointState> state_serial_kinematics_;

  std::vector<CoupledCommand> commands_coupled_kinematics_;
  std::vector<SerialCommand> commands_serial_kinematics_;
  std::vector<AdvancedPositionCommandLimiter> position_limiters_;

  std::unordered_map<std::string, SupportedVariant> state_interface_pre_mapping_;
  std::unordered_map<std::string, SupportedVariant> command_interface_pre_mapping_;

  StateInterfaceMapping state_interface_mapping_;
  CommandInterfaceMapping command_interface_mapping_;

  hardware_interface::CommandInterface::SharedPtr freeze_mode_interface_;

  rclcpp::Logger logger_;

  std::set<std::string> currently_active_interfaces_;
  rsl_drive_sdk::mode::ModeEnum current_active_drive_mode_{ rsl_drive_sdk::mode::ModeEnum::Freeze };

  // Internal methods
  std::vector<double> parse_initial_positions(std::string initial_positions_str)
  {
    std::vector<double> initial_positions;

    // Remove brackets
    initial_positions_str.erase(std::remove(initial_positions_str.begin(), initial_positions_str.end(), '['),
                                initial_positions_str.end());
    initial_positions_str.erase(std::remove(initial_positions_str.begin(), initial_positions_str.end(), ']'),
                                initial_positions_str.end());

    // Split by comma and convert to doubles
    std::stringstream ss(initial_positions_str);
    std::string item;

    while (std::getline(ss, item, ',')) {
      // Trim whitespace
      item.erase(0, item.find_first_not_of(" \t"));
      item.erase(item.find_last_not_of(" \t") + 1);

      try {
        double value = std::stod(item);
        initial_positions.push_back(value);
      } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to parse initial position value: " << item);
        initial_positions.push_back(0.0);
      }
    }
    return initial_positions;
  }
};

}  // namespace duatic::duadrive_interface
