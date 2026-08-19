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

/*stl*/
#include <filesystem>

/*project*/
#include "duatic_duadrive_interface/duadrive_interface.hpp"
#include "duatic_duadrive_interface/interface_utils.hpp"

namespace duatic::duadrive_interface
{

DuaDriveInterface::DuaDriveInterface(rclcpp::Logger logger) : DuaDriveInterfaceBase(logger)
{
}
DuaDriveInterface::~DuaDriveInterface()
{
  RCLCPP_INFO_STREAM(logger_, "Destructor of DuaDriveHardwareInterface Hardware Interface called");
  if (ecat_bus_handle_.ecat_bus) {
    RCLCPP_INFO_STREAM(logger_, "Releasing ethercat bus");
    EthercatBusSingleton::instance().release_bus(ecat_bus_handle_);
    ecat_bus_handle_.ecat_bus.reset();
  }
}

hardware_interface::CallbackReturn DuaDriveInterface::init(const DuaDriveInterfaceParameters& params)
{
  params_ = params;
  generate_state_interface_descriptions();
  generate_command_interface_desriptions();
  // configure ethercat bus and drives
  const auto ethercat_bus = params.ethercat_bus;
  const ethercat_interface::EthercatBus::Parameters ecat_master_config = {
    .interface = ethercat_bus, .dc_cycle_time = std::chrono::nanoseconds(1000000)
  };  // TODO(firesurfer) set timestep according to the update rate of ros2control (or spin asynchronously)

  // Obtain an instance of the bus from the singleton - if there is no instance it will be created
  ecat_bus_handle_ = EthercatBusSingleton::instance().aquire_bus(
      ecat_master_config, std::bind(&DuaDriveInterface::on_bus_startup_finished, this));

  const auto joint_name = params.joint_name;
  logger_ = rclcpp::get_logger("DuaDriveHardwareInterface_" + joint_name);

  const ethercat_interface::DeviceId address = static_cast<ethercat_interface::DeviceId>(params.device_address);
  // Obtain the parameter file for the currently processed drive
  std::string device_file_path = params.drive_parameter_file_path;
  // If there is no configuration available for the current joint in the passed parameter folder we abort
  if (!std::filesystem::exists(device_file_path)) {
    RCLCPP_FATAL_STREAM(logger_, "No configuration found for joint: " << joint_name << " in: " << device_file_path);
    return hardware_interface::CallbackReturn::FAILURE;
  }
  drive_ = std::make_shared<duadrive_sdk::v1::DuaDrive>(duadrive_sdk::v1::RxPdoType::A, duadrive_sdk::v1::TxPdoType::E);

  // And attach it to the ethercat master
  ecat_bus_handle_.ecat_bus->attach_device(address, drive_->get_ethercat_device());

  RCLCPP_INFO_STREAM(logger_, "Registered drive: " << joint_name << " at bus address: " << address);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterface::configure()
{
  EthercatBusSingleton::instance().mark_as_ready(ecat_bus_handle_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DuaDriveInterface::on_bus_startup_finished()
{
  using namespace duadrive_sdk::v1;  // NOLINT(build/namespaces)

  // Log the firmware information of the drive. Might be useful for debugging issues at customer

  BuildInfo info = drive_->configuration_interface().read_firmware_build_info();

  std::string drive_model = drive_->configuration_interface().read_drive_model();

  drive_info_ = { .drive_name = drive_->get_ethercat_device_info().name,
                  .drive_model = drive_model,
                  .drive_build_tag = info.git_tag };

  Version fw_version = drive_->configuration_interface().read_firmware_version();

  RCLCPP_INFO_STREAM(logger_, "Drive info:\n   Name: " << get_name() << "\n   Drive model: " << drive_model
                                                       << "\n   Build date: " << info.build_date << "\n   Git tag: "
                                                       << info.git_tag << "\n   Git hash: " << info.git_hash
                                                       << "\n   Firmware version: " << fw_version);
  const auto gains =
      drive_->configuration_interface().read_control_gains(ControlMode::JointPositionVelocityTorque);

  command_.p_gain = gains.p;
  command_.i_gain = gains.i;
  command_.d_gain = gains.d;

  RCLCPP_INFO_STREAM(logger_, "PID Gains: " << gains);

  // Maximum torque/velocity values
  // We use this to provide the possibility to scale down the safety values
  configured_max_torque_ = drive_->configuration_interface().read_maximum_joint_torque();
  configured_gear_ratio_ = static_cast<float>(drive_->configuration_interface().read_gearbox_ratio());

  double max_motor_velocity = drive_->configuration_interface().read_maximum_motor_velocity();
  // Actually store the velocity in joint coordinates
  configured_max_velocity_ = max_motor_velocity / configured_gear_ratio_;

  current_max_torque_ = configured_max_torque_;
  current_max_velocity_ = configured_max_velocity_;

  RCLCPP_INFO_STREAM(logger_, "Maximum joint torque: " << configured_max_torque_ << "Nm "
                                                       << " maximum joint velocity: " << configured_max_velocity_
                                                       << " rad/s");
  current_brake_state_ = drive_->configuration_interface().read_current_brake_state();

  RCLCPP_INFO_STREAM(logger_, "Current brake state: " << current_brake_state_);
}
hardware_interface::CallbackReturn DuaDriveInterface::activate()
{
  using namespace duadrive_sdk::v1;  // NOLINT(build/namespaces)

  RCLCPP_INFO_STREAM(logger_, "Activate drive: " << drive_->get_ethercat_device_info().name);
  // We are now in the realtime loop
  // In case we are in error state clear the error and try again
  auto status_word = drive_->read_status_word();
  // sync data into the drive object
  drive_->sync_read();

  // Print the current drive state (e.g. warnings, errors, fatals)
  // TODO(firesurfer) re-add more specific printing functions
  RCLCPP_INFO_STREAM(logger_, status_word);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (status_word.get_fsm_state() == FSMState::Fatal) {
    RCLCPP_FATAL_STREAM(logger_, "Drive: " << get_name() << " is in Fatal state - aborting startup!");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Check if the drive is in error state
  // If so we try to clear the errors
  const bool is_in_error_state = status_word.get_fsm_state() == FSMState::Error;
  if (is_in_error_state) {
    RCLCPP_WARN_STREAM(logger_, "Drive: " << get_name() << " is in Error state - trying to reset");
    drive_->set_fsm_goal_state(FSMState::Standby);  // Requesting standby clears the errors if possible
    drive_->sync_write();
    drive_->sync_read();

    drive_->wait_until_goal_state_reached(std::chrono::milliseconds{ 100 });
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // Put into Configure
  RCLCPP_WARN_STREAM(logger_, "Setting to configure state, current state: "
                                  << drive_->get_latest_reading().status_word.get_fsm_state());
  drive_->set_fsm_goal_state(FSMState::Configure);

  if (!drive_->wait_until_goal_state_reached(std::chrono::milliseconds{ 500 })) {
    RCLCPP_FATAL_STREAM(logger_, "Drive: " << get_name() << " failed to put drive into configure");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // In case the drive was in error state we check again if we managed to clear the errors (aka state machine is now in
  // any mode != Error or Fatal)
  if (is_in_error_state) {
    status_word = drive_->read_status_word();
    if (status_word.get_fsm_state() == FSMState::Error || status_word.get_fsm_state() == FSMState::Fatal) {
      RCLCPP_FATAL_STREAM(logger_,
                          "Drive: " << get_name() << " could not reset/clear errors at startup - aborting startup!");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  // We need to give the drive a bit of time otherwise we do not get valid readings
  int retries = 0;
  while (!drive_->fsm_goal_state_reached()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    retries++;
    if (retries > 100) {
      RCLCPP_WARN_STREAM(logger_, "Drive hasn't reach goal state within 100ms - trying to continue nevertheless - "
                                  "usually it reaches the goal state afterwards");
      break;
    }
  }

  // Perform the initial readout to set the current positions as targets
  if (read(rclcpp::Time{}, rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
    RCLCPP_ERROR_STREAM(logger_, "Initial readout failed for: " << get_name() << " - this is critical!");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO_STREAM(logger_, "Initial joint position for joint:" << get_name() << " " << state_.joint_position);
  // Validate initial readout - if it is exactly 0.0 something went wrong
  if (state_.joint_position == 0.0) {
    RCLCPP_FATAL_STREAM(logger_, "Initial joint position reading was 0.0 - this is a critical error");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // We want to enforce disable mode at startup and workaround the SDK limitation of implicitly starting in freeze mode
  drive_->stage_command(DisableCommand());

  // Set joint position command to current position
  command_.joint_position = state_.joint_position;

  drive_->set_fsm_goal_state(FSMState::ControlOp);
  if (!drive_->wait_until_goal_state_reached(std::chrono::milliseconds{ 500 })) {
    RCLCPP_FATAL_STREAM(logger_, "Drive: " << get_name() << " failed to put drive into control op");
    return hardware_interface::CallbackReturn::ERROR;
  }

  last_reading_update_ = std::chrono::system_clock::now();

  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn DuaDriveInterface::deactivate()
{
  if (drive_) {
    if (params_.has_brake) {
      // in case we have a brake -> disable drive and engage the brake

      drive_->stage_command(duadrive_sdk::v1::DisableCommand());
      drive_->configuration_interface().write_target_brake_state(duadrive_sdk::v1::BrakeState::Engaged);
    } else {
      // in case we have no brake -> enable freeze mode (NOTE: the drives will go into standby state during shutdown and
      // will nevertheless result into a falling arm)

      drive_->stage_command(duadrive_sdk::v1::FreezeCommand());
    }
    //Otherwise we have a pdo timeout
    drive_->set_fsm_goal_state(duadrive_sdk::v1::FSMState::Standby);
    drive_->sync_write();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DuaDriveInterface::read([[maybe_unused]] const rclcpp::Time& time,
                                                        [[maybe_unused]] const rclcpp::Duration& period)
{
  drive_->sync_read();
  // Obtain the latest reading from the drive (note: we assume asynchronous spinning)
  duadrive_sdk::v1::Reading reading = drive_->get_latest_reading();

  // Print any status word changes (e.g. motor temperature warning has appeared)
  // TODO(firesurfer) this might be bad to have in the real time loop
  const auto current_status_word = reading.status_word;
  if (last_status_word_) {
    duadrive_sdk::v1::StatusEvent event(last_status_word_.value(), current_status_word);
    if (event.has_changes()) {
      RCLCPP_INFO_STREAM(logger_, event);
    }
  }
  last_status_word_ = current_status_word;
  // last_reading_update_ = reading.time_stamp;

  // Now update the state vector
  state_.joint_position = reading.joint_position;
  state_.joint_velocity = reading.joint_velocity;
  state_.joint_acceleration = reading.joint_acceleration;
  state_.joint_torque = reading.joint_torque;

  state_.current_q = reading.current_q;
  state_.current_d = reading.current_d.value();
  state_.current_coil_A = reading.phase_currents->at(0);
  state_.current_coil_B = reading.phase_currents->at(1);
  state_.current_coil_C = reading.phase_currents->at(2);

  state_.bus_voltage = reading.dc_bus_voltage;
  state_.voltage_coil_A = reading.phase_voltages->at(0);
  state_.voltage_coil_B = reading.phase_voltages->at(1);
  state_.voltage_coil_C = reading.phase_voltages->at(2);

  state_.temperature_system = reading.system_temperature;
  state_.temperature_coil_A = reading.coil_temperatures->at(0);
  state_.temperature_coil_B = reading.coil_temperatures->at(1);
  state_.temperature_coil_C = reading.coil_temperatures->at(2);

  // These fields are reused
  state_.power_active = reading.active_motor_power;
  state_.power_reactive = reading.reactive_motor_power;

  state_.joint_position_commanded = reading.joint_position_commanded.value();
  state_.joint_velocity_commanded = reading.joint_velocity_commanded.value();
  // This is just as a feedback for certain controllers - we cannot command acceleration to the drive
  state_.joint_acceleration_commanded = command_.joint_acceleration;
  state_.joint_torque_commanded = reading.joint_torque_commanded.value();
  state_.current_q_commanded = reading.current_q_commanded.value();
  // Some control mode information
  state_.joint_freeze_mode_commanded = command_.joint_freeze_mode;
  state_.current_drive_mode = active_mode_;
  state_.current_drive_state = reading.status_word.get_fsm_state();
  // state_.current_drive_state = state.getStatusword().getStateEnum();

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type DuaDriveInterface::write([[maybe_unused]] const rclcpp::Time& time,
                                                         [[maybe_unused]] const rclcpp::Duration& period)
{
  drive_->sync_write();
  // Only write the command if we are already in the correct state
  if (drive_->fsm_goal_state_reached()) {
    // Convert command vector into an rsl_drive_sdk::Command
    // Make sure to be in the right mode

    duadrive_sdk::v1::PidGains gains{ command_.p_gain, command_.i_gain, command_.d_gain };

    // Sanitize inputs - might result in weird behaviour otherwise
    sanitize_command_input(command_.joint_position);
    sanitize_command_input(command_.joint_velocity);
    sanitize_command_input(command_.joint_torque);

    // In case the brake is active we do not allow the freeze mode to be enabled as it might vibrate on the brake
    if (params_.has_brake && current_brake_state_ == duadrive_sdk::v1::BrakeState::Engaged &&
        (active_mode_ == duadrive_sdk::v1::ControlMode::Freeze || command_.joint_freeze_mode == 1.0)) {
      drive_->stage_command(duadrive_sdk::v1::DisableCommand());
    } else {
      // Enforce freeze mode if desired - otherwise set the currently active mode
      if (command_.joint_freeze_mode == 1.0) {
        drive_->stage_command(duadrive_sdk::v1::FreezeCommand());
      } else {
        const auto cmd =
            build_command(active_mode_, command_.joint_position, command_.joint_velocity, command_.joint_torque, gains);
        drive_->stage_command(cmd);
      }
    }

    if (params_.has_brake && command_.target_brake_state != current_target_brake_state) {
      current_target_brake_state = command_.target_brake_state;
      // Safety check that only valid values are commanded
      if (current_target_brake_state >= 0 && current_target_brake_state < 3) {
        duadrive_sdk::v1::BrakeState target_brake_state =
            static_cast<duadrive_sdk::v1::BrakeState>(current_target_brake_state);
        RCLCPP_INFO_STREAM(logger_, "New brake target state:" << target_brake_state);
        drive_->configuration_interface().write_target_brake_state(target_brake_state);

        current_brake_state_ = target_brake_state;
      }
    }

  } else {
    const auto reading = drive_->get_latest_reading();
    RCLCPP_ERROR_STREAM(logger_, get_name() << " Is not in target FSM Mode: ControlOP actual mode: "
                                            << reading.status_word.get_fsm_state()
                                            << " Raw status word: " << reading.status_word.get_raw());
  }

  // From this part of the drive API we do not get any feedback. Therefore we can only return OK here
  return hardware_interface::return_type::OK;
}

}  // namespace duatic::duadrive_interface
