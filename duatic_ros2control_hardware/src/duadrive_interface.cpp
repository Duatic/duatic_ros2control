#include "duatic_ros2control_hardware/duadrive_interface.hpp"

#include "ethercat_sdk_master/EthercatMasterSingleton.hpp"
#include <filesystem>
#include "duatic_ros2control_hardware/interface_utils.hpp"

namespace duatic_ros2control_hardware
{

DuaDriveInterface::DuaDriveInterface(rclcpp::Logger logger) : DuaDriveInterfaceBase(logger)
{
  generate_state_interface_descriptions();
  generate_command_interface_desriptions();
}
DuaDriveInterface::~DuaDriveInterface()
{
  RCLCPP_INFO_STREAM(logger_, "Destructor of DuaDriveHardwareInterface Hardware Interface called");
  if (ecat_master_handle_.ecat_master) {
    RCLCPP_INFO_STREAM(logger_, "Releasing ethercat master");
    ecat_master::EthercatMasterSingleton::instance().releaseMaster(ecat_master_handle_);
    ecat_master_handle_.ecat_master.reset();
  }
}

hardware_interface::CallbackReturn DuaDriveInterface::init(const DuaDriveInterfaceParameters& params)
{
  params_ = params;
  // configure ethercat bus and drives
  const auto ethercat_bus = params.ethercat_bus;
  const ecat_master::EthercatMasterConfiguration ecat_master_config = {
    .name = "DuaDriveHardwareInterface", .networkInterface = ethercat_bus, .timeStep = 0.001
  };  // TODO(firesurfer) set timestep according to the update rate of ros2control (or spin asynchronously)

  // Obtain an instance of the bus from the singleton - if there is no instance it will be created
  ecat_master_handle_ = ecat_master::EthercatMasterSingleton::instance().aquireMaster(ecat_master_config);

  const auto joint_name = params.joint_name;
  logger_ = rclcpp::get_logger("DuaDriveHardwareInterface_" + joint_name);

  const auto address = params.device_address;
  // Obtain the parameter file for the currently processed drive
  std::string device_file_path = params.drive_parameter_file_path;
  // If there is no configuration available for the current joint in the passed parameter folder we load it from the
  // default folder
  if (!std::filesystem::exists(device_file_path)) {
    RCLCPP_WARN_STREAM(logger_, "No configuration found for joint: " << joint_name << " in: " << device_file_path
                                                                     << " Loading default drive parameters");

    device_file_path = params.drive_default_parameter_file_path;
    RCLCPP_INFO_STREAM(logger_, "Drive file path " << device_file_path);
  }
  drive_ = rsl_drive_sdk::DriveEthercatDevice::deviceFromFile(device_file_path, joint_name, address,

                                                              rsl_drive_sdk::PdoTypeEnum::E);

  // And attach it to the ethercat master
  if (!ecat_master_handle_.ecat_master->attachDevice(drive_)) {
    RCLCPP_ERROR_STREAM(logger_, "Could not attach the slave drive to the master.");
  }

  RCLCPP_INFO_STREAM(logger_, "Registered drive: " << joint_name << " at bus address: " << address);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterface::configure()
{
  ecat_master::EthercatMasterSingleton::instance().markAsReady(ecat_master_handle_);
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn DuaDriveInterface::activate()
{
  // We are now in the realtime loop
  // In case we are in error state clear the error and try again
  rsl_drive_sdk::Statusword status_word;
  drive_->getStatuswordSdo(status_word);
  if (status_word.getStateEnum() == rsl_drive_sdk::fsm::StateEnum::Error) {
    RCLCPP_WARN_STREAM(logger_, "Drive: " << get_name() << " is in Error state - trying to reset");
    drive_->setControlword(RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY);
    drive_->updateWrite();
    drive_->updateRead();
    if (!drive_->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10)) {
      RCLCPP_FATAL_STREAM(logger_, "Drive: " << get_name() << " did not go into ControlOP");
    } else {
      RCLCPP_INFO_STREAM(logger_, "Drive: " << get_name() << " went into ControlOp successfully");
    }
  }

  // Put into controlOP, in blocking mode.
  if (!drive_->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 1, 10)) {
    RCLCPP_FATAL_STREAM(logger_, "Drive: " << get_name()
                                           << " did not go into ControlOP - this is trouble some and a reason to "
                                              "abort. Try to reboot the hardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Log the firmware information of the drive. Might be useful for debugging issues at customer
  rsl_drive_sdk::common::BuildInfo info;
  drive_->getBuildInfo(info);

  std::string drive_model;
  drive_->getDriveModel(drive_model);
  RCLCPP_INFO_STREAM(logger_, "Drive info: " << get_name() << " Drive model: " << drive_model << " Build date: "
                                             << info.buildDate << " tag: " << info.gitTag << " hash: " << info.gitHash);

  drive_info_ = { .drive_name = drive_->getName(), .drive_model = drive_model, .drive_build_tag = info.gitTag };

  // Update the command with the current state (avoid weird motions)
  rsl_drive_sdk::mode::PidGainsF gains;
  drive_->getControlGains(rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorquePidGains, gains);
  command_.p_gain = gains.getP();
  command_.i_gain = gains.getI();
  command_.d_gain = gains.getD();

  RCLCPP_INFO_STREAM(logger_, "PID Gains: " << gains);

  // Perform the initial readout to set the current positions as targets
  if (read() != hardware_interface::return_type::OK) {
    RCLCPP_ERROR_STREAM(logger_, "Initial readout failed for: " << get_name() << " - this is critical!");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Validate initial readout - if it is exactly 0.0 something went wrong
  if (state_.joint_position == 0.0) {
    RCLCPP_FATAL_STREAM(logger_, "Initial joint position reading was 0.0 - this is a critical error");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Set joint position command to current position
  command_.joint_position = state_.joint_position;

  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn DuaDriveInterface::deactivate()
{
  // Put drive into freeze mode at shutdown
  if (drive_) {
    drive_->setFSMGoalState(rsl_drive_sdk::fsm::StateEnum::ControlOp, true, 3.0, 0.01);

    rsl_drive_sdk::Command cmd;
    cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::Freeze);
    drive_->setCommand(cmd);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DuaDriveInterface::read()
{
  // Obtain the latest reading from the drive (note: we assume asynchronous spinning)
  rsl_drive_sdk::ReadingExtended reading;
  drive_->getReading(reading);

  const auto& state = reading.getState();

  // Print any status word changes (e.g. motor temperature warning has appeared)
  // TODO(firesurfer) this might be bad to have in the real time loop
  const auto current_status_word = state.getStatusword();
  print_drive_status_changes(get_name(), current_status_word, last_status_word_, logger_);
  last_status_word_ = current_status_word;

  // Now update the state vector
  state_.joint_position = state.getJointPosition();
  state_.joint_velocity = state.getJointVelocity();
  state_.joint_acceleration = state.getJointAcceleration();
  state_.joint_torque = state.getJointVelocity();

  state_.current_q = state.getMeasuredCurrentQ();
  state_.current_d = state.getMeasuredCurrentD();
  state_.current_coil_A = state.getMeasuredCurrentPhaseU();
  state_.current_coil_B = state.getMeasuredCurrentPhaseV();
  state_.current_coil_C = state.getMeasuredCurrentPhaseW();

  state_.bus_voltage = state.getVoltage();
  state_.voltage_coil_A = state.getMeasuredVoltagePhaseU();
  state_.voltage_coil_B = state.getMeasuredVoltagePhaseV();
  state_.voltage_coil_C = state.getMeasuredVoltagePhaseW();

  state_.temperature_system = state.getTemperature();
  state_.temperature_coil_A = state.getCoilTemp1();
  state_.temperature_coil_B = state.getCoilTemp2();
  state_.temperature_coil_C = state.getCoilTemp3();

  // These fields are reused
  state_.power_active = state.getGearPosition();
  state_.power_reactive = state.getGearVelocity();

  state_.joint_position_commanded = reading.getCommanded().getJointPosition();
  state_.joint_velocity_commanded = reading.getCommanded().getJointVelocity();
  // This is just as a feedback for certain controllers - we cannot command acceleration to the drive
  state_.joint_acceleration_commanded = command_.joint_acceleration;
  state_.joint_torque_commanded = reading.getCommanded().getJointTorque();
  state_.current_q_commanded = reading.getCommanded().getCurrent();
  state_.joint_freeze_mode_commanded = command_.joint_freeze_mode;

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type DuaDriveInterface::write()
{
  // Only write the command if we are already in the correct state
  if (drive_->goalStateHasBeenReached()) {
    // Convert command vector into an rsl_drive_sdk::Command
    // Make sure to be in the right mode
    rsl_drive_sdk::Command cmd;

    rsl_drive_sdk::mode::PidGainsF gains;
    gains.setP(command_.p_gain);
    gains.setI(command_.i_gain);
    gains.setD(command_.d_gain);

    // Sanitize inputs - might result in weird behaviour otherwise
    sanitize_command_input(command_.joint_position);
    sanitize_command_input(command_.joint_velocity);
    sanitize_command_input(command_.joint_torque);

    cmd.setJointPosition(command_.joint_position);
    cmd.setJointVelocity(command_.joint_velocity);
    cmd.setJointTorque(command_.joint_torque);
    cmd.setPidGains(gains);

    // Enforce freeze mode if desired - otherwise set the currently active mode
    if (command_.joint_freeze_mode == 1.0) {
      cmd.setModeEnum(rsl_drive_sdk::mode::ModeEnum::Freeze);
    } else {
      cmd.setModeEnum(active_mode_);
    }

    // We always fill all command fields but depending on the mode only a subset is used
    drive_->setCommand(cmd);
  }

  // From this part of the drive API we do not get any feedback. Therefore we can only return OK here
  return hardware_interface::return_type::OK;
}

}  // namespace duatic_ros2control_hardware
