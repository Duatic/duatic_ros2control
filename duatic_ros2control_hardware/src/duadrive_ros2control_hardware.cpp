#include "duatic_ros2control_hardware/duadrive_ros2control_hardware.hpp"

#include "ethercat_sdk_master/EthercatMasterSingleton.hpp"
#include <filesystem>

namespace duatic_ros2control_hardware
{

DuaDriveInterface::DuaDriveInterface(rclcpp::Logger logger) : logger_(logger)
{
}
hardware_interface::CallbackReturn DuaDriveInterface::on_init(const DuaDriveInterfaceParameters& params)
{
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
std::vector<hardware_interface::StateInterface> DuaDriveInterface::export_state_interfaces() const
{
}
std::vector<hardware_interface::CommandInterface> DuaDriveInterface::export_command_interfaces() const
{
}

hardware_interface::CallbackReturn DuaDriveInterface::on_activate()
{
}
hardware_interface::CallbackReturn DuaDriveInterface::on_configure()
{
}
hardware_interface::CallbackReturn DuaDriveInterface::on_deactivate()
{
}

hardware_interface::return_type DuaDriveInterface::read()
{
}
hardware_interface::return_type DuaDriveInterface::write()
{
}

hardware_interface::return_type
DuaDriveInterface::prepare_command_mode_switch([[maybe_unused]] const std::vector<std::string>& start_interfaces,
                                               [[maybe_unused]] const std::vector<std::string>& stop_interfaces)
{
}
hardware_interface::return_type
DuaDriveInterface::perform_command_mode_switch([[maybe_unused]] const std::vector<std::string>& start_interfaces,
                                               [[maybe_unused]] const std::vector<std::string>& stop_interfaces)
{
}

}  // namespace duatic_ros2control_hardware
