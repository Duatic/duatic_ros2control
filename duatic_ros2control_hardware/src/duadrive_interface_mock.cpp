#include "duatic_ros2control_hardware/duadrive_interface_mock.hpp"

namespace duatic_ros2control_hardware
{
DuaDriveInterfaceMock::DuaDriveInterfaceMock(rclcpp::Logger logger) : DuaDriveInterfaceBase(logger)
{
}

DuaDriveInterfaceMock::~DuaDriveInterfaceMock()
{
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::init(const DuaDriveInterfaceParameters& params)
{
  params_ = params;
  logger_ = rclcpp::get_logger("DuaDriveHardwareInterfaceMock_" + params.joint_name);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::activate()
{
  drive_info_.drive_name = "mock";
  drive_info_.drive_model = "DuaDrive_mock";
  drive_info_.drive_build_tag = "none";
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::configure()
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DuaDriveInterfaceMock::deactivate()
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DuaDriveInterfaceMock::read()
{
  state_.joint_position_commanded = command_.joint_position;
  state_.joint_velocity_commanded = command_.joint_velocity;
  state_.joint_acceleration_commanded = command_.joint_acceleration;
  state_.joint_torque_commanded = command_.joint_torque;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DuaDriveInterfaceMock::write()
{
  state_.joint_torque = command_.joint_torque;
  state_.joint_acceleration = command_.joint_acceleration;
  state_.joint_velocity = command_.joint_velocity;
  state_.joint_position = command_.joint_position;
  return hardware_interface::return_type::OK;
}
}  // namespace duatic_ros2control_hardware
