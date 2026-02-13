#include "duatic_ros2control_hardware/duadrive_interface_base.hpp"

#include "duatic_ros2control_hardware/interface_utils.hpp"

namespace duatic_ros2control_hardware
{

DuaDriveInterfaceBase::DuaDriveInterfaceBase(rclcpp::Logger logger) : logger_(logger)
{
}
DuaDriveInterfaceBase::~DuaDriveInterfaceBase()
{
}
const std::string& DuaDriveInterfaceBase::get_name() const
{
  return params_.joint_name;
}

void DuaDriveInterfaceBase::generate_state_interface_descriptions()
{
  state_interface_descriptions_ = {
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_POSITION, 0.0),
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_VELOCITY, 0.0),
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_ACCELERATION, 0.0),
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_EFFORT, 0.0),

    create_interface_description<double>(get_name(), "position_commanded", 0.0),
    create_interface_description<double>(get_name(), "velocity_commanded", 0.0),
    create_interface_description<double>(get_name(), "acceleration_commanded", 0.0),
    create_interface_description<double>(get_name(), "effort_commanded", 0.0),

    create_interface_description<double>(get_name(), "temperature_system", 0.0),
    create_interface_description<double>(get_name(), "temperature_coil_A", 0.0),
    create_interface_description<double>(get_name(), "temperature_coil_B", 0.0),
    create_interface_description<double>(get_name(), "temperature_coil_C", 0.0),

    create_interface_description<double>(get_name(), "bus_voltage", 0.0),

    create_interface_description<double>(get_name(), "current_d", 0.0),
    create_interface_description<double>(get_name(), "current_q", 0.0),
    create_interface_description<double>(get_name(), "current_coil_A", 0.0),
    create_interface_description<double>(get_name(), "current_coil_B", 0.0),
    create_interface_description<double>(get_name(), "current_coil_C", 0.0),

    create_interface_description<double>(get_name(), "voltage_coil_A", 0.0),
    create_interface_description<double>(get_name(), "voltage_coil_B", 0.0),
    create_interface_description<double>(get_name(), "voltage_coil_C", 0.0),

  };

  state_interface_mapping_.insert({ state_interface_descriptions_[0].get_name(), &state_.joint_position });
  state_interface_mapping_.insert({ state_interface_descriptions_[1].get_name(), &state_.joint_velocity });
  state_interface_mapping_.insert({ state_interface_descriptions_[2].get_name(), &state_.joint_position });
  state_interface_mapping_.insert({ state_interface_descriptions_[3].get_name(), &state_.joint_acceleration });
  state_interface_mapping_.insert({ state_interface_descriptions_[4].get_name(), &state_.joint_torque });

  state_interface_mapping_.insert({ state_interface_descriptions_[5].get_name(), &state_.joint_position_commanded });
  state_interface_mapping_.insert({ state_interface_descriptions_[6].get_name(), &state_.joint_velocity_commanded });
  state_interface_mapping_.insert(
      { state_interface_descriptions_[7].get_name(), &state_.joint_acceleration_commanded });
  state_interface_mapping_.insert({ state_interface_descriptions_[7].get_name(), &state_.joint_torque_commanded });

  state_interface_mapping_.insert({ state_interface_descriptions_[8].get_name(), &state_.temperature_system });
  state_interface_mapping_.insert({ state_interface_descriptions_[9].get_name(), &state_.temperature_coil_A });
  state_interface_mapping_.insert({ state_interface_descriptions_[10].get_name(), &state_.temperature_coil_B });
  state_interface_mapping_.insert({ state_interface_descriptions_[11].get_name(), &state_.temperature_coil_C });

  state_interface_mapping_.insert({ state_interface_descriptions_[12].get_name(), &state_.bus_voltage });
  state_interface_mapping_.insert({ state_interface_descriptions_[13].get_name(), &state_.current_d });
  state_interface_mapping_.insert({ state_interface_descriptions_[14].get_name(), &state_.current_q });
  state_interface_mapping_.insert({ state_interface_descriptions_[15].get_name(), &state_.current_coil_A });
  state_interface_mapping_.insert({ state_interface_descriptions_[16].get_name(), &state_.current_coil_B });
  state_interface_mapping_.insert({ state_interface_descriptions_[17].get_name(), &state_.current_coil_C });

  state_interface_mapping_.insert({ state_interface_descriptions_[18].get_name(), &state_.voltage_coil_A });
  state_interface_mapping_.insert({ state_interface_descriptions_[19].get_name(), &state_.voltage_coil_B });
  state_interface_mapping_.insert({ state_interface_descriptions_[20].get_name(), &state_.voltage_coil_C });
}
void DuaDriveInterfaceBase::generate_command_interface_desriptions()
{
  command_interface_descriptions_ = {
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_POSITION, 0.0),
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_VELOCITY, 0.0),
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_ACCELERATION, 0.0),
    create_interface_description<double>(get_name(), hardware_interface::HW_IF_EFFORT, 0.0),
    create_interface_description<double>(get_name(), "p_gain", 0.0),
    create_interface_description<double>(get_name(), "i_gain", 0.0),
    create_interface_description<double>(get_name(), "d_gain", 0.0),
    create_interface_description<bool>(get_name(), "freeze_mode", true)
  };

  command_interface_mapping_.insert({ command_interface_descriptions_[0].get_name(), &command_.joint_position });
  command_interface_mapping_.insert({ command_interface_descriptions_[1].get_name(), &command_.joint_velocity });
  command_interface_mapping_.insert({ command_interface_descriptions_[2].get_name(), &command_.joint_acceleration });
  command_interface_mapping_.insert({ command_interface_descriptions_[3].get_name(), &command_.joint_torque });
  command_interface_mapping_.insert({ command_interface_descriptions_[4].get_name(), &command_.p_gain });
  command_interface_mapping_.insert({ command_interface_descriptions_[5].get_name(), &command_.i_gain });
  command_interface_mapping_.insert({ command_interface_descriptions_[6].get_name(), &command_.d_gain });
  command_interface_mapping_.insert({ command_interface_descriptions_[7].get_name(), &command_.joint_freeze_mode });
}

hardware_interface::CallbackReturn DuaDriveInterfaceBase::configure()
{
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn DuaDriveInterfaceBase::activate()
{
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn DuaDriveInterfaceBase::deactivate()
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DuaDriveInterfaceBase::read()
{
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type DuaDriveInterfaceBase::write()
{
  return hardware_interface::return_type::OK;
}

}  // namespace duatic_ros2control_hardware
