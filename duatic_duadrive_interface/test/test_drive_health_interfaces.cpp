/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <variant>

#include <rclcpp/rclcpp.hpp>

#include "faultable_mock_drive.hpp"

using duatic::duadrive_interface::DuaDriveInterfaceParameters;
using duatic::duadrive_interface::test::FaultableMockDrive;

namespace
{
DuaDriveInterfaceParameters mock_parameters()
{
  DuaDriveInterfaceParameters params;
  params.joint_name = "joint1";
  params.device_address = 1;
  params.maximum_joint_velocity = 1.0;
  return params;
}

double mapped_value(FaultableMockDrive& drive, const std::string& name)
{
  return *std::get<double*>(drive.get_default_state_mapping().at(name));
}

void read(FaultableMockDrive& drive)
{
  ASSERT_EQ(drive.read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}
}  // namespace

TEST(DriveHealthInterfaces, ExportsDriveStateAndCommunicationTimeout)
{
  FaultableMockDrive drive(rclcpp::get_logger("test"));
  ASSERT_EQ(drive.init(mock_parameters()), hardware_interface::CallbackReturn::SUCCESS);

  const auto descriptions = drive.get_state_interface_descriptions();
  for (const std::string name : { "joint1/drive_state", "joint1/communication_timeout" }) {
    const auto found = std::any_of(descriptions.begin(), descriptions.end(),
                                   [&](const auto& description) { return description.get_name() == name; });
    EXPECT_TRUE(found) << name;
    EXPECT_NO_THROW(drive.get_default_state_mapping().at(name)) << name;
  }
}

TEST(DriveHealthInterfaces, ReportsDriveStateAsProtocolId)
{
  FaultableMockDrive drive(rclcpp::get_logger("test"));
  ASSERT_EQ(drive.init(mock_parameters()), hardware_interface::CallbackReturn::SUCCESS);

  read(drive);
  EXPECT_EQ(mapped_value(drive, "joint1/drive_state"), RSL_DRIVE_STATE_ID_CONTROLOP);

  drive.set_drive_state(rsl_drive_sdk::fsm::StateEnum::Fatal);
  read(drive);
  EXPECT_EQ(mapped_value(drive, "joint1/drive_state"), RSL_DRIVE_STATE_ID_FATAL);
}

TEST(DriveHealthInterfaces, ReportsCommunicationTimeoutOfStaleReading)
{
  FaultableMockDrive drive(rclcpp::get_logger("test"));
  ASSERT_EQ(drive.init(mock_parameters()), hardware_interface::CallbackReturn::SUCCESS);

  read(drive);
  EXPECT_EQ(mapped_value(drive, "joint1/communication_timeout"), 0.0);

  drive.set_readings_stale(true);
  read(drive);
  EXPECT_EQ(mapped_value(drive, "joint1/communication_timeout"), 1.0);
}
