/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include "duatic_duadrive_interface/coupled_kinematics_hardware_interface.hpp"
#include "duatic_duadrive_interface/coupled_kinematics_invariant_mapping.hpp"
#include "faultable_mock_drive.hpp"

namespace
{
using duatic::duadrive_interface::CoupledKinematicsHardwareInterfaceBase;
using duatic::duadrive_interface::test::FaultableMockDrive;
using rsl_drive_sdk::fsm::StateEnum;

class TwoDriveSystem
  : public CoupledKinematicsHardwareInterfaceBase<FaultableMockDrive,
                                                  duatic::duadrive_interface::kinematics::InvariantKinematicsMapping<2>>
{
public:
  FaultableMockDrive& drive(const std::size_t index)
  {
    return *drives_.at(index);
  }
};

std::string joint(const std::string& name, const std::string& drive_parameter_file,
                  const std::string& communication_timeout_ms)
{
  return R"(
    <joint name=")" +
         name + R"(">
      <param name="ethercat_address">1</param>
      <param name="ethercat_bus">none</param>
      <param name="parent_unit">test</param>
      <param name="drive_parameter_file_path">)" +
         drive_parameter_file + R"(</param>
      <param name="has_brake">false</param>
      <param name="brake_clearance">0.9</param>
      <param name="communication_timeout">)" +
         communication_timeout_ms + R"(</param>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      <state_interface name="acceleration"/>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
    </joint>)";
}

std::string urdf(const std::string& drive_parameter_file, const std::string& joint2_communication_timeout_ms)
{
  return R"(<?xml version="1.0"?>
<robot name="two_joints">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="10" velocity="5"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="10" velocity="5"/>
  </joint>
  <ros2_control name="TwoJointSystem" type="system">
    <hardware><plugin>unused</plugin></hardware>)" +
         joint("joint1", drive_parameter_file, "100") +
         joint("joint2", drive_parameter_file, joint2_communication_timeout_ms) + R"(
  </ros2_control>
</robot>
)";
}

class CoupledErrorActiveTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    drive_parameter_file_ =
        std::filesystem::temp_directory_path() / ("test_coupled_error_active_" + std::to_string(getpid()) + ".yaml");
    std::ofstream(drive_parameter_file_) << "{}\n";

    ASSERT_EQ(system_.init(params_with_joint2_timeout("250")), hardware_interface::CallbackReturn::SUCCESS);
    system_.on_export_state_interfaces();
    system_.on_export_command_interfaces();
    ASSERT_EQ(system_.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system_.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    std::filesystem::remove(drive_parameter_file_);
  }

  hardware_interface::HardwareComponentParams params_with_joint2_timeout(const std::string& communication_timeout_ms)
  {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info =
        hardware_interface::parse_control_resources_from_urdf(urdf(drive_parameter_file_, communication_timeout_ms))[0];
    params.clock = std::make_shared<rclcpp::Clock>();
    return params;
  }

  void read()
  {
    ASSERT_EQ(system_.read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
  }

  double state(const std::string& name)
  {
    return system_.get_state<double>(name);
  }

  std::filesystem::path drive_parameter_file_;
  TwoDriveSystem system_;
};
}  // namespace

TEST_F(CoupledErrorActiveTest, DrivesUseTheCommunicationTimeoutFromTheUrdf)
{
  EXPECT_EQ(system_.drive(0).communication_timeout(), std::chrono::milliseconds(100));
  EXPECT_EQ(system_.drive(1).communication_timeout(), std::chrono::milliseconds(250));
}

TEST_F(CoupledErrorActiveTest, StaleReadingsHelperRestoresTheConfiguredTimeout)
{
  system_.drive(1).set_readings_stale(true);
  system_.drive(1).set_readings_stale(false);
  EXPECT_EQ(system_.drive(1).communication_timeout(), std::chrono::milliseconds(250));
}

TEST_F(CoupledErrorActiveTest, NonPositiveCommunicationTimeoutFailsInit)
{
  for (const std::string timeout_ms : { "0", "-5" }) {
    TwoDriveSystem system;
    EXPECT_EQ(system.init(params_with_joint2_timeout(timeout_ms)), hardware_interface::CallbackReturn::FAILURE)
        << timeout_ms;
  }
}

TEST_F(CoupledErrorActiveTest, NothingLatchedWhileHealthy)
{
  read();
  for (const std::string joint_name : { "joint1", "joint2" }) {
    EXPECT_EQ(state(joint_name + "/error_active"), 0.0) << joint_name;
    EXPECT_EQ(state(joint_name + "/fault_latched"), 0.0) << joint_name;
    EXPECT_EQ(state(joint_name + "/communication_timeout_latched"), 0.0) << joint_name;
  }
}

TEST_F(CoupledErrorActiveTest, DriveFaultLatchesSystemAndNamesTheDrive)
{
  system_.drive(0).set_drive_state(StateEnum::Error);
  read();
  // The drive recovers, the latches do not
  system_.drive(0).set_drive_state(StateEnum::ControlOp);
  read();

  EXPECT_EQ(state("joint1/drive_state"), RSL_DRIVE_STATE_ID_CONTROLOP);
  EXPECT_EQ(state("joint1/error_active"), 1.0);
  EXPECT_EQ(state("joint1/fault_latched"), 1.0);
  EXPECT_EQ(state("joint1/communication_timeout_latched"), 0.0);
  EXPECT_EQ(state("joint2/error_active"), 1.0);
  EXPECT_EQ(state("joint2/fault_latched"), 0.0);
}

TEST_F(CoupledErrorActiveTest, CommunicationTimeoutLatchesSystemAndNamesTheDrive)
{
  system_.drive(1).set_readings_stale(true);
  read();
  system_.drive(1).set_readings_stale(false);
  read();

  EXPECT_EQ(state("joint2/communication_timeout"), 0.0);
  EXPECT_EQ(state("joint2/error_active"), 1.0);
  EXPECT_EQ(state("joint2/communication_timeout_latched"), 1.0);
  EXPECT_EQ(state("joint2/fault_latched"), 0.0);
  EXPECT_EQ(state("joint1/error_active"), 1.0);
  EXPECT_EQ(state("joint1/communication_timeout_latched"), 0.0);
}

TEST_F(CoupledErrorActiveTest, FailuresAfterTheFreezeAreNotNamed)
{
  system_.drive(0).set_drive_state(StateEnum::Error);
  read();
  system_.drive(1).set_readings_stale(true);
  system_.drive(1).set_drive_state(StateEnum::Fatal);
  read();

  EXPECT_EQ(state("joint1/fault_latched"), 1.0);
  EXPECT_EQ(state("joint2/communication_timeout"), 1.0);
  EXPECT_EQ(state("joint2/communication_timeout_latched"), 0.0);
  EXPECT_EQ(state("joint2/fault_latched"), 0.0);
}

TEST_F(CoupledErrorActiveTest, FailuresInTheSameCycleAreAllNamed)
{
  system_.drive(0).set_drive_state(StateEnum::Error);
  system_.drive(1).set_readings_stale(true);
  read();

  EXPECT_EQ(state("joint1/fault_latched"), 1.0);
  EXPECT_EQ(state("joint2/communication_timeout_latched"), 1.0);
}

TEST_F(CoupledErrorActiveTest, LatchSurvivesReactivation)
{
  system_.drive(0).set_drive_state(StateEnum::Fatal);
  read();
  system_.drive(0).set_drive_state(StateEnum::ControlOp);
  ASSERT_EQ(system_.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  read();

  EXPECT_EQ(state("joint1/error_active"), 1.0);
  EXPECT_EQ(state("joint1/fault_latched"), 1.0);
}
