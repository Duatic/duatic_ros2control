/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <duatic_controller_msgs/msg/drive_state.hpp>
#include <duatic_controller_msgs/msg/drive_state_collection.hpp>
#include <duatic_controllers/duadrive_status_broadcaster.hpp>

using duatic_controller_msgs::msg::DriveState;
using duatic_controller_msgs::msg::DriveStateCollection;

namespace
{
const std::vector<std::string> kJoints = { "joint1", "joint2" };

// Values not listed stay at 1.0
using InterfaceValues = std::map<std::string, double>;

hardware_interface::InterfaceDescription describe(const std::string& full_name)
{
  const auto slash = full_name.rfind('/');
  hardware_interface::InterfaceInfo info;
  info.name = full_name.substr(slash + 1);
  info.data_type = "double";
  info.initial_value = "1.0";
  return hardware_interface::InterfaceDescription(full_name.substr(0, slash), info);
}

/// Activate a broadcaster on interfaces holding the given values and return the first message it publishes.
std::optional<DriveStateCollection> first_message(const InterfaceValues& values)
{
  duatic::controllers::StatusBroadcaster controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "duadrive_status_broadcaster";
  params.update_rate = 100;
  params.controller_manager_update_rate = 100;
  params.node_options =
      controller.define_custom_node_options().parameter_overrides({ rclcpp::Parameter("joints", kJoints) });
  if (controller.init(params) != controller_interface::return_type::OK ||
      controller.get_node()->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return std::nullopt;
  }

  // The interfaces must outlive the loans handed to the controller
  std::vector<hardware_interface::StateInterface::SharedPtr> state_storage;
  std::vector<hardware_interface::LoanedStateInterface> state_loans;
  for (const auto& name : controller.state_interface_configuration().names) {
    state_storage.push_back(std::make_shared<hardware_interface::StateInterface>(describe(name)));
    const auto value = values.find(name);
    if (value != values.end() && !state_storage.back()->set_value(value->second)) {
      return std::nullopt;
    }
    state_loans.emplace_back(state_storage.back(), nullptr);
  }
  controller.assign_interfaces({}, std::move(state_loans));
  if (controller.get_node()->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return std::nullopt;
  }

  auto listener = std::make_shared<rclcpp::Node>("listener");
  std::optional<DriveStateCollection> received;
  auto subscription = listener->create_subscription<DriveStateCollection>(
      "/duadrive_status_broadcaster/state", 10, [&](const DriveStateCollection& msg) { received = msg; });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    if (controller.update(rclcpp::Time(), rclcpp::Duration(0, 10000000)) != controller_interface::return_type::OK) {
      break;
    }
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  controller.release_interfaces();
  return received;
}
}  // namespace

class StatusBroadcasterTest : public testing::Test
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
};

TEST_F(StatusBroadcasterTest, PublishesDriveHealth)
{
  const auto received = first_message({
      { "joint1/drive_state", DriveState::DRIVE_STATE_ERROR },
      { "joint1/communication_timeout", 1.0 },
      { "joint1/error_active", 1.0 },
      { "joint1/fault_latched", 1.0 },
      { "joint1/communication_timeout_latched", 0.0 },
      { "joint2/drive_state", DriveState::DRIVE_STATE_CONTROLOP },
      { "joint2/communication_timeout", 0.0 },
      { "joint2/error_active", 1.0 },
      { "joint2/fault_latched", 0.0 },
      { "joint2/communication_timeout_latched", 1.0 },
  });
  ASSERT_TRUE(received);
  ASSERT_EQ(received->states.size(), 2u);

  EXPECT_EQ(received->states[0].drive_state, DriveState::DRIVE_STATE_ERROR);
  EXPECT_TRUE(received->states[0].communication_timeout);
  EXPECT_TRUE(received->states[0].error_active);
  EXPECT_TRUE(received->states[0].fault_latched);
  EXPECT_FALSE(received->states[0].communication_timeout_latched);

  EXPECT_EQ(received->states[1].drive_state, DriveState::DRIVE_STATE_CONTROLOP);
  EXPECT_FALSE(received->states[1].communication_timeout);
  EXPECT_TRUE(received->states[1].error_active);
  EXPECT_FALSE(received->states[1].fault_latched);
  EXPECT_TRUE(received->states[1].communication_timeout_latched);
}

TEST_F(StatusBroadcasterTest, PublishesInvalidDriveStateAsNa)
{
  const auto received = first_message({
      { "joint1/drive_state", std::numeric_limits<double>::quiet_NaN() },
      { "joint2/drive_state", DriveState::DRIVE_STATE_DEVICE_MISSING + 1.0 },
  });
  ASSERT_TRUE(received);
  ASSERT_EQ(received->states.size(), 2u);

  EXPECT_EQ(received->states[0].drive_state, DriveState::DRIVE_STATE_NA);
  EXPECT_EQ(received->states[1].drive_state, DriveState::DRIVE_STATE_NA);
}
