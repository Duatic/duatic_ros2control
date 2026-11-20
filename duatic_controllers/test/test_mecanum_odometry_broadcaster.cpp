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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <duatic_controllers/mecanum_odometry_broadcaster.hpp>

namespace
{
constexpr double kWheelsRadius = 0.1;
constexpr double kCenterProjection = 0.5;
}  // namespace

class MecanumOdometryBroadcasterTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  /// Start the ROS context shared by the tests of this suite.
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  /// Stop the ROS context shared by the tests of this suite.
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  /// Bring up an active broadcaster whose wheel velocity state interfaces all start at zero.
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "mecanum_odometry_broadcaster";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_options = controller_.define_custom_node_options().parameter_overrides({
        { "front_left_wheel_joint_name", "fl" },
        { "front_right_wheel_joint_name", "fr" },
        { "rear_right_wheel_joint_name", "rr" },
        { "rear_left_wheel_joint_name", "rl" },
        { "kinematics.wheels_radius", kWheelsRadius },
        { "kinematics.sum_of_robot_center_projection_on_X_Y_axis", kCenterProjection },
        { "pose_covariance_diagonal", std::vector<double>{ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 } },
    });
    ASSERT_EQ(controller_.init(params), controller_interface::return_type::OK);
    ASSERT_EQ(controller_.get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(controller_.command_interface_configuration().type,
              controller_interface::interface_configuration_type::NONE);

    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    for (const auto& full_name : controller_.state_interface_configuration().names) {
      const auto slash = full_name.rfind('/');
      hardware_interface::InterfaceInfo info;
      info.name = full_name.substr(slash + 1);
      info.data_type = "double";
      info.initial_value = "0.0";
      state_storage_.push_back(std::make_shared<hardware_interface::StateInterface>(
          hardware_interface::InterfaceDescription(full_name.substr(0, slash), info)));
      state_loans.emplace_back(state_storage_.back(), nullptr);
    }
    controller_.assign_interfaces({}, std::move(state_loans));
    ASSERT_EQ(controller_.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    listener_ = std::make_shared<rclcpp::Node>("odometry_listener");
    subscription_ = listener_->create_subscription<nav_msgs::msg::Odometry>(
        "/mecanum_odometry_broadcaster/odometry", rclcpp::SystemDefaultsQoS(),
        [this](const nav_msgs::msg::Odometry& msg) {
          if (rclcpp::Time(msg.header.stamp) >= since_) {
            received_ = msg;
          }
        });
  }

  void TearDown() override
  /// Return the loaned interfaces before the storage backing them goes away.
  {
    controller_.release_interfaces();
  }

  /**
   * @brief Set the velocity the hardware reports for a wheel joint.
   *
   * @param joint Joint name as passed in the `*_wheel_joint_name` parameters.
   * @param velocity Wheel velocity in rad/s.
   */
  void set_velocity(const std::string& joint, double velocity)
  {
    for (const auto& interface : state_storage_) {
      if (interface->get_name() == joint + "/velocity") {
        ASSERT_TRUE(interface->set_value(velocity));
        return;
      }
    }
    FAIL() << "Velocity state interface not claimed: " << joint;
  }

  /**
   * @brief Run 10 ms updates until an odometry message from one of them arrives on the topic.
   *
   * Messages still queued from earlier updates are ignored.
   *
   * @return The first message received, or nothing if none arrived within 5 s.
   */
  std::optional<nav_msgs::msg::Odometry> receive()
  {
    received_.reset();
    since_ = controller_.get_node()->now();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received_ && std::chrono::steady_clock::now() < deadline) {
      EXPECT_EQ(controller_.update(controller_.get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
                controller_interface::return_type::OK);
      rclcpp::spin_some(listener_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received_;
  }

  duatic::controllers::MecanumOdometryBroadcaster controller_;
  // The interfaces must outlive the loans handed to the controller
  std::vector<hardware_interface::StateInterface::SharedPtr> state_storage_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::optional<nav_msgs::msg::Odometry> received_;
  rclcpp::Time since_{ 0, 0, RCL_ROS_TIME };
};

TEST_F(MecanumOdometryBroadcasterTest, PublishesBodyTwistOfMecanumForwardKinematics)
{
  // Distinct speeds per wheel, so swapping any two wheels changes the twist
  set_velocity("fl", 1.0);
  set_velocity("fr", 2.0);
  set_velocity("rr", 4.0);
  set_velocity("rl", 8.0);

  const auto msg = receive();
  ASSERT_TRUE(msg);
  const double r = kWheelsRadius;
  EXPECT_NEAR(msg->twist.twist.linear.x, 0.25 * r * (1.0 + 2.0 + 4.0 + 8.0), 1e-12);
  EXPECT_NEAR(msg->twist.twist.linear.y, 0.25 * r * (-1.0 + 2.0 - 4.0 + 8.0), 1e-12);
  EXPECT_NEAR(msg->twist.twist.angular.z, 0.25 * r / kCenterProjection * (-1.0 + 2.0 + 4.0 - 8.0), 1e-12);
  EXPECT_EQ(msg->header.frame_id, "odom");
  EXPECT_EQ(msg->child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(msg->pose.covariance[7], 2.0);
  EXPECT_DOUBLE_EQ(msg->pose.covariance[35], 6.0);
}

TEST_F(MecanumOdometryBroadcasterTest, IntegratesStraightDriveIntoPosition)
{
  ASSERT_TRUE(receive());
  for (const auto* joint : { "fl", "fr", "rr", "rl" }) {
    set_velocity(joint, 10.0);
  }

  // 1 m/s forward for 100 updates of 10 ms
  for (int i = 0; i < 100; i++) {
    ASSERT_EQ(controller_.update(controller_.get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
              controller_interface::return_type::OK);
  }
  const auto msg = receive();
  ASSERT_TRUE(msg);
  // receive() runs at least one more update before the message it returns
  EXPECT_GE(msg->pose.pose.position.x, 1.0 + 0.01 - 1e-9);
  EXPECT_NEAR(msg->pose.pose.position.y, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(msg->pose.pose.orientation.w, 1.0);
}

TEST_F(MecanumOdometryBroadcasterTest, IgnoresNonFiniteWheelVelocities)
{
  ASSERT_TRUE(receive());
  for (const auto* joint : { "fl", "fr", "rr", "rl" }) {
    set_velocity(joint, 10.0);
  }

  for (const double invalid : { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() }) {
    set_velocity("fl", invalid);
    for (int i = 0; i < 10; i++) {
      ASSERT_EQ(controller_.update(controller_.get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
                controller_interface::return_type::OK);
    }
  }

  for (const auto* joint : { "fl", "fr", "rr", "rl" }) {
    set_velocity(joint, 0.0);
  }
  const auto msg = receive();
  ASSERT_TRUE(msg);
  EXPECT_DOUBLE_EQ(msg->pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(msg->pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(msg->pose.pose.orientation.w, 1.0);
}

TEST_F(MecanumOdometryBroadcasterTest, KeepsHeadingWithinHalfTurnWhileSpinning)
{
  ASSERT_TRUE(receive());
  // Spin at 2.5 pi rad/s for 1 s, so the unbounded heading would reach 2.5 pi
  const double wheel_velocity = 2.5 * std::numbers::pi * kCenterProjection / kWheelsRadius;
  set_velocity("fl", -wheel_velocity);
  set_velocity("rl", -wheel_velocity);
  set_velocity("fr", wheel_velocity);
  set_velocity("rr", wheel_velocity);
  for (int i = 0; i < 100; i++) {
    ASSERT_EQ(controller_.update(controller_.get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
              controller_interface::return_type::OK);
  }

  for (const auto* joint : { "fl", "fr", "rr", "rl" }) {
    set_velocity(joint, 0.0);
  }
  const auto msg = receive();
  ASSERT_TRUE(msg);
  // A heading in [-pi, pi] gives a quaternion with w >= 0, 2.5 pi would give cos(1.25 pi) < 0
  EXPECT_GE(msg->pose.pose.orientation.w, 0.0);
  EXPECT_NEAR(2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w), 0.5 * std::numbers::pi,
              1e-9);
}

TEST_F(MecanumOdometryBroadcasterTest, IntegratesStepsShorterThanATenthOfAMillisecond)
{
  ASSERT_TRUE(receive());
  for (const auto* joint : { "fl", "fr", "rr", "rl" }) {
    set_velocity(joint, 10.0);
  }

  // 1 m/s forward for 1000 updates of 0.05 ms
  for (int i = 0; i < 1000; i++) {
    ASSERT_EQ(controller_.update(controller_.get_node()->now(), rclcpp::Duration::from_seconds(0.00005)),
              controller_interface::return_type::OK);
  }
  for (const auto* joint : { "fl", "fr", "rr", "rl" }) {
    set_velocity(joint, 0.0);
  }
  const auto msg = receive();
  ASSERT_TRUE(msg);
  EXPECT_NEAR(msg->pose.pose.position.x, 0.05, 1e-9);
}
