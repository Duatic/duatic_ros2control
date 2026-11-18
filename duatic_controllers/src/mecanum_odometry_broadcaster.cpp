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

#include <duatic_controllers/mecanum_odometry_broadcaster.hpp>

#include <cmath>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <duatic_controllers/ros2_control_compat.hpp>

namespace duatic::controllers
{

controller_interface::InterfaceConfiguration MecanumOdometryBroadcaster::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL, {} };
}

controller_interface::InterfaceConfiguration MecanumOdometryBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Same order as WheelIndex
  for (const auto& joint : { params_.front_left_wheel_joint_name, params_.front_right_wheel_joint_name,
                             params_.rear_right_wheel_joint_name, params_.rear_left_wheel_joint_name }) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn MecanumOdometryBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<mecanum_odometry_broadcaster::ParamListener>(get_node());
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MecanumOdometryBroadcaster::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  params_ = param_listener_->get_params();

  // Prefix without a leading and with a trailing '/', the controller's namespace if no prefix is given
  std::string tf_prefix;
  if (params_.tf_frame_prefix_enable) {
    tf_prefix = params_.tf_frame_prefix.empty() ? std::string(get_node()->get_namespace()) : params_.tf_frame_prefix;
    if (tf_prefix.back() != '/') {
      tf_prefix += "/";
    }
    if (tf_prefix.front() == '/') {
      tf_prefix.erase(0, 1);
    }
  }
  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  odometry_msg_.header.frame_id = odom_frame_id;
  odometry_msg_.child_frame_id = base_frame_id;
  constexpr std::size_t dimensions = 6;
  for (std::size_t i = 0; i < dimensions; ++i) {
    odometry_msg_.pose.covariance[dimensions * i + i] = params_.pose_covariance_diagonal[i];
    odometry_msg_.twist.covariance[dimensions * i + i] = params_.twist_covariance_diagonal[i];
  }

  tf_msg_.transforms.resize(1);
  tf_msg_.transforms[0].header.frame_id = odom_frame_id;
  tf_msg_.transforms[0].child_frame_id = base_frame_id;

  odometry_pub_ = get_node()->create_publisher<OdometryMsg>("~/odometry", rclcpp::SystemDefaultsQoS());
  odometry_pub_rt_ = std::make_unique<realtime_tools::RealtimePublisher<OdometryMsg>>(odometry_pub_);
  tf_pub_ = get_node()->create_publisher<TfMsg>("~/tf_odometry", rclcpp::SystemDefaultsQoS());
  tf_pub_rt_ = std::make_unique<realtime_tools::RealtimePublisher<TfMsg>>(tf_pub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumOdometryBroadcaster::update(const rclcpp::Time& time,
                                                                     const rclcpp::Duration& period)
{
  param_listener_->try_update_params(params_);

  const auto front_left = compat::try_get_value(state_interfaces_[FRONT_LEFT]);
  const auto front_right = compat::try_get_value(state_interfaces_[FRONT_RIGHT]);
  const auto rear_right = compat::try_get_value(state_interfaces_[REAR_RIGHT]);
  const auto rear_left = compat::try_get_value(state_interfaces_[REAR_LEFT]);
  if (!front_left || !front_right || !rear_right || !rear_left) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Unable to retrieve the velocity of every wheel");
    return controller_interface::return_type::OK;
  }
  if (!std::isnan(*front_left) && !std::isnan(*front_right) && !std::isnan(*rear_right) && !std::isnan(*rear_left)) {
    integrate(*front_left, *front_right, *rear_right, *rear_left, period.seconds());
  }

  // Quaternion of a rotation by yaw about z
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(0.5 * yaw_);
  orientation.w = std::cos(0.5 * yaw_);

  odometry_msg_.header.stamp = time;
  odometry_msg_.pose.pose.position.x = x_;
  odometry_msg_.pose.pose.position.y = y_;
  odometry_msg_.pose.pose.orientation = orientation;
  odometry_msg_.twist.twist.linear.x = v_x_;
  odometry_msg_.twist.twist.linear.y = v_y_;
  odometry_msg_.twist.twist.angular.z = omega_;
  compat::publish_rt(odometry_pub_rt_, odometry_msg_);

  if (params_.enable_odom_tf) {
    auto& transform = tf_msg_.transforms.front();
    transform.header.stamp = time;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation = orientation;
    compat::publish_rt(tf_pub_rt_, tf_msg_);
  }

  return controller_interface::return_type::OK;
}

void MecanumOdometryBroadcaster::integrate(const double front_left, const double front_right, const double rear_right,
                                           const double rear_left, const double dt)
{
  // Too short to integrate with
  if (dt < 0.0001) {
    return;
  }

  // Body twist at the robot center
  const double radius = params_.kinematics.wheels_radius;
  const double v_x_center = 0.25 * radius * (front_left + rear_left + rear_right + front_right);
  const double v_y_center = 0.25 * radius * (-front_left + rear_left - rear_right + front_right);
  omega_ = 0.25 * radius / params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
           (-front_left - rear_left + rear_right + front_right);

  // Rotate into the base frame and add the lever arm of the base frame offset
  const auto& offset = params_.kinematics.base_frame_offset;
  const double cos_offset = std::cos(-offset.theta);
  const double sin_offset = std::sin(-offset.theta);
  const double v_x_rotated = cos_offset * v_x_center - sin_offset * v_y_center;
  const double v_y_rotated = sin_offset * v_x_center + cos_offset * v_y_center;
  const double lever_x = -cos_offset * offset.x + sin_offset * offset.y;
  const double lever_y = -sin_offset * offset.x - cos_offset * offset.y;
  v_x_ = v_x_rotated + lever_y * omega_;
  v_y_ = v_y_rotated - lever_x * omega_;

  // Integrate the pose in the odometry frame, rotating the twist by the already updated heading
  yaw_ += omega_ * dt;
  const double cos_yaw = std::cos(yaw_);
  const double sin_yaw = std::sin(yaw_);
  x_ += (cos_yaw * v_x_ - sin_yaw * v_y_) * dt;
  y_ += (sin_yaw * v_x_ + cos_yaw * v_y_) * dt;
}

}  // namespace duatic::controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::MecanumOdometryBroadcaster, controller_interface::ControllerInterface)
