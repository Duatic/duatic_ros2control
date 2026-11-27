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

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>

#include <control_msgs/srv/set_odometry.hpp>
#include <controller_interface/controller_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <duatic_controllers/mecanum_odometry_broadcaster_parameters.hpp>

namespace duatic::controllers
{
/**
 * @brief Publishes the wheel odometry of a four wheel mecanum base without commanding the wheels.
 *
 * Computes and publishes odometry the same way as `mecanum_drive_controller/MecanumDriveController` of
 * ros2_controllers, on `~/odometry` and, if `enable_odom_tf` is set, `~/tf_odometry`. It only claims the wheel
 * velocity state interfaces, so it can run next to a mecanum drive controller or while that one is inactive.
 *
 * The `~/set_odometry` service moves the integrated pose to a requested one.
 */
class MecanumOdometryBroadcaster : public controller_interface::ControllerInterface
{
public:
  /**
   * @copydoc controller_interface::ControllerInterfaceBase::command_interface_configuration
   * @note None, the wheels are never commanded.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @copydoc controller_interface::ControllerInterfaceBase::state_interface_configuration
   * @note The velocity interface of each wheel, in the order of WheelIndex.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @copydoc controller_interface::ControllerInterfaceBase::update
   * @note Neither integrates nor publishes while a wheel velocity is unavailable or not finite.
   */
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// @copydoc controller_interface::ControllerInterfaceBase::on_init
  controller_interface::CallbackReturn on_init() override;

  /**
   * @copydoc rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::on_configure
   * @note Keeps the integrated pose, so it continues across a cleanup and configure.
   */
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

private:
  /// @brief Index of each wheel in the claimed state interfaces and in WheelVelocities.
  enum WheelIndex : std::size_t
  {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_RIGHT = 2,
    REAR_LEFT = 3,
    WHEEL_COUNT = 4,
  };

  /// @brief Velocity of each wheel in rad/s, indexed by WheelIndex.
  using WheelVelocities = std::array<double, WheelIndex::WHEEL_COUNT>;

  /**
   * @brief Advances the odometry by one step of mecanum forward kinematics.
   *
   * @param wheel_velocities Velocity of each wheel in rad/s.
   * @param dt Step in s.
   */
  void integrate(const WheelVelocities& wheel_velocities, double dt);

  using SetOdometrySrv = control_msgs::srv::SetOdometry;

  /**
   * @brief Hands a requested pose to update(), which continues the odometry from it.
   *
   * @param request Pose in the odometry frame, x and y in m and yaw in rad.
   * @param response Success unless the controller is not active.
   */
  void set_odometry(const SetOdometrySrv::Request& request, SetOdometrySrv::Response& response);

  std::shared_ptr<mecanum_odometry_broadcaster::ParamListener> param_listener_;
  mecanum_odometry_broadcaster::Params params_;

  // Kinematics constants derived from the parameters in on_configure
  double linear_factor_ = 0.0;   ///< Wheel velocity sum [rad/s] to body velocity [m/s]
  double angular_factor_ = 0.0;  ///< Wheel velocity sum [rad/s] to yaw rate [rad/s]
  double cos_offset_ = 1.0;      ///< Cosine of the inverted base frame offset angle
  double sin_offset_ = 0.0;      ///< Sine of the inverted base frame offset angle
  double lever_x_ = 0.0;         ///< [m] Robot center relative to the base frame, in the base frame
  double lever_y_ = 0.0;         ///< [m]

  // Pose in the odometry frame and twist in the base frame
  double x_ = 0.0;      ///< [m]
  double y_ = 0.0;      ///< [m]
  double yaw_ = 0.0;    ///< [rad] in [-pi, pi]
  double v_x_ = 0.0;    ///< [m/s]
  double v_y_ = 0.0;    ///< [m/s]
  double omega_ = 0.0;  ///< [rad/s]

  using OdometryMsg = nav_msgs::msg::Odometry;
  using TfMsg = tf2_msgs::msg::TFMessage;
  rclcpp::Publisher<OdometryMsg>::SharedPtr odometry_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<OdometryMsg>> odometry_pub_rt_;
  rclcpp::Publisher<TfMsg>::SharedPtr tf_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<TfMsg>> tf_pub_rt_;
  OdometryMsg odometry_msg_;
  TfMsg tf_msg_;

  rclcpp::Service<SetOdometrySrv>::SharedPtr set_odometry_service_;
  realtime_tools::RealtimeThreadSafeBox<SetOdometrySrv::Request> requested_pose_;
  std::atomic<bool> set_odometry_requested_{ false };
};
}  // namespace duatic::controllers
