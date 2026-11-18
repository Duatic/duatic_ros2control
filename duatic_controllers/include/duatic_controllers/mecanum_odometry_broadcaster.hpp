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

#include <cstddef>
#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_publisher.hpp>
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
 */
class MecanumOdometryBroadcaster : public controller_interface::ControllerInterface
{
public:
  /**
   * @brief Claims no command interfaces.
   * @return An empty individual interface configuration.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Claims the velocity state interface of each wheel.
   * @return The interfaces in the order front left, front right, rear right, rear left, which update() relies on.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Integrates the wheel velocities over period and publishes the odometry stamped with time.
   *
   * Skips integration while a wheel velocity is NaN and publishes nothing while a wheel velocity is unavailable.
   *
   * @param time Stamp of the published messages.
   * @param period Integration step; steps below 0.1 ms leave the odometry unchanged.
   * @return Always OK.
   */
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * @brief Creates the parameter listener.
   * @return ERROR if the parameters cannot be declared, SUCCESS otherwise.
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Reads the parameters, sets the kinematics and frames and creates the publishers.
   *
   * The integrated pose is kept, so it continues across a cleanup and configure.
   *
   * @return SUCCESS.
   * @throws rclcpp::exceptions::RCLError If a publisher cannot be created.
   */
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

private:
  /// @brief Index of each wheel in the claimed state interfaces.
  enum WheelIndex : std::size_t
  {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_RIGHT = 2,
    REAR_LEFT = 3,
  };

  /**
   * @brief Advances the odometry by one step of mecanum forward kinematics.
   *
   * @param front_left Wheel velocity in rad/s.
   * @param front_right Wheel velocity in rad/s.
   * @param rear_right Wheel velocity in rad/s.
   * @param rear_left Wheel velocity in rad/s.
   * @param dt Step in s; below 0.1 ms neither pose nor twist change.
   */
  void integrate(double front_left, double front_right, double rear_right, double rear_left, double dt);

  std::shared_ptr<mecanum_odometry_broadcaster::ParamListener> param_listener_;
  mecanum_odometry_broadcaster::Params params_;

  // Pose in the odometry frame and twist in the base frame
  double x_ = 0.0;      ///< [m]
  double y_ = 0.0;      ///< [m]
  double yaw_ = 0.0;    ///< [rad]
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
};
}  // namespace duatic::controllers
