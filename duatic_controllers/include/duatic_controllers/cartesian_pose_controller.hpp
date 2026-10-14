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

// C++ system headers
#include <cassert>
#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

// Pinocchio
#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

// Optimization
#include <proxsuite/proxqp/dense/dense.hpp>

// Other headers
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

// ROS2
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// Project
#include <duatic_controllers/cartesian_pose_controller_parameters.hpp>

namespace duatic::controllers
{

/**
 * @brief Moves a target frame to a Cartesian target pose given relative to a base frame.
 *
 * Controls all joints on the chain between the base frame and the target frame.
 */
class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
  using Self = CartesianPoseController;

  // ros2_control's controller_manager calls update(time, period) with 'time' on RCL_ROS_TIME (verified
  // via the clock-type mismatch this constant guards against; see the assert in update()).
  static constexpr rcl_clock_type_t rcl_time_source = RCL_ROS_TIME;

  // lower bound for dt and a general numeric tolerance for the QP box constraints, see update()/run_pose_diff_ik()
  static constexpr double numeric_epsilon = 1.0e-6;
  static constexpr double numeric_epsilon_inv = 1.0 / numeric_epsilon;
  static_assert(numeric_epsilon > 0.0, "numeric_epsilon must be real positive");
  static_assert(numeric_epsilon_inv > numeric_epsilon, "numeric_epsilon_inv must be real greater than numeric_epsilon");

  // this implementation relies on fast, small update steps; see the check in on_configure()
  static constexpr unsigned int min_update_rate_hz = 500;

  /// @brief Creates the controller with default parameters.
  inline CartesianPoseController()
    : controller_interface::ControllerInterface(), params_(std::make_shared<cartesian_pose_controller::Params>())
  {
  }

  /**
   * @copydoc controller_interface::ControllerInterfaceBase::command_interface_configuration
   * @note Position, and if command_velocities is set velocity, interfaces of all controlled joints; none if dry_run.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @copydoc controller_interface::ControllerInterfaceBase::state_interface_configuration
   * @note Position, and if velocity_feedback is positive velocity, interfaces of all controlled joints.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @copydoc controller_interface::ControllerInterfaceBase::update
   * @throws duatic::controllers::exceptions::MissingInterfaceValue if a state interface has no value.
   */
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// @copydoc controller_interface::ControllerInterfaceBase::on_init
  controller_interface::CallbackReturn on_init() override;

  /**
   * @copydoc rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::on_configure
   * @note Fails if the update rate is below min_update_rate_hz, a frame is unknown or the chain between the frames
   *       has no or unsupported joints.
   */
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /// @copydoc rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::on_cleanup
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @copydoc rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::on_activate
   * @throws duatic::controllers::exceptions::MissingInterfaceValue if a state interface has no value.
   */
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  // parsed target pose: linear position and angular orientation, decoupled from the wire message format
  using target_type = std::tuple<Eigen::Vector3d, Eigen::Quaterniond>;

  std::unique_ptr<cartesian_pose_controller::ParamListener> param_listener_;
  std::shared_ptr<cartesian_pose_controller::Params> params_;

  rclcpp::Logger::Level logger_level_;

  double linear_error_weight_;  // derived from params_ at on_configure, see there
  double target_filter_rate_;   // -1.0 / target_filter; floored

  pinocchio::Model robot_model_;
  pinocchio::FrameIndex base_frame_idx_;  // frame all published poses/twists are expressed relative to
  pinocchio::FrameIndex target_frame_idx_;

  pinocchio::Data state_data_;
  Eigen::VectorXd state_q_;
  Eigen::VectorXd state_v_;

  Eigen::VectorXd blend_delta_q_;

  Eigen::VectorXd control_q_;
  Eigen::VectorXd control_v_;

  // input topic subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_msg_sub_;

  // NonRT->RT unidirectional data exchange buffer holding the latest received target pose
  realtime_tools::RealtimeBuffer<target_type> target_buffer_;
  target_type control_target_;  // filtered target

  /* Quadratic programming solver
   *   min_x 1/2 x^T H x + x^T g
       s.t.  l_box <= x <= u_box
   *
   * H = w * J_lin^T * J_lin + J_ang^T * J_ang + ik_damping * I  (dense, recomputed every cycle)
   * g = -problem_scale * (w * J_lin^T * diff_linear + J_ang^T * diff_angular)
   * with x = theta, the (problem_scale-scaled) joint displacement solution and w = linear_error_weight_
   */
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_solver_;  // initialized with dimensions in on_configure()
  Eigen::MatrixXd qp_solver_H_;                                      // QP Hessian, see above
  Eigen::VectorXd qp_solver_g_;                                      // QP linear cost term, see above
  Eigen::VectorXd qp_solver_l_box_;                                  // lower box bounds
  Eigen::VectorXd qp_solver_u_box_;                                  // upper box bounds
  Eigen::VectorXd joint_velocity_box_;                               // theta bound from joint velocity limits
  Eigen::VectorXd joint_velocity_limit_;                             // floored per-joint velocity limit (rad/s)
  Eigen::VectorXd pose_diff_ik_result_;                              // solution vector
  Eigen::Matrix<double, 6, Eigen::Dynamic> qp_jacobian_;             // Jacobian memory space

  // state publication topics
  double topics_pub_period_;
  double topics_pub_next_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::UniquePtr target_pose_pub_realtime_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>::UniquePtr target_twist_pub_realtime_;

  /**
   * @brief Stores a received target pose for the following control cycles.
   *
   * Ignores, with a warning, targets whose frame_id is neither empty nor base_frame, or whose orientation is not a
   * unit quaternion.
   * @param msg Target pose relative to base_frame.
   */
  void handle_target_msg_sub(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Reads the joint positions, and if velocity_feedback is positive the joint velocities.
   * @throws duatic::controllers::exceptions::MissingInterfaceValue if a state interface has no value.
   */
  void read_states();

  /**
   * @brief Combines the measured joint state with the last command into the joint state used for control.
   * @param velocity_feedback_weight Weight of the measured joint velocities, in [0, 1]; 0 ignores them.
   */
  void blend_states(const double velocity_feedback_weight);

  /// @brief Updates the base and target frame poses and velocities from the joint state.
  void update_state();

  /**
   * @brief Computes the joint displacement that moves the target frame by the given pose difference.
   *
   * Respects the joint position and velocity limits; the result is stored in pose_diff_ik_result_.
   * @param problem_scale Numerical scaling of the problem, motion_horizon / dt.
   * @param target_diff_linear Position difference, in meters, world-aligned.
   * @param target_diff_angular Orientation difference as rotation vector, in radians, world-aligned.
   * @param verbose Print the solver output.
   * @return True if solved; otherwise an approximate result is stored.
   */
  bool run_pose_diff_ik(const double problem_scale, const Eigen::Vector3d& target_diff_linear,
                        const Eigen::Vector3d& target_diff_angular, const bool verbose = false);

  /// @brief Writes the joint position, and if command_velocities is set the joint velocity, commands.
  void command_controls();

  /**
   * @brief Logs the last IK problem, its solution and the current joint state.
   * @param target_v Current target frame twist, world-aligned.
   */
  void log_statistics(const pinocchio::Motion& target_v) const;

  /**
   * @brief Publishes the target frame pose and twist relative to base_frame, the twist at the target frame origin.
   * @param stamp Header stamp of the published messages.
   */
  void publish_topics(const rclcpp::Time& stamp);

  /// @brief Returns the current target frame pose relative to the robot model root.
  inline const pinocchio::SE3& target_pose() const
  {
    return state_data_.oMf[target_frame_idx_];
  }

  /**
   * @brief Scales a vector down, keeping its direction, so its norm does not exceed a limit.
   * @tparam Vector Eigen vector type.
   * @param v Vector to scale, in place.
   * @param limit Maximum norm, must be positive.
   */
  template <typename Vector>
  inline static void scale_limit(Vector&& v, const double limit)
  {
    assert(limit > 0.0);
    const double limit_sq = limit * limit;
    const double scale = std::sqrt(limit_sq / std::fmax(limit_sq, v.squaredNorm()));
    assert((0.0 <= scale) && (scale <= 1.0));
    v *= scale;
  }
};

}  // namespace duatic::controllers
