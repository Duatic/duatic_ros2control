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
#include <new>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

// Pinocchio
#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>

// Optimization
#include <proxsuite/proxqp/dense/dense.hpp>

// Other headers
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "controller_interface/chainable_controller_interface.hpp"

// ROS2
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_publisher.hpp>

// Project
#include <duatic_concurrency/unidirectional_buffer.hpp>
#include <duatic_controllers/cartesian_pose_controller_parameters.hpp>
#include <duatic_controllers/interface_utils.hpp>
#include <duatic_geometry/geometry.hpp>
#include <duatic_geometry_msgs/duatic_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <duatic_trajectory/trajectory.hpp>

namespace duatic::controllers
{

class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
  using Self = CartesianPoseController;

  struct KinematicTrajectorySettingsParams : cartesian_pose_controller::Params
  {
    using ScalarType = double;

    ScalarType velocity_limit_linear() const
    {
      return limits.velocity.linear;
    }

    ScalarType velocity_limit_angular() const
    {
      return limits.velocity.angular;
    }

    ScalarType acceleration_limit_linear() const
    {
      return limits.acceleration.linear;
    }

    ScalarType acceleration_limit_angular() const
    {
      return limits.acceleration.angular;
    }

    KinematicTrajectorySettingsParams& operator=(cartesian_pose_controller::Params&& rhs)
    {
      cartesian_pose_controller::Params::operator=(std::move(rhs));
      return *this;
    }
  };
  static_assert(trajectory::is_kinematic_trajectory_settings_v<KinematicTrajectorySettingsParams>);

  using trajectory_type = trajectory::ExponentialApproachPose3DC2<
      double, rclcpp::Time,
      trajectory::KinematicTrajectorySettingsExponentialApproachDefault<double, KinematicTrajectorySettingsParams>>;
  static constexpr geometry::KinematicOrder trajectory_eval_order_depth = geometry::KinematicOrder::Twist;
  static constexpr geometry::KinematicOrder trajectory_start_order_depth = geometry::KinematicOrder::Twist;

  static constexpr auto required_state_order_depth =
      geometry::KinematicOrder::Twist;  // state order depth required by the internal functionality and pinocchio model
  static constexpr auto state_order_depth = std::max(required_state_order_depth, trajectory_start_order_depth);

  using ScalarType = typename trajectory_type::ScalarType;
  using TimestampType = typename trajectory_type::TimestampType;
  using trajectory_settings_type = typename trajectory_type::KinematicTrajectorySettingsType;

  using trajectory_target_type = typename trajectory_type::TrajectoryDescriptionType;
  using trajectory_target_msg_type = duatic_geometry_msgs::msg_stamped_t<trajectory_target_type>;

  using trajectory_update_state_type = typename trajectory_type::UpdateStateType;

  using trajectory_eval_type = typename trajectory_type::template KinematicState<trajectory_eval_order_depth>;
  static_assert(geometry::is_kinematic_state_v<trajectory_eval_type>);

  // ros2_control's controller_manager calls update(time, period) with 'time' on RCL_ROS_TIME (verified
  // via the clock-type mismatch this constant guards against; see the assert in update()).
  static constexpr rcl_clock_type_t rcl_time_source = RCL_ROS_TIME;

  inline CartesianPoseController()
    : controller_interface::ControllerInterface()
    , params_(std::make_shared<trajectory_settings_type>())
    , trajectory_buffer_(params_)
  {
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  std::unique_ptr<cartesian_pose_controller::ParamListener> param_listener_;
  std::shared_ptr<trajectory_settings_type> params_;

  // optimization horizon and limits
  double motion_horizon_;
  double v_limit_lin_, v_limit_ang_;
  double linear_error_weight_;

  pinocchio::Model robot_model_;
  std::vector<pinocchio::JointIndex> joint_model_idx_;
  std::vector<Eigen::Index> joint_q_idx_;
  std::vector<Eigen::Index> joint_v_idx_;
  pinocchio::FrameIndex base_frame_idx_;  // frame all published poses/twists are expressed relative to
  pinocchio::FrameIndex target_frame_idx_;
  std::vector<pinocchio::FrameIndex> observed_frame_indices_;  // aligned with params_->limits.velocity.observed_frames_linear

  pinocchio::Data state_data_;
  Eigen::VectorXd state_q_;
  Eigen::VectorXd state_v_;

  pinocchio::Data control_data_;
  Eigen::VectorXd control_q_;
  Eigen::VectorXd control_v_;

  // input topic subscriptions
  std::shared_ptr<rclcpp::Subscription<trajectory_target_msg_type>> target_msg_sub_;

  // lock-free RT-NonRT unidirectional data exchange buffer
  duatic::concurrency::UnidirectionalBuffer<TimestampType> current_time_buffer_;
  duatic::concurrency::UnidirectionalBuffer<trajectory_type> trajectory_buffer_;

  trajectory_eval_type trajectory_eval_target_;
  geometry::Twist3Dd trajectory_eval_target_pose_diff_;
  geometry::Twist3Dd solution_pose_diff_;

  /* Quadratic programming solver
   *   min_x 1/2 x^T H x + x^T g
       s.t.  l_box <= x <= u_box
   *
   * H = 1 + J^T * J  (dense, recomputed every cycle; the '1' from ik_damping)
   * g = -J^T * pose_diff
   * with x = theta, the (problem_scale-scaled) joint displacement solution
   */
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_solver_;  // initialized with dimensions in the contrustor
  Eigen::MatrixXd qp_solver_H_;                                      // I + w * J^T * J (dense, recomputed every cycle)
  Eigen::VectorXd qp_solver_g_;                                      // -w * J^T * pose_diff (linear cost term)
  Eigen::VectorXd qp_solver_l_box_;                                  // lower box bounds
  Eigen::VectorXd qp_solver_u_box_;                                  // upper box bounds
  Eigen::VectorXd pose_diff_ik_result_;                              // solution vector
  Eigen::Matrix<double, 6, Eigen::Dynamic> qp_jacobian_;             // Jacobian memory space
  Eigen::Matrix<double, Eigen::Dynamic, 6> qp_jacobian_t_;           // Jacobian transpose memory space

  // state publication topics
  double topics_pub_period_;
  double topics_pub_next_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::UniquePtr target_pose_pub_realtime_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>::UniquePtr target_twist_pub_realtime_;
  // one pose/twist publisher pair per observed frame, aligned with observed_frame_indices_
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> observed_frame_pose_pub_;
  std::vector<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::UniquePtr>
      observed_frame_pose_pub_realtime_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> observed_frame_twist_pub_;
  std::vector<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>::UniquePtr>
      observed_frame_twist_pub_realtime_;

  void handle_target_msg_sub(const trajectory_target_msg_type::SharedPtr msg);

  void update_state(const bool use_hw_positions);

  void run_pose_diff_ik(const double problem_scale, const geometry::Twist3Dd& scaled_target_diff,
                        const bool verbose = false);

  double apply_frame_velocity_limits(const double period_seconds);

  void command_controls();

  void log_statistics() const;

  void publish_topics();

  inline const pinocchio::SE3& target_pose() const
  {  // for convenience
    return state_data_.oMf[target_frame_idx_];
  }

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
