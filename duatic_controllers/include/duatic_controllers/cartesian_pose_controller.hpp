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
#include <string>
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
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// Project
#include <duatic_concurrency/unidirectional_buffer.hpp>
#include <duatic_controllers/cartesian_pose_controller_parameters.hpp>

namespace duatic::controllers
{

class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
  using Self = CartesianPoseController;

  // ros2_control's controller_manager calls update(time, period) with 'time' on RCL_ROS_TIME (verified
  // via the clock-type mismatch this constant guards against; see the assert in update()).
  static constexpr rcl_clock_type_t rcl_time_source = RCL_ROS_TIME;

  // lower bound for dt and a general numeric tolerance for the QP box constraints, see update()/run_pose_diff_ik()
  static constexpr double numeric_epsilon = 1.0e-6;
  static_assert(numeric_epsilon > 0.0, "numeric_epsilon must be real positive");

  inline CartesianPoseController()
    : controller_interface::ControllerInterface(), params_(std::make_shared<cartesian_pose_controller::Params>())
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
  std::shared_ptr<cartesian_pose_controller::Params> params_;

  double linear_error_weight_;  // derived from params_ at on_configure, see there

  pinocchio::Model robot_model_;
  std::vector<Eigen::Index> joint_q_idx_;
  std::vector<Eigen::Index> joint_v_idx_;
  pinocchio::FrameIndex base_frame_idx_;  // frame all published poses/twists are expressed relative to
  pinocchio::FrameIndex target_frame_idx_;

  pinocchio::Data state_data_;
  Eigen::VectorXd state_q_;
  Eigen::VectorXd state_v_;

  Eigen::VectorXd control_q_;
  Eigen::VectorXd control_v_;

  // input topic subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_msg_sub_;

  // lock-free RT-NonRT unidirectional data exchange buffer holding the latest received target pose
  duatic::concurrency::UnidirectionalBuffer<geometry_msgs::msg::PoseStamped> target_buffer_;

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
  Eigen::VectorXd joint_velocity_box_;                               // theta bound from joint velocity limits
  Eigen::VectorXd pose_diff_ik_result_;                              // solution vector
  Eigen::Matrix<double, 6, Eigen::Dynamic> qp_jacobian_;             // Jacobian memory space

  // state publication topics
  double topics_pub_period_;
  double topics_pub_next_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::UniquePtr target_pose_pub_realtime_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>::UniquePtr target_twist_pub_realtime_;

  void handle_target_msg_sub(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void read_states(const double velocity_feedback_weight);

  void update_state();

  bool run_pose_diff_ik(const double problem_scale, const Eigen::Vector3d& target_diff_linear,
                        const Eigen::Vector3d& target_diff_angular, const bool verbose = false);

  void command_controls();

  void log_statistics(const pinocchio::Motion& target_v) const;

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
