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
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Project
#include <duatic_concurrency/unidirectional_buffer.hpp>
#include <duatic_controllers/cartesian_pose_controller_parameters.hpp>
#include <duatic_controllers/interface_utils.hpp>
#include <duatic_geometry/twist_3d.hpp>
#include <duatic_geometry/state_3d.hpp>
#include <duatic_geometry/trajectory.hpp>

namespace duatic::controllers
{

class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
  using trajectory_type = geometry::ConstantTargetPose;  // TODO(patrick): convert into a template parameter

  inline CartesianPoseController() : controller_interface::ControllerInterface()
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
  cartesian_pose_controller::Params params_;

  // optimization horizon and limits
  double motion_horizon_;
  double v_limit_lin_, v_limit_ang_;
  enum class VelocityLimitState
  {  // first bit -> angular limit, last bit -> linear limit
    None = 0b00,
    LinearSync = 0b01,
    AngularOnly = 0b10,
    Both = 0b11
  };
  using VelocityLimitStateT = std::underlying_type_t<VelocityLimitState>;
  static_assert(std::is_literal_type_v<VelocityLimitStateT>);
  inline VelocityLimitState velocity_limit_state() const
  {
    assert(v_limit_lin_ >= 0.0);
    assert(v_limit_ang_ >= 0.0);
    return static_cast<VelocityLimitState>((static_cast<VelocityLimitStateT>(v_limit_ang_ > 0.0) << 1) |
                                           static_cast<VelocityLimitStateT>(v_limit_lin_ > 0.0));
  }
  static_assert(static_cast<VelocityLimitStateT>(bool(false)) == 0b0);
  static_assert(static_cast<VelocityLimitStateT>(bool(true)) == 0b1);

  pinocchio::Model robot_model_;
  std::vector<pinocchio::JointIndex> joint_model_idx_;
  std::vector<Eigen::Index> joint_q_idx_;
  std::vector<Eigen::Index> joint_v_idx_;
  pinocchio::FrameIndex target_frame_idx_;
  std::vector<pinocchio::FrameIndex> limit_frame_indices_;

  pinocchio::Data state_data_;
  Eigen::VectorXd state_q_;
  Eigen::VectorXd state_v_;
  pinocchio::Motion state_target_twist_;

  Eigen::VectorXd control_q_;
  Eigen::VectorXd control_v_;

  // input topic subscriptions
  std::shared_ptr<rclcpp::Subscription<trajectory_type::UpdateInformation::msg>> target_msg_sub_;

  // lock-free RT Non-RT data exchange buffer
  duatic::concurrency::UnidirectionalBuffer<geometry::TimedState3Dd> current_state_buffer_;
  duatic::concurrency::UnidirectionalBuffer<trajectory_type> trajectory_buffer_;

  geometry::State3Dd target_state_;

  /* Quadratic programming solver
   *   min_x 1/2 x^T H x + x^T g
       s.t.  l_box <= x <= u_box
   *
   * H = [1 + w * J^T * J | 0 ]  +  W_L * [ C^T * C | C^T]
   *     [      0         | 0 ]           [    C    |  1 ]
   * g = [-J^T * pose_diff ]
   *     [      0          ]
   * with C being the constraints-projection matrix and W_L being the ik_limit_frames_weight
   * and the result x being structured as x = [theta; c] with c being the linearized constraints variables
   */
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_solver_;  // initialized with dimensions in the contrustor
  Eigen::MatrixXd qp_solver_H_;                                      // I + w * J^T * J (dense, recomputed every cycle)
  Eigen::VectorXd qp_solver_g_;                                      // -w * J^T * pose_diff (linear cost term)
  Eigen::Matrix<double, 6, Eigen::Dynamic> qp_jacobian_;             // end-effector Jacobian J
  Eigen::VectorXd qp_solver_l_box_;                                  // lower box bounds
  Eigen::VectorXd qp_solver_u_box_;                                  // upper box bounds
  Eigen::VectorXd pose_diff_ik_result_;                              // solution vector

  // state publication topics
  double topics_pub_period_;
  double topics_pub_next_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::UniquePtr target_pose_pub_realtime_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>::UniquePtr target_twist_pub_realtime_;

  void handle_target_msg_sub(const trajectory_type::UpdateInformation::msg::SharedPtr msg);

  void update_state();

  void update_rt_state_buffer(const rclcpp::Time& now);

  void run_pose_diff_ik(const double problem_scale, const geometry::Twist3Dd& scaled_target_diff,
                        const bool verbose = false);

  void command_controls();

  void log_statistics() const;

  void publish_topics();

  inline const pinocchio::SE3& target_pose() const
  {  // for convenience
    return state_data_.oMf[target_frame_idx_];
  }
  inline const pinocchio::Motion& target_twist() const
  {  // for convenience
    return state_target_twist_;
  }

  template <typename Vector>
  inline static double scale_limit(Vector&& v, const double limit)
  {
    assert(limit > 0.0);
    const double limit_sq = limit * limit;
    const double scale = std::sqrt(limit_sq / std::fmax(limit_sq, v.squaredNorm()));
    assert((0.0 <= scale) && (scale <= 1.0));
    v *= scale;
    return scale;
  }
};

}  // namespace duatic::controllers
