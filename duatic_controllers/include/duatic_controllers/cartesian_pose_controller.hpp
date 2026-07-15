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
#include <geometry_msgs/msg/twist_stamped.hpp>

// Project
#include <duatic_concurrency/unidirectional_buffer.hpp>
#include <duatic_controller_msgs/msg/pose_twist_stamped.hpp>
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
  inline CartesianPoseController() : controller_interface::ControllerInterface()
  {
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

private:
  std::unique_ptr<cartesian_pose_controller::ParamListener> param_listener_;
  cartesian_pose_controller::Params params_;

  pinocchio::Model robot_model_;
  std::vector<pinocchio::JointIndex> joint_model_idx_;
  std::vector<Eigen::Index> joint_q_idx_;
  std::vector<Eigen::Index> joint_v_idx_;
  pinocchio::FrameIndex end_effector_frame_idx_;

  pinocchio::Data state_data_;
  Eigen::VectorXd state_q_;
  Eigen::VectorXd state_v_;
  pinocchio::Motion state_end_effector_twist_;

  Eigen::VectorXd control_q_;
  Eigen::VectorXd control_v_;

  // input topic subscriptions
  std::shared_ptr<rclcpp::Subscription<duatic_controller_msgs::msg::PoseTwistStamped>> target_pose_twist_sub_;

  // lock-free RT Non-RT data exchange buffer
  duatic::concurrency::UnidirectionalBuffer<geometry::StampedState3Dd> current_state_buffer_;
  duatic::concurrency::UnidirectionalBuffer<geometry::ConstantTargetPoseTwist> trajectory_buffer_;

  geometry::State3Dd target_state_;
  geometry::Twist3Dd target_pose_diff_;

  /* Quadratic programming solver
   *   min x=[theta,error] 1/2 x^T H_diag x
   *   subject to: J theta + error = pose_diff    <=>  [J 1] x = pose_diff
   */
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_solver_;  // initialized with dimensions in the contrustor
  Eigen::MatrixXd qp_solver_H_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> qp_solver_A_;

  // state publication topics
  double topics_pub_period_;
  double topics_pub_next_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>::UniquePtr end_effector_pose_pub_realtime_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr end_effector_twist_pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>::UniquePtr end_effector_twist_pub_realtime_;

  void handle_target_pose_twist_sub(const duatic_controller_msgs::msg::PoseTwistStamped::SharedPtr msg);

  void update_state();

  void update_rt_state_buffer(const rclcpp::Time& now);

  void computeStateJointMotion(const rclcpp::Duration& period, const bool verbose = false);

  void command_controls();

  void publish_statistics();

  void publish_topics();

  inline const pinocchio::SE3& end_effector_pose()
  {  // for convenience
    return state_data_.oMf[end_effector_frame_idx_];
  }
  inline const pinocchio::Motion& end_effector_twist()
  {  // for convenience
    return state_end_effector_twist_;
  }

  ////////////////////////////////////////////////////////
  /// OLD UNDERNEATH
  ////////////////////////////////////////////////////////
  pinocchio::GeometryModel pinocchio_geom_;
};

}  // namespace duatic::controllers
