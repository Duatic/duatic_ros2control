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
#include <memory>
#include <algorithm>

// Pinocchio
#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"

// Other headers
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "controller_interface/chainable_controller_interface.hpp"

// ROS2
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// Project
#include <duatic_controller_msgs/msg/pose_twist_stamped.hpp>
#include <duatic_controllers/cartesian_pose_controller_parameters.hpp>
#include <duatic_controllers/interface_utils.hpp>
#include <duatic_geometry/trajectory.hpp>

namespace duatic::controllers
{

class CartesianPoseController : public controller_interface::ControllerInterface
{
private:
  static constexpr uint8_t c_buff_update = (1 << 7);      // MSB
  static constexpr uint8_t c_buff_MASK = !c_buff_update;  // all non-msb bits

public:
  inline CartesianPoseController()
    : controller_interface::ControllerInterface()
    // INVARIANT: ENSURE THOSE THREE INDIZES ALWAYS HOLD THESE EXACT VALUES IN ARB. PERMUTATION (MSB used for
    // Update-indication)
    , rt_idx_(0)
    , buff_idx_(1)
    , sub_idx_(2)
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

  // input topic subscriptions
  std::shared_ptr<rclcpp::Subscription<duatic_controller_msgs::msg::PoseTwistStamped>> target_pose_twist_sub_;

  // the official realtime buffer is not meant to be used in the direction from rt to non-rt, thus implement a CPU-level
  // lock-free one-way buffer in the following:
  // -> ensure cache-line aligned data for the exchange between rt-update and non-rt subscription
  alignas(std::hardware_destructive_interference_size)
      std::array<geometry::TrajectoryUpdateInformation, 3> trajectory_update_buffer_;
  alignas(std::hardware_destructive_interference_size) uint8_t rt_idx_;
  alignas(std::hardware_destructive_interference_size) std::atomic_uint8_t buff_idx_;
  alignas(std::hardware_destructive_interference_size) uint8_t sub_idx_;

  static_assert(std::atomic_uint8_t::is_always_lock_free, "This hardware is not suitable for lock-free atomic uint8_t "
                                                          "operations. Cannot compile this code!");

  realtime_tools::RealtimeBuffer<geometry::ConstantTargetPoseTwist>
      trajectory_buffer_;  // use the available impl. because it's there not because it's better ... and because the
                           // other buffer is not yet separated into a dedicated class

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

  void read_rt_state_buffer();

  void publish_topics(const rclcpp::Time& now);

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

  /*
    std::optional<PinocchioState> build_current_state();
    std::optional<IKResult> run_ik(const geometry_msgs::msg::PoseStamped& msg);
    std::optional<pinocchio::SE3> get_current_ee_pose();
    std::optional<IKResult> compute_ik(const pinocchio::Model& model, pinocchio::Data& data,
                                       const pinocchio::SE3& target_pose,
                                       const pinocchio::FrameIndex target_pose_frame_id, const Eigen::VectorXd& q_in,
                                       rclcpp::Logger logger);
  */
  /**
   * @brief stage a new target which is eventually commanded and store its time of staging for
   */
  /*  void stage_new_target(const IKResult& new_target)
    {
      if (!last_system_state_) {
        RCLCPP_FATAL_STREAM(get_node()->get_logger(), "Cannot stage new target as there is currently no system state "
                                                      "available");
        return;
      }

      staged_target_ = StagingState{ .time_of_staging = get_node()->now(),
                                     .system_state_of_staging = last_system_state_.value(),
                                     .target = new_target };
    }
  */
};

}  // namespace duatic::controllers
