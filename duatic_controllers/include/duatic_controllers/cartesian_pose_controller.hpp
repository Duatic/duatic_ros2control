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
#include <unordered_map>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

// Pinocchio
#include <Eigen/Dense>
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

// Project
#include <duatic_controllers/cartesian_pose_controller_parameters.hpp>
#include <duatic_controllers/interface_utils.hpp>

namespace duatic::controllers
{
class CartesianPoseController : public controller_interface::ControllerInterface
{
public:
  struct PinocchioState
  {
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
  };
  struct IKResult
  {
    Eigen::VectorXd q_out;
  };
  CartesianPoseController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
private:
  // Access to controller parameters via generate_parameter_library
  std::unique_ptr<cartesian_pose_controller::ParamListener> param_listener_;
  cartesian_pose_controller::Params params_;

  pinocchio::Model pinocchio_model_;
  pinocchio::Data pinocchio_data_;
  pinocchio::GeometryModel pinocchio_geom_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interfaces_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_acceleration_state_interfaces_;
  std::atomic_bool active_{ false };

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> pose_cmd_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> buffer_pose_cmd_;
  std::vector<pinocchio::JointIndex> joint_indices_;
  pinocchio::FrameIndex endeffector_frame_id_;
  pinocchio::FrameIndex base_frame_id_;

  std::optional<PinocchioState> last_system_state_;
  std::optional<PinocchioState> next_target_state_;

  std::optional<PinocchioState> build_current_state();
  std::optional<IKResult> run_ik(const geometry_msgs::msg::PoseStamped& msg);
  std::optional<pinocchio::SE3> get_current_ee_pose();
  std::optional<IKResult> compute_ik(const pinocchio::Model& model, pinocchio::Data& data,
                                     const pinocchio::SE3& target_pose,
                                     const pinocchio::FrameIndex target_pose_frame_id, const Eigen::VectorXd& q_in,
                                     rclcpp::Logger logger);
};

inline std::ostream& operator<<(std::ostream& os, const CartesianPoseController::PinocchioState& s)
{
  os << "{\n";
  os << "  q: " << s.q.transpose() << "\n";
  os << "  v: " << s.v.transpose() << "\n";
  os << "  a: " << s.a.transpose() << "\n";
  os << "}";
  return os;
}
}  // namespace duatic::controllers
