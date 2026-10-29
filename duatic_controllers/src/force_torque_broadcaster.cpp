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

#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include <duatic_controllers/force_torque_broadcaster.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

namespace duatic::controllers
{

void apply_wrench_offset(const geometry_msgs::msg::WrenchStamped& msg, const geometry_msgs::msg::WrenchStamped& offset,
                         geometry_msgs::msg::WrenchStamped& corrected_msg)
{
  // Copy assignment reuses the frame_id capacity of corrected_msg
  corrected_msg = msg;

  // Subtract the offset from the force components
  corrected_msg.wrench.force.x = msg.wrench.force.x - offset.wrench.force.x;
  corrected_msg.wrench.force.y = msg.wrench.force.y - offset.wrench.force.y;
  corrected_msg.wrench.force.z = msg.wrench.force.z - offset.wrench.force.z;

  // Subtract the offset from the torque components
  corrected_msg.wrench.torque.x = msg.wrench.torque.x - offset.wrench.torque.x;
  corrected_msg.wrench.torque.y = msg.wrench.torque.y - offset.wrench.torque.y;
  corrected_msg.wrench.torque.z = msg.wrench.torque.z - offset.wrench.torque.z;
}

WrenchSolver::WrenchSolver(const Eigen::Index nv)
  : J_reg_(nv, 6), svd_(nv, 6, Eigen::ComputeThinU | Eigen::ComputeThinV)
{
}

bool WrenchSolver::solve(const Jacobian& J, const Eigen::VectorXd& tau_ext, const double lambda,
                         const bool use_svd_solver, Wrench& wrench)
{
  // Eigen 3.4 SVD solves read an uninitialised rank on non-finite input and can segfault
  if (!J.allFinite() || !tau_ext.allFinite()) {
    return false;
  }

  // Solve for wrench: F = (J^T)^+ * tau_ext
  if (use_svd_solver) {
    J_reg_ = J.transpose();
    J_reg_.diagonal().array() += lambda;
    svd_.compute(J_reg_);
    // Same result as svd_.solve(tau_ext), which allocates a temporary
    const Eigen::Index rank = svd_.rank();
    Wrench scaled = Wrench::Zero();
    scaled.head(rank).noalias() = svd_.matrixU().leftCols(rank).transpose() * tau_ext;
    scaled.head(rank).array() /= svd_.singularValues().head(rank).array();
    wrench.noalias() = svd_.matrixV().leftCols(rank) * scaled.head(rank);
    return true;
  }
  const Eigen::Matrix<double, 6, 6> A = J * J.transpose() + lambda * Eigen::Matrix<double, 6, 6>::Identity();
  wrench = A.ldlt().solve(J * tau_ext);
  return true;
}

ForceTorqueBroadcaster::ForceTorqueBroadcaster() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration ForceTorqueBroadcaster::command_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::InterfaceConfiguration ForceTorqueBroadcaster::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    config.names.emplace_back(joint + "/" + "acceleration_commanded");
  }

  return config;
}

controller_interface::CallbackReturn ForceTorqueBroadcaster::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<force_torque_broadcaster::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // check if joints are empty
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Building Pinocchio model from XML");
  pinocchio::Model new_model;  // rebuilding into the old model causes data-model inconsistency
  pinocchio::urdf::buildModelFromXML(get_robot_description(), new_model);
  pinocchio_model_ = std::move(new_model);
  pinocchio_data_ = std::move(pinocchio_model_.createData());
  if (!pinocchio_model_.check(pinocchio_data_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio data check failed, 'pinocchio_data_' is not consistent with "
                                           "'pinocchio_model_'");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto& joint_name : params_.joints) {
    if (!pinocchio_model_.existJointName(joint_name)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  frame_index_ = pinocchio_model_.getFrameId(params_.endeffector_frame);

  // Build joint index cache
  joint_indices_.clear();
  for (const auto& joint : params_.joints) {
    joint_indices_.push_back(pinocchio_model_.getJointId(joint));
  }

  // Entries of joints not in params_.joints stay zero, and so do the J_ columns getFrameJacobian never writes
  q_ = Eigen::VectorXd::Zero(pinocchio_model_.nq);
  v_ = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  a_ = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  torque_meas_ = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  tau_ext_ = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  J_ = WrenchSolver::Jacobian::Zero(6, pinocchio_model_.nv);
  wrench_solver_ = WrenchSolver(pinocchio_model_.nv);

  torques_msg_.joints = params_.joints;
  torques_msg_.torques.assign(params_.joints.size(), 0.0);

  // The wrench publisher
  wrench_pub_ =
      get_node()->create_publisher<WrenchStamped>("~/wrench", 10);  // TODO(firesurfer) what is the right qos ?
  wrench_pub_rt_ = std::make_unique<WrenchStampedPublisher>(wrench_pub_);

  external_torques_pub_ = get_node()->create_publisher<MeasuredTorques>("~/torques", 10);
  external_torques_pub__rt_ = std::make_unique<MeasuredTorquesPublisher>(external_torques_pub_);

  zero_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/zero",
      [&]([[maybe_unused]] std_srvs::srv::Trigger::Request::SharedPtr req,
          [[maybe_unused]] std_srvs::srv::Trigger::Response::SharedPtr res) { wrench_offset_ = wrench_raw_msg_; });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // clear out vectors in case of restart
  joint_effort_state_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  joint_acceleration_state_interfaces_.clear();

  // get the actual interface in an ordered way (same order as the joints parameter)
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_POSITION, joint_position_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, hardware_interface::HW_IF_VELOCITY, joint_velocity_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - velocity");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, "acceleration_commanded",
                                                    joint_acceleration_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - acceleration");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT,
                                                    joint_effort_state_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered state interfaces - effort");
    return controller_interface::CallbackReturn::FAILURE;
  }

  active_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueBroadcaster::update([[maybe_unused]] const rclcpp::Time& time,
                                                                 [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE || !active_) {
    return controller_interface::return_type::OK;
  }

  const std::size_t joint_count = joint_position_state_interfaces_.size();

  // Map: Pinocchio joint name -> index in q/v
  for (std::size_t i = 0; i < joint_count; i++) {
    const std::string& joint_name = params_.joints[i];
    const auto idx = joint_indices_[i];
    if (idx == 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
      return controller_interface::return_type::ERROR;
    }
    // Pinocchio joint index starts at 1, q/v index is idx-1
    q_[pinocchio_model_.joints[idx].idx_q()] =
        duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());
    v_[pinocchio_model_.joints[idx].idx_v()] =
        duatic::controllers::compat::require_value(joint_velocity_state_interfaces_.at(i).get());
    a_[pinocchio_model_.joints[idx].idx_v()] =
        duatic::controllers::compat::require_value(joint_acceleration_state_interfaces_.at(i).get());
    torque_meas_[pinocchio_model_.joints[idx].idx_v()] =
        duatic::controllers::compat::require_value(joint_effort_state_interfaces_.at(i).get());
  }

  forwardKinematics(pinocchio_model_, pinocchio_data_, q_, v_, a_);
  const auto& tau = pinocchio::rnea(pinocchio_model_, pinocchio_data_, q_, v_, a_);
  // These are the actual external torques
  // Note: inverted because of sign convention
  tau_ext_ = tau - torque_meas_;

  pinocchio::computeJointJacobians(pinocchio_model_, pinocchio_data_, q_);
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);
  pinocchio::getFrameJacobian(pinocchio_model_, pinocchio_data_, frame_index_, pinocchio::LOCAL, J_);

  if (!wrench_solver_.solve(J_, tau_ext_, params_.lambda_regularization, params_.use_svd_solver, wrench_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Non-finite Jacobian or joint torques, skipping wrench estimation for this cycle");
    return controller_interface::return_type::OK;
  }

  // Publish the wrench as wrench stamped

  wrench_raw_msg_.header.stamp = time;
  wrench_raw_msg_.header.frame_id = params_.endeffector_frame;

  // Force
  wrench_raw_msg_.wrench.force.x = wrench_(0);
  wrench_raw_msg_.wrench.force.y = wrench_(1);
  wrench_raw_msg_.wrench.force.z = wrench_(2);

  // Torque
  wrench_raw_msg_.wrench.torque.x = wrench_(3);
  wrench_raw_msg_.wrench.torque.y = wrench_(4);
  wrench_raw_msg_.wrench.torque.z = wrench_(5);

  apply_wrench_offset(wrench_raw_msg_, wrench_offset_, wrench_msg_);

  // and we try to have our realtime publisher publish the message
  // if this doesn't succeed - well it will probably next time
  duatic::controllers::compat::publish_rt(wrench_pub_rt_, wrench_msg_);

  torques_msg_.header = wrench_msg_.header;
  // Publish the calculated external torques per joint
  for (std::size_t i = 0; i < joint_count; i++) {
    torques_msg_.torques[i] = tau_ext_[pinocchio_model_.joints[joint_indices_[i]].idx_v()];
  }

  duatic::controllers::compat::publish_rt(external_torques_pub__rt_, torques_msg_);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ForceTorqueBroadcaster::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace duatic::controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::ForceTorqueBroadcaster, controller_interface::ControllerInterface)
