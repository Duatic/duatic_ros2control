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

#include <duatic_controllers/gravity_compensation_controller.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/introspection.hpp>
#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

namespace duatic::controllers
{

GravityCompensationController::GravityCompensationController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration GravityCompensationController::command_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  if (params_.dry_run) {
    config.type = controller_interface::interface_configuration_type::NONE;
  } else {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const auto joints = params_.joints;
    for (auto& joint : joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration GravityCompensationController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<gravity_compensation_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update parameters
  param_listener_->refresh_dynamic_parameters();
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

  // (Re)Build full-size vectors for all robot joints (Pinocchio expects this)
  q_ = Eigen::VectorXd::Zero(pinocchio_model_.nq);
  v_ = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  tau_gravity_ = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  tau_coriolis_ = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  joint_effort_.clear();
  joint_effort_.resize(params_.joints.size(), 0.0);

  // Build pinocchio-joint index caches
  joint_q_idx_.resize(params_.joints.size());
  joint_v_idx_.resize(params_.joints.size());
  for (std::size_t i = 0; i < params_.joints.size(); i++) {
    const std::string& joint_name = params_.joints[i];
    if (!pinocchio_model_.existJointName(joint_name)) {  // evaluate model
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    auto& joint = pinocchio_model_.joints[pinocchio_model_.getJointId(joint_name)];
    joint_q_idx_[i] = joint.idx_q();
    joint_v_idx_[i] = joint.idx_v();
  }

  // The status publisher
  status_pub_ = get_node()->create_publisher<StatusMsg>("~/state", 10);  // TODO(firesurfer) what is the right qos ?
  status_pub_rt_ = std::make_unique<StatusMsgPublisher>(status_pub_);

  // ros2control introspection
  if (params_.enable_introspection) {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring ROS2control Introspection for internal state monitoring.");
    for (std::size_t i = 0; i < params_.joints.size(); i++) {
      REGISTER_ROS2_CONTROL_INTROSPECTION("tau_gravity_" + std::to_string(i), &tau_gravity_[joint_v_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("tau_coriolis_" + std::to_string(i), &tau_coriolis_[joint_v_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("joint_effort_" + std::to_string(i), &joint_effort_[i]);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  if (params_.enable_introspection) {
    RCLCPP_INFO(get_node()->get_logger(), "Unconfiguring ROS2control Introspection.");
    for (std::size_t i = 0; i < params_.joints.size(); i++) {
      using hardware_interface::DEFAULT_REGISTRY_KEY;  // BUGFIX IN ROS2CONTROL !  The actual macro is missing to
                                                       // include this namespace natively as done in REGISTER
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("tau_gravity_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("tau_coriolis_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("joint_effort_" + std::to_string(i));
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // clear out vectors in case of restart
  joint_effort_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();
  joint_velocity_state_interfaces_.clear();
  initial_joint_positions_.clear();

  // Do allocation one but clear it in the activate function to to be sure
  state_msg_.joints.clear();
  for (std::size_t i = 0; i < params_.joints.size(); i++) {
    const std::string& joint_name = params_.joints[i];
    state_msg_.joints.push_back(joint_name);
    state_msg_.commanded_torque.push_back(0.0);
  }

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

  if (!params_.dry_run) {
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT, joint_effort_command_interfaces_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - effort");
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  // Obtain the joint positions during startup which we need for the startup jump check
  for (std::size_t i = 0; i < joint_position_state_interfaces_.size(); i++) {
    try {
      initial_joint_positions_.push_back(
          duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get()));
    } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read initial joint position for joint '%s': %s",
                   params_.joints[i].c_str(), e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCompensationController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  activation_time_set_ = false;
  for (auto effort_if : joint_effort_command_interfaces_) {
    (void)effort_if.get().set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                        [[maybe_unused]] const rclcpp::Duration& period)
{
  // Set activation_time_ only once, using the same time source as 'time'
  if (!activation_time_set_) {
    activation_time_ = time;
    activation_time_set_ = true;
  }

  const std::size_t joint_count = joint_position_state_interfaces_.size();
  assert(joint_count == joint_velocity_state_interfaces_.size());
  // Map: Pinocchio joint name -> index in q/v
  try {
    for (std::size_t i = 0; i < joint_count; i++) {
      q_[joint_q_idx_[i]] = duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());
      v_[joint_v_idx_[i]] = duatic::controllers::compat::require_value(joint_velocity_state_interfaces_.at(i).get());
    }
  } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint state values: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  // Perform startup jump check if enabled
  // A jump might happen if the configured urdf does not match the hardware
  // So for the first 0.5s after activation we check if there was a jump of more than (default 0.5) x rad
  if (params_.enable_startup_check && (time - activation_time_ < rclcpp::Duration(std::chrono::milliseconds(500)))) {
    bool has_jump = false;
    try {
      for (std::size_t i = 0; i < joint_count; i++) {
        const double pos_now = duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());

        if (std::abs(pos_now - initial_joint_positions_.at(i)) > params_.max_jump_startup) {
          has_jump = true;
        }
      }
    } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Startup check failed: no position value available: %s", e.what());
      return controller_interface::return_type::ERROR;
    }

    if (has_jump) {
      RCLCPP_ERROR(get_node()->get_logger(), "Detected jump directly after startup- this is an error");
      return controller_interface::return_type::ERROR;
    }
  }

  // Gravity Compensation
  tau_gravity_ = pinocchio::computeGeneralizedGravity(pinocchio_model_, pinocchio_data_, q_);

  // Coriolis Compensation
  if (params_.enable_coriolis_compensation) {  // else: stays zero as initialized in on_configure
    tau_coriolis_ = pinocchio::computeCoriolisMatrix(pinocchio_model_, pinocchio_data_, q_, v_) * v_;
  }

  // combine into joint_effort command
  for (std::size_t i = 0; i < joint_count; i++) {
    joint_effort_[i] = tau_gravity_[joint_v_idx_[i]] + tau_coriolis_[joint_v_idx_[i]];
  }

  // command to hw
  if (!params_.dry_run) {
    for (std::size_t i = 0; i < joint_count; i++) {
      if (!joint_effort_command_interfaces_.at(i).get().set_value(joint_effort_[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set new effort value for joint interface at index %zu.", i);
        return controller_interface::return_type::ERROR;
      }
    }
  }

  // publish state topic
  if (params_.enable_state_topic) {
    state_msg_.timestamp = time;
    for (std::size_t i = 0; i < joint_count; i++) {
      state_msg_.commanded_torque[i] = joint_effort_[i];
    }
    duatic::controllers::compat::publish_rt(status_pub_rt_, state_msg_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace duatic::controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::GravityCompensationController, controller_interface::ControllerInterface)
