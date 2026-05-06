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

#include <duatic_controllers/brake_release_controller.hpp>

// C++ system headers
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

#include <pinocchio/collision/collision.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/check-data.hpp>

// Other headers
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace duatic::controllers
{

BrakeReleaseController::BrakeReleaseController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration BrakeReleaseController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration BrakeReleaseController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const auto joints = params_.joints;
  for (auto& joint : joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.emplace_back(joint + "/" + "acceleration_commanded");
  }

  return config;
}

controller_interface::CallbackReturn BrakeReleaseController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<brake_release_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Start IK worker thread
  // IK worker will be started in on_activate after Pinocchio model is built

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
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

  try {
    // 1. build the pinocchio model from the urdf
    RCLCPP_INFO(get_node()->get_logger(), "Building Pinocchio model from URDF...");
    {
      pinocchio::Model full_model;
      pinocchio::urdf::buildModelFromXML(get_robot_description(), full_model);

      // So pinocchio is very counter intuitive. We specify joints that are marked as FIXED in the reduced model
      // We we built an inversed list
      // This is needed so that the controller will only do the IK for the specified joint list
      std::unordered_set<pinocchio::JointIndex> keep;

      for (const auto& joint : params_.joints) {
        pinocchio::JointIndex id = full_model.getJointId(joint);
        keep.insert(id);

        RCLCPP_INFO_STREAM(get_node()->get_logger(), "keeping joint: " << joint << " id=" << id);
      }

      std::vector<pinocchio::JointIndex> indices;

      for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full_model.njoints); ++j) {
        if (keep.find(j) == keep.end()) {
          indices.push_back(j);
          RCLCPP_INFO_STREAM(get_node()->get_logger(), "locking joint: " << full_model.names[j] << " id=" << j);
        }
      }
      Eigen::VectorXd q0 = pinocchio::neutral(full_model);
      pinocchio::buildReducedModel(full_model, indices, q0, pinocchio_model_);
      RCLCPP_INFO_STREAM(get_node()->get_logger(), pinocchio_model_.njoints << " " << pinocchio_model_.nv);
      for (pinocchio::JointIndex j = 0; j < static_cast<pinocchio::JointIndex>(full_model.njoints); ++j) {
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Joint " << j << ": " << pinocchio_model_.names[j]);
      }
      pinocchio_data_ = pinocchio::Data(pinocchio_model_);
      RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model built with %zu joints",
                  pinocchio_model_.joints.size() - 1);
    }

    // 2. Validate that all controller joints exist in the Pinocchio model
    RCLCPP_INFO(get_node()->get_logger(), "Validating controller joints...");
    std::vector<pinocchio::JointIndex> controller_joint_indices;
    for (const auto& joint_name : params_.joints) {
      if (!pinocchio_model_.existJointName(joint_name)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      auto joint_id = pinocchio_model_.getJointId(joint_name);
      controller_joint_indices.push_back(joint_id);
      RCLCPP_INFO(get_node()->get_logger(), "Joint '%s' found with index %ld", joint_name.c_str(), joint_id);
    }

    // 3. Validate kinematic chain structure (only if more than one joint)
    if (params_.joints.size() > 1) {
      RCLCPP_INFO(get_node()->get_logger(), "Validating kinematic chain structure...");
      for (size_t i = 1; i < controller_joint_indices.size(); ++i) {
        auto current_joint_id = controller_joint_indices[i];
        auto previous_joint_id = controller_joint_indices[i - 1];

        // Check if current joint is a descendant of the previous joint in the kinematic tree
        bool is_valid_chain = false;
        auto parent_id = pinocchio_model_.parents[current_joint_id];

        // Traverse up the kinematic tree to see if we find the previous joint
        while (parent_id != 0) {
          if (parent_id == previous_joint_id) {
            is_valid_chain = true;
            break;
          }
          parent_id = pinocchio_model_.parents[parent_id];
        }

        RCLCPP_INFO(get_node()->get_logger(), "Chain validation for joint '%s' (id %ld) -> '%s' (id %ld): %s",
                    params_.joints[i].c_str(), current_joint_id, params_.joints[i - 1].c_str(), previous_joint_id,
                    is_valid_chain ? "VALID" : "INVALID");

        if (!is_valid_chain) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Invalid kinematic chain: Joint '%s' (index %zu) is not a descendant of joint '%s' (index %zu) "
                       "in the kinematic tree.",
                       params_.joints[i].c_str(), i, params_.joints[i - 1].c_str(), i - 1);
          RCLCPP_ERROR(get_node()->get_logger(), "Please check that the 'joints' parameter lists the joints in the "
                                                 "correct kinematic order.");
          return controller_interface::CallbackReturn::ERROR;
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during Pinocchio model setup: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // clear out vectors in case of restart (reactivation to be precise)
  joint_position_command_interfaces_.clear();

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

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints,
                                                    hardware_interface::HW_IF_POSITION,
                                                    joint_position_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }

  const std::size_t joint_count = joint_position_state_interfaces_.size();

  // Build full-size vectors for all robot joints (Pinocchio expects this)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);

  for (std::size_t i = 0; i < joint_count; i++) {
    const std::string& joint_name = params_.joints[i];
    const auto idx = pinocchio_model_.getJointId(joint_name);

    try {
      q[pinocchio_model_.joints[idx].idx_q()] =
          duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());

      v[pinocchio_model_.joints[idx].idx_v()] =
          duatic::controllers::compat::require_value(joint_velocity_state_interfaces_.at(i).get());

      a[pinocchio_model_.joints[idx].idx_v()] =
          duatic::controllers::compat::require_value(joint_acceleration_state_interfaces_.at(i).get());
    } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read state for joint '%s': %s", joint_name.c_str(), e.what());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Current configuration: " << q.transpose());

  // Calculate the necessary torques to hold the arm at is current location
  Eigen::VectorXd tau = pinocchio::computeGeneralizedGravity(pinocchio_model_, pinocchio_data_, q);

  // Determine the direction we need to move into in order to unload the brake pins
  Eigen::VectorXd direction = Eigen::VectorXd::Zero(tau.size());

  for (int i = 0; i < tau.size(); ++i) {
    if (tau[i] > 1e-6)
      direction[i] = 1.0;
    else if (tau[i] < -1e-6)
      direction[i] = -1.0;
  }
  // Define the new targets
  Eigen::VectorXd q_target = q + direction * params_.position_kick;

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Command new target positions: " << q_target.transpose());

  // And command it
  for (std::size_t i = 0; i < params_.joints.size(); i++) {
    const std::string& joint_name = params_.joints[i];
    const auto idx = pinocchio_model_.getJointId(joint_name);
    if (!joint_position_command_interfaces_.at(i).get().set_value<double>(
            q_target[pinocchio_model_.joints[idx].idx_q()])) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command for joint '%s'", joint_name.c_str());
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BrakeReleaseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                 [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  return controller_interface::return_type::OK;
}
controller_interface::CallbackReturn
BrakeReleaseController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
BrakeReleaseController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace duatic::controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::BrakeReleaseController, controller_interface::ControllerInterface)
