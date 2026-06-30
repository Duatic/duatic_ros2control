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

#include <duatic_controllers/cartesian_impedance_controller.hpp>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <pinocchio/parsers/urdf.hpp>

namespace duatic::controllers
{
CartesianImpedanceController::CartesianImpedanceController() = default;

controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const std::string &joint : params_.joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const std::string &joint : params_.joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const std::string &joint : params_.joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    param_listener_ = std::make_unique<cartesian_impedance_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();

  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianImpedanceController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update parameters
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // check if joints are empty
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // create pinocchio model
  pinocchio::urdf::buildModelFromXML(get_robot_description(), robot_model_);

  // Build joint index caches
  joint_model_idx_.clear();
  joint_model_idx_.reserve(params_.joints.size());
  joint_q_idx_.clear();
  joint_q_idx_.reserve(params_.joints.size());
  joint_v_idx_.clear();
  joint_v_idx_.reserve(params_.joints.size());
  for (const std::string &joint : params_.joints) {
    if (!robot_model_.existJointName(joint)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in Pinocchio model.", joint.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      const pinocchio::JointIndex jidx = robot_model_.getJointId(joint);
      joint_model_idx_.push_back(jidx);
      joint_q_idx_.push_back(robot_model_.idx_qs[jidx]);
      joint_v_idx_.push_back(robot_model_.idx_vs[jidx]);
    }
  }
  assert(joint_model_idx_.size() == params_.joints.size());
  assert(joint_q_idx_.size() == params_.joints.size());
  assert(joint_v_idx_.size() == params_.joints.size());

  // Store reference frame index
  const std::string &reference_frame = params_.reference_frame;
  if (!robot_model_.existJointName(reference_frame)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Reference frame '%s' not found in Pinocchio model.", reference_frame.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  } else {
    reference_frame_idx_ = robot_model_.getFrameId(reference_frame);
  }

  // size internal state variables
  state_data_ = pinocchio::Data(robot_model_);
  state_q_ = Eigen::VectorXd::Zero(robot_model_.nq);
  state_v_ = Eigen::VectorXd::Zero(robot_model_.nv);
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianImpedanceController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating CartesianImpedanceController on frame '%s' using joints [%s].",
    params_.reference_frame.c_str(),
    fmt::format("{}", fmt::join(params_.joints, ", ")).c_str()
  );

  // initialize robot state variables
  update_state();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianImpedanceController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Deactivating CartesianImpedanceController on frame '" << params_.reference_frame << "'");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                       [[maybe_unused]] const rclcpp::Duration& period)
{
  
  
  return controller_interface::return_type::OK;
}

void CartesianImpedanceController::update_state(const rclcpp::Duration &period) {
  auto interface_iter = state_interfaces_.begin();
  for (const auto idx : joint_q_idx_) {
    state_q_[idx] = interface_iter->get_optional().value_or(state_q_[idx]);
    interface_iter++;
  }
  for (const auto idx : joint_v_idx_) {
    state_v_[idx] = interface_iter->get_optional().value_or(state_v_[idx]);
    interface_iter++;
  }
  assert(interface_iter == state_interfaces_.end());

  //TODO: insert hack for manual velocity calculation

  // run forward kinematics and update frames
  pinocchio::forwardKinematics(robot_model_, state_data_, state_q_, state_v_);
  pinocchio::updateFramePlacements(robot_model_, state_data_);
}

}  // namespace duatic::controllers

PLUGINLIB_EXPORT_CLASS(duatic::controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
