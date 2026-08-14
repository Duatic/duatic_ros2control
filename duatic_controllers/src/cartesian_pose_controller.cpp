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
#include <duatic_controllers/cartesian_pose_controller.hpp>

// C++ system headers
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <functional>
#include <numbers>

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

controller_interface::InterfaceConfiguration CartesianPoseController::command_interface_configuration() const
{
  // Claim the necessary command interfaces
  controller_interface::InterfaceConfiguration config;
  if (params_->dry_run) {
    config.type = controller_interface::interface_configuration_type::NONE;
  } else {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // ensure the exact same order as for the state interfaces! (Used in on_activate)
    for (const std::string& joint : params_->joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      RCLCPP_DEBUG(get_node()->get_logger(), "Require command interface %s", config.names.back().c_str());
    }
    if constexpr (trajectory_eval_order_depth >= geometry::KinematicOrder::Twist) {
      for (const std::string& joint : params_->joints) {
        config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        RCLCPP_DEBUG(get_node()->get_logger(), "Require command interface %s", config.names.back().c_str());
      }
    }
    static_assert(trajectory_eval_order_depth <= geometry::KinematicOrder::Twist,  // line break
                  "Control outputs of higher derivative than Twist are not yet supported");
  }
  return config;
}

controller_interface::InterfaceConfiguration CartesianPoseController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // ensure the exact same order as for the command interfaces! (Used in on_activate)
  for (const std::string& joint : params_->joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    RCLCPP_DEBUG(get_node()->get_logger(), "Require state interface %s", config.names.back().c_str());
  }
  if constexpr (state_order_depth >= geometry::KinematicOrder::Twist) {
    for (const std::string& joint : params_->joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
      RCLCPP_DEBUG(get_node()->get_logger(), "Require state interface %s", config.names.back().c_str());
    }
  }
  static_assert(state_order_depth <= geometry::KinematicOrder::Twist,  // line break
                "State inputs of higher derivative than Twist are not yet supported");
  return config;
}

controller_interface::CallbackReturn CartesianPoseController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<cartesian_pose_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    *params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // log some internal configurations
  RCLCPP_INFO(get_node()->get_logger(), "Using 'trajectory_start_order_depth' = %s",
              geometry::to_string(trajectory_start_order_depth).c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Using 'trajectory_eval_order_depth' = %s",
              geometry::to_string(trajectory_eval_order_depth).c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Using 'state_order_depth' = %s",
              geometry::to_string(state_order_depth).c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update parameters
  try {
    param_listener_->refresh_dynamic_parameters();
    *params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller configuration: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // create the pinocchio model from the urdf
  RCLCPP_INFO(get_node()->get_logger(), "Building Pinocchio model from XML");
  pinocchio::Model new_model;  // rebuilding into the old model causes data-model inconsistency
  pinocchio::urdf::buildModelFromXML(get_robot_description(), new_model);
  robot_model_ = std::move(new_model);
  state_data_ = robot_model_.createData();
  if (!robot_model_.check(state_data_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio data check failed, 'state_data_' is not consistent with "
                                           "'robot_model_'");
    return controller_interface::CallbackReturn::ERROR;
  }
  state_q_ = Eigen::VectorXd::Zero(robot_model_.nq);
  state_v_ = Eigen::VectorXd::Zero(robot_model_.nv);
  // control output variables
  control_data_ = robot_model_.createData();
  if (!robot_model_.check(state_data_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio data check failed, 'control_data_' is not consistent with "
                                           "'robot_model_'");
    return controller_interface::CallbackReturn::ERROR;
  }
  control_q_ = Eigen::VectorXd::Zero(robot_model_.nq);
  control_v_ = Eigen::VectorXd::Zero(robot_model_.nv);

  // evaluate joits given
  if (params_->joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty. Abort configuration.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // evaluate optimization horizon, respective limits, and error weights
  motion_horizon_ = params_->motion_horizon;
  assert(motion_horizon_ > 0.0);
  RCLCPP_INFO(get_node()->get_logger(), "Configuring optimizing with a motion horizon of %.2f seconds.",
              motion_horizon_);
  v_limit_lin_ = params_->limits.velocity.linear * motion_horizon_;  // displacement instead of velocity
  assert(v_limit_lin_ >= 0.0);
  RCLCPP_INFO(get_node()->get_logger(), "Linear displacement limit at %.2f m.", v_limit_lin_);
  v_limit_ang_ = params_->limits.velocity.angular * motion_horizon_;  // displacement instead of velocity;
  assert(v_limit_ang_ >= 0.0);
  RCLCPP_INFO(get_node()->get_logger(), "Angular displacement limit at %.2f rad.", v_limit_ang_);

  linear_error_weight_ = params_->ik_meter_to_revolution_error_correlation * (2.0 * std::numbers::pi);
  RCLCPP_INFO(get_node()->get_logger(), "Linear error weight: %.2f", linear_error_weight_);

  // build model joint caches
  joint_model_idx_.clear();
  joint_model_idx_.reserve(params_->joints.size());
  joint_q_idx_.clear();
  joint_q_idx_.reserve(params_->joints.size());
  joint_v_idx_.clear();
  joint_v_idx_.reserve(params_->joints.size());
  for (const std::string& joint : params_->joints) {
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
  assert(joint_model_idx_.size() == params_->joints.size());
  assert(joint_q_idx_.size() == params_->joints.size());
  assert(joint_v_idx_.size() == params_->joints.size());

  // Store base frame index (all published poses/twists are expressed relative to this frame)
  if (!robot_model_.existFrame(params_->base_frame)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Base frame '%s' not found in Pinocchio model. Abort configuration.",
                 params_->base_frame.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  } else {
    base_frame_idx_ = robot_model_.getFrameId(params_->base_frame);
  }

  // Store end effector frame index
  if (!robot_model_.existFrame(params_->target_frame)) {
    RCLCPP_ERROR(get_node()->get_logger(), "End effector frame '%s' not found in Pinocchio model. Abort configuration.",
                 params_->target_frame.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  } else {
    target_frame_idx_ = robot_model_.getFrameId(params_->target_frame);
  }

  // Store observed frame indices
  observed_frame_indices_.clear();
  if (!params_->limits.observed_frames.empty()) {
    observed_frame_indices_.reserve(params_->limits.observed_frames.size());
    for (const std::string& frame : params_->limits.observed_frames) {
      if (!robot_model_.existFrame(frame)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Observed frame '%s' not found in Pinocchio model.", frame.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }  // else
      observed_frame_indices_.push_back(robot_model_.getFrameId(frame));
    }
    if (params_->limits.velocity.observed_frames_linear.size() < observed_frame_indices_.size()) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "'limits.velocity.observed_frames_linear' (size %zu) is smaller than 'limits.observed_frames' "
                  "(size %zu). Extending the missing entries with the regular linear velocity limit (%.2f).",
                  params_->limits.velocity.observed_frames_linear.size(), observed_frame_indices_.size(),
                  params_->limits.velocity.linear);
      params_->limits.velocity.observed_frames_linear.resize(observed_frame_indices_.size(),
                                                             params_->limits.velocity.linear);
    }
  }

  // setup and initialize QP
  assert(robot_model_.nv == params_->joints.size() && "At this stage, it is assumed that ALL joints are to be "
                                                      "controlled! -> This is an open TODO");
  // TODO(patrick): always make a full model and use the joints parameters to define actively controlled joints

  // create QP solver
  RCLCPP_INFO(get_node()->get_logger(), "Setting up QP solver with %d result variables", robot_model_.nv);
  qp_solver_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
      robot_model_.nv, 0, 0,  // variables, eq-constraints != 0, in-eq-constraints != 0
      true,                   // box_constrained
      proxsuite::proxqp::HessianType::Dense);
  assert(qp_solver_->model.dim == robot_model_.nv);
  assert(params_->ik_max_iterations >= 3 && "'ik_max_iterations' must be at least 3. Please update the parameter "
                                            "bounds accordingly.");

  // settings
  qp_solver_->settings.eps_abs = params_->ik_precision;
  qp_solver_->settings.eps_rel = 0.1 * params_->ik_precision;
  qp_solver_->settings.max_iter = params_->ik_max_iterations;
  qp_solver_->settings.max_iter_in = params_->ik_max_iterations - 2;
  qp_solver_->settings.verbose = params_->enable_debug_log;
  qp_solver_->settings.compute_timings = (params_->enable_debug_log || params_->enable_introspection);
  qp_solver_->settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;  // ! IMPORTANT ! Don't reset this on
                                                                               // error, it will cause way more required
                                                                               // iterations, whyever ...
  // variables setup
  qp_solver_H_ =
      Eigen::MatrixXd::Identity(qp_solver_->model.dim, qp_solver_->model.dim);  // init save as identity matrix.
  qp_solver_g_ = Eigen::VectorXd::Zero(qp_solver_->model.dim);
  qp_jacobian_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, robot_model_.nv);
  qp_jacobian_t_ = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(robot_model_.nv, 6);
  // position (box) limits
  qp_solver_l_box_ = Eigen::VectorXd::Constant(qp_solver_->model.dim, -1e10);
  qp_solver_u_box_ = Eigen::VectorXd::Constant(qp_solver_->model.dim, 1e10);
  pose_diff_ik_result_ = Eigen::VectorXd::Zero(robot_model_.nv);

  // subscriptions
  const std::string topic_prefix =
      (params_->topic_prefix.empty() ? get_node()->get_name() : params_->topic_prefix) + "/";
  const std::string target_topic_prefix = topic_prefix + params_->target_frame + "/";

  // TARGET INPUT
  target_msg_sub_ = get_node()->create_subscription<trajectory_target_msg_type>(
      target_topic_prefix + params_->target_topic_suffix, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&CartesianPoseController::handle_target_msg_sub, this, std::placeholders::_1));

  // create RT topic publishers for the controller end effector pose and twist
  if (params_->topic_pub_frequency > 0.0) {
    topics_pub_period_ = 1.0 / params_->topic_pub_frequency;
    topics_pub_next_time_ = get_node()->now().seconds();  // force immediate publish on first update

    target_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_topic_prefix + "pose", rclcpp::QoS(1).durability_volatile());
    target_pose_pub_realtime_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(target_pose_pub_);

    target_twist_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        target_topic_prefix + "twist", rclcpp::QoS(1).durability_volatile());
    target_twist_pub_realtime_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(target_twist_pub_);

    // create RT topic publishers for each observed frame's pose and twist
    observed_frame_pose_pub_.clear();
    observed_frame_pose_pub_realtime_.clear();
    observed_frame_twist_pub_.clear();
    observed_frame_twist_pub_realtime_.clear();
    observed_frame_pose_pub_.reserve(observed_frame_indices_.size());
    observed_frame_pose_pub_realtime_.reserve(observed_frame_indices_.size());
    observed_frame_twist_pub_.reserve(observed_frame_indices_.size());
    observed_frame_twist_pub_realtime_.reserve(observed_frame_indices_.size());
    for (std::size_t i = 0; i < observed_frame_indices_.size(); i++) {
      const std::string observed_frame_topic_prefix =
          topic_prefix + "observed/" + params_->limits.observed_frames[i] + "/";

      observed_frame_pose_pub_.push_back(get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          observed_frame_topic_prefix + "pose", rclcpp::QoS(1).durability_volatile()));
      observed_frame_pose_pub_realtime_.push_back(
          std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
              observed_frame_pose_pub_.back()));

      observed_frame_twist_pub_.push_back(get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
          observed_frame_topic_prefix + "twist", rclcpp::QoS(1).durability_volatile()));
      observed_frame_twist_pub_realtime_.push_back(
          std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
              observed_frame_twist_pub_.back()));
    }
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Topic frequency is set to 0.0, not publishing anything");
    topics_pub_period_ = 0.0;
    topics_pub_next_time_ = std::numeric_limits<double>::max();
    target_pose_pub_ = nullptr;
    target_pose_pub_realtime_ = nullptr;
    target_twist_pub_ = nullptr;
    target_twist_pub_realtime_ = nullptr;
    observed_frame_pose_pub_.clear();
    observed_frame_pose_pub_realtime_.clear();
    observed_frame_twist_pub_.clear();
    observed_frame_twist_pub_realtime_.clear();
  }

  // ros2control introspection
  if (params_->enable_introspection) {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring ROS2control Introspection for internal state monitoring.");
    for (std::size_t i = 0; i < params_->joints.size(); i++) {
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_q_" + std::to_string(i), &state_q_[joint_q_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_v_" + std::to_string(i), &state_v_[joint_v_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_q_" + std::to_string(i), &control_q_[joint_q_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_v_" + std::to_string(i), &control_v_[joint_v_idx_[i]]);
    }
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_x",
                                        &trajectory_eval_target_.pose().linear()(0));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_y",
                                        &trajectory_eval_target_.pose().linear()(1));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_z",
                                        &trajectory_eval_target_.pose().linear()(2));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_rx",
                                        &trajectory_eval_target_.pose().angular().x());
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_ry",
                                        &trajectory_eval_target_.pose().angular().y());
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_rz",
                                        &trajectory_eval_target_.pose().angular().z());
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_rw",
                                        &trajectory_eval_target_.pose().angular().w());
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_x",
                                        &trajectory_eval_target_pose_diff_.vector()(0));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_y",
                                        &trajectory_eval_target_pose_diff_.vector()(1));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_z",
                                        &trajectory_eval_target_pose_diff_.vector()(2));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_rx",
                                        &trajectory_eval_target_pose_diff_.vector()(3));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_ry",
                                        &trajectory_eval_target_pose_diff_.vector()(4));
    REGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_rz",
                                        &trajectory_eval_target_pose_diff_.vector()(5));
    REGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_x", &solution_pose_diff_.vector()(0));
    REGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_y", &solution_pose_diff_.vector()(1));
    REGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_z", &solution_pose_diff_.vector()(2));
    REGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_rx", &solution_pose_diff_.vector()(3));
    REGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_ry", &solution_pose_diff_.vector()(4));
    REGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_rz", &solution_pose_diff_.vector()(5));
    for (Eigen::Index i = 0; i < qp_solver_->model.dim; i++) {
      REGISTER_ROS2_CONTROL_INTROSPECTION("QP_result_" + std::to_string(i), &(qp_solver_->results.x[i]));
      REGISTER_ROS2_CONTROL_INTROSPECTION("QP_bound_u_box_" + std::to_string(i), &(qp_solver_u_box_[i]));
      REGISTER_ROS2_CONTROL_INTROSPECTION("QP_bound_l_box_" + std::to_string(i), &(qp_solver_l_box_[i]));
    }
    REGISTER_ROS2_CONTROL_INTROSPECTION("QP_iterations_inner", &(qp_solver_->results.info.iter));
    REGISTER_ROS2_CONTROL_INTROSPECTION("QP_iterations_outer", &(qp_solver_->results.info.iter_ext));
    REGISTER_ROS2_CONTROL_INTROSPECTION("QP_time_setup", &(qp_solver_->results.info.setup_time));
    REGISTER_ROS2_CONTROL_INTROSPECTION("QP_time_solve", &(qp_solver_->results.info.solve_time));
    REGISTER_ROS2_CONTROL_INTROSPECTION("QP_time_run", &(qp_solver_->results.info.run_time));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  if (params_->enable_introspection) {
    RCLCPP_INFO(get_node()->get_logger(), "Unconfiguring ROS2control Introspection.");
    using namespace hardware_interface;  // BUGFIX IN ROS2CONTROL !  The actual macro is missing to include this
                                         // namespace natively as done in the REGISTER function
    for (std::size_t i = 0; i < params_->joints.size(); i++) {
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("state_q_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("state_v_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("control_q_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("control_v_" + std::to_string(i));
    }
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_x");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_y");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_z");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_rx");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_ry");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_rz");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_state_pose_rw");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_y");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_z");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_rx");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_ry");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("trajectory_eval_target_pose_diff_rz");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_x");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_y");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_z");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_rx");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_ry");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("solution_pose_diff_rz");
    for (Eigen::Index i = 0; i < qp_solver_->model.dim; i++) {
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_result_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_bound_u_box_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_bound_l_box_" + std::to_string(i));
    }
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_iterations_inner");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_iterations_outer");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_time_setup");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_time_solve");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("QP_time_run");
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  if (!params_->dry_run) {
    auto command_itr = command_interfaces_.begin();
    auto state_itr = state_interfaces_.begin();
    // initialize the command interfaces to the current states as far as they are available
    while ((command_itr != command_interfaces_.end()) && (state_itr != state_interfaces_.end())) {
      const auto state = state_itr->get_optional();
      if (!state) {
        RCLCPP_ERROR(get_node()->get_logger(), "State interface '%s' is not available. Abort activation.",
                     state_itr->get_name().c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      if (!command_itr->set_value(state.value())) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to initialize command '%s'. Abort activation.",
                    command_itr->get_name().c_str());
        return controller_interface::CallbackReturn::FAILURE;
      };
      ++command_itr;
      ++state_itr;
    }
    // initialize all other commands to zero
    while (command_itr != command_interfaces_.end()) {
      if (!command_itr->set_value(0.0)) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to initialize command '%s'. Abort activation.",
                    command_itr->get_name().c_str());
        return controller_interface::CallbackReturn::FAILURE;
      };
      ++command_itr;
    }
  }
  // initialize current state (data availability is guaranteed by the previous loop)
  update_state(true);

  // initialize state and trajectory buffers, ensure the correct time source is present within the buffer
  current_time_buffer_.publish_write(rclcpp::Clock(rcl_time_source).now());
  // init to the trajectories "neutral" goal, expressed relative to 'base_frame' as expected by the trajectory
  // (targets fed via handle_target_msg_sub() are validated to be in 'base_frame' too, see there)
  trajectory_update_state_type current_state;
  current_state.data().setNeutral();
  current_state.time() = current_time_buffer_.update_read();
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];
  const pinocchio::SE3 base_to_target = oMbase.actInv(target_pose());
  current_state.pose().linear() = base_to_target.translation();
  current_state.pose().angular() = base_to_target.rotation();
  if constexpr (trajectory_start_order_depth >= geometry::KinematicOrder::Twist) {
    const pinocchio::Motion base_v_local =
        pinocchio::getFrameVelocity(robot_model_, state_data_, base_frame_idx_, pinocchio::ReferenceFrame::LOCAL);
    const pinocchio::Motion target_v_local =
        pinocchio::getFrameVelocity(robot_model_, state_data_, target_frame_idx_, pinocchio::ReferenceFrame::LOCAL);
    const pinocchio::Motion target_v_in_base = base_to_target.act(target_v_local) - base_v_local;
    current_state.twist().linear() = target_v_in_base.linear();
    current_state.twist().angular() = target_v_in_base.angular();
  }
  static_assert(trajectory_start_order_depth <= geometry::KinematicOrder::Twist,  // line break
                "Starting a trajectory with higher continuity order than Twist is not yet supported");
  trajectory_buffer_.write().calculate_neutral(current_state);
  trajectory_buffer_.publish_write();

  // initialize IK QP
  qp_jacobian_.setZero();
  pinocchio::computeFrameJacobian(robot_model_, state_data_, state_q_, target_frame_idx_,
                                  pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, qp_jacobian_);
  qp_solver_H_.block(0, 0, robot_model_.nv, robot_model_.nv) =
      qp_jacobian_.transpose() * qp_jacobian_;  // neglect angular weight
  qp_solver_H_.diagonal().segment(0, robot_model_.nv).array() += params_->ik_damping;
  qp_solver_g_.setZero();  // pose_diff is zero on initialization
  // TODO(patrick): maybe it's better to initialize with the current velocity
  qp_solver_->init(qp_solver_H_, qp_solver_g_,                                  // optimization criteria
                   proxsuite::nullopt, proxsuite::nullopt,                      // no equality constraints
                   proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,  // inequality constraints
                   Eigen::VectorXd::Constant(qp_solver_->model.dim, -0.1),
                   Eigen::VectorXd::Constant(qp_solver_->model.dim, 0.1)  // box constraints (joint position and frame
                                                                          // displacement limits)
  );
  qp_solver_->solve(Eigen::VectorXd::Zero(qp_solver_->model.dim), proxsuite::nullopt, proxsuite::nullopt);
  if (!qp_solver_->results.x.isZero(params_->ik_precision)) {
    RCLCPP_ERROR(get_node()->get_logger(), "QP solver did not converge on zero-initialization. Abort Activation.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (params_->topic_pub_frequency > 0.0) {
    publish_topics();
  }

  // Everything functional and ready for RT operation
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianPoseController::handle_target_msg_sub(const trajectory_target_msg_type::SharedPtr msg)
{
  // Accept targets given in 'base_frame'; an empty frame_id is treated as implicitly 'base_frame' too.
  if (msg->header.frame_id.empty() || (msg->header.frame_id == params_->base_frame)) {
    // Update the new trajectory outside the realtime-loop
    trajectory_target_type target_goal{};
    duatic_geometry_msgs::decode(*msg, target_goal);
    trajectory_buffer_.write().update_from(trajectory_buffer_.read(), current_time_buffer_.update_read(), target_goal);
    assert(current_time_buffer_.read().get_clock_type() == Self::rcl_time_source);
    trajectory_buffer_.publish_write();
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring target message with frame_id '%s'; expected '%s' or an empty frame_id.",
                msg->header.frame_id.c_str(), params_->base_frame.c_str());
  }
}

controller_interface::return_type CartesianPoseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                  [[maybe_unused]] const rclcpp::Duration& period)
{
  assert((time.get_clock_type() == Self::rcl_time_source) && "time provided by the wrong time source");

  // Publish the current end effector pose and twist iff the time is right
  const double seconds = time.seconds();
  const bool do_publications = (seconds > topics_pub_next_time_);
  const double problem_scale = motion_horizon_ / period.seconds();

  update_state(false);
  current_time_buffer_.publish_write(time);  // update non-rt time

  // Get 6D-diff between current pose and target pose
  trajectory_buffer_.update_read().evaluate<trajectory_eval_order_depth>(time, trajectory_eval_target_);
  // trajectory_eval_target_.pose() is given relative to 'base_frame' (matching how targets are interpreted in
  // handle_target_msg_sub() and how the trajectory's neutral state was seeded in on_activate()); convert it directly
  // into the world-aligned target_frame_idx frame expected by the rest of this function: position relative to the
  // current target_frame_idx origin, orientation kept in world-frame axes. Translation and orientation are folded
  // by hand (quaternion algebra) instead of composing full SE3 objects, avoiding an unneeded matrix conversion.
  const Eigen::Quaterniond base_orientation(state_data_.oMf[base_frame_idx_].rotation());
  trajectory_eval_target_.pose().linear() = base_orientation * trajectory_eval_target_.pose().linear() +
                                            state_data_.oMf[base_frame_idx_].translation() -
                                            target_pose().translation();  // from now on in local-world-aligned frame
  trajectory_eval_target_.pose().angular() = (base_orientation * trajectory_eval_target_.pose().angular()).normalized();
  trajectory_eval_target_pose_diff_ =
      trajectory_eval_target_.pose() - geometry::Pose3Dd(Eigen::Vector3d::Zero(), target_pose().rotation());

  geometry::Twist3Dd problem_pose_diff =
      trajectory_eval_target_pose_diff_ * problem_scale;  // local-world-aligned frame
  scale_limit(problem_pose_diff.linear(), v_limit_lin_);
  scale_limit(problem_pose_diff.angular(), v_limit_ang_);

  // Run the Optimization
  run_pose_diff_ik(problem_scale, problem_pose_diff, params_->enable_debug_log && do_publications);

  // integrate joint positions
  assert(robot_model_.nv == state_q_.size());
  control_q_ = (state_q_ + pose_diff_ik_result_)
                   .cwiseMax(robot_model_.lowerPositionLimit.cwiseMin(state_q_))
                   .cwiseMin(robot_model_.upperPositionLimit.cwiseMax(state_q_));  // soft limits !

  // verify result
  pinocchio::forwardKinematics(robot_model_, control_data_, control_q_);
  pinocchio::updateFramePlacement(robot_model_, control_data_, target_frame_idx_);  // update target frame

  // express the candidate solution pose in the same original world-aligned local reference frame as
  // trajectory_eval_target_
  const geometry::Pose3Dd solution_pose(control_data_.oMf[target_frame_idx_].translation() -
                                            target_pose().translation(),
                                        control_data_.oMf[target_frame_idx_].rotation());
  solution_pose_diff_ = trajectory_eval_target_.pose() - solution_pose;

  // backtrack to prevent overshoot
  const double backtracking_scale =
      std::fmax(-1.0, std::fmin((trajectory_eval_target_pose_diff_.vector().transpose() * solution_pose_diff_.vector() +
                                 params_->ik_precision) /
                                    (solution_pose_diff_.vector().squaredNorm() + params_->ik_precision),
                                1.0));
  control_q_ += (backtracking_scale - 1.0) * pose_diff_ik_result_;

  // scale down further if the target or any observed frame's actual resulting speed would exceed its configured limit
  const double frame_velocity_scale = apply_frame_velocity_limits(period.seconds());
  if (params_->enable_debug_log && (frame_velocity_scale < 1.0)) {
    RCLCPP_INFO(get_node()->get_logger(), "Scaling joint motion by %.3f to respect frame speed limits.",
                frame_velocity_scale);
  }  // TODO(patrick): this is only for debugging, remove this message again
  // TODO(patrick): this scsale should be forwarded into the (not yet existing velocity calculation)
  // TODO(patrick): decide what to do with the time if the motion is downscaled... (delay/catch-up) ?

  // update control_v_
  control_v_ = 0.95 * (control_q_ - state_q_) / period.seconds();  // TODO: do correct or do not !

  // Write to HW
  if (!params_->dry_run) {
    command_controls();
  }

  // Publish the current end effector pose and twist iff the time is right
  if (do_publications) {
    if (params_->enable_debug_log) {
      log_statistics();
    }
    publish_topics();

    // update the next publish time
    topics_pub_next_time_ = std::fmax(seconds, topics_pub_next_time_ + topics_pub_period_);
  }

  return controller_interface::return_type::OK;
}

void CartesianPoseController::update_state(const bool use_hw_positions)
{
  static_assert(required_state_order_depth >= geometry::KinematicOrder::Twist,  // line break
                "The controller requires at least Twist order state updates");
  static_assert(state_order_depth >= required_state_order_depth);
  auto interface_iter = state_interfaces_.begin();
  for (const auto idx : joint_q_idx_) {
    state_q_[idx] = interface_iter->get_optional().value_or(state_q_[idx]);
    interface_iter++;
  }
  for (const auto idx : joint_v_idx_) {
    state_v_[idx] = interface_iter->get_optional().value_or(state_v_[idx]);
    interface_iter++;
  }

  if (use_hw_positions) {
    control_q_ = state_q_;
  } else {
    state_q_ = control_q_;  // TODO(patrick): Bad workaround: the robot is not fast enough to follow the control, so the
                            // control would follow the robot if real state updates would be used here!
  }

  // run forward kinematics and update end effector frame state
  pinocchio::forwardKinematics(robot_model_, state_data_, state_q_, state_v_);
  pinocchio::computeJointJacobians(robot_model_, state_data_);  // Note, that if there is only a single target and none
                                                                // or only a single limit frame, computing all might be
                                                                // slower but this should be fine anyhow.
  pinocchio::updateFramePlacements(robot_model_, state_data_);  // update all frames
}

void CartesianPoseController::run_pose_diff_ik(const double problem_scale, const geometry::Twist3Dd& scaled_target_diff,
                                               const bool verbose)
{
  // Construct Target Error Problem
  qp_jacobian_.setZero();
  pinocchio::getFrameJacobian(robot_model_, state_data_, target_frame_idx_,
                              pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, qp_jacobian_);
  qp_jacobian_t_ = qp_jacobian_.transpose();
  qp_jacobian_t_.block(0, 0, robot_model_.nv, 3) *= linear_error_weight_;
  static_assert(!decltype(qp_solver_H_)::IsRowMajor, "qp_solver_H_ is assumed to be column-major for accessing upper "
                                                     "triangular columns of H");
  qp_solver_H_.triangularView<Eigen::Upper>() = qp_jacobian_t_ * qp_jacobian_;
  qp_solver_H_.diagonal().array() += params_->ik_damping;
  qp_solver_g_ = -(qp_jacobian_t_ * scaled_target_diff.vector());
  // the entire upper triangle of H is now assembled -- mirror it down into the strictly lower triangle.
  qp_solver_H_.triangularView<Eigen::StrictlyLower>() = qp_solver_H_.transpose();

  // Fill QP box bounds with soft position displacement limits, scaled to match theta's own 'problem_scale' scaling
  qp_solver_l_box_ = (robot_model_.lowerPositionLimit - state_q_).cwiseMin(0.0) * problem_scale;
  qp_solver_u_box_ = (robot_model_.upperPositionLimit - state_q_).cwiseMax(0.0) * problem_scale;

  // Update and solve QP
  qp_solver_->settings.verbose = verbose;
  qp_solver_->update(qp_solver_H_, qp_solver_g_,                                  // optimization criteria
                     proxsuite::nullopt, proxsuite::nullopt,                      // no equality constraints
                     proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,  // inequality constraints
                     qp_solver_l_box_,
                     qp_solver_u_box_,  // box constraints (joint position limits)
                     true               // update_preconditioner
  );
  qp_solver_->solve();  // warm start with previous result by settings
  // ERROR HANDLING
  if (qp_solver_->results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "CartesianPoseController: QP solver did not converge, result status is "
            << static_cast<std::underlying_type_t<proxsuite::proxqp::QPSolverOutput>>(qp_solver_->results.info.status));
    log_statistics();
    qp_solver_->results.x *= 0.5;

    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Continue with half the unfinished solution: " << qp_solver_->results.x.transpose());
  }

  // rescale result
  pose_diff_ik_result_ = qp_solver_->results.x.segment(0, robot_model_.nv) / problem_scale;
}

double CartesianPoseController::apply_frame_velocity_limits(const double period_seconds)
{
  const Eigen::VectorXd delta_q = control_q_ - state_q_;
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];

  qp_jacobian_.setZero();
  pinocchio::getFrameJacobian(robot_model_, state_data_, base_frame_idx_, pinocchio::ReferenceFrame::LOCAL,
                              qp_jacobian_);
  const Eigen::Vector3d base_displacement_lin = qp_jacobian_.block(0, 0, 3, robot_model_.nv) * delta_q;

  double scale = 1.0;
  const auto limit_frame_speed = [&](const pinocchio::FrameIndex frame_idx, const double v_limit) {
    qp_jacobian_.setZero();  // reuse jacobian memory
    pinocchio::getFrameJacobian(robot_model_, state_data_, frame_idx, pinocchio::ReferenceFrame::LOCAL, qp_jacobian_);
    const Eigen::Vector3d frame_displacement_lin = qp_jacobian_.block(0, 0, 3, robot_model_.nv) * delta_q;
    const Eigen::Matrix3d base_to_frame_rotation = oMbase.actInv(state_data_.oMf[frame_idx]).rotation();
    const double displacement_lin_in_base =
        ((base_to_frame_rotation * frame_displacement_lin) - base_displacement_lin).norm();
    const double displacement_limit = v_limit * period_seconds;
    if (displacement_lin_in_base > displacement_limit) {
      scale = std::fmin(scale, displacement_limit / displacement_lin_in_base);
    }
  };

  // the target frame's own actual resulting speed, using the general linear velocity limit
  limit_frame_speed(target_frame_idx_, params_->limits.velocity.linear);
  for (std::size_t i = 0; i < observed_frame_indices_.size(); i++) {
    limit_frame_speed(observed_frame_indices_[i], params_->limits.velocity.observed_frames_linear[i]);
  }

  control_q_ = state_q_ + scale * delta_q;
  return scale;
}

void CartesianPoseController::command_controls()
{
  auto command_itr = command_interfaces_.begin();
  // make sure to have the same order as initially claimed within 'command_interface_configuration'
  for (std::size_t i = 0; i < params_->joints.size(); i++, command_itr++) {
    // set position
    if (!command_itr->set_value(control_q_[joint_q_idx_[i]])) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command for joint '%s'",
                  params_->joints[i].c_str());
    }
  }
  if constexpr (trajectory_eval_order_depth >= geometry::KinematicOrder::Twist) {
    for (std::size_t i = 0; i < params_->joints.size(); i++, command_itr++) {
      // set velocity
      if (!command_itr->set_value(control_v_[joint_v_idx_[i]])) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set velocity command for joint '%s'",
                    params_->joints[i].c_str());
      }
    }
  }
  static_assert(trajectory_eval_order_depth <= geometry::KinematicOrder::Twist,  // line break
                "Control commands of higher derivative than Twist are not yet supported");
  assert(command_itr == command_interfaces_.end());
}

void CartesianPoseController::log_statistics() const
{
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "IK solver: Problem Description"
                                                   << std::endl  // print out the entire QP Problem
                                                   << " - Hessian (I + w * J^T * J)" << std::endl
                                                   << qp_solver_H_ << std::endl
                                                   << " - Linear term (-w * J^T * pose_diff)" << std::endl
                                                   << qp_solver_g_ << std::endl
                                                   << " - Lower Box Bounds" << std::endl
                                                   << qp_solver_l_box_ << std::endl
                                                   << " - Upper Box Bounds" << std::endl
                                                   << qp_solver_u_box_);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "IK solver: Statistics"
                         << std::endl  // Print solver statistics
                         << " - time - setup: " << qp_solver_->results.info.setup_time << " µs" << std::endl
                         << " - time - solve: " << qp_solver_->results.info.solve_time << " µs" << std::endl
                         << " - time - run: " << qp_solver_->results.info.run_time << " µs" << std::endl
                         << " - inner iterations: " << qp_solver_->results.info.iter << std::endl
                         << " - outer iterations: " << qp_solver_->results.info.iter_ext << std::endl);
  Eigen::VectorXd control_positions = Eigen::VectorXd::Zero(control_q_.size());
  Eigen::VectorXd control_velocities = Eigen::VectorXd::Zero(control_v_.size());
  assert(control_q_.size() == control_v_.size());
  assert(control_q_.size() == static_cast<Eigen::Index>(params_->joints.size()));
  for (Eigen::Index i = 0; i < control_q_.size(); ++i) {
    control_positions[i] = control_q_[joint_q_idx_[static_cast<size_t>(i)]];
    control_velocities[i] = control_v_[joint_v_idx_[static_cast<size_t>(i)]];
  }
  // target pose/twist relative to 'base_frame' (matching what is actually published), plus 'base_frame' itself in
  // world coordinates for reference -- see publish_topics() for the same relative-pose/-twist formulas.
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];
  const pinocchio::Motion base_v_local =
      pinocchio::getFrameVelocity(robot_model_, state_data_, base_frame_idx_, pinocchio::ReferenceFrame::LOCAL);
  const pinocchio::Motion base_v_world_aligned = pinocchio::getFrameVelocity(
      robot_model_, state_data_, base_frame_idx_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  const pinocchio::SE3 base_to_target = oMbase.actInv(target_pose());
  const pinocchio::Motion target_v_local =
      pinocchio::getFrameVelocity(robot_model_, state_data_, target_frame_idx_, pinocchio::ReferenceFrame::LOCAL);
  const pinocchio::Motion target_v_in_base = base_to_target.act(target_v_local) - base_v_local;
  RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "IK Target:" << std::endl  // Print the solver's solution
                   << " - target - base_frame pose : "
                   << geometry::Pose3Dd(base_to_target.translation(), base_to_target.rotation()) << std::endl
                   << " - target - base_frame twist: "
                   << geometry::Twist3Dd(target_v_in_base.linear(), target_v_in_base.angular()) << std::endl
                   << " - base_frame - world pose : " << geometry::Pose3Dd(oMbase.translation(), oMbase.rotation())
                   << std::endl
                   << " - base_frame - world twist: "
                   << geometry::Twist3Dd(base_v_world_aligned.linear(), base_v_world_aligned.angular())
                   << " - trajectory eval target state: " << trajectory_eval_target_);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "IK solver: Problem Solution"
                         << std::endl  // Print the solver's solution
                         << " - solver - solution   : " << qp_solver_->results.x.transpose() << std::endl
                         << " - result              : " << pose_diff_ik_result_.transpose() << std::endl
                         << " - state   - position: " << state_q_.transpose() << std::endl
                         << " - control - position: " << control_positions.transpose() << std::endl
                         << " - state   - velocity: " << state_v_.transpose() << std::endl
                         << " - control - velocity: " << control_velocities.transpose());
}

void CartesianPoseController::publish_topics()
{
  // All published poses/twists are expressed relative to 'base_frame', matching the message header. 'oMbase' is the
  // base frame's placement in the Pinocchio root frame, and 'base_v_local' its own body twist (zero for a frame
  // rigidly/fixed-attached to the root, generally nonzero otherwise, e.g. for a moving base).
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];
  const pinocchio::Motion base_v_local =
      pinocchio::getFrameVelocity(robot_model_, state_data_, base_frame_idx_, pinocchio::ReferenceFrame::LOCAL);

  // Publish Pose
  const pinocchio::SE3 base_to_target = oMbase.actInv(target_pose());
  assert(target_pose_pub_realtime_ != nullptr);
  if (target_pose_pub_realtime_->trylock()) {
    target_pose_pub_realtime_->msg_.header.stamp = get_node()->now();
    target_pose_pub_realtime_->msg_.header.frame_id = params_->base_frame;
    const geometry::Pose3Dd current_target_pose(base_to_target.translation(), base_to_target.rotation());
    duatic_geometry_msgs::encode(current_target_pose, target_pose_pub_realtime_->msg_.pose);
    target_pose_pub_realtime_->unlockAndPublish();
  }
  // Publish Twist
  assert(target_twist_pub_realtime_ != nullptr);
  if (target_twist_pub_realtime_->trylock()) {
    const pinocchio::Motion target_v_local =
        pinocchio::getFrameVelocity(robot_model_, state_data_, target_frame_idx_, pinocchio::ReferenceFrame::LOCAL);
    const pinocchio::Motion target_v_in_base = base_to_target.act(target_v_local) - base_v_local;
    target_twist_pub_realtime_->msg_.header.stamp = get_node()->now();
    target_twist_pub_realtime_->msg_.header.frame_id = params_->base_frame;
    const geometry::Twist3Dd current_target_twist(target_v_in_base.linear(), target_v_in_base.angular());
    duatic_geometry_msgs::encode(current_target_twist, target_twist_pub_realtime_->msg_.twist);
    target_twist_pub_realtime_->unlockAndPublish();
  }

  // Publish each observed frame's pose and twist
  assert(observed_frame_pose_pub_realtime_.size() == observed_frame_indices_.size());
  assert(observed_frame_twist_pub_realtime_.size() == observed_frame_indices_.size());
  for (std::size_t i = 0; i < observed_frame_indices_.size(); i++) {
    const pinocchio::FrameIndex frame_idx = observed_frame_indices_[i];
    const pinocchio::SE3 base_to_frame = oMbase.actInv(state_data_.oMf[frame_idx]);
    if (observed_frame_pose_pub_realtime_[i]->trylock()) {
      observed_frame_pose_pub_realtime_[i]->msg_.header.stamp = get_node()->now();
      observed_frame_pose_pub_realtime_[i]->msg_.header.frame_id = params_->base_frame;
      const geometry::Pose3Dd current_observed_pose(base_to_frame.translation(), base_to_frame.rotation());
      duatic_geometry_msgs::encode(current_observed_pose, observed_frame_pose_pub_realtime_[i]->msg_.pose);
      observed_frame_pose_pub_realtime_[i]->unlockAndPublish();
    }
    if (observed_frame_twist_pub_realtime_[i]->trylock()) {
      const pinocchio::Motion observed_v_local =
          pinocchio::getFrameVelocity(robot_model_, state_data_, frame_idx, pinocchio::ReferenceFrame::LOCAL);
      const pinocchio::Motion observed_v_in_base = base_to_frame.act(observed_v_local) - base_v_local;
      observed_frame_twist_pub_realtime_[i]->msg_.header.stamp = get_node()->now();
      observed_frame_twist_pub_realtime_[i]->msg_.header.frame_id = params_->base_frame;
      const geometry::Twist3Dd current_observed_twist(observed_v_in_base.linear(), observed_v_in_base.angular());
      duatic_geometry_msgs::encode(current_observed_twist, observed_frame_twist_pub_realtime_[i]->msg_.twist);
      observed_frame_twist_pub_realtime_[i]->unlockAndPublish();
    }
  }
}

}  // namespace duatic::controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::CartesianPoseController, controller_interface::ControllerInterface)
