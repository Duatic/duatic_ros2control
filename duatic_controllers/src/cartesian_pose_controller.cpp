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

controller_interface::InterfaceConfiguration CartesianPoseController::command_interface_configuration() const
{
  // Claim the necessary command interfaces
  controller_interface::InterfaceConfiguration config;
  if (params_.dry_run) {
    config.type = controller_interface::interface_configuration_type::NONE;
  } else {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // ensure the exact same order as for the state interfaces! (Used in on_activate)
    for (const std::string& joint : params_.joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    for (const std::string& joint : params_.joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration CartesianPoseController::state_interface_configuration() const
{
  // Claim the necessary state interfaces
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // ensure the exact same order as for the command interfaces! (Used in on_activate)
  for (const std::string& joint : params_.joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const std::string& joint : params_.joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::CallbackReturn CartesianPoseController::on_init()
{
  try {
    // Obtains necessary parameters
    param_listener_ = std::make_unique<cartesian_pose_controller::ParamListener>(get_node());
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller init: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // update parameters
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // create the pinocchio model from the urdf
  RCLCPP_INFO(get_node()->get_logger(), "Building Pinocchio model from XML");
  pinocchio::Model new_model;  // rebuilding into the old model causes data-model inconsistency
  pinocchio::urdf::buildModelFromXML(get_robot_description(), new_model);
  robot_model_ = std::move(new_model);
  state_data_ = std::move(robot_model_.createData());
  if (!robot_model_.check(state_data_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio data check failed, 'state_data_' is not consistent with "
                                           "'robot_model_'");
    return controller_interface::CallbackReturn::ERROR;
  }
  state_q_ = Eigen::VectorXd::Zero(robot_model_.nq);
  state_v_ = Eigen::VectorXd::Zero(robot_model_.nv);
  // control output variables
  control_q_ = Eigen::VectorXd::Zero(robot_model_.nq);
  control_v_ = Eigen::VectorXd::Zero(robot_model_.nv);

  // evaluate joits given
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty. Abort configuration.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // evaluate optimization horizon and respective limits
  motion_horizon_ = params_.motion_horizon;
  assert(motion_horizon_ > 0.0);
  RCLCPP_INFO(get_node()->get_logger(), "Configuring optimizing with a motion horizon of %.2f seconds.",
              motion_horizon_);
  v_limit_lin_ = params_.linear_velocity_limit * motion_horizon_;  // displacement instead of velocity
  assert(v_limit_lin_ >= 0.0);
  v_limit_ang_ = params_.angular_velocity_limit * motion_horizon_;  // displacement instead of velocity;
  assert(v_limit_ang_ >= 0.0);
  // Log limits
  switch (velocity_limit_state()) {
    case VelocityLimitState::None:
      RCLCPP_WARN(get_node()->get_logger(), "Working with unlimited motions");
      break;
    case VelocityLimitState::LinearSync:
      RCLCPP_INFO(get_node()->get_logger(), "Linear displacement limit at %.2f m. Angular displacement synchronized.",
                  v_limit_lin_);
      break;
    case VelocityLimitState::AngularOnly:
      RCLCPP_WARN(get_node()->get_logger(), "Linear velocity unlimited. Angular displacement limit at %.2f rad.",
                  v_limit_ang_);
      break;
    case VelocityLimitState::Both:
      RCLCPP_INFO(get_node()->get_logger(),
                  "Linear displacement limit at %.2f m. Angular displacement limit at %.2f rad.", v_limit_lin_,
                  v_limit_ang_);
      break;
  }

  // build model joint caches
  joint_model_idx_.clear();
  joint_model_idx_.reserve(params_.joints.size());
  joint_q_idx_.clear();
  joint_q_idx_.reserve(params_.joints.size());
  joint_v_idx_.clear();
  joint_v_idx_.reserve(params_.joints.size());
  for (const std::string& joint : params_.joints) {
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

  // Store end effector frame index
  if (!robot_model_.existFrame(params_.target_frame)) {
    RCLCPP_ERROR(get_node()->get_logger(), "End effector frame '%s' not found in Pinocchio model. Abort configuration.",
                 params_.target_frame.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  } else {
    target_frame_idx_ = robot_model_.getFrameId(params_.target_frame);
  }

  // Store limit frame indices
  limit_frame_indices_.clear();
  limit_frame_indices_.reserve(params_.limit_frames.size());
  for (const std::string& frame : params_.limit_frames) {
    if (!robot_model_.existFrame(frame)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Limit frame '%s' not found in Pinocchio model.", frame.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    } else {
      limit_frame_indices_.push_back(robot_model_.getFrameId(frame));
    }
  }

  // setup and initialize QP
  assert(robot_model_.nv == params_.joints.size() && "At this stage, it is assumed that ALL joints are to be "
                                                     "controlled! -> This is an open TODO");
  // TODO(patrick): always make a full model and use the joints parameters to define actively controlled joints

  // accumulate constraints
  Eigen::Index qp_constraints_n = 0;
  switch (velocity_limit_state()) {
    case VelocityLimitState::None:
      break;
    case VelocityLimitState::LinearSync:
    case VelocityLimitState::AngularOnly:
      qp_constraints_n += static_cast<Eigen::Index>(limit_frame_indices_.size());  // linear or angular limits
      break;
    case VelocityLimitState::Both:
      qp_constraints_n += static_cast<Eigen::Index>(limit_frame_indices_.size()) * 2;  // both limits
      break;
  }
  // create QP solver
  RCLCPP_INFO(get_node()->get_logger(), "Setting up QP solver with %d result variables and %d constraint variables",
              robot_model_.nv, static_cast<int>(qp_constraints_n));
  qp_solver_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
      robot_model_.nv + qp_constraints_n, 0, 0,  // variables, eq-constraints != 0, in-eq-constraints != 0
      true,                                      // box_constrained
      proxsuite::proxqp::HessianType::Dense);
  assert(qp_solver_->model.dim == robot_model_.nv + static_cast<Eigen::Index>(qp_constraints_n));
  // settings
  qp_solver_->settings.eps_abs = params_.ik_precision;
  qp_solver_->settings.eps_rel = 0.1 * params_.ik_precision;
  qp_solver_->settings.max_iter = params_.ik_max_iterations;
  qp_solver_->settings.max_iter_in = params_.ik_max_iterations - 2;
  qp_solver_->settings.verbose = params_.enable_debug_log;
  qp_solver_->settings.compute_timings = (params_.enable_debug_log || params_.enable_introspection);
  qp_solver_->settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;  // ! IMPORTANT ! Don't reset this on
                                                                               // error, it will cause way more required
                                                                               // iterations, whyever ...
  // variables setup
  qp_solver_H_ =
      Eigen::MatrixXd::Identity(qp_solver_->model.dim, qp_solver_->model.dim);  // init save as identity matrix.
  if (qp_constraints_n > 0) {  // initialize the lower-right weighted identity block matrix within H (it stays constant)
    qp_solver_H_.diagonal().segment(robot_model_.nv, qp_constraints_n).setConstant(params_.ik_limit_frames_weight);
  }
  qp_solver_g_ = Eigen::VectorXd::Zero(qp_solver_->model.dim);
  qp_jacobian_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, robot_model_.nv);
  // position (box) limits
  qp_solver_l_box_ = Eigen::VectorXd::Constant(qp_solver_->model.dim, -1e10);
  qp_solver_u_box_ = Eigen::VectorXd::Constant(qp_solver_->model.dim, 1e10);
  pose_diff_ik_result_ = Eigen::VectorXd::Zero(robot_model_.nv);

  // subscriptions
  const std::string topic_prefix = (params_.topic_prefix.empty() ? get_node()->get_name() : params_.topic_prefix) + "/";
  const std::string target_topic_prefix = topic_prefix + params_.target_frame + "/";

  // TARGET INPUT
  target_msg_sub_ = get_node()->create_subscription<trajectory_type::UpdateInformation::msg>(
      target_topic_prefix + params_.target_topic_suffix, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&CartesianPoseController::handle_target_msg_sub, this, std::placeholders::_1));

  // create RT topic publishers for the controller end effector pose and twist
  if (params_.topic_pub_frequency > 0.0) {
    topics_pub_period_ = 1.0 / params_.topic_pub_frequency;
    topics_pub_next_time_ = get_node()->now().seconds();  // force immediate publish on first update

    target_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        target_topic_prefix + "pose", rclcpp::QoS(1).durability_volatile());
    target_pose_pub_realtime_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(target_pose_pub_);

    target_twist_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        target_topic_prefix + "twist", rclcpp::QoS(1).durability_volatile());
    target_twist_pub_realtime_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(target_twist_pub_);
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Topic frequency is set to 0.0, not publishing anything");
    topics_pub_period_ = 0.0;
    topics_pub_next_time_ = std::numeric_limits<double>::max();
    target_pose_pub_ = nullptr;
    target_pose_pub_realtime_ = nullptr;
    target_twist_pub_ = nullptr;
    target_twist_pub_realtime_ = nullptr;
  }

  // ros2control introspection
  if (params_.enable_introspection) {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring ROS2control Introspection for internal state monitoring.");
    for (std::size_t i = 0; i < params_.joints.size(); i++) {
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_q_" + std::to_string(i), &state_q_[joint_q_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_v_" + std::to_string(i), &state_v_[joint_v_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_q_" + std::to_string(i), &control_q_[joint_q_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_v_" + std::to_string(i), &control_v_[joint_v_idx_[i]]);
    }
    REGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_x", &target_pose_diff_.vector(0));
    REGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_y", &target_pose_diff_.vector(1));
    REGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_z", &target_pose_diff_.vector(2));
    REGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_rx", &target_pose_diff_.vector(3));
    REGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_ry", &target_pose_diff_.vector(4));
    REGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_rz", &target_pose_diff_.vector(5));
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
  if (params_.enable_introspection) {
    RCLCPP_INFO(get_node()->get_logger(), "Unconfiguring ROS2control Introspection.");
    using namespace hardware_interface;  // BUGFIX IN ROS2CONTROL !  The actual macro is missing to include this
                                         // namespace natively as done in the REGISTER function
    for (std::size_t i = 0; i < params_.joints.size(); i++) {
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("state_q_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("state_v_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("control_q_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("control_v_" + std::to_string(i));
    }
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_x");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_y");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_z");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_rx");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_ry");
    UNREGISTER_ROS2_CONTROL_INTROSPECTION("target_pose_diff_rz");
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
  if (!params_.dry_run) {
    // validation check
    if (command_interfaces_.size() != state_interfaces_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Number of command interfaces (%zu) does not match expected number of state interfaces (%zu). Abort "
                   "activation.",
                   command_interfaces_.size(), state_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    // initialize the command interfaces to the current state to avoid jumps on activation
    assert(state_interfaces_.size() == command_interfaces_.size());
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      const auto state = state_interfaces_[i].get_optional();
      if (!state) {
        RCLCPP_ERROR(get_node()->get_logger(), "State interface %s is not available. Abort activation.",
                     state_interfaces_[i].get_name().c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      if (!command_interfaces_[i].set_value(state.value())) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to initialize command '%s'",
                    command_interfaces_[i].get_full_name().c_str());
      };
    }
  }

  // initialize current state (data availability is guaranteed by the previous loop)
  update_state();

  // initialize state and trajectory buffers
  current_state_buffer_.write().time = get_node()->now();
  current_state_buffer_.write().pose.position = target_pose().translation();
  current_state_buffer_.write().pose.orientation = target_pose().rotation();
  current_state_buffer_.write().twist = target_twist();
  current_state_buffer_.publish_write();
  trajectory_buffer_.write().update(current_state_buffer_.read(),
                                    trajectory_type::NeutralUpdate(current_state_buffer_.update_read()));
  trajectory_buffer_.publish_write();

  // initialize IK QP
  qp_jacobian_.setZero();
  pinocchio::computeFrameJacobian(robot_model_, state_data_, state_q_, target_frame_idx_,
                                  pinocchio::ReferenceFrame::WORLD, qp_jacobian_);
  qp_solver_H_.block(0, 0, robot_model_.nv, robot_model_.nv) = qp_jacobian_.transpose() * qp_jacobian_;
  qp_solver_H_.diagonal().segment(0, robot_model_.nv).array() += params_.ik_damping;
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
  if (!qp_solver_->results.x.isZero(params_.ik_precision)) {
    RCLCPP_ERROR(get_node()->get_logger(), "QP solver did not converge on zero-initialization. Abort Activation.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  publish_topics();

  // Everything functional and ready for RT operation
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianPoseController::handle_target_msg_sub(const trajectory_type::UpdateInformation::msg::SharedPtr msg)
{
  // Update/Calculate new trajectory outside the realtime-loop
  trajectory_buffer_.write().update(current_state_buffer_.update_read(), trajectory_type::UpdateInformation(*msg));
  trajectory_buffer_.publish_write();
}

controller_interface::return_type CartesianPoseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                  [[maybe_unused]] const rclcpp::Duration& period)
{
  // Publish the current end effector pose and twist iff the time is right
  const double seconds = time.seconds();
  const bool do_publications = (seconds > topics_pub_next_time_);
  const double problem_scale = motion_horizon_ / period.seconds();

  update_state();

  update_rt_state_buffer(time);

  // Get 6D-diff between current pose and target pose
  trajectory_buffer_.update_read().evaluate_at(time, target_state_);
  target_pose_diff_ = target_state_.pose - geometry::Pose3Dd(target_pose().translation(), target_pose().rotation());
  geometry::Twist3Dd problem_pose_diff = target_pose_diff_ * problem_scale;

  // Limit error to safe target frame velocity limits
  // ! IMPORTANT: This estimation is based on the assumption of consistent time steps !
  switch (velocity_limit_state()) {
    case VelocityLimitState::None:
      break;
    case VelocityLimitState::LinearSync:
      problem_pose_diff.Rotation() *= scale_limit(problem_pose_diff.Translation(), v_limit_lin_);
      break;
    case VelocityLimitState::AngularOnly:
      scale_limit(problem_pose_diff.Rotation(), v_limit_ang_);
      // Continue !
    case VelocityLimitState::Both:
      scale_limit(problem_pose_diff.Translation(), v_limit_lin_);
      break;
  }

  // Run the Optimization
  run_pose_diff_ik(problem_scale, problem_pose_diff, params_.enable_debug_log && do_publications);

  // integrate joint position
  assert(robot_model_.nv == state_q_.size());
  control_q_ = (state_q_ + pose_diff_ik_result_)
                   .cwiseMax(robot_model_.lowerPositionLimit.cwiseMin(state_q_))
                   .cwiseMin(robot_model_.upperPositionLimit.cwiseMax(state_q_));  // soft limits !
  // TODO: update control_v_

  // Write to HW
  if (!params_.dry_run) {
    command_controls();
  }

  // Publish the current end effector pose and twist iff the time is right
  if (do_publications) {
    if (params_.enable_debug_log) {
      log_statistics();
    }
    publish_topics();

    // update the next publish time
    topics_pub_next_time_ = std::fmax(seconds, topics_pub_next_time_ + topics_pub_period_);
  }

  return controller_interface::return_type::OK;
}

void CartesianPoseController::update_state()
{
  auto interface_iter = state_interfaces_.begin();
  for (const auto idx : joint_q_idx_) {
    //    x = (1-a)*x + a*y
    //      = x - ax + ay
    //      = x + a(y -x)
    // TODO: parameterize or remove these filters!
    state_q_[idx] += 1.0 * (interface_iter->get_optional().value_or(state_q_[idx]) - state_q_[idx]);
    interface_iter++;
  }
  for (const auto idx : joint_v_idx_) {
    state_v_[idx] += 0.1 * (interface_iter->get_optional().value_or(state_v_[idx]) - state_v_[idx]);
    interface_iter++;
  }
  assert(interface_iter == state_interfaces_.end());

  // run forward kinematics and update end effector frame state
  pinocchio::forwardKinematics(robot_model_, state_data_, state_q_, state_v_);
  pinocchio::computeJointJacobians(robot_model_, state_data_);  // Note, that if there is only a single target and none
                                                                // or only a single limit frame, computing all might be
                                                                // slower but this should be fine anyhow.
  pinocchio::updateFramePlacements(robot_model_, state_data_);  // update all frames
  state_target_twist_ =
      pinocchio::getFrameVelocity(robot_model_, state_data_, target_frame_idx_, pinocchio::ReferenceFrame::WORLD);
}

void CartesianPoseController::update_rt_state_buffer(const rclcpp::Time& now)
{
  // update rt state buffer
  current_state_buffer_.write().time = now;
  current_state_buffer_.write().pose.position = target_pose().translation();
  current_state_buffer_.write().pose.orientation = target_pose().rotation();
  current_state_buffer_.write().twist = target_twist();
  current_state_buffer_.publish_write();
}

void CartesianPoseController::run_pose_diff_ik(const double problem_scale, const geometry::Twist3Dd& scaled_target_diff,
                                               const bool verbose)
{
  // Construct Target Error Problem
  qp_jacobian_.setZero();
  pinocchio::getFrameJacobian(robot_model_, state_data_, target_frame_idx_, pinocchio::ReferenceFrame::WORLD,
                              qp_jacobian_);
  qp_solver_H_.block(0, 0, robot_model_.nv, robot_model_.nv) = qp_jacobian_.transpose() * qp_jacobian_;
  qp_solver_H_.diagonal().segment(0, robot_model_.nv).array() += params_.ik_damping;
  qp_solver_g_.segment(0, robot_model_.nv) = -(qp_jacobian_.transpose() * scaled_target_diff.vector);
  // Fill QP box bounds with soft position displacement limits
  qp_solver_l_box_.segment(0, robot_model_.nv) = (robot_model_.lowerPositionLimit - state_q_).cwiseMin(0.0);
  qp_solver_u_box_.segment(0, robot_model_.nv) = (robot_model_.upperPositionLimit - state_q_).cwiseMax(0.0);
  // Construct Constraints Problem
  switch (velocity_limit_state()) {
    case VelocityLimitState::None:
      assert(qp_solver_->model.dim == robot_model_.nv);
      break;
    case VelocityLimitState::LinearSync:
    {
      assert(qp_solver_->model.dim == robot_model_.nv + limit_frame_indices_.size());
      Eigen::Index H_idx = robot_model_.nv;
      for (const auto frame_idx : limit_frame_indices_) {
        qp_jacobian_.setZero();  // reuse jacobian memory
        pinocchio::getFrameJacobian(robot_model_, state_data_, frame_idx, pinocchio::ReferenceFrame::LOCAL,
                                    qp_jacobian_);
        const pinocchio::Motion frame_motion =
            pinocchio::getFrameVelocity(robot_model_, state_data_, frame_idx, pinocchio::ReferenceFrame::LOCAL);
        qp_solver_H_.block(H_idx, 0, 1, robot_model_.nv) =
            frame_motion.linear().transpose() * qp_jacobian_.block(0, 0, 3, robot_model_.nv);
        // avoid potential division by zero in the case of no frame movement by scaling the box-constraints instead
        const double frame_v_lin_length = frame_motion.linear().norm();
        qp_solver_u_box_(H_idx) = std::fmax(v_limit_lin_ * frame_v_lin_length,
                                            params_.ik_precision);  // guarantee some minimal numerical space
        qp_solver_l_box_(H_idx) = -qp_solver_u_box_(H_idx);
        // move on to the next index
        H_idx++;
      }
      assert(H_idx == qp_solver_->model.dim);
      break;
    }
    case VelocityLimitState::AngularOnly:
      assert(qp_solver_->model.dim == robot_model_.nv + limit_frame_indices_.size());
      assert(false && "Not Yet Implemented");  // TODO(patrick): implement
      break;
    case VelocityLimitState::Both:
      assert(qp_solver_->model.dim == robot_model_.nv + 2 * limit_frame_indices_.size());
      assert(false && "Not Yet Implemented");  // TODO(patrick): implement
      break;
  }
  // copy over C to the rest of H
  const Eigen::Index constraints_n = qp_solver_->model.dim - robot_model_.nv;
  if (constraints_n > 0) {  // if there are constraints available
    auto qp_solver_C = qp_solver_H_.block(robot_model_.nv, 0, constraints_n, robot_model_.nv);
    qp_solver_H_.block(0, 0, robot_model_.nv, robot_model_.nv) +=
        params_.ik_limit_frames_weight * qp_solver_C.transpose() * qp_solver_C;
    qp_solver_C *= params_.ik_limit_frames_weight;
    qp_solver_H_.block(0, robot_model_.nv, robot_model_.nv, constraints_n) = qp_solver_C.transpose();
  }
  // scale box constraints
  qp_solver_l_box_ *= problem_scale;
  qp_solver_u_box_ *= problem_scale;
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
  // push toward zero:
  qp_solver_->results.x *= 0.9;
  qp_solver_->results.y *= 0.9;
  qp_solver_->results.z *= 0.9;
}

void CartesianPoseController::command_controls()
{
  // make sure to have the same order as initially claimed within 'command_interface_configuration'
  auto command_itr = command_interfaces_.begin();
  for (std::size_t i = 0; i < params_.joints.size(); i++, command_itr++) {
    // set position
    if (!command_itr->set_value(control_q_[joint_q_idx_[i]])) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command for joint '%s'", params_.joints[i].c_str());
    }
  }
  for (std::size_t i = 0; i < params_.joints.size(); i++, command_itr++) {
    // set velocity
    if (!command_itr->set_value(control_v_[joint_v_idx_[i]])) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set velocity command for joint '%s'", params_.joints[i].c_str());
    }
  }
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
  assert(control_q_.size() == static_cast<Eigen::Index>(params_.joints.size()));
  for (Eigen::Index i = 0; i < control_q_.size(); ++i) {
    control_positions[i] = control_q_[joint_q_idx_[static_cast<size_t>(i)]];
    control_velocities[i] = control_v_[joint_v_idx_[static_cast<size_t>(i)]];
  }
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "IK solver: Problem Solution"
                         << std::endl  // Print the solver's solution
                         << " - pose  - target: "
                         << geometry::Pose3Dd(target_pose().translation(), target_pose().rotation()) << std::endl
                         << " - pose  - target      : " << target_state_.pose << std::endl
                         << " - twist - target      : "
                         << geometry::Twist3Dd(target_twist().linear(), target_twist().angular()) << std::endl
                         << " - twist - target      : " << target_state_.twist << std::endl
                         << " - solver - solution   : " << qp_solver_->results.x.transpose() << std::endl
                         << " - result              : " << pose_diff_ik_result_.transpose() << std::endl
                         << " - state   - position: " << state_q_.transpose() << std::endl
                         << " - control - position: " << control_positions.transpose() << std::endl
                         << " - state   - velocity: " << state_v_.transpose() << std::endl
                         << " - control - velocity: " << control_velocities.transpose());
}

void CartesianPoseController::publish_topics()
{
  // Publish Pose
  if (target_pose_pub_realtime_->trylock()) {
    target_pose_pub_realtime_->msg_.header.stamp = get_node()->now();
    target_pose_pub_realtime_->msg_.header.frame_id =
        params_.base_frame;  // TODO(patrick) this frame_id is not yet taken into account in the rest of the code !
    const auto& pose_trans = target_pose().translation();
    target_pose_pub_realtime_->msg_.pose.position.x = pose_trans(0);
    target_pose_pub_realtime_->msg_.pose.position.y = pose_trans(1);
    target_pose_pub_realtime_->msg_.pose.position.z = pose_trans(2);
    const Eigen::Quaterniond pose_quat(target_pose().rotation());
    target_pose_pub_realtime_->msg_.pose.orientation.w = pose_quat.w();
    target_pose_pub_realtime_->msg_.pose.orientation.x = pose_quat.x();
    target_pose_pub_realtime_->msg_.pose.orientation.y = pose_quat.y();
    target_pose_pub_realtime_->msg_.pose.orientation.z = pose_quat.z();
    target_pose_pub_realtime_->unlockAndPublish();
  }
  // Publish Twist
  if (target_twist_pub_realtime_->trylock()) {
    target_twist_pub_realtime_->msg_.header.stamp = get_node()->now();
    target_twist_pub_realtime_->msg_.header.frame_id = params_.base_frame;
    const auto& twist_lin = target_twist().linear();
    target_twist_pub_realtime_->msg_.twist.linear.x = twist_lin(0);
    target_twist_pub_realtime_->msg_.twist.linear.y = twist_lin(1);
    target_twist_pub_realtime_->msg_.twist.linear.z = twist_lin(2);
    const auto& twist_ang = target_twist().angular();
    target_twist_pub_realtime_->msg_.twist.angular.x = twist_ang(0);
    target_twist_pub_realtime_->msg_.twist.angular.y = twist_ang(1);
    target_twist_pub_realtime_->msg_.twist.angular.z = twist_ang(2);
    target_twist_pub_realtime_->unlockAndPublish();
  }
}

}  // namespace duatic::controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::CartesianPoseController, controller_interface::ControllerInterface)
