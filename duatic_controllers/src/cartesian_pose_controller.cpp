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
#include <functional>
#include <numbers>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/check-data.hpp>
#include <pinocchio/spatial.hpp>  // pinocchio::log3

// Other headers
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <controller_interface/helpers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace duatic::controllers
{

namespace
{
inline void assign(const Eigen::Vector3d& t, const Eigen::Quaterniond& q, geometry_msgs::msg::Pose& msg)
{
  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
}

inline void assign(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular, geometry_msgs::msg::Twist& msg)
{
  msg.linear.x = linear.x();
  msg.linear.y = linear.y();
  msg.linear.z = linear.z();
  msg.angular.x = angular.x();
  msg.angular.y = angular.y();
  msg.angular.z = angular.z();
}
}  // namespace

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
    if (params_->command_velocities) {
      for (const std::string& joint : params_->joints) {
        config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        RCLCPP_DEBUG(get_node()->get_logger(), "Require command interface %s", config.names.back().c_str());
      }
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
  for (const std::string& joint : params_->joints) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    RCLCPP_DEBUG(get_node()->get_logger(), "Require state interface %s", config.names.back().c_str());
  }
  if (params_->velocity_feedback > 0.0) {
    for (const std::string& joint : params_->joints) {
      config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
      RCLCPP_DEBUG(get_node()->get_logger(), "Require state interface %s", config.names.back().c_str());
    }
  }
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
  control_q_ = Eigen::VectorXd::Zero(robot_model_.nq);
  control_v_ = Eigen::VectorXd::Zero(robot_model_.nv);

  // evaluate joits given
  if (params_->joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty. Abort configuration.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  assert(params_->motion_horizon > 0.0);
  RCLCPP_INFO(get_node()->get_logger(), "Linear limits: %.2f m/s, %.2f m/s^2.", params_->limits.velocity.linear,
              params_->limits.acceleration.linear);
  RCLCPP_INFO(get_node()->get_logger(), "Angular limits: %.2f rad/s, %.2f rad/s^2.", params_->limits.velocity.angular,
              params_->limits.acceleration.angular);

  linear_error_weight_ = params_->ik_meter_to_revolution_error_correlation * (2.0 * std::numbers::pi);
  RCLCPP_INFO(get_node()->get_logger(), "Linear error weight: %.2f", linear_error_weight_);

  // build model joint caches
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
      joint_q_idx_.push_back(robot_model_.idx_qs[jidx]);
      joint_v_idx_.push_back(robot_model_.idx_vs[jidx]);
    }
  }
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

  // setup and initialize QP
  // TODO(patrick): always make a full model and use the joints parameters to define actively controlled joints
  if (robot_model_.nv != static_cast<int>(params_->joints.size())) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'joints' (%zu) must currently cover every DOF of the URDF model (%d); a strict subset is not yet "
                 "supported. Abort configuration.",
                 params_->joints.size(), robot_model_.nv);
    return controller_interface::CallbackReturn::FAILURE;
  }

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
  // position (box) limits
  qp_solver_l_box_ = Eigen::VectorXd::Constant(qp_solver_->model.dim, -1e10);
  qp_solver_u_box_ = Eigen::VectorXd::Constant(qp_solver_->model.dim, 1e10);
  pose_diff_ik_result_ = Eigen::VectorXd::Zero(robot_model_.nv);
  // velocity as displacement limit: limit * dt * (scale := motion_horizon / dt) = limit * motion_horizon = const
  joint_velocity_box_ = robot_model_.velocityLimit * params_->motion_horizon;
  for (std::size_t i = 0; i < params_->joints.size(); i++) {
    if (robot_model_.velocityLimit[joint_v_idx_[i]] <= 0.0) {
      joint_velocity_box_[joint_v_idx_[i]] = 1.0 / numeric_epsilon;
      RCLCPP_WARN(get_node()->get_logger(),
                  "Joint '%s' has no velocity limit in the URDF (parsed as 0); setting its displacement bound to "
                  "%.2e instead of locking it.",
                  params_->joints[i].c_str(), joint_velocity_box_[joint_v_idx_[i]]);
    }
  }

  // subscriptions
  const std::string topic_prefix =
      (params_->topic_prefix.empty() ? get_node()->get_name() : params_->topic_prefix) + "/";
  const std::string target_topic_prefix = topic_prefix + params_->target_frame + "/";

  // TARGET INPUT
  target_msg_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
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
  if (params_->enable_introspection) {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring ROS2control Introspection for internal state monitoring.");
    for (std::size_t i = 0; i < params_->joints.size(); i++) {
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_q_" + std::to_string(i), &state_q_[joint_q_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_v_" + std::to_string(i), &state_v_[joint_v_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_q_" + std::to_string(i), &control_q_[joint_q_idx_[i]]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_v_" + std::to_string(i), &control_v_[joint_v_idx_[i]]);
    }
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
  if (params_->enable_introspection && qp_solver_) {  // qp_solver_ is null if on_configure() failed before creating it
    RCLCPP_INFO(get_node()->get_logger(), "Unconfiguring ROS2control Introspection.");
    using namespace hardware_interface;  // BUGFIX IN ROS2CONTROL !  The actual macro is missing to include this
                                         // namespace natively as done in the REGISTER function
    for (std::size_t i = 0; i < params_->joints.size(); i++) {
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("state_q_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("state_v_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("control_q_" + std::to_string(i));
      UNREGISTER_ROS2_CONTROL_INTROSPECTION("control_v_" + std::to_string(i));
    }
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

  // release resources acquired in on_configure(), returning to an unconfigured state
  target_msg_sub_.reset();
  target_pose_pub_realtime_.reset();
  target_pose_pub_.reset();
  target_twist_pub_realtime_.reset();
  target_twist_pub_.reset();
  qp_solver_.reset();

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
    // zero any remaining command interfaces (e.g. velocity, when there is no matching state feedback to seed from)
    while (command_itr != command_interfaces_.end()) {
      if (!command_itr->set_value(0.0)) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to initialize command '%s'. Abort activation.",
                    command_itr->get_name().c_str());
        return controller_interface::CallbackReturn::FAILURE;
      };
      ++command_itr;
    }
  }
  // reset for a deactivate/activate cycle, which does not re-run on_configure()'s zero-init
  control_q_.setZero();
  control_v_.setZero();
  // initialize current state purely from HW (if available)
  read_states(1.0);
  update_state();
  control_q_ = state_q_;
  control_v_ = state_v_;

  // Seed the target buffer with the current pose, so a cycle running before the first target message arrives
  // does not command a jump towards frame-origin/identity.
  geometry_msgs::msg::PoseStamped initial_target;
  initial_target.header.frame_id = params_->base_frame;
  const pinocchio::SE3 base_to_target = state_data_.oMf[base_frame_idx_].actInv(target_pose());
  assign(base_to_target.translation(), Eigen::Quaterniond(base_to_target.rotation()), initial_target.pose);
  target_buffer_.publish_write(initial_target);  // one-time init: having this illegal second producer is safe herein

  // initialize IK QP
  qp_jacobian_.setZero();
  pinocchio::computeFrameJacobian(robot_model_, state_data_, state_q_, target_frame_idx_,
                                  pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, qp_jacobian_);
  qp_solver_H_.setZero();
  qp_solver_H_.selfadjointView<Eigen::Upper>().rankUpdate(qp_jacobian_.transpose());  // neglect angular weight
  qp_solver_H_.diagonal().array() += params_->ik_damping;
  qp_solver_g_.setZero();  // pose_diff is zero on initialization
  qp_solver_H_.triangularView<Eigen::StrictlyLower>() = qp_solver_H_.transpose();
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
    topics_pub_next_time_ = get_node()->now().seconds() + topics_pub_period_;
  }

  // Everything functional and ready for RT operation
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianPoseController::handle_target_msg_sub(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Accept targets given in 'base_frame'; an empty frame_id is treated as implicitly 'base_frame' too.
  if (msg->header.frame_id.empty() || (msg->header.frame_id == params_->base_frame)) {
    const auto& q = msg->pose.orientation;
    const double q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (std::abs(q_norm_sq - 1.0) <= 1e-2) {  // allow some tolerance to real unit quaternions
      target_buffer_.publish_write(*msg);
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Ignoring target message with non-unit quaternion (x:%.4f, y:%.4f, z:%.4f, w:%.4f).", q.x, q.y, q.z,
                  q.w);
    }
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring target message with frame_id '%s'; expected '%s' or an empty frame_id.",
                msg->header.frame_id.c_str(), params_->base_frame.c_str());
  }
}

controller_interface::return_type CartesianPoseController::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  assert((time.get_clock_type() == Self::rcl_time_source) && "time provided by the wrong time source");

  const bool do_publications = (time.seconds() > topics_pub_next_time_);
  const double dt = std::fmax(period.seconds(), numeric_epsilon);  // never zero/negative, so problem_scale stays finite
  const double problem_scale = params_->motion_horizon / dt;

  read_states(params_->velocity_feedback);
  update_state();

  // Cartesian distance to the last received target, expressed local-world-aligned at the target frame origin
  const geometry_msgs::msg::PoseStamped& target_msg = target_buffer_.update_read();
  const Eigen::Vector3d target_position(target_msg.pose.position.x, target_msg.pose.position.y,
                                        target_msg.pose.position.z);
  const Eigen::Quaterniond target_orientation(target_msg.pose.orientation.w, target_msg.pose.orientation.x,
                                              target_msg.pose.orientation.y, target_msg.pose.orientation.z);
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];
  const pinocchio::SE3& oMtarget = target_pose();
  const Eigen::Vector3d target_position_world = oMbase.translation() + oMbase.rotation() * target_position;
  const Eigen::Matrix3d target_rotation_world = oMbase.rotation() * target_orientation.normalized().toRotationMatrix();

  Eigen::Vector3d diff_linear = target_position_world - oMtarget.translation();
  Eigen::Vector3d diff_angular =
      pinocchio::log3(Eigen::Matrix3d(target_rotation_world * oMtarget.rotation().transpose()));

  // Ramp this cycle's velocity ceiling from the target frame's actual current speed, bounded by the acceleration
  // limits, then shrink the raw distance so it cannot imply exceeding that ceiling.
  const pinocchio::Motion target_v = pinocchio::getFrameVelocity(robot_model_, state_data_, target_frame_idx_,
                                                                 pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  const double v_limit_lin_eff =
      std::fmin(params_->limits.velocity.linear, target_v.linear().norm() + params_->limits.acceleration.linear * dt);
  const double v_limit_ang_eff = std::fmin(params_->limits.velocity.angular,
                                           target_v.angular().norm() + params_->limits.acceleration.angular * dt);
  if (v_limit_lin_eff > 0.0) {
    scale_limit(diff_linear, v_limit_lin_eff * dt);
  } else {
    diff_linear.setZero();
  }
  if (v_limit_ang_eff > 0.0) {
    scale_limit(diff_angular, v_limit_ang_eff * dt);
  } else {
    diff_angular.setZero();
  }

  // Run the Optimization
  const bool ik_unsolved =
      !run_pose_diff_ik(problem_scale, diff_linear, diff_angular, params_->enable_debug_log && do_publications);

  // integrate joint positions and derive the commanded velocity from the applied step
  assert(robot_model_.nv == state_q_.size());
  control_q_ = (state_q_ + pose_diff_ik_result_)
                   .cwiseMax(robot_model_.lowerPositionLimit.cwiseMin(state_q_))
                   .cwiseMin(robot_model_.upperPositionLimit.cwiseMax(state_q_));  // soft limits !
  control_v_ = (control_q_ - state_q_) / dt;

  // Never command or carry forward a non-finite result (e.g. from a degenerate period or solver failure); hold
  // the current state instead so a bad cycle cannot poison the next one via control_v_'s feedback blend.
  if (!(control_q_.allFinite() && control_v_.allFinite())) {
    RCLCPP_ERROR(get_node()->get_logger(), "Computed a non-finite joint command; holding current state instead.");
    control_q_ = state_q_;
    control_v_.setZero();
  }

  // Write to HW
  if (!params_->dry_run) {
    command_controls();
  }

  if ((do_publications && params_->enable_debug_log) || ik_unsolved) {
    log_statistics(target_v);
  }
  // Publish the current end effector pose and twist iff the time is right
  if (do_publications) {
    publish_topics();

    // update the next publish time
    topics_pub_next_time_ = std::fmax(time.seconds(), topics_pub_next_time_ + topics_pub_period_);
  }

  return controller_interface::return_type::OK;
}

void CartesianPoseController::read_states(const double velocity_feedback_weight)
{
  auto interface_iter = state_interfaces_.begin();
  for (const auto idx : joint_q_idx_) {
    state_q_[idx] = duatic::controllers::compat::require_value(*interface_iter);
    interface_iter++;
  }
  if (params_->velocity_feedback > 0.0) {
    for (const auto idx : joint_v_idx_) {
      const double hw_v = duatic::controllers::compat::require_value(*interface_iter);
      state_v_[idx] = control_v_[idx] + velocity_feedback_weight * (hw_v - control_v_[idx]);
      interface_iter++;
    }
  } else {
    state_v_ = control_v_;
  }
  assert(interface_iter == state_interfaces_.end() && "Not all state interfaces were read");
}

void CartesianPoseController::update_state()
{
  // run forward kinematics and update end effector frame state
  pinocchio::forwardKinematics(robot_model_, state_data_, state_q_, state_v_);
  // Computing only the single required Jacobian, computeFrameJacobian(target_frame_idx_), would redo forwardKinematics'
  // placements for its whole support chain. Since the target usually sits at the tip of the kinematic chain, there
  // would be no actual saving here.
  pinocchio::computeJointJacobians(robot_model_, state_data_);
  pinocchio::updateFramePlacements(robot_model_, state_data_);  // update all frames
}

bool CartesianPoseController::run_pose_diff_ik(const double problem_scale, const Eigen::Vector3d& target_diff_linear,
                                               const Eigen::Vector3d& target_diff_angular, const bool verbose)
{
  // Construct Target Error Problem
  qp_jacobian_.setZero();
  pinocchio::getFrameJacobian(robot_model_, state_data_, target_frame_idx_,
                              pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, qp_jacobian_);
  const auto J_lin = qp_jacobian_.topRows<3>();
  const auto J_ang = qp_jacobian_.bottomRows<3>();

  // H = w * J_lin^T * J_lin + J_ang^T * J_ang + damping * I; only the upper triangle is ever computed/stored.
  static_assert(!decltype(qp_solver_H_)::IsRowMajor, "qp_solver_H_ is assumed to be column-major for accessing upper "
                                                     "triangular columns of H");
  qp_solver_H_.setZero();
  qp_solver_H_.selfadjointView<Eigen::Upper>().rankUpdate(J_lin.transpose(), linear_error_weight_);
  qp_solver_H_.selfadjointView<Eigen::Upper>().rankUpdate(J_ang.transpose());
  qp_solver_H_.diagonal().array() += params_->ik_damping;
  qp_solver_g_ = -problem_scale * ((J_lin.transpose() * (target_diff_linear * linear_error_weight_)) +
                                   (J_ang.transpose() * target_diff_angular));
  // the entire upper triangle of H is now assembled -- mirror it down into the strictly lower triangle.
  qp_solver_H_.triangularView<Eigen::StrictlyLower>() = qp_solver_H_.transpose();

  // Fill QP box bounds with soft position and joint velocity displacement limits, scaled to theta units
  qp_solver_l_box_ = ((robot_model_.lowerPositionLimit - state_q_) * problem_scale)
                         .cwiseMin(+numeric_epsilon)
                         .cwiseMax(-joint_velocity_box_);
  qp_solver_u_box_ = ((robot_model_.upperPositionLimit - state_q_) * problem_scale)
                         .cwiseMax(-numeric_epsilon)
                         .cwiseMin(joint_velocity_box_);

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
  const bool solved = (qp_solver_->results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED);
  if (!solved) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "CartesianPoseController: QP solver did not converge, result status is "
            << static_cast<std::underlying_type_t<proxsuite::proxqp::QPSolverOutput>>(qp_solver_->results.info.status));
    // the warm-started iterate is not guaranteed to respect the box constraints on non-convergence; re-clamp before
    // using it, then halve as before.
    qp_solver_->results.x = qp_solver_->results.x.cwiseMax(qp_solver_l_box_).cwiseMin(qp_solver_u_box_);
    qp_solver_->results.x *= 0.5;

    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Continue with half the unfinished solution: " << qp_solver_->results.x.transpose());
  }

  // rescale result
  pose_diff_ik_result_ = qp_solver_->results.x / problem_scale;
  return solved;
}

void CartesianPoseController::command_controls()
{
  auto command_itr = command_interfaces_.begin();
  // make sure to have the same order as initially claimed within 'command_interface_configuration'
  for (std::size_t i = 0; i < params_->joints.size(); i++, command_itr++) {
    if (!command_itr->set_value(control_q_[joint_q_idx_[i]])) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command for joint '%s'",
                  params_->joints[i].c_str());
    }
  }
  if (params_->command_velocities) {
    for (std::size_t i = 0; i < params_->joints.size(); i++, command_itr++) {
      if (!command_itr->set_value(control_v_[joint_v_idx_[i]])) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set velocity command for joint '%s'",
                    params_->joints[i].c_str());
      }
    }
  }
  assert(command_itr == command_interfaces_.end());
}

void CartesianPoseController::log_statistics(const pinocchio::Motion& target_v) const
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
  // target pose/twist relative to 'base_frame' (matching what is actually published) -- see publish_topics().
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];
  const pinocchio::SE3 base_to_target = oMbase.actInv(target_pose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "IK Target:" << std::endl
                                  << " - target - base_frame position : " << base_to_target.translation().transpose()
                                  << std::endl
                                  << " - target - world twist         : " << target_v << std::endl
                                  << " - result              : " << pose_diff_ik_result_.transpose() << std::endl
                                  << " - state   - position: " << state_q_.transpose() << std::endl
                                  << " - state   - velocity: " << state_v_.transpose() << std::endl
                                  << " - control - position: " << control_q_.transpose() << std::endl
                                  << " - control - velocity: " << control_v_.transpose());
}

void CartesianPoseController::publish_topics()
{
  // All published poses/twists are expressed relative to 'base_frame', matching the message header.
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];
  const pinocchio::Motion base_v_local =
      pinocchio::getFrameVelocity(robot_model_, state_data_, base_frame_idx_, pinocchio::ReferenceFrame::LOCAL);
  const pinocchio::SE3 base_to_target = oMbase.actInv(target_pose());

  assert(target_pose_pub_realtime_ != nullptr);
  if (target_pose_pub_realtime_->trylock()) {
    target_pose_pub_realtime_->msg_.header.stamp = get_node()->now();
    target_pose_pub_realtime_->msg_.header.frame_id = params_->base_frame;
    assign(base_to_target.translation(), Eigen::Quaterniond(base_to_target.rotation()),
           target_pose_pub_realtime_->msg_.pose);
    target_pose_pub_realtime_->unlockAndPublish();
  }
  assert(target_twist_pub_realtime_ != nullptr);
  if (target_twist_pub_realtime_->trylock()) {
    const pinocchio::Motion target_v_local =
        pinocchio::getFrameVelocity(robot_model_, state_data_, target_frame_idx_, pinocchio::ReferenceFrame::LOCAL);
    const pinocchio::Motion target_v_in_base = base_to_target.act(target_v_local) - base_v_local;
    target_twist_pub_realtime_->msg_.header.stamp = get_node()->now();
    target_twist_pub_realtime_->msg_.header.frame_id = params_->base_frame;
    assign(target_v_in_base.linear(), target_v_in_base.angular(), target_twist_pub_realtime_->msg_.twist);
    target_twist_pub_realtime_->unlockAndPublish();
  }
}

}  // namespace duatic::controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::CartesianPoseController, controller_interface::ControllerInterface)
