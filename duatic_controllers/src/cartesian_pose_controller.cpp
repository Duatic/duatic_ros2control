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
#include <numbers>  // NOLINT(build/include_order)

#include <pinocchio/algorithm/check-data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>  // pinocchio::neutral
#include <pinocchio/algorithm/model.hpp>                // pinocchio::buildReducedModel
#include <pinocchio/spatial.hpp>                        // pinocchio::log3

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
    assert((robot_model_.names[0] == "universe") && "joint 0 is expected to be Pinocchio's universe joint");
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // ensure the exact same order as for the state interfaces! (Used in on_activate)
    for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(robot_model_.njoints); j++) {
      config.names.emplace_back(robot_model_.names[j] + "/" + hardware_interface::HW_IF_POSITION);
      RCLCPP_DEBUG(get_node()->get_logger(), "Require command interface %s", config.names.back().c_str());
    }
    if (params_->command_velocities) {
      for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(robot_model_.njoints); j++) {
        config.names.emplace_back(robot_model_.names[j] + "/" + hardware_interface::HW_IF_VELOCITY);
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
  assert((robot_model_.names[0] == "universe") && "joint 0 is expected to be Pinocchio's universe joint");
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // ensure the exact same order as for the command interfaces! (Used in on_activate)
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(robot_model_.njoints); j++) {
    config.names.emplace_back(robot_model_.names[j] + "/" + hardware_interface::HW_IF_POSITION);
    RCLCPP_DEBUG(get_node()->get_logger(), "Require state interface %s", config.names.back().c_str());
  }
  if (params_->velocity_feedback > 0.0) {
    for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(robot_model_.njoints); j++) {
      config.names.emplace_back(robot_model_.names[j] + "/" + hardware_interface::HW_IF_VELOCITY);
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
  if (get_update_rate() < min_update_rate_hz) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "%s (CartesianPoseController) requires a control frequency of at least %u Hz but only got %u Hz. "
                 "Abort configuration",
                 get_node()->get_name(), min_update_rate_hz, get_update_rate());
    return controller_interface::CallbackReturn::FAILURE;
  }

  // update parameters
  try {
    param_listener_->refresh_dynamic_parameters();
    *params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception during controller configuration: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // update log level setting
  logger_level_ = get_node()->get_logger().get_effective_level();

  // build the full pinocchio model from the urdf, used only to find the joint chain between base_frame and
  // target_frame; robot_model_ itself becomes the reduced model built from exactly that chain, below.
  RCLCPP_INFO(get_node()->get_logger(), "Building Pinocchio model from XML");
  pinocchio::Model full_model;
  pinocchio::urdf::buildModelFromXML(get_robot_description(), full_model);

  if (!full_model.existFrame(params_->base_frame)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Base frame '%s' not found in Pinocchio model. Abort configuration.",
                 params_->base_frame.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (!full_model.existFrame(params_->target_frame)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Target frame '%s' not found in Pinocchio model. Abort configuration.",
                 params_->target_frame.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Building reduced Pinocchio model for joint chain between base_frame '%s' and target_frame '%s'.",
              params_->base_frame.c_str(), params_->target_frame.c_str());
  // find the joint chain between base_frame and target_frame: each frame's ancestor chain up to their common ancestor
  const auto ancestors = [&full_model, this](pinocchio::JointIndex joint_id, pinocchio::JointIndex sentinel) {
    std::vector<pinocchio::JointIndex> chain{ sentinel };  // never a real joint id
    assert((sentinel > static_cast<pinocchio::JointIndex>(full_model.njoints)) && "sentinel must not be a real joint "
                                                                                  "id");
    while (joint_id != 0) {
      chain.push_back(joint_id);
      RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "Joint " << joint_id << ": " << full_model.names[joint_id]);
      joint_id = full_model.parents[joint_id];
    }
    return chain;
  };

  RCLCPP_DEBUG(get_node()->get_logger(), "Target Chain:");
  static constexpr pinocchio::JointIndex sentinel_target_chain = static_cast<pinocchio::JointIndex>(-1);
  const std::vector<pinocchio::JointIndex> target_chain =
      ancestors(full_model.frames[full_model.getFrameId(params_->target_frame)].parentJoint, sentinel_target_chain);

  RCLCPP_DEBUG(get_node()->get_logger(), "Base Chain:");
  static constexpr pinocchio::JointIndex sentinel_base_chain = static_cast<pinocchio::JointIndex>(-2);
  const std::vector<pinocchio::JointIndex> base_chain =
      ancestors(full_model.frames[full_model.getFrameId(params_->base_frame)].parentJoint, sentinel_base_chain);

  static_assert((sentinel_target_chain != sentinel_base_chain) && "chain sentinels must be different to ensure "
                                                                  "exception-free detection of chain divergence");

  // walk inward until both chains diverge
  auto target_it = target_chain.rbegin();
  auto base_it = base_chain.rbegin();
  while (*target_it == *base_it) {
    ++target_it;
    ++base_it;
  }

  // model joint ids increase root-to-tip, matching target_it/base_it's walk direction: advance a cursor whenever
  // it points at the current joint, and it's on the chain; an untouched movable joint gets locked.
  std::vector<pinocchio::JointIndex> joints_to_lock;
  std::size_t active_count = 0;
  RCLCPP_INFO(get_node()->get_logger(), "Configure Active Joints:");
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full_model.njoints); j++) {
    bool on_chain = false;
    if (*target_it == j) {
      ++target_it;
      on_chain = true;
    } else if (*base_it == j) {
      ++base_it;
      on_chain = true;
    }
    const int joint_nq = full_model.joints[j].nq();
    if (joint_nq == 0) {
      continue;  // already a fixed joint, nothing to lock or activate
    }
    if (!on_chain) {
      joints_to_lock.push_back(j);
      continue;
    }
    if (joint_nq != 1) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint '%s' has %d DOF; only 1-DOF joints are supported. Abort "
                   "configuration.",
                   full_model.names[j].c_str(), joint_nq);
      return controller_interface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_node()->get_logger(), " - %s", full_model.names[j].c_str());
    active_count++;
  }
  if ((*target_it != sentinel_target_chain) || (*base_it != sentinel_base_chain)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Internal error: base_frame or target_frame chain were not fully consumed "
                                           "during the construction of the reduced model. Abort configuration.");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (active_count == 0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "No movable joint found between base_frame '%s' and target_frame '%s'. Abort configuration.",
                 params_->base_frame.c_str(), params_->target_frame.c_str());
    return controller_interface::CallbackReturn::FAILURE;
  }

  // build the reduced model, welding every other movable joint at its neutral position
  pinocchio::buildReducedModel(full_model, joints_to_lock, pinocchio::neutral(full_model), robot_model_);
  assert((robot_model_.nq == robot_model_.nv) && (robot_model_.nv == robot_model_.njoints - 1) &&
         "all joints are mandatorily 1-DOF, so nq, nv and njoints-1 must all agree");
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
  blend_delta_q_ = Eigen::VectorXd::Zero(robot_model_.nq);

  assert(params_->motion_horizon > 0.0);
  RCLCPP_INFO(get_node()->get_logger(), "Linear limits: %.2f m/s, %.2f m/s^2.", params_->limits.velocity.linear,
              params_->limits.acceleration.linear);
  RCLCPP_INFO(get_node()->get_logger(), "Angular limits: %.2f rad/s, %.2f rad/s^2.", params_->limits.velocity.angular,
              params_->limits.acceleration.angular);

  linear_error_weight_ = params_->ik_meter_to_revolution_error_correlation * (2.0 * std::numbers::pi);
  RCLCPP_INFO(get_node()->get_logger(), "Linear error weight: %.2f", linear_error_weight_);

  target_filter_rate_ = -1.0 / std::fmax(params_->target_filter, numeric_epsilon);
  RCLCPP_INFO(get_node()->get_logger(), "Target filter rate: %.6f", target_filter_rate_);

  // Store base frame index (all published poses/twists are expressed relative to this frame)
  assert(robot_model_.existFrame(params_->base_frame) && "buildReducedModel must preserve frame names");
  base_frame_idx_ = robot_model_.getFrameId(params_->base_frame);

  // Store target frame index
  assert(robot_model_.existFrame(params_->target_frame) && "buildReducedModel must preserve frame names");
  target_frame_idx_ = robot_model_.getFrameId(params_->target_frame);

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
  qp_solver_->settings.verbose = (logger_level_ <= rclcpp::Logger::Level::Debug);
  qp_solver_->settings.compute_timings = (qp_solver_->settings.verbose || params_->enable_introspection);
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
  joint_velocity_box_ = Eigen::VectorXd::Zero(robot_model_.nv);
  joint_velocity_limit_ = Eigen::VectorXd::Zero(robot_model_.nv);
  assert((robot_model_.names[0] == "universe") && "joint 0 is expected to be Pinocchio's universe joint");
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(robot_model_.njoints); j++) {
    const Eigen::Index v_idx = robot_model_.idx_vs[j];
    const double velocity_limit = robot_model_.velocityLimit[v_idx];
    if (velocity_limit >= 0.0) {
      joint_velocity_limit_[v_idx] = std::fmax(velocity_limit, numeric_epsilon);
      RCLCPP_INFO(get_node()->get_logger(), "Setting joint velocity limit for '%s' to %.2f rad/s.",
                  robot_model_.names[j].c_str(), joint_velocity_limit_[v_idx]);
    } else {
      joint_velocity_limit_[v_idx] = numeric_epsilon_inv;
      RCLCPP_WARN(get_node()->get_logger(),
                  "No valid velocity limit found for Joint '%s': using default velocity limit %.2f rad/s.",
                  robot_model_.names[j].c_str(), joint_velocity_limit_[v_idx]);
    }
  }
  joint_velocity_box_ = joint_velocity_limit_ * params_->motion_horizon;

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
    assert((robot_model_.nq == robot_model_.nv) && "all joints are mandatorily 1-DOF, so nq and nv must agree");
    for (Eigen::Index i = 0; i < robot_model_.nq; i++) {
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_q_" + std::to_string(i), &state_q_[i]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("state_v_" + std::to_string(i), &state_v_[i]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_q_" + std::to_string(i), &control_q_[i]);
      REGISTER_ROS2_CONTROL_INTROSPECTION("control_v_" + std::to_string(i), &control_v_[i]);
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
    // BUGFIX IN ROS2CONTROL !  UNREGISTER_ROS2_CONTROL_INTROSPECTION uses DEFAULT_REGISTRY_KEY unqualified
    using hardware_interface::DEFAULT_REGISTRY_KEY;
    assert((robot_model_.nq == robot_model_.nv) && "all joints are mandatorily 1-DOF, so nq and nv must agree");
    for (std::size_t i = 0; i < static_cast<std::size_t>(robot_model_.nq); i++) {
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
      }
      ++command_itr;
      ++state_itr;
    }
    // zero any remaining command interfaces (e.g. velocity, when there is no matching state feedback to seed from)
    while (command_itr != command_interfaces_.end()) {
      if (!command_itr->set_value(0.0)) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to initialize command '%s'. Abort activation.",
                    command_itr->get_name().c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      ++command_itr;
    }
  }
  // reset for a deactivate/activate cycle, which does not re-run on_configure()'s zero-init
  control_q_.setZero();
  control_v_.setZero();
  // initialize current state purely from HW (if available)
  read_states();
  control_q_ = state_q_;  // reinit control states to current states
  control_v_ = state_v_;
  blend_states(1.0);
  update_state();

  // Seed the target buffer with the current pose, so a cycle running before the first target message arrives
  // does not command a jump towards frame-origin/identity.
  const pinocchio::SE3 base_to_target = state_data_.oMf[base_frame_idx_].actInv(target_pose());
  auto& [position, orientation] = target_buffer_.write();
  position = base_to_target.translation();
  orientation = Eigen::Quaterniond(base_to_target.rotation());
  control_target_ = target_type(position, orientation);
  target_buffer_.publish_write();  // one-time init: having this illegal second producer is safe herein

  // initialize IK QP
  qp_jacobian_.setZero();
  pinocchio::computeFrameJacobian(robot_model_, state_data_, state_q_, target_frame_idx_,
                                  pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, qp_jacobian_);
  qp_solver_H_.setZero();
  qp_solver_H_.selfadjointView<Eigen::Upper>().rankUpdate(qp_jacobian_.transpose());  // neglect angular weight
  qp_solver_H_.diagonal().array() += params_->ik_damping;
  qp_solver_g_.setZero();  // pose_diff is zero on initialization
  qp_solver_H_.triangularView<Eigen::StrictlyLower>() = qp_solver_H_.transpose();
  qp_solver_->init(qp_solver_H_, qp_solver_g_,                                  // optimization criteria
                   proxsuite::nullopt, proxsuite::nullopt,                      // no equality constraints
                   proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,  // inequality constraints
                   -joint_velocity_box_,
                   joint_velocity_box_  // box constraints (joint velocity displacement limits)
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
    auto& [position, orientation] = target_buffer_.write();
    position.x() = msg->pose.position.x;
    position.y() = msg->pose.position.y;
    position.z() = msg->pose.position.z;
    orientation.x() = msg->pose.orientation.x;
    orientation.y() = msg->pose.orientation.y;
    orientation.z() = msg->pose.orientation.z;
    orientation.w() = msg->pose.orientation.w;
    if (std::abs(orientation.squaredNorm() - 1.0) <= 1e-2) {  // allow some tolerance to real unit quaternions
      orientation.normalize();
      target_buffer_.publish_write();
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Ignoring target message with non-unit quaternion (x:%.4f, y:%.4f, z:%.4f, w:%.4f).",
                  msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
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
  const bool verbose = (logger_level_ <= rclcpp::Logger::Level::Debug) && do_publications;
  const double dt = std::fmax(period.seconds(), numeric_epsilon);  // never zero/negative, so problem_scale stays finite
  const double problem_scale = params_->motion_horizon / dt;

  read_states();
  blend_states(params_->velocity_feedback);
  update_state();

  // filter target
  const auto& [target_position, target_orientation] = target_buffer_.update_read();
  const double target_filter_alpha = -std::expm1(dt * target_filter_rate_);
  auto& [control_target_position, control_target_orientation] = control_target_;
  control_target_position += target_filter_alpha * (target_position - control_target_position);
  control_target_orientation = control_target_orientation.slerp(target_filter_alpha, target_orientation);

  // Cartesian distance to the filtered target, expressed local-world-aligned at the target frame origin
  const pinocchio::SE3& oMbase = state_data_.oMf[base_frame_idx_];
  const pinocchio::SE3& oMtarget = target_pose();
  const Eigen::Vector3d target_position_world = oMbase.translation() + oMbase.rotation() * control_target_position;
  const Eigen::Matrix3d target_rotation_world =
      oMbase.rotation() * control_target_orientation.normalized().toRotationMatrix();

  Eigen::Vector3d diff_linear = target_position_world - oMtarget.translation();
  const double diff_lin_norm = diff_linear.norm();
  Eigen::Vector3d diff_angular =
      pinocchio::log3(Eigen::Matrix3d(target_rotation_world * oMtarget.rotation().transpose()));
  const double diff_ang_norm = diff_angular.norm();

  // Ramp this cycle's velocity ceiling from the target frame's actual current speed, bounded by the acceleration
  // limits, then shrink the raw distance so it cannot imply exceeding that ceiling.
  const pinocchio::Motion target_v = pinocchio::getFrameVelocity(robot_model_, state_data_, target_frame_idx_,
                                                                 pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  const double target_v_lin_norm = target_v.linear().norm();
  const double target_v_ang_norm = target_v.angular().norm();

  const double v_limit_lin_eff =
      std::fmin(params_->limits.velocity.linear,
                std::fmin(target_v_lin_norm + params_->limits.acceleration.linear * dt,            // acceleration limit
                          std::sqrt(2.0 * params_->limits.acceleration.linear * diff_lin_norm)));  // deceleration limit
  const double v_limit_ang_eff = std::fmin(
      params_->limits.velocity.angular,
      std::fmin(target_v_ang_norm + params_->limits.acceleration.angular * dt,            // acceleration limit
                std::sqrt(2.0 * params_->limits.acceleration.angular * diff_ang_norm)));  // deceleration limit
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
  const bool ik_unsolved = !run_pose_diff_ik(problem_scale, diff_linear, diff_angular, verbose);

  // integrate joint positions and limit to the joint position limits (soft limits)
  assert(robot_model_.nv == state_q_.size());
  control_q_ = (state_q_ + pose_diff_ik_result_)
                   .cwiseMax(robot_model_.lowerPositionLimit.cwiseMin(state_q_))
                   .cwiseMin(robot_model_.upperPositionLimit.cwiseMax(state_q_));

  // piecewise-linear velocity
  control_v_ = control_q_ - state_q_;  // average displacement over this cycle
  const double step_end_lin = std::fmax((2.0 * diff_lin_norm) - (target_v_lin_norm * dt), 0.0);
  const double step_end_ang = std::fmax((2.0 * diff_ang_norm) - (target_v_ang_norm * dt), 0.0);
  const double scale_lin = (step_end_lin + numeric_epsilon) / (diff_lin_norm + numeric_epsilon);
  const double scale_ang = (step_end_ang + numeric_epsilon) / (diff_ang_norm + numeric_epsilon);
  const double scale_joint =
      (joint_velocity_limit_.array() * dt / (control_v_.cwiseAbs().array() + numeric_epsilon * dt))
          .minCoeff();  // epsilon margin keeps the result strictly below the limit
  control_v_ *= std::fmin(scale_lin, std::fmin(scale_ang, scale_joint)) / dt;  // most conservative scale
  assert((control_v_.cwiseAbs().array() <= joint_velocity_limit_.array() + numeric_epsilon).all() &&  //
         "control_v_ exceeds a joint's velocity limit");

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

  if (verbose || ik_unsolved) {
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

void CartesianPoseController::read_states()
{
  auto interface_iter = state_interfaces_.begin();
  for (auto& state : state_q_) {
    state = duatic::controllers::compat::require_value(*interface_iter);
    interface_iter++;
  }
  if (params_->velocity_feedback > 0.0) {
    for (auto& state : state_v_) {
      state = duatic::controllers::compat::require_value(*interface_iter);
      interface_iter++;
    }
  } else {
    state_v_ = control_v_;
  }
  assert(interface_iter == state_interfaces_.end() && "Not all state interfaces were read");
}

void CartesianPoseController::blend_states(const double velocity_feedback_weight)
{
  // blend position state_q_ with feedforward control_q_:
  // use capped feedforward so no feedforward joint moves further from its state than velocity_limit * motion_horizon
  blend_delta_q_ = control_q_ - state_q_;
  const double ratio_q = blend_delta_q_.cwiseAbs().cwiseQuotient(joint_velocity_box_).maxCoeff();
  const double weight_q = (ratio_q > 1.0) ? (1.0 / ratio_q) : 1.0;
  state_q_ += weight_q * blend_delta_q_;

  // blend velocity state_v_ with feedforward control-based velocities
  state_v_ = control_v_ + velocity_feedback_weight * (state_v_ - control_v_);
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
                         .cwiseMax(-joint_velocity_box_)  // lower bound
                         .cwiseMin(+numeric_epsilon);     // upper bound
  qp_solver_u_box_ = ((robot_model_.upperPositionLimit - state_q_) * problem_scale)
                         .cwiseMin(joint_velocity_box_)  // upper bound
                         .cwiseMax(-numeric_epsilon);    // lower bound
  assert((qp_solver_l_box_.array() <= qp_solver_u_box_.array()).all() && "QP box bounds are inconsistent");

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
  for (const auto& control : control_q_) {
    if (!command_itr->set_value(control)) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command");
    }
    command_itr++;
  }
  if (params_->command_velocities) {
    for (const auto& control : control_v_) {
      if (!command_itr->set_value(control)) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set velocity command");
      }
      command_itr++;
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
