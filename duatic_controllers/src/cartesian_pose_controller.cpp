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

#include <duatic_controllers/cartesian_pose_controller.hpp>

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

std::optional<CartesianPoseController::IKResult> CartesianPoseController::compute_ik(
    const pinocchio::Model& model, pinocchio::Data& data, const pinocchio::SE3& target_pose,
    const pinocchio::FrameIndex target_pose_frame_id, const Eigen::VectorXd& q_in, rclcpp::Logger logger)
{
  // Update once
  pinocchio::forwardKinematics(model, data, q_in);
  pinocchio::updateFramePlacement(model, data, target_pose_frame_id);
  const pinocchio::SE3 desired_pose_world = data.oMf[target_pose_frame_id] * target_pose;

  Eigen::VectorXd q_out = q_in;

  // Tunable parameters
  const double eps = 1e-5;  // Convergence threshold
  const int IT_MAX = 4000;  // Max iterations
  const double DT = 5e-2;   // Step size
  const double w_rot = 0.5;
  const double w_trans = 1.0;
  const double w_q = 0.1;

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  Eigen::VectorXd v(model.nv);
  Eigen::Matrix<double, 6, 1> err;
  bool success = false;

  for (int i = 0; i < IT_MAX; i++) {
    // Update kinematics
    pinocchio::forwardKinematics(model, data, q_out);
    pinocchio::updateFramePlacement(model, data, target_pose_frame_id);

    // Compute error (target_pose vs current end-effector pose)
    const pinocchio::SE3 dMi = desired_pose_world.actInv(data.oMf[target_pose_frame_id]);
    err = pinocchio::log6(dMi).toVector();
    err.head<3>() *= w_rot;
    err.tail<3>() *= w_trans;
    // Log error for debugging
    /*if (i % 100 == 0) {
      RCLCPP_INFO_STREAM(logger, "q " << q_out.transpose());
      RCLCPP_INFO_STREAM(logger, "error" << err.transpose());
      RCLCPP_INFO(logger, "Iteration %d: error norm = %f", i, err.norm());
    }*/

    // Check convergence
    if (err.norm() < eps) {
      success = true;
      RCLCPP_INFO_STREAM(logger, "Converged after: " << i << " iteration with error: " << err.norm());
      break;
    }

    // Compute Jacobian at the end-effector
    pinocchio::computeFrameJacobian(model, data, q_out, target_pose_frame_id, pinocchio::LOCAL, J);

    // Check if Jacobian is ill-conditioned
    if (J.isZero(1e-10)) {
      RCLCPP_ERROR(logger, "Jacobian is ill-conditioned at iteration %d.", i);
      return std::nullopt;
    }
    // SVD solver
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

    auto U = svd.matrixU();
    auto V = svd.matrixV();
    auto S = svd.singularValues();

    // Dynamic lambda
    double sigma_max = S(0);
    double sigma_min = S(S.size() - 1);

    if (sigma_min < 1e-4)  // near singularity
      std::cout << "sigma_min: " << sigma_min << std::endl;

    double kappa = sigma_max / (sigma_min + 1e-12);

    double lambda = 1e-3 * kappa;
    lambda = std::clamp(lambda, 1e-4, 1e-1);

    Eigen::VectorXd Sinv = S;
    for (int j = 0; j < S.size(); j++) {
      Sinv(j) = S(j) / (S(j) * S(j) + lambda * lambda);
    }
    v = -V * Sinv.asDiagonal() * U.transpose() * err;

    Eigen::MatrixXd J_pinv = V * Sinv.asDiagonal() * U.transpose();
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(model.nv, model.nv) - J_pinv * J;

    Eigen::VectorXd q_err = q_in - q_out;
    v += w_q * N * q_err;

    // Update joint configuration
    q_out = pinocchio::integrate(model, q_out, v * DT);
  }
  success = true;
  if (success) {
    RCLCPP_INFO_STREAM(logger, err.norm());
    CartesianPoseController::IKResult result;
    result.q_out = q_out;
    return result;
  } else {
    RCLCPP_ERROR_STREAM(logger, "Inverse kinematics did not converge within: " << IT_MAX
                                                                               << " iterations. Error: " << err.norm());
    return std::nullopt;
  }
}
static pinocchio::SE3 ROS_pose_to_SE3(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
  return pinocchio::SE3(q.normalized(), t);
}

static std::optional<pinocchio::SE3> transform_pose_to_model_base_frame(
    const std::string& frame_id, const pinocchio::FrameIndex& base_frame_idx, const pinocchio::SE3& target_pose,
    const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q_current, rclcpp::Logger logger)
{
  // If no frame_id specified or it's "world", assume pose is already in model base frame
  if (frame_id.empty() || frame_id == "world") {
    return target_pose;
  }

  // Get the transformation from the target frame to the model base frame
  pinocchio::FrameIndex target_frame_id;
  try {
    target_frame_id = model.getFrameId(frame_id);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Frame '%s' not found in Pinocchio model: %s", frame_id.c_str(), e.what());
    // Return original pose with failure flag
    return std::nullopt;
  }

  // Compute forward kinematics to get the transformation
  pinocchio::forwardKinematics(model, data, q_current);
  pinocchio::updateFramePlacements(model, data);

  const pinocchio::SE3& world_T_base = data.oMf[base_frame_idx];
  const pinocchio::SE3& world_T_ref = data.oMf[target_frame_id];

  // Get the pose of the target frame in model base frame
  pinocchio::SE3 base_T_ref = world_T_base.inverse() * world_T_ref;
  pinocchio::SE3 base_T_target = base_T_ref * target_pose;

  return base_T_target;
}
std::optional<CartesianPoseController::IKResult>
CartesianPoseController::run_ik(const geometry_msgs::msg::PoseStamped& msg)
{
  const auto start_time = std::chrono::system_clock::now();
  // Run IK in the non rt thread
  const auto pose_for_ik = transform_pose_to_model_base_frame(
      msg.header.frame_id, endeffector_frame_id_, ROS_pose_to_SE3(msg.pose), pinocchio_model_, pinocchio_data_,
      last_system_state_->q, get_node()->get_logger());

  if (!pose_for_ik) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to transform pose to IK chain base frame");
    return std::nullopt;
  }

  const auto ik_result = compute_ik(pinocchio_model_, pinocchio_data_, pose_for_ik.value(), endeffector_frame_id_,
                                    last_system_state_->q, get_node()->get_logger());
  const auto ik_duration = std::chrono::system_clock::now() - start_time;
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "IK took: " << std::chrono::duration_cast<std::chrono::milliseconds>(ik_duration));
  return ik_result;
}
std::optional<pinocchio::SE3> CartesianPoseController::get_current_ee_pose()
{
  // Compute forward kinematics to get the transformation
  pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, last_system_state_->q);
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);

  const pinocchio::SE3& oM_ee = pinocchio_data_.oMf[endeffector_frame_id_];
  const pinocchio::SE3& oM_ref = pinocchio_data_.oMf[base_frame_id_];
  pinocchio::SE3 refM_ee = oM_ref.inverse() * oM_ee;

  return refM_ee;
}

CartesianPoseController::CartesianPoseController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration CartesianPoseController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration CartesianPoseController::state_interface_configuration() const
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

  // Start IK worker thread
  // IK worker will be started in on_activate after Pinocchio model is built

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
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
    pinocchio::urdf::buildModelFromXML(get_robot_description(), pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model built with %zu joints", pinocchio_model_.joints.size() - 1);

    // 2. Build the collision model from urdf and srdf (only if SRDF is provided)
    if (!params_.srdf.empty()) {
      RCLCPP_INFO(get_node()->get_logger(), "Building collision geometry...");
      std::stringstream urdf_stream;
      urdf_stream << get_robot_description();
      pinocchio::urdf::buildGeom(pinocchio_model_, urdf_stream, pinocchio::COLLISION, pinocchio_geom_);

      pinocchio_geom_.addAllCollisionPairs();
      pinocchio::srdf::removeCollisionPairsFromXML(pinocchio_model_, pinocchio_geom_, params_.srdf);
      RCLCPP_INFO(get_node()->get_logger(), "Collision geometry built with %zu collision pairs",
                  pinocchio_geom_.collisionPairs.size());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "No SRDF provided - collision checking will be disabled");
    }

    // 3. Validate that all controller joints exist in the Pinocchio model
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

    // 4. Validate kinematic chain structure (only if more than one joint)
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

    // 5. Validate end effector frame exists
    if (!pinocchio_model_.existFrame(params_.end_effector_frame)) {
      RCLCPP_ERROR(get_node()->get_logger(), "End effector frame '%s' not found in Pinocchio model.",
                   params_.end_effector_frame.c_str());

      // Debug: List all available frames
      RCLCPP_ERROR(get_node()->get_logger(), "Available frames in Pinocchio model:");
      for (size_t i = 0; i < pinocchio_model_.frames.size(); ++i) {
        RCLCPP_ERROR(get_node()->get_logger(), "  [%zu]: %s", i, pinocchio_model_.frames[i].name.c_str());
      }
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(),
                "Successfully configured controller with %zu joints and end effector frame '%s'", params_.joints.size(),
                params_.end_effector_frame.c_str());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during Pinocchio model setup: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Build joint index cache
  joint_indices_.clear();
  for (const auto& joint : params_.joints) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Joint: " << joint << " pinnochio id: " << pinocchio_model_.getJointId(joint));
    joint_indices_.push_back(pinocchio_model_.getJointId(joint));
  }

  for (auto& val : joint_indices_) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), val);
  }
  // Cache end effector frame id:
  endeffector_frame_id_ = pinocchio_model_.getFrameId(params_.end_effector_frame);
  base_frame_id_ = pinocchio_model_.getFrameId(params_.base_frame);

  // Setup the pose listener
  pose_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", 10, [&](const geometry_msgs::msg::PoseStamped& msg) {
        // Write raw to RT buffer for real-time consumers
        buffer_pose_cmd_.writeFromNonRT(msg);
        const auto ik_result = run_ik(msg);
        if (!ik_result) {
          RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to solve IK");
          return;
        }
        next_target_state_ = PinocchioState{ .q = ik_result->q_out };
      });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = true;

  // clear out vectors in case of restart
  joint_position_command_interfaces_.clear();
  joint_velocity_command_interfaces_.clear();

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
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints,
                                                    hardware_interface::HW_IF_VELOCITY,
                                                    joint_velocity_command_interfaces_)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not get ordered command interfaces - position");
    return controller_interface::CallbackReturn::FAILURE;
  }

  last_system_state_ = build_current_state();
  if (!last_system_state_) {
    RCLCPP_FATAL_STREAM(get_node()->get_logger(), "Could not obtain the full current system state during activation - "
                                                  "this is fatal, aborting");
    return controller_interface::CallbackReturn::FAILURE;
  }
  next_target_state_ = last_system_state_;

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial system state: " << last_system_state_.value());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

std::optional<CartesianPoseController::PinocchioState> CartesianPoseController::build_current_state()
{
  const std::size_t joint_count = joint_position_state_interfaces_.size();

  // Build full-size vectors for all robot joints (Pinocchio expects this)
  PinocchioState state{ .q = Eigen::VectorXd::Zero(pinocchio_model_.nq),
                        .v = Eigen::VectorXd::Zero(pinocchio_model_.nv),
                        .a = Eigen::VectorXd::Zero(pinocchio_model_.nv) };

  // Map: Pinocchio joint name -> index in q/v
  for (std::size_t i = 0; i < joint_count; i++) {
    const std::string& joint_name = params_.joints[i];
    const auto idx = joint_indices_[i];

    try {
      state.q[pinocchio_model_.joints[idx].idx_q()] =
          duatic::controllers::compat::require_value(joint_position_state_interfaces_.at(i).get());

      state.v[pinocchio_model_.joints[idx].idx_v()] =
          duatic::controllers::compat::require_value(joint_velocity_state_interfaces_.at(i).get());

      state.a[pinocchio_model_.joints[idx].idx_v()] =
          duatic::controllers::compat::require_value(joint_acceleration_state_interfaces_.at(i).get());
    } catch (const duatic::controllers::exceptions::MissingInterfaceValue& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read state for joint '%s': %s", joint_name.c_str(), e.what());
      return std::nullopt;
    }
  }
  return state;
}

controller_interface::return_type CartesianPoseController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                  [[maybe_unused]] const rclcpp::Duration& period)
{
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE || !active_) {
    return controller_interface::return_type::OK;
  }

  last_system_state_ = build_current_state();
  if (!last_system_state_) {
    RCLCPP_FATAL_STREAM(get_node()->get_logger(), "Could not obtain the full current system state during "
                                                  "operation...wtf");
    return controller_interface::return_type::ERROR;
  }

  if (next_target_state_) {
    for (std::size_t i = 0; i < params_.joints.size(); i++) {
      const std::string& joint_name = params_.joints[i];
      const auto idx = joint_indices_[i];
      if (!joint_position_command_interfaces_.at(i).get().set_value<double>(
              next_target_state_->q[pinocchio_model_.joints[idx].idx_q()])) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command for joint '%s'", joint_name.c_str());
      }
    }
  }
  return controller_interface::return_type::OK;
}
controller_interface::CallbackReturn
CartesianPoseController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_error([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianPoseController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace duatic::controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(duatic::controllers::CartesianPoseController, controller_interface::ControllerInterface)
