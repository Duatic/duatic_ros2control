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
#include <algorithm>

// Other headers
#include <controller_interface/controller_interface.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "controller_interface/chainable_controller_interface.hpp"

// ROS2
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Project
#include <duatic_controllers/brake_release_controller_parameters.hpp>
#include <duatic_controllers/interface_utils.hpp>

namespace duatic::controllers
{
class BrakeReleaseController : public controller_interface::ControllerInterface
{
public:
  enum class State
  {
    Init,
    Commanding,
    Holding
  };

  enum class Mode
  {
    Auto,
    Manual,
  };

  BrakeReleaseController();
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
  // Per drive state of the torque sweep in mode "auto", the drives are named like the joints
  struct JointProbe
  {
    // Drive output torque in [Nm] added on top of the gravity compensation
    double offset{};
    // Index of the sweep level the offset moves towards, the search is over once it is past the last level
    std::size_t level{ 0 };
    // Position loop gains of the drive before the search, restored when the joint is handed back
    double p_gain{};
    double i_gain{};
  };

  // Access to controller parameters via generate_parameter_library
  std::unique_ptr<brake_release_controller::ParamListener> param_listener_;
  brake_release_controller::Params params_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_p_gain_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_i_gain_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> effort_reference_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> brake_target_command_interfaces_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> brake_released_state_interfaces_;

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Time last_status_log_time_;
  std::vector<JointProbe> probes_;
  std::vector<double> joint_velocities_;

  /**
   * @brief whether every drive reports its brake as released
   */
  bool all_brakes_released() const;
  /**
   * @brief comma separated names of the joints whose brakes are not released yet
   */
  std::string unreleased_joints() const;
  /**
   * @brief command all brakes into the holding state
   */
  void hold_brakes();
  /**
   * @brief command a joint position target at zero velocity
   */
  void command_joint(std::size_t index, double position);
  /**
   * @brief command the joint torques that put the torque offset of each drive on its motor
   */
  void command_offsets();
  /**
   * @brief velocity of a drive in [rad/s] in drive coordinates, from the last read joint velocities
   */
  double drive_velocity(std::size_t index) const;
  /**
   * @brief give a drive back to its position loop without a torque offset
   */
  void hand_back(std::size_t index);
  /**
   * @brief the configured joints as the gravity compensation prefixes their effort references
   */
  std::vector<std::string> reference_joint_names() const;
  /**
   * @brief ask the controller manager to deactivate this controller
   */
  void request_deactivation();

  State current_state_{ State::Init };
  std::optional<rclcpp::Time> last_state_change_time_;
  Mode mode_{ Mode::Auto };

  /**
   * @brief convert the mode argument in the parameter file into a better manageable Mode enum
   */
  Mode mode_from_str(const std::string& str)
  {
    if (str == "auto") {
      return Mode::Auto;
    } else if (str == "manual") {
      return Mode::Manual;
    }
    throw std::invalid_argument("Invalid mode: '" + str + "'");
  }
};

}  // namespace duatic::controllers
