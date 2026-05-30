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

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>

#include <duatic_controllers/interface_utils.hpp>
#include <duatic_controllers/pin_release_controller_parameters.hpp>

/**
 * @file pin_release_controller.hpp
 * @brief Declares the ROS 2 control controller used to release and hold brake pins.
 */

namespace duatic::controllers
{
/**
 * @brief Controller that releases brake pins for a configured duration and then holds them.
 *
 * The controller claims the `target_brake_state` command interface for each configured joint.
 * After activation, it commands the pins to the releasing state until `hold_timeout` seconds
 * have elapsed, then switches the command to the holding state.
 */
class PinReleaseController : public controller_interface::ControllerInterface
{
public:
  /**
   * @brief Internal command state of the pin release sequence.
   */
  enum class State
  {
    /// Command pins into the released state.
    Releasing,
    /// Command pins into the holding state.
    Holding
  };

  /**
   * @brief Construct a pin release controller instance.
   */
  PinReleaseController();

  /**
   * @brief Return the command interfaces required by the controller.
   * @return Individual `target_brake_state` command interfaces for all configured joints.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Return the state interfaces required by the controller.
   * @return No state interfaces are required.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Run one control update of the release-and-hold state machine.
   * @param time Current controller manager time.
   * @param period Elapsed time since the previous update.
   * @return `OK` after writing the current brake pin command.
   */
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * @brief Initialize generated parameter handling.
   * @return `SUCCESS` if parameters are available, otherwise `ERROR`.
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Refresh and validate controller parameters.
   * @param previous_state Lifecycle state before configure was requested.
   * @return `SUCCESS` when at least one joint is configured, otherwise `FAILURE`.
   */
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Claim ordered brake pin command interfaces and reset the release sequence.
   * @param previous_state Lifecycle state before activation was requested.
   * @return `SUCCESS` when all command interfaces are available, otherwise `FAILURE`.
   */
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Handle lifecycle deactivation.
   * @param previous_state Lifecycle state before deactivation was requested.
   * @return `SUCCESS`.
   */
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Handle lifecycle cleanup.
   * @param previous_state Lifecycle state before cleanup was requested.
   * @return `SUCCESS`.
   */
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Handle lifecycle error processing.
   * @param previous_state Lifecycle state before error processing was requested.
   * @return `SUCCESS`.
   */
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Handle lifecycle shutdown.
   * @param previous_state Lifecycle state before shutdown was requested.
   * @return `SUCCESS`.
   */
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
  /// Generated-parameter-library listener for controller parameters.
  std::unique_ptr<pin_release_controller::ParamListener> param_listener_;
  /// Cached controller parameters.
  pin_release_controller::Params params_;

  /// Ordered `target_brake_state` command interfaces for the configured joints.
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> brake_target_command_interfaces_;

  /// Current release sequence state.
  State current_state_{ State::Releasing };
  /// Controller time at which the current activation began.
  std::optional<rclcpp::Time> activate_time_;
};

}  // namespace duatic::controllers
