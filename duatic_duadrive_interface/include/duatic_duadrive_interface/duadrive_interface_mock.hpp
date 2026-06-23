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
 * products derived from this software without  hardware_interface::CallbackReturn activate();t specific prior written
 * permission.
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

// System
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <type_traits>

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"

// Local
#include "duatic_duadrive_interface/duadrive_utils.hpp"
#include "duatic_duadrive_interface/duadrive_interface_base.hpp"

namespace duatic::duadrive_interface
{

/**
 * @brief the DuaDriveInterface class intends to make a single Duatic DuaDrive easier to use within ros2control hardware
 * interfaces
 *
 * It manages internally a single instance of the SDK drive object.
 * The ethercat bus is always handled asynchronously in the background
 */
class DuaDriveInterfaceMock : public DuaDriveInterfaceBase
{
public:
  static constexpr double default_abs_velocity_limit = 2.0 * M_2_PI;// i.e. 2 rotations per second

  explicit DuaDriveInterfaceMock(rclcpp::Logger logger);
  virtual ~DuaDriveInterfaceMock();
  using UniquePtr = std::unique_ptr<DuaDriveInterfaceMock>;
  /**
   * @brief perform initialization of the duadrive interface component
   */
  hardware_interface::CallbackReturn init(const DuaDriveInterfaceParameters& params) final;
  /**
   * @brief perform the activation procedure of this drive component
   * @note this will perform a read internally trying to obtain the current position values
   */
  hardware_interface::CallbackReturn activate() final;
  /**
   * @brief perform the configuration procedure of this drive component
   * This will simply attach the drive representation to the ethercat bus
   */
  hardware_interface::CallbackReturn configure() final;
  /**
   * @brief perform the deactivation procedure of this drive component
   * This will try to put the drive into freeze mode
   */
  hardware_interface::CallbackReturn deactivate() final;
  /**
   * @brief perform a single read on the drive component
   */
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  /**
   * @brief perform a single write on the drive component
   */
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  
  /**
   * @brief Allow in the mocked hardware to explicitly enforce a specific position (e.g. at startup)
   */
  void enforce_position(const double position);
  /**
   * @brief Allow in the mocked hardware to explicitly enforce positive and negative velocity limits (e.g. during dynamic simulation)
   */
  void limit_velocity(const double max_velocity, const double min_velocity);
  /**
   * @brief Allow in the mocked hardware to explicitly enforce equal velocity limits (e.g. during dynamic simulation)
   */
  void limit_velocity(const double max_abs_velocity) {
    limit_velocity(max_abs_velocity, -max_abs_velocity);
  }
  /**
   * @brief Register a mock dynamics model with the mock hardware
   */
  void register_mock_dynamics(const std::function<double()>& acceleration_callback)
  {
    mock_acceleration_callback_ = acceleration_callback;
  }

private:
  double min_velocity_;
  double max_velocity_;
  std::function<double()> mock_acceleration_callback_;
};

/*
 * type trait to simplify duadrive mock interface request
 */
template <typename T>
constexpr inline bool is_dua_drive_interface_mock_v = std::is_base_of_v<DuaDriveInterfaceMock, T>;

}  // namespace duatic::duadrive_interface
