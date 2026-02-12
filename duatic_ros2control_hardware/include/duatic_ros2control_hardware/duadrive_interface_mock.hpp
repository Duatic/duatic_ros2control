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

// ros2_control hardware_interface
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"

// Local
#include "duatic_ros2control_hardware/duadrive_utils.hpp"
#include "duatic_ros2control_hardware/duadrive_interface_base.hpp"

namespace duatic_ros2control_hardware
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
  DuaDriveInterfaceMock(rclcpp::Logger logger);
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
  hardware_interface::return_type read() final;
  /**
   * @brief perform a single write on the drive component
   */
  hardware_interface::return_type write() final;

private:
};
}  // namespace duatic_ros2control_hardware
