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

#include <string>
#include <filesystem>
#include <vector>
#include <fstream>
#include <iterator>

#include "duatic_duadrive_scanner/ethercat_utils.hpp"

namespace duadrive_scanner
{

class Scanner
{
public:
  explicit Scanner(const std::string& interface) : interface_(interface)
  {
  }
  virtual ~Scanner()
  {
    // Make sure to close the connection at exit. Otherwise we might start accumulating open handles
    close();
  }

  /**
   * @brief Initialize the ethercat master on the configured bus interface
   */
  void connect();

  bool is_connected() const
  {
    return connected_;
  }
  int device_count() const
  {
    if (!is_connected())
      throw std::runtime_error("not connected to the bus");
    return ecat_context.ecatSlavecount_;
  }
  void close()
  {
    ecx_close(&ecat_context.context);
    connected_ = false;
  }

  bool is_duadrive(const uint16_t device_id);

  /**
   * @brief Read build information about the firmware from the given device
   */
  firmware::BuildInfo get_firmware_build_info(const uint16_t device_id);
  /**
   * @brief Read drive information from the given device
   */
  firmware::DriveInfo get_firmware_drive_info(const uint16_t device_id);

  const std::string& getInterface() const
  {
    return interface_;
  }

  std::string get_device_name(const uint16_t device_id);

private:
  std::string interface_{};
  ethercat::EthercatContext ecat_context;
  bool connected_{ false };
};

}  // namespace duadrive_scanner
