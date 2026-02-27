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

#include "duatic_duadrive_scanner/Scanner.hpp"
#include <thread>
namespace duadrive_scanner
{

void Scanner::connect()
{
  if (ecx_init(&ecat_context.context, interface_.c_str()) <= 0) {
    throw std::runtime_error("Failed to open interface: " + interface_ + " Run as root!");
  }

  if (!ethercat::initialize_devices(ecat_context)) {
    throw std::runtime_error("No devices found on the bus");
  }
  connected_ = true;
}

firmware::BuildInfo Scanner::get_firmware_build_info(const uint16_t device_id)
{
  if (!ethercat::has_device(ecat_context, device_id))
    throw std::runtime_error("Device: " + std::to_string(device_id) + " not found");

  const auto build_info = ethercat::read_build_info(ecat_context, device_id);
  if (!build_info)
    throw std::runtime_error("Could not read build info from device: " + std::to_string(device_id));

  return build_info.value();
}

firmware::DriveInfo Scanner::get_firmware_drive_info(const uint16_t device_id)
{
  if (!ethercat::has_device(ecat_context, device_id))
    throw std::runtime_error("Device: " + std::to_string(device_id) + " not found");
  const auto drive_info = ethercat::read_drive_info(ecat_context, device_id);

  if (!drive_info)
    throw std::runtime_error("Could not read drive info from device: " + std::to_string(device_id));

  return drive_info.value();
}
std::string Scanner::get_device_name(const uint16_t device_id)
{
  if (!ethercat::has_device(ecat_context, device_id))
    throw std::runtime_error("Device: " + std::to_string(device_id) + " not found");

  return ethercat::read_device_name(ecat_context, device_id).value();
}

}  // namespace duadrive_scanner
