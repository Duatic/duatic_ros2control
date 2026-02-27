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

#include <optional>
#include <string>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>
#include "duatic_duadrive_scanner/ethercat_context.hpp"
#include "duatic_duadrive_scanner/firmware_info.hpp"
namespace duadrive_scanner::ethercat
{

template <typename T>
// TODO(firesurfer) define concept for allowed types
inline std::optional<T> read_sdo(EthercatContext& context, const uint16_t device_id, const uint16_t index,
                                 const uint8_t sub_index = 0, const bool complete_access = false)
{
  static_assert(!std::is_same_v<T, std::string>);

  if (static_cast<int>(device_id) > context.ecatSlavecount_) {
    throw std::runtime_error("device not found on bus");
  }

  const int requested_size = sizeof(T);
  int actual_size = requested_size;
  T data{};

  const int wkc = ecx_SDOread(&context.context, device_id, index, sub_index, static_cast<boolean>(complete_access),
                              &actual_size, reinterpret_cast<void*>(&data), EC_TIMEOUTRXM);

  if (wkc <= 0) {
    std::cerr << "Device id " << device_id << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x"
              << std::setfill('0') << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0')
              << std::setw(2) << std::hex << static_cast<uint16_t>(sub_index) << ")." << std::endl;
    return std::nullopt;
  }

  if (requested_size != actual_size) {
    std::cerr << "Device id  " << device_id << ": Size mismatch (expected " << requested_size << " bytes, read "
              << actual_size << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex
              << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
              << static_cast<uint16_t>(sub_index) << ")." << std::endl;
    return std::nullopt;
  }
  return data;
}

template <>
inline std::optional<std::string> read_sdo(EthercatContext& context, const uint16_t device_id, const uint16_t index,
                                           const uint8_t sub_index, [[maybe_unused]] const bool complete_access)
{
  if (device_id > context.ecatSlavecount_) {
    std::cerr << "Slave ID (" << device_id << ") is not valid!" << std::endl;

    return std::nullopt;
  }

  // Read char casted as an uint32 array.

  std::vector<char> buffer(128, 0);
  int size = buffer.size();

  int wkc_ = ecx_SDOread(&context.context, device_id, index, sub_index, 0u, &size, buffer.data(), EC_TIMEOUTRXM);

  if (wkc_ != 1) {
    std::cerr << "Working counter (" << wkc_ << ") too low. - for actual string read" << std::endl;

    return std::nullopt;
  }

  auto end = buffer.begin();
  std::advance(end, size);
  std::string str(buffer.begin(), end);
  return str;
}

inline std::optional<firmware::BuildInfo> read_build_info(EthercatContext& context, uint16_t device_id)
{
  firmware::BuildInfo info;

  const auto build_date = read_sdo<std::string>(
      context, device_id, static_cast<uint16_t>(firmware::ObjectDictionary::BuildInfo), 0x01, false);
  const auto git_tag = read_sdo<std::string>(context, device_id,
                                             static_cast<uint16_t>(firmware::ObjectDictionary::BuildInfo), 0x02, false);
  const auto git_hash = read_sdo<std::string>(
      context, device_id, static_cast<uint16_t>(firmware::ObjectDictionary::BuildInfo), 0x03, false);
  auto capabilities_raw = read_sdo<uint32_t>(
      context, device_id, static_cast<uint16_t>(firmware::ObjectDictionary::Capabilities), 0x00, false);

  if (!build_date || !git_tag || !git_hash || !capabilities_raw) {
    return std::nullopt;
  }

  info.build_date = build_date.value();
  info.git_tag = git_tag.value();
  info.git_hash = git_hash.value();
  info.capabilities = *reinterpret_cast<firmware::Capabilities*>(&capabilities_raw.value());
  return info;
}

inline std::optional<firmware::DriveInfo> read_drive_info(EthercatContext& context, uint16_t device_id)
{
  const auto drive_type =
      read_sdo<std::string>(context, device_id, static_cast<uint16_t>(firmware::ObjectDictionary::DriveType), 0x01);

  const auto drive_model =
      read_sdo<std::string>(context, device_id, static_cast<uint16_t>(firmware::ObjectDictionary::DriveModel), 0x0);
  const auto drive_name =
      read_sdo<std::string>(context, device_id, static_cast<uint16_t>(firmware::ObjectDictionary::DriveName), 0x01);

  if (!drive_type || !drive_model || !drive_name) {
    return std::nullopt;
  }

  return firmware::DriveInfo{ .drive_type = drive_type.value(),
                              .drive_model = drive_model.value(),
                              .drive_name = drive_name.value() };
}

inline std::optional<int> initialize_devices(EthercatContext& context)
{
  const int device_count = ecx_config_init(&context.context, FALSE);
  if (device_count <= 0) {
    std::cerr << "No devices found in the bus" << std::endl;
    return std::nullopt;
  }
  return device_count;
}

inline bool has_device(EthercatContext& context, const uint16_t device_id)
{
  return device_id <= context.ecatSlavecount_;
}

inline std::optional<std::string> read_device_name(EthercatContext& context, const uint16_t device_id)
{
  return std::string(context.ecatSlavelist_[device_id].name);
}
inline bool switch_to_state(EthercatContext& context, const uint16_t device_id, const ec_state state)
{
  context.ecatSlavelist_[device_id].state = state;
  ecx_writestate(&context.context, device_id);
  return ecx_statecheck(&context.context, device_id, state, EC_TIMEOUTSTATE * 4) == state;
}
inline bool switch_to_bootstate(EthercatContext& context, const uint16_t device_id)
{
  if (!switch_to_state(context, device_id, EC_STATE_INIT)) {
    std::cerr << "Could not set device '" << device_id << "' to EC_STATE_INIT state." << std::endl;
    return false;
  }

  if (!switch_to_state(context, device_id, EC_STATE_BOOT)) {
    std::cerr << "Could not set device '" << device_id << "' to EC_STATE_BOOT state." << std::endl;
    return false;
  }
  return true;
}

inline std::vector<std::string> list_adapters()
{
  ec_adaptert* adapter = nullptr;

  adapter = ec_find_adapters();
  if (!adapter)
    throw std::runtime_error("Error calling ec_find_adapters");

  std::vector<std::string> result;

  auto iter = adapter;
  while (iter != nullptr) {
    result.push_back(iter->name);
    iter = iter->next;
  }

  ec_free_adapters(adapter);
  return result;
}

}  // namespace duadrive_scanner::ethercat
