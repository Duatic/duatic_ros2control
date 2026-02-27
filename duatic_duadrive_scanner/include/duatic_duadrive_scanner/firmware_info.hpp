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

#include <cstdint>
#include <string>

/**
 * This namespace contains types that are mirrored from the firmware
 */
namespace duadrive_scanner::firmware
{

enum class ObjectDictionary : uint16_t
{
  DriveModel = 0x1008,
  DriveName = 0x7073,
  Capabilities = 0x7079,
  BuildInfo = 0x7078,
  DriveType = 0x7077,
};

struct Capabilities
{
  uint32_t experimental : 1;
  uint32_t async_spi : 1;
  uint32_t async_i2c : 1;
  uint32_t watchdog : 1;
  uint32_t shunt_05 : 1;
  uint32_t timing_measurements : 1;
  uint32_t fieldweakening : 1;
};
struct BuildInfo
{
  std::string build_date = "";
  std::string git_tag = "";
  std::string git_hash = "";
  Capabilities capabilities;
};
struct DriveInfo
{
  std::string drive_type = "";
  std::string drive_model = "";
  std::string drive_name = "";
};

}  // namespace duadrive_scanner::firmware
