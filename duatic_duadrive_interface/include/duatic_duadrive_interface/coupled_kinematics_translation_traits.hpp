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
#include <span>

#include "duatic_duadrive_interface/coupled_kinematics_types.hpp"

namespace duatic::duadrive_interface::kinematics
{
// Concept every class that wants to act as KinematicsTranslator needs to fulfill
template <typename T>
concept KinematicsTranslator = requires(std::span<const CoupledJointState> cjs_in, std::span<SerialJointState> sjs_out,
                                        std::span<const SerialJointState> sjs_in, std::span<CoupledJointState> cjs_out,
                                        std::span<const CoupledCommand> cc_in, std::span<SerialCommand> sc_out,
                                        std::span<const SerialCommand> sc_in, std::span<CoupledCommand> cc_out)
{
  // JointState mappings
  {
    T::map_from_coupled_to_serial(cjs_in, sjs_out)
    } -> std::same_as<void>;
  {
    T::map_from_serial_to_coupled(sjs_in, cjs_out)
    } -> std::same_as<void>;

  // Command mappings
  {
    T::map_from_coupled_to_serial(cc_in, sc_out)
    } -> std::same_as<void>;
  {
    T::map_from_serial_to_coupled(sc_in, cc_out)
    } -> std::same_as<void>;
};

}  // namespace duatic::duadrive_interface::kinematics
