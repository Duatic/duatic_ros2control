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
#include <stdexcept>

#include "duatic_duadrive_interface/coupled_kinematics_invariant_translator.hpp"

namespace duatic::duadrive_interface::kinematics
{
void InvariantKinematicsTranslator::map_from_coupled_to_serial(std::span<const CoupledJointState> in,
                                                               std::span<SerialJointState> out)
{
  if (in.size() != out.size()) {
    throw std::runtime_error("In/Out sizes do not match");
  }
  for (std::size_t i = 0; i < in.size(); i++) {
    out[i].position = in[i].position;
    out[i].position_commanded = in[i].position_commanded;
    out[i].velocity = in[i].velocity;
    out[i].velocity_commanded = in[i].velocity_commanded;
    out[i].acceleration = in[i].acceleration;
    out[i].acceleration_commanded = in[i].acceleration_commanded;
    out[i].torque = in[i].torque;
    out[i].torque_commanded = in[i].torque_commanded;
  }
}

void InvariantKinematicsTranslator::map_from_serial_to_coupled(std::span<const SerialJointState> in,
                                                               std::span<CoupledJointState> out)
{
  if (in.size() != out.size()) {
    throw std::runtime_error("In/Out sizes do not match");
  }
  for (std::size_t i = 0; i < in.size(); i++) {
    out[i].position = in[i].position;
    out[i].position_commanded = in[i].position_commanded;
    out[i].velocity = in[i].velocity;
    out[i].velocity_commanded = in[i].velocity_commanded;
    out[i].acceleration = in[i].acceleration;
    out[i].acceleration_commanded = in[i].acceleration_commanded;
    out[i].torque = in[i].torque;
    out[i].torque_commanded = in[i].torque_commanded;
  }
}

void InvariantKinematicsTranslator::map_from_coupled_to_serial(std::span<const CoupledCommand> in,
                                                               std::span<SerialCommand> out)
{
  if (in.size() != out.size()) {
    throw std::runtime_error("In/Out sizes do not match");
  }
  for (std::size_t i = 0; i < in.size(); i++) {
    out[i].position = in[i].position;
    out[i].velocity = in[i].velocity;
    out[i].acceleration = in[i].acceleration;
    out[i].torque = in[i].torque;
  }
}

void InvariantKinematicsTranslator::map_from_serial_to_coupled(std::span<const SerialCommand> in,
                                                               std::span<CoupledCommand> out)
{
  if (in.size() != out.size()) {
    throw std::runtime_error("In/Out sizes do not match");
  }
  for (std::size_t i = 0; i < in.size(); i++) {
    out[i].position = in[i].position;
    out[i].velocity = in[i].velocity;
    out[i].acceleration = in[i].acceleration;
    out[i].torque = in[i].torque;
  }
}
}  // namespace duatic::duadrive_interface::kinematics
