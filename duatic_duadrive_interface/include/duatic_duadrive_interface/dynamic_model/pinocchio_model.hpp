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

#include <tuple>
#include <vector>

#include <urdf/model.h> // NOLINT(build/include_order)
#include "hardware_interface/system_interface.hpp"
#include "duatic_duadrive_interface/coupled_kinematics_types.hpp"

#include "Eigen/Eigen"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace duatic::duadrive_interface::dynamic_model
{

class PinocchioModel
{
public:
  void init(const hardware_interface::HardwareComponentInterfaceParams& system_info,
            const std::shared_ptr<urdf::Model> urdf_model);

  void mock_effort_accelerations(const std::span<const SerialJointState> serial_joint_state_span,
                                 const std::span<const SerialCommand> serial_command_span);

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd tau_;
  Eigen::VectorX<Eigen::Index> joint_model_map_q_;  // containing idx_q
  Eigen::VectorX<Eigen::Index> joint_model_map_v_;  // containing idx_v

public:
  Eigen::VectorXd mocked_accelerations;
};

}  // namespace duatic::duadrive_interface::dynamic_model
