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

#include "duatic_duadrive_interface/dynamic_model/dynamic_model_pinocchio.hpp"

#include <exception>

#include "Eigen/Dense"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/aba.hpp>

namespace duatic::duadrive_interface::dynamic_model
{

void DynamicModelPinocchio::init(const hardware_interface::HardwareComponentInterfaceParams& system_info,
                                 const std::shared_ptr<urdf::Model> urdf_model)
{
  // build robot model from urdf
  pinocchio::urdf::buildModel(urdf_model, model_);
  data_ = pinocchio::Data(model_);

  // setup internal data storages
  const std::size_t drive_cnt = system_info.hardware_info.joints.size();
  joint_model_map_q_.resize(drive_cnt);
  joint_model_map_v_.resize(drive_cnt);
  mocked_accelerations = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(drive_cnt));

  // build drive-index map
  for (std::size_t i = 0; i < drive_cnt; i++) {
    const std::string& joint_name = system_info.hardware_info.joints[i].name;
    if (model_.existJointName(joint_name)) {
      const auto& joint = model_.joints[model_.getJointId(joint_name)];
      joint_model_map_q_[i] = joint.idx_q();
      joint_model_map_v_[i] = joint.idx_v();
    } else {
      throw std::runtime_error("Could not find drive name '" + joint_name + "' within the robot hardware model.");
    }
  }
}

void DynamicModelPinocchio::mock_effort_accelerations(const std::span<const SerialJointState> serial_joint_state_span,
                                                      const std::span<const SerialCommand> serial_command_span)
{
  Eigen::VectorXd q(model_.nq);
  Eigen::VectorXd v(model_.nv);
  Eigen::VectorXd tau(model_.nv);

  for (Eigen::Index i = 0; i < joint_model_map_q_.size(); i++) {
    q[joint_model_map_q_[i]] = serial_joint_state_span[i].position;
    v[joint_model_map_v_[i]] = serial_joint_state_span[i].velocity;
    tau[joint_model_map_v_[i]] = serial_command_span[i].torque;
  }

  // simple forward dynamics with no collision or contact constraints being considered
  pinocchio::aba(model_, data_, q, v, tau);

  // reverse map
  for (Eigen::Index i = 0; i < joint_model_map_q_.size(); i++) {
    mocked_accelerations[i] = data_.ddq[joint_model_map_v_[i]];
  }
}

}  // namespace duatic::duadrive_interface::dynamic_model
