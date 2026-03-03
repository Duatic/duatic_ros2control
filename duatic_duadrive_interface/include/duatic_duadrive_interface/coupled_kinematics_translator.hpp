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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "duatic_duadrive_interface/coupled_kinematics_types.hpp"

namespace duatic::duadrive_interface::kinematics
{
template <typename T>
concept CoupledSerialMapping = requires(const Eigen::VectorXd& v)
{
  // Static functions returning the specified vector type
  {
    T::map_from_coupled_to_serial_coordinates(v)
    } -> std::same_as<Eigen::VectorXd>;
  {
    T::map_from_coupled_to_serial_torques(v)
    } -> std::same_as<Eigen::VectorXd>;
  {
    T::map_from_serial_to_coupled_coordinates(v)
    } -> std::same_as<Eigen::VectorXd>;
  {
    T::map_from_serial_to_coupled_torques(v)
    } -> std::same_as<Eigen::VectorXd>;

  // Static size function
  {
    T::input_size()
    } -> std::convertible_to<std::size_t>;
};

template <typename Mapper>
struct KinematicsTranslator
{
  using VectorType = Mapper::VectorType;
  static void map_from_coupled_to_serial(std::span<const CoupledJointState> input, std::span<SerialJointState> output)
  {
    assert(input.size() == output.size());
    if (input.size() != Mapper::input_size()) {
      throw std::runtime_error("Wrong input size");
    }
    const std::size_t n = input.size();
    VectorType p_c(n);
    VectorType v_c(n);
    VectorType a_c(n);
    VectorType t_c(n);

    VectorType p_commanded_c(n);
    VectorType v_commanded_c(n);
    VectorType a_commanded_c(n);
    VectorType t_commanded_c(n);

    for (std::size_t i = 0; i < input.size(); i++) {
      p_c[i] = input[i].position;
      v_c[i] = input[i].velocity;
      a_c[i] = input[i].acceleration;
      t_c[i] = input[i].torque;

      p_commanded_c[i] = input[i].position_commanded;
      v_commanded_c[i] = input[i].velocity_commanded;
      a_commanded_c[i] = input[i].acceleration_commanded;
      t_commanded_c[i] = input[i].torque_commanded;
    }

    VectorType p_s = Mapper::map_from_coupled_to_serial_coordinates(p_c);
    VectorType v_s = Mapper::map_from_coupled_to_serial_coordinates(v_c);
    VectorType a_s = Mapper::map_from_coupled_to_serial_coordinates(a_c);
    VectorType t_s = Mapper::map_from_coupled_to_serial_torques(t_c);

    VectorType p_commanded_s = Mapper::map_from_coupled_to_serial_coordinates(p_commanded_c);
    VectorType v_commanded_s = Mapper::map_from_coupled_to_serial_coordinates(v_commanded_c);
    VectorType a_commanded_s = Mapper::map_from_coupled_to_serial_coordinates(a_commanded_c);
    VectorType t_commanded_s = Mapper::map_from_coupled_to_serial_torques(t_commanded_c);

    for (std::size_t i = 0; i < input.size(); i++) {
      output[i].position = p_s[i];
      output[i].velocity = v_s[i];
      output[i].acceleration = a_s[i];
      output[i].torque = t_s[i];

      output[i].position_commanded = p_commanded_s[i];
      output[i].velocity_commanded = v_commanded_s[i];
      output[i].acceleration_commanded = a_commanded_s[i];
      output[i].torque_commanded = t_commanded_s[i];
    }
  }

  static void map_from_serial_to_coupled(std::span<const SerialJointState> input, std::span<CoupledJointState> output)
  {
    assert(input.size() == output.size());
    if (input.size() != Mapper::input_size()) {
      throw std::runtime_error("Wrong input size");
    }
    const std::size_t n = input.size();
    VectorType p_s(n);
    VectorType v_s(n);
    VectorType a_s(n);
    VectorType t_s(n);

    VectorType p_commanded_s(n);
    VectorType v_commanded_s(n);
    VectorType a_commanded_s(n);
    VectorType t_commanded_s(n);

    for (std::size_t i = 0; i < input.size(); i++) {
      p_s[i] = input[i].position;
      v_s[i] = input[i].velocity;
      a_s[i] = input[i].acceleration;
      t_s[i] = input[i].torque;

      p_commanded_s[i] = input[i].position_commanded;
      v_commanded_s[i] = input[i].velocity_commanded;
      a_commanded_s[i] = input[i].acceleration_commanded;
      t_commanded_s[i] = input[i].torque_commanded;
    }

    VectorType p_c = Mapper::map_from_serial_to_coupled_coordinates(p_s);
    VectorType v_c = Mapper::map_from_serial_to_coupled_coordinates(v_s);
    VectorType a_c = Mapper::map_from_serial_to_coupled_coordinates(a_s);
    VectorType t_c = Mapper::map_from_serial_to_coupled_torques(t_s);

    VectorType p_commanded_c = Mapper::map_from_serial_to_coupled_coordinates(p_commanded_s);
    VectorType v_commanded_c = Mapper::map_from_serial_to_coupled_coordinates(v_commanded_s);
    VectorType a_commanded_c = Mapper::map_from_serial_to_coupled_coordinates(a_commanded_s);
    VectorType t_commanded_c = Mapper::map_from_serial_to_coupled_torques(t_commanded_s);

    for (std::size_t i = 0; i < input.size(); i++) {
      output[i].position = p_c[i];
      output[i].velocity = v_c[i];
      output[i].acceleration = a_c[i];
      output[i].torque = t_c[i];

      output[i].position_commanded = p_commanded_c[i];
      output[i].velocity_commanded = v_commanded_c[i];
      output[i].acceleration_commanded = a_commanded_c[i];
      output[i].torque_commanded = t_commanded_c[i];
    }
  }

  static void map_from_coupled_to_serial(std::span<const CoupledCommand> input, std::span<SerialCommand> output)
  {
    assert(input.size() == output.size());
    if (input.size() != Mapper::input_size()) {
      throw std::runtime_error("Wrong input size");
    }
    const std::size_t n = input.size();
    VectorType p_c(n);
    VectorType v_c(n);
    VectorType a_c(n);
    VectorType t_c(n);

    for (std::size_t i = 0; i < input.size(); i++) {
      p_c[i] = input[i].position;
      v_c[i] = input[i].velocity;
      a_c[i] = input[i].acceleration;
      t_c[i] = input[i].torque;
    }

    VectorType p_s = Mapper::map_from_coupled_to_serial_coordinates(p_c);
    VectorType v_s = Mapper::map_from_coupled_to_serial_coordinates(v_c);
    VectorType a_s = Mapper::map_from_coupled_to_serial_coordinates(a_c);
    VectorType t_s = Mapper::map_from_coupled_to_serial_torques(t_c);

    for (std::size_t i = 0; i < input.size(); i++) {
      output[i].position = p_s[i];
      output[i].velocity = v_s[i];
      output[i].acceleration = a_s[i];
      output[i].torque = t_s[i];
    }
  }

  static void map_from_serial_to_coupled(std::span<const SerialCommand> input, std::span<CoupledCommand> output)
  {
    assert(input.size() == output.size());
    if (input.size() != Mapper::input_size()) {
      throw std::runtime_error("Wrong input size");
    }
    const std::size_t n = input.size();
    VectorType p_s(n);
    VectorType v_s(n);
    VectorType a_s(n);
    VectorType t_s(n);

    for (std::size_t i = 0; i < input.size(); i++) {
      p_s[i] = input[i].position;
      v_s[i] = input[i].velocity;
      a_s[i] = input[i].acceleration;
      t_s[i] = input[i].torque;
    }

    VectorType p_c = Mapper::map_from_serial_to_coupled_coordinates(p_s);
    VectorType v_c = Mapper::map_from_serial_to_coupled_coordinates(v_s);
    VectorType a_c = Mapper::map_from_serial_to_coupled_coordinates(a_s);
    VectorType t_c = Mapper::map_from_serial_to_coupled_torques(t_s);

    for (std::size_t i = 0; i < input.size(); i++) {
      output[i].position = p_c[i];
      output[i].velocity = v_c[i];
      output[i].acceleration = a_c[i];
      output[i].torque = t_c[i];
    }
  }
};

}  // namespace duatic::duadrive_interface::kinematics
