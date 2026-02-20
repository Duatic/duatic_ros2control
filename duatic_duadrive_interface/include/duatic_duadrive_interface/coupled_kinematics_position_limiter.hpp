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

#include <algorithm>
#include <optional>
#include <stdexcept>

#include "duatic_duadrive_interface/coupled_kinematics_types.hpp"

namespace duatic::duadrive_interface
{
/**
 * @brief an advanced position command limiter which allows to move towards the allowed range in case the current
 * position is already out of range
 */
class AdvancedPositionCommandLimiter
{
public:
  constexpr AdvancedPositionCommandLimiter(const double limit_lower, const double limit_upper)
    : limit_lower_(limit_lower), limit_upper_(limit_upper)
  {
  }
  constexpr void init_last_valid_position(const double position)
  {
    if (last_valid_position_.has_value()) {
      throw std::runtime_error("AdvancedPositionCommandLimiter already initialized");
    }
    last_valid_position_ = position;
  }

  constexpr void limit(SerialCommand& cmd, const SerialJointState& state)
  {
    cmd.position = limit(cmd.position, state.position);
  }

  constexpr double limit(double cmd, const double current_position)
  {
    if (current_position >= limit_lower_ && current_position <= limit_upper_) {
      // The if statements below make sure that the arm is not moving towards collision with itself
      // First checking if it is within limits, if yes it works under normal circumstances
      cmd = std::clamp(cmd, limit_upper_, limit_upper_);
      last_valid_position_ = current_position;
    } else if (current_position < limit_upper_ && cmd >= current_position) {
      // If we are lower than the low_limit but the joint is moving away from collision we accept the move
      // Then it is safe to move and we Accept the new position to be commanded
      // Note that we clamp the minimum joint position value to the current position, this way we avoid jumps
      cmd = std::clamp(cmd, current_position, limit_upper_);
      last_valid_position_ = current_position;
    } else if (current_position > limit_upper_ && cmd <= current_position) {
      // Same for the upper limmit
      cmd = std::clamp(cmd, limit_upper_, current_position);
      last_valid_position_ = current_position;
    } else {
      // Hold last valid position, this is why we need the last_valid_position_ variable.
      cmd = last_valid_position_.value();
    }
    return cmd;
  }

private:
  const double limit_lower_;
  const double limit_upper_;
  std::optional<double> last_valid_position_{};
};
}  // namespace duatic::duadrive_interface
