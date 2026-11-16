/*
 * Copyright 2026 Duatic AG
 */

#pragma once

#include <chrono>
#include <memory>
#include <optional>

#include "duatic_duadrive_interface/duadrive_interface_mock.hpp"

namespace duatic::duadrive_interface::test
{
// A mock drive whose state machine state and reading age can be forced
class FaultableMockDrive : public DuaDriveInterfaceMock
{
public:
  using DuaDriveInterfaceMock::DuaDriveInterfaceMock;
  using UniquePtr = std::unique_ptr<FaultableMockDrive>;

  void set_drive_state(const rsl_drive_sdk::fsm::StateEnum drive_state)
  {
    state_.current_drive_state = drive_state;
  }

  // A negative timeout makes every reading count as stale
  void set_readings_stale(const bool stale)
  {
    if (!configured_timeout_) {
      configured_timeout_ = params_.communication_timeout;
    }
    params_.communication_timeout = stale ? std::chrono::milliseconds(-1) : *configured_timeout_;
  }

  std::chrono::milliseconds communication_timeout() const
  {
    return params_.communication_timeout;
  }

private:
  std::optional<std::chrono::milliseconds> configured_timeout_;
};
}  // namespace duatic::duadrive_interface::test
