/*
 * Copyright 2026 Duatic AG
 */

#pragma once

#include <chrono>
#include <memory>

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
    params_.communication_timeout = std::chrono::milliseconds(stale ? -1 : 100);
  }
};
}  // namespace duatic::duadrive_interface::test
