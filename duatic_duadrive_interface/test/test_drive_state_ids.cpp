/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include <duatic_controller_msgs/msg/drive_state.hpp>
#include <rsl_drive_sdk/fsm/StateEnum.hpp>

using duatic_controller_msgs::msg::DriveState;

// The message constants are the protocol ids the hardware interface exports
static_assert(DriveState::DRIVE_STATE_NA == RSL_DRIVE_STATE_ID_NA);
static_assert(DriveState::DRIVE_STATE_COLDSTART == RSL_DRIVE_STATE_ID_COLDSTART);
static_assert(DriveState::DRIVE_STATE_WARMSTART == RSL_DRIVE_STATE_ID_WARMSTART);
static_assert(DriveState::DRIVE_STATE_CONFIGURE == RSL_DRIVE_STATE_ID_CONFIGURE);
static_assert(DriveState::DRIVE_STATE_CALIBRATE == RSL_DRIVE_STATE_ID_CALIBRATE);
static_assert(DriveState::DRIVE_STATE_STANDBY == RSL_DRIVE_STATE_ID_STANDBY);
static_assert(DriveState::DRIVE_STATE_MOTOROP == RSL_DRIVE_STATE_ID_MOTOROP);
static_assert(DriveState::DRIVE_STATE_CONTROLOP == RSL_DRIVE_STATE_ID_CONTROLOP);
static_assert(DriveState::DRIVE_STATE_ERROR == RSL_DRIVE_STATE_ID_ERROR);
static_assert(DriveState::DRIVE_STATE_FATAL == RSL_DRIVE_STATE_ID_FATAL);
static_assert(DriveState::DRIVE_STATE_MOTORPREOP == RSL_DRIVE_STATE_ID_MOTORPREOP);
static_assert(DriveState::DRIVE_STATE_DEVICE_MISSING == RSL_DRIVE_STATE_ID_DEVICE_MISSING);

TEST(DriveStateIds, MatchTheDriveProtocol)
{
  SUCCEED();
}
