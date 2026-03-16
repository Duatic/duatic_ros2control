/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>
#include <memory>

#include "duatic_duadrive_interface/coupled_kinematics_position_limiter.hpp"

namespace duatic
{
namespace duadrive_interface
{

class AdvancedPositionCommandLimiterTest : public testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize with a joint name and limits [-1.0, 1.0]
    limiter = std::make_unique<AdvancedPositionCommandLimiter>("test_joint", -1.0, 1.0);
    limiter->init_last_valid_position(0.0);
  }

  std::unique_ptr<AdvancedPositionCommandLimiter> limiter;
};

}  // namespace duadrive_interface
}  // namespace duatic

using duatic::duadrive_interface::AdvancedPositionCommandLimiter;
using duatic::duadrive_interface::AdvancedPositionCommandLimiterTest;
using duatic::duadrive_interface::SerialCommand;
using duatic::duadrive_interface::SerialJointState;

TEST_F(AdvancedPositionCommandLimiterTest, TestWithinLimits)
{
  // Command 0.5 when at 0.0 -> Should remain 0.5
  EXPECT_NEAR(limiter->limit(0.5, 0.0), 0.5, 1e-6);

  // Command -0.5 when at 0.5 -> Should remain -0.5
  EXPECT_NEAR(limiter->limit(-0.5, 0.5), -0.5, 1e-6);
}

TEST_F(AdvancedPositionCommandLimiterTest, TestClamping)
{
  // Command 1.5 when at 0.0 -> Should be clamped to 1.0
  EXPECT_NEAR(limiter->limit(1.5, 0.0), 1.0, 1e-6);

  // Command -1.5 when at 0.0 -> Should be clamped to -1.0
  EXPECT_NEAR(limiter->limit(-1.5, 0.0), -1.0, 1e-6);
}

TEST_F(AdvancedPositionCommandLimiterTest, TestRecoveryFromLowerBound)
{
  // Current position is -1.2 (out of bounds), command -1.1 (moving towards bounds)
  // Should be allowed but clamped to current position to avoid jumps if it was
  // moving further out, but here cmd >= current_position, so it uses current_position as minimum
  EXPECT_NEAR(limiter->limit(-1.1, -1.2), -1.1, 1e-6);

  // Command 0.0 when at -1.2 -> Should be allowed
  EXPECT_NEAR(limiter->limit(0.0, -1.2), 0.0, 1e-6);
}

TEST_F(AdvancedPositionCommandLimiterTest, TestRecoveryFromUpperBound)
{
  // Current position is 1.2 (out of bounds), command 1.1 (moving towards bounds)
  EXPECT_NEAR(limiter->limit(1.1, 1.2), 1.1, 1e-6);

  // Command 0.0 when at 1.2 -> Should be allowed
  EXPECT_NEAR(limiter->limit(0.0, 1.2), 0.0, 1e-6);
}

TEST_F(AdvancedPositionCommandLimiterTest, TestHoldPositionIfMovingFurtherOutside)
{
  // Last valid was 0.0
  // Current position is -1.2, command -1.3 (moving further out)
  // Should return last valid position
  EXPECT_NEAR(limiter->limit(-1.3, -1.2), 0.0, 1e-6);

  // Current position is 1.2, command 1.3 (moving further out)
  EXPECT_NEAR(limiter->limit(1.3, 1.2), 0.0, 1e-6);
}

TEST_F(AdvancedPositionCommandLimiterTest, TestInfiniteLimits)
{
  // Create limiter with very large limits (effectively infinity)
  AdvancedPositionCommandLimiter infinite_limiter("infinite_joint", -1e305, 1e305);
  infinite_limiter.init_last_valid_position(0.0);

  // Should not clamp at all
  EXPECT_NEAR(infinite_limiter.limit(5000.0, 0.0), 5000.0, 1e-6);
  EXPECT_NEAR(infinite_limiter.limit(-5000.0, 0.0), -5000.0, 1e-6);
}

TEST_F(AdvancedPositionCommandLimiterTest, TestSerialCommandLimit)
{
  SerialCommand cmd;
  cmd.position = 1.5;
  SerialJointState state;
  state.position = 0.0;

  // limiter has limits [-1.0, 1.0]
  // Should return true because position changed (1.5 -> 1.0)
  bool shifted = limiter->limit(cmd, state);

  EXPECT_TRUE(shifted);
  EXPECT_NEAR(cmd.position, 1.0, 1e-6);

  // Second call with same state and already limited command
  shifted = limiter->limit(cmd, state);
  EXPECT_FALSE(shifted);
}
