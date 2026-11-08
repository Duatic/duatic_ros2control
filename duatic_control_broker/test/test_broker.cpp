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

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "duatic_control_broker/broker.hpp"

using duatic_control_broker::Broker;
using duatic_control_broker::Config;

namespace
{

// Two arms with a controller of their own per mode, and a freedrive controller that moves both.
Config two_arms()
{
  Config config;
  config.heartbeat_timeout_s = 1.0;
  for (const std::string side : { "left", "right" }) {
    auto& group = config.groups["arm_" + side];
    group.default_mode = "hold";
    group.modes["hold"] = { "freeze_" + side };
    group.modes["trajectory"] = { "jtc_" + side };
    group.modes["freedrive"] = { "freedrive" };
  }
  return config;
}

Broker started()
{
  Broker broker(two_arms());
  broker.commit_modes(broker.plan_defaults());
  return broker;
}

std::string claim(Broker& broker, const std::string& owner, const std::vector<std::string>& groups,
                  const std::vector<std::string>& modes, double now = 0.0)
{
  return broker.commit_claim(owner, broker.plan_claim(owner, groups, modes), now);
}

std::vector<std::string> sorted(std::vector<std::string> items)
{
  std::sort(items.begin(), items.end());
  return items;
}

}  // namespace

TEST(Broker, DefaultsActivateEveryHoldController)
{
  Broker broker(two_arms());
  const auto plan = broker.plan_defaults();
  ASSERT_TRUE(plan.ok);
  EXPECT_EQ(sorted(plan.change.activate), (std::vector<std::string>{ "freeze_left", "freeze_right" }));
  EXPECT_EQ(sorted(plan.change.deactivate), (std::vector<std::string>{ "freedrive", "jtc_left", "jtc_right" }));
}

TEST(Broker, AClaimMadeBeforeTheDefaultsIsNotUndone)
{
  Broker broker(two_arms());
  claim(broker, "program", { "arm_left" }, { "trajectory" });
  const auto plan = broker.plan_defaults();
  ASSERT_TRUE(plan.ok) << plan.message;
  EXPECT_EQ(plan.change.activate, (std::vector<std::string>{ "freeze_right" }));
  EXPECT_EQ(plan.modes.count("arm_left"), 0u);
  broker.commit_modes(plan);
  EXPECT_EQ(broker.groups()[0].mode, "trajectory");
  EXPECT_EQ(broker.groups()[0].owner, "program");
}

TEST(Broker, ClaimSwitchesOnlyTheClaimedGroup)
{
  auto broker = started();
  const auto plan = broker.plan_claim("program", { "arm_left" }, { "trajectory" });
  ASSERT_TRUE(plan.ok) << plan.message;
  EXPECT_EQ(plan.change.activate, (std::vector<std::string>{ "jtc_left" }));
  EXPECT_EQ(plan.change.deactivate, (std::vector<std::string>{ "freedrive", "freeze_left" }));
  broker.commit_claim("program", plan, 0.0);
  const auto groups = broker.groups();
  EXPECT_EQ(groups[0].mode, "trajectory");
  EXPECT_EQ(groups[0].owner, "program");
  EXPECT_EQ(groups[1].mode, "hold");
  EXPECT_TRUE(groups[1].owner.empty());
}

TEST(Broker, AHeldGroupIsRefusedToAnotherOwner)
{
  auto broker = started();
  claim(broker, "program", { "arm_left" }, { "trajectory" });
  const auto plan = broker.plan_claim("jog", { "arm_left" }, { "hold" });
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.message, "Group arm_left is held by program");
  EXPECT_TRUE(broker.plan_claim("program", { "arm_left" }, { "hold" }).ok);
}

TEST(Broker, AControllerSharedWithAnotherGroupNeedsBothClaimed)
{
  auto broker = started();
  const std::string refused = "Controller freedrive also drives group arm_right, claim it as well";
  const auto one = broker.plan_claim("program", { "arm_left" }, { "freedrive" });
  EXPECT_FALSE(one.ok);
  EXPECT_EQ(one.message, refused);
  const auto both = broker.plan_claim("program", { "arm_left", "arm_right" }, { "freedrive", "freedrive" });
  ASSERT_TRUE(both.ok) << both.message;
  EXPECT_EQ(both.change.activate, (std::vector<std::string>{ "freedrive" }));
}

TEST(Broker, LeavingASharedModeNeedsBothClaimed)
{
  auto broker = started();
  claim(broker, "teach", { "arm_left", "arm_right" }, { "freedrive", "freedrive" });
  const std::string refused = "Controller freedrive also drives group arm_right, claim it as well";
  const auto plan = broker.plan_claim("teach", { "arm_left" }, { "hold" });
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.message, refused);
}

TEST(Broker, ReleaseReturnsTheGroupsToTheirDefaultMode)
{
  auto broker = started();
  const auto lease = claim(broker, "program", { "arm_left" }, { "trajectory" });
  const auto plan = broker.plan_release(lease);
  ASSERT_TRUE(plan.ok);
  EXPECT_EQ(plan.change.activate, (std::vector<std::string>{ "freeze_left" }));
  broker.commit_release(lease, plan, true);
  EXPECT_EQ(broker.groups()[0].mode, "hold");
  EXPECT_TRUE(broker.groups()[0].owner.empty());
  EXPECT_FALSE(broker.plan_release(lease).ok);
}

TEST(Broker, AFailedSwitchBackLeavesTheModeUnknown)
{
  auto broker = started();
  const auto lease = claim(broker, "program", { "arm_left" }, { "trajectory" });
  broker.commit_release(lease, broker.plan_release(lease), false);
  EXPECT_EQ(broker.groups()[0].mode, "");
  EXPECT_TRUE(broker.groups()[0].owner.empty());
}

TEST(Broker, ALeaseWithoutHeartbeatExpires)
{
  auto broker = started();
  const auto lease = claim(broker, "jog", { "arm_left" }, { "trajectory" }, 10.0);
  EXPECT_TRUE(broker.expired(10.5).empty());
  EXPECT_TRUE(broker.heartbeat(lease, 10.8));
  EXPECT_TRUE(broker.expired(11.5).empty());
  EXPECT_EQ(broker.expired(12.0), (std::vector<std::string>{ lease }));
  EXPECT_FALSE(broker.heartbeat("lease-unknown", 12.0));
}

TEST(Broker, ReclaimingMovesTheGroupToTheNewLease)
{
  auto broker = started();
  const auto first = claim(broker, "program", { "arm_left" }, { "trajectory" });
  const auto second = claim(broker, "program", { "arm_left" }, { "hold" });
  EXPECT_NE(first, second);
  EXPECT_FALSE(broker.plan_release(first).ok);
  EXPECT_EQ(broker.groups()[0].lease_id, second);
}

TEST(Broker, BadClaimsAreRefused)
{
  auto broker = started();
  const std::string unnamed = "Name at least one group, and one mode per group";
  EXPECT_EQ(broker.plan_claim("x", {}, {}).message, unnamed);
  EXPECT_EQ(broker.plan_claim("x", { "leg" }, { "hold" }).message, "No group leg");
  EXPECT_EQ(broker.plan_claim("x", { "arm_left" }, { "dance" }).message, "Group arm_left has no mode dance");
  EXPECT_EQ(broker.plan_claim("x", { "arm_left", "arm_left" }, { "hold", "hold" }).message, "Group arm_left is named "
                                                                                            "twice");
}
