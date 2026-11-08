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

#include "duatic_control_broker/broker.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace duatic_control_broker
{

namespace
{

bool contains(const std::vector<std::string>& items, const std::string& item)
{
  return std::find(items.begin(), items.end(), item) != items.end();
}

Plan refuse(std::string message)
{
  Plan plan;
  plan.message = std::move(message);
  return plan;
}

}  // namespace

Broker::Broker(Config config) : config_(std::move(config))
{
}

Plan Broker::plan_defaults() const
{
  std::map<std::string, std::string> modes;
  for (const auto& [group, config] : config_.groups) {
    if (holder_.count(group) == 0) {
      modes[group] = config.default_mode;
    }
  }
  return plan_modes(modes, true);
}

Plan Broker::plan_claim(const std::string& owner, const std::vector<std::string>& groups,
                        const std::vector<std::string>& modes) const
{
  if (groups.empty() || groups.size() != modes.size()) {
    return refuse("Name at least one group, and one mode per group");
  }
  std::map<std::string, std::string> wanted;
  for (std::size_t i = 0; i < groups.size(); ++i) {
    if (!wanted.emplace(groups[i], modes[i]).second) {
      return refuse("Group " + groups[i] + " is named twice");
    }
    const auto problem = owner_problem(owner, groups[i], modes[i]);
    if (!problem.empty()) {
      return refuse(problem);
    }
  }
  return plan_modes(wanted, true);
}

std::string Broker::owner_problem(const std::string& owner, const std::string& group, const std::string& mode) const
{
  const auto config = config_.groups.find(group);
  if (config == config_.groups.end()) {
    return "No group " + group;
  }
  if (config->second.modes.count(mode) == 0) {
    return "Group " + group + " has no mode " + mode;
  }
  const auto holder = holder_.find(group);
  if (holder != holder_.end() && leases_.at(holder->second).owner != owner) {
    return "Group " + group + " is held by " + leases_.at(holder->second).owner;
  }
  return "";
}

Plan Broker::plan_modes(const std::map<std::string, std::string>& modes, bool check_others) const
{
  std::set<std::string> activate;
  std::set<std::string> deactivate;
  for (const auto& [group, mode] : modes) {
    for (const auto& controller : config_.groups.at(group).modes.at(mode)) {
      activate.insert(controller);
    }
  }
  for (const auto& [group, mode] : modes) {
    for (const auto& [other_mode, controllers] : config_.groups.at(group).modes) {
      for (const auto& controller : controllers) {
        if (other_mode != mode && activate.count(controller) == 0) {
          deactivate.insert(controller);
        }
      }
    }
  }
  if (check_others) {
    // A controller can drive more than one group. Switching it must not change a group outside the plan:
    // deactivating one its current mode needs, or activating one of its other modes, would.
    for (const auto& [other, config] : config_.groups) {
      if (modes.count(other) != 0) {
        continue;
      }
      const auto current = current_.find(other);
      const auto in_use = current == current_.end() ? config.modes.end() : config.modes.find(current->second);
      const std::vector<std::string> none;
      const auto& used = in_use == config.modes.end() ? none : in_use->second;
      for (const auto& [mode, controllers] : config.modes) {
        for (const auto& controller : controllers) {
          const bool needed = contains(used, controller);
          if ((needed && deactivate.count(controller) != 0) || (!needed && activate.count(controller) != 0)) {
            return refuse("Controller " + controller + " also drives group " + other + ", claim it as well");
          }
        }
      }
    }
  }
  Plan plan;
  plan.ok = true;
  plan.change.activate.assign(activate.begin(), activate.end());
  plan.change.deactivate.assign(deactivate.begin(), deactivate.end());
  plan.modes = modes;
  return plan;
}

std::string Broker::commit_claim(const std::string& owner, const Plan& plan, double now)
{
  Lease lease;
  lease.id = "lease-" + std::to_string(next_lease_++);
  lease.owner = owner;
  lease.expires_at = now + config_.heartbeat_timeout_s;
  for (const auto& [group, mode] : plan.modes) {
    // A group the owner already holds moves from its old lease to the new one.
    const auto holder = holder_.find(group);
    if (holder != holder_.end()) {
      auto& old = leases_.at(holder->second).groups;
      old.erase(std::remove(old.begin(), old.end(), group), old.end());
      if (old.empty()) {
        leases_.erase(holder->second);
      }
    }
    holder_[group] = lease.id;
    current_[group] = mode;
    lease.groups.push_back(group);
  }
  leases_[lease.id] = lease;
  return lease.id;
}

Plan Broker::plan_release(const std::string& lease_id) const
{
  const auto lease = leases_.find(lease_id);
  if (lease == leases_.end()) {
    return refuse("No lease " + lease_id);
  }
  std::map<std::string, std::string> modes;
  for (const auto& group : lease->second.groups) {
    modes[group] = config_.groups.at(group).default_mode;
  }
  return plan_modes(modes, false);
}

void Broker::commit_release(const std::string& lease_id, const Plan& plan, bool switched)
{
  const auto lease = leases_.find(lease_id);
  if (lease == leases_.end()) {
    return;
  }
  for (const auto& group : lease->second.groups) {
    holder_.erase(group);
    // A group whose switch failed is in a mode nobody knows.
    current_[group] = switched ? plan.modes.at(group) : "";
  }
  leases_.erase(lease);
}

void Broker::commit_modes(const Plan& plan)
{
  for (const auto& [group, mode] : plan.modes) {
    current_[group] = mode;
  }
}

bool Broker::heartbeat(const std::string& lease_id, double now)
{
  const auto lease = leases_.find(lease_id);
  if (lease == leases_.end()) {
    return false;
  }
  lease->second.expires_at = now + config_.heartbeat_timeout_s;
  return true;
}

std::vector<std::string> Broker::expired(double now) const
{
  std::vector<std::string> out;
  for (const auto& [id, lease] : leases_) {
    if (lease.expires_at < now) {
      out.push_back(id);
    }
  }
  return out;
}

std::vector<GroupState> Broker::groups() const
{
  std::vector<GroupState> out;
  for (const auto& [group, config] : config_.groups) {
    GroupState state;
    state.group = group;
    const auto current = current_.find(group);
    state.mode = current == current_.end() ? "" : current->second;
    const auto holder = holder_.find(group);
    if (holder != holder_.end()) {
      state.lease_id = holder->second;
      state.owner = leases_.at(holder->second).owner;
    }
    out.push_back(state);
  }
  return out;
}

}  // namespace duatic_control_broker
