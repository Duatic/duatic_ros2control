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

#ifndef DUATIC_CONTROL_BROKER__BROKER_HPP_
#define DUATIC_CONTROL_BROKER__BROKER_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace duatic_control_broker
{

struct GroupConfig
{
  std::string default_mode;
  // Each mode and the controllers that are active in it.
  std::map<std::string, std::vector<std::string>> modes;
};

struct Config
{
  std::map<std::string, GroupConfig> groups;
  double heartbeat_timeout_s = 1.0;
};

// Controllers to activate and deactivate. The node narrows both to what controller_manager reports.
struct Switch
{
  std::vector<std::string> activate;
  std::vector<std::string> deactivate;
};

// A planned change: whether it is allowed, and if so the switch and the mode each group ends in.
struct Plan
{
  bool ok = false;
  std::string message;
  Switch change;
  std::map<std::string, std::string> modes;
};

struct Lease
{
  std::string id;
  std::string owner;
  std::vector<std::string> groups;
  double expires_at = 0.0;
};

struct GroupState
{
  std::string group;
  std::string mode;
  std::string owner;
  std::string lease_id;
};

// Hands out groups of joints to one owner at a time and plans the controller switches that go with it.
// Plans are separate from commits so the caller can run the switch first and commit only if it worked.
class Broker
{
public:
  explicit Broker(Config config);

  // Every group nobody holds in its default mode, for taking over a robot in an unknown state.
  Plan plan_defaults() const;
  Plan plan_claim(const std::string& owner, const std::vector<std::string>& groups,
                  const std::vector<std::string>& modes) const;
  std::string commit_claim(const std::string& owner, const Plan& plan, double now);
  Plan plan_release(const std::string& lease_id) const;
  // switched says whether the plan's switch worked. If not, the groups end in an unknown mode.
  void commit_release(const std::string& lease_id, const Plan& plan, bool switched);
  void commit_modes(const Plan& plan);

  bool heartbeat(const std::string& lease_id, double now);
  std::vector<std::string> expired(double now) const;

  std::vector<GroupState> groups() const;

private:
  // check_others refuses a switch that would change a group outside `modes`.
  Plan plan_modes(const std::map<std::string, std::string>& modes, bool check_others) const;
  std::string owner_problem(const std::string& owner, const std::string& group, const std::string& mode) const;

  Config config_;
  std::map<std::string, std::string> current_;
  std::map<std::string, std::string> holder_;
  std::map<std::string, Lease> leases_;
  std::uint64_t next_lease_ = 1;
};

}  // namespace duatic_control_broker

#endif  // DUATIC_CONTROL_BROKER__BROKER_HPP_
