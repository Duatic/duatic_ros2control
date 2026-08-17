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

#pragma once

#include <map>

#include <duatic_duadrive_sdk/v1/duadrive.hpp>
#include <duatic_ethercat_interface/executor.hpp>
#include <duatic_message_logger/log.hpp>

namespace duatic::duadrive_interface
{
/**
 *  @brief Provides the only method how we can use the same ethercat bus in multiple ros2control hardware interfaces
 * The idea is that we centrally manage the instances of the EthercatMasters and each hardware interface may attach its
 * devices to it
 */
class EthercatBusSingleton
{
public:
  using StartupFinishedCb = std::function<void(void)>;

private:
  using EthercatBus = ethercat_interface::EthercatBus;
  using SingleBusExecutor = ethercat_interface::SingleBusExecutor;
  // This represents and internal handle which contains all necessary information in order to manage multiple
  // EthercatMasterInstances
  struct InternalHandle
  {
    std::shared_ptr<EthercatBus> ecat_bus;
    std::unique_ptr<SingleBusExecutor> executor;
    std::atomic_bool running{ false };
    int reference_count{ 0 };
    std::map<int, bool> handles_ready;
    std::vector<StartupFinishedCb> startup_finished_callbacks{ nullptr };
    InternalHandle(const std::shared_ptr<EthercatBus>& ecat_bus_, StartupFinishedCb cb_startup_finished_)
      : ecat_bus(ecat_bus_), startup_finished_callbacks({ cb_startup_finished_ })
    {
    }
    InternalHandle(InternalHandle&& o)
      : ecat_bus(o.ecat_bus)
      , reference_count(o.reference_count)
      , handles_ready(o.handles_ready)
      , startup_finished_callbacks(o.startup_finished_callbacks)
    {
    }
  };

public:
  // This represents a public handle which can be used to uniquely identify the result of an aquireMaster operation
  struct Handle
  {
    int id{ 0 };
    std::shared_ptr<EthercatBus> ecat_bus;
    std::atomic_bool* running{ nullptr };
  };

  static EthercatBusSingleton& instance();  // Method has to be in cpp file in order to work properly

  /**
   * @brief get a shared pointer to an ecat bus instance
   * If there is already a bus active for the given instance we simply reuse it and return it
   * @note Using this methods enforces asynchronous spinning of the master in this class
   * @note In case the "new" ethercat master configuration does not match the existing one only a warning is printed!
   */
  Handle aquire_bus(const EthercatBus::Parameters& config, StartupFinishedCb cb_startup_finished = nullptr)
  {
    // Give it at least some thread safety
    std::lock_guard<std::mutex> guard(lock_);

    // Check if we already have a master up and running for the given networkinterface
    if (handles_.find(config.interface) == handles_.end()) {
      logging::info() << "Setting up new EthercatMaster on interface: " << config.interface << " and updating it";
      auto master = std::make_shared<EthercatBus>(config);
      master->initialize();

      handles_.emplace(config.interface, InternalHandle{ master, cb_startup_finished });
    } else {
      handles_.at(config.interface).startup_finished_callbacks.push_back(cb_startup_finished);
    }

    // Increment its reference counter
    handles_.at(config.interface).reference_count += 1;
    // Mark the new handle as not ready
    handles_.at(config.interface).handles_ready.emplace(handles_.at(config.interface).reference_count, false);

    // if (config != handles_.at(config.interface).ecat_bus->get_parameters()) {
    //  Print warning or abort if the configuration does not match!
    // logging::warn() << "Ethercat master configurations do not match for bus: " << config.interface;
    //}

    // Create and return a handle in which we use the reference count as an id
    return Handle{ handles_.at(config.interface).reference_count, handles_.at(config.interface).ecat_bus,
                   &handles_.at(config.interface).running };
  }
  /**
   * @brief mark a specific handle as ready. If all handles acquired via aquireMaster are ready the bus gets activated
   * and is spun in a separate thread
   * @return true if the ethercat is now activated
   */
  bool mark_as_ready(const Handle& handle)
  {
    // 1. find the corresponding internal handle
    const auto& network_interface = handle.ecat_bus->get_parameters().interface;

    if (!has_bus(network_interface)) {
      throw std::logic_error("EthercatMaster for interface: " + network_interface +
                             " is not handled by this singleton");
    }

    auto& internal_handle = handles_.at(network_interface);

    // 2. Check if the handle is already marked as ready
    if (internal_handle.handles_ready.at(handle.id)) {
      throw std::runtime_error("Handle with id: " + std::to_string(handle.id) + " on interface: " + network_interface +
                               " was already marked as ready!");
    }

    // 3. Mark it as ready
    internal_handle.handles_ready.at(handle.id) = true;

    // 4. Check if all handles are ready
    bool all_ready = true;
    for (auto& [id, ready] : internal_handle.handles_ready) {
      if (!ready) {
        all_ready = false;
        break;
      }
    }

    if (!all_ready) {
      logging::info() << "Not all handles ready - deferring start";
      return false;
    }

    // 5. Perform the startup (bus startup - this just starts setting up communication) and spin
    internal_handle.ecat_bus->startup();

    // 6. Call callback to allow clients to perform any work before going into PDO communication which is timing
    // sensitive
    for (auto& cb : internal_handle.startup_finished_callbacks) {
      if (cb) {
        cb();
      }
    }

    logging::info() << "Starting asynchronous worker thread for ethercat master on network interface: "
                    << network_interface;

    // Start executor
    internal_handle.ecat_bus->activate();
    internal_handle.executor = std::make_unique<SingleBusExecutor>(internal_handle.ecat_bus);
    return true;
  }
  /**
   * @brief check if an ethercat master is active and managed by this implementation for the given configuration
   * @note only checks the interface name
   */
  bool has_bus(const EthercatBus::Parameters& config)
  {
    return handles_.find(config.interface) != handles_.end();
  }
  bool has_bus(const std::string& networkInterface)
  {
    return handles_.find(networkInterface) != handles_.end();
  }

  /**
   * @brief release_bus - release your handle obtain via aquireMaster
   * This method decrements the internal reference counter for the given EthercatMaster and performs the shutdown if no
   * references are living anymore
   */
  bool release_bus(const Handle& handle)
  {
    // Give it at least some thread safety
    std::lock_guard<std::mutex> guard(lock_);

    const auto& network_interface = handle.ecat_bus->get_parameters().interface;
    // First check if we even handle this ethercat bus
    if (!has_bus(network_interface)) {
      throw std::logic_error("EthercatBus for interface: " + network_interface + " is not handled by this singleton");
    }

    // Decrement the reference counter and check if it is zero
    handles_.at(network_interface).reference_count -= 1;

    if (handles_.at(network_interface).reference_count <= 0) {
      logging::info() << "Shutting down EthercatBus for interface: " << network_interface;
      // Perform the actual shutdown if all callers have called shutdown on their reference
      shutdown_bus(handle.ecat_bus);
      return true;
    }

    return false;
  }
  /**
   * @brief shutdown a given EthercatBus even though the reference counter is not 0
   * @note this is really unsafe as other instances using the ethercat bus will most likely crash
   * But ins some cases it is better to properly close the bus than not to crash the control program on the PC
   */
  void force_shutdown_bus(const std::shared_ptr<EthercatBus>& bus)
  {
    // Give it at least some thread safety
    std::lock_guard<std::mutex> guard(lock_);

    shutdown_bus(bus);
  }

private:
  void shutdown_bus(const std::shared_ptr<EthercatBus>& bus)
  {
    const auto& network_interface = bus->get_parameters().interface;
    if (!has_bus(network_interface)) {
      throw std::logic_error("EthercatMaster for interface: " + network_interface +
                             " is not handled by this singleton");
    }
    logging::info() << "Shutting down ethercat bus: " << network_interface;

    // Perform the actual shutdown
    handles_.at(network_interface).executor->stop();

    // And remove all entries
    handles_.erase(network_interface);

    logging::info() << "Shutdown of ethercat bus: " << network_interface << " finished";
  }

  EthercatBusSingleton() = default;
  ~EthercatBusSingleton()
  {
    // Tell every update thread to stop spinning
    for (auto& [interface, handle] : handles_) {
      handle.executor->stop();
    }
  }

  std::map<std::string, InternalHandle> handles_;
  std::mutex lock_;
};
}  // namespace duatic::duadrive_interface
