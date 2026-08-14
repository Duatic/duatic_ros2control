#include "duatic_duadrive_interface/ethercat_bus_singleton.hpp"

namespace duatic::duadrive_interface
{
EthercatBusSingleton& EthercatBusSingleton::instance()
{
  static EthercatBusSingleton instance_;
  return instance_;
}
}  // namespace duatic::duadrive_interface
