#pragma once

#include <rclcpp/time.hpp>

namespace duatic::geometry
{

template <typename DataT, typename TimeStampT = rclcpp::Time>
class Stamped : public DataT
{
public:
  using DataType = DataT;
  using TimestampType = TimeStampT;
  using Self = Stamped<DataType, TimestampType>;

  TimestampType time;

  inline constexpr Stamped() = default;
  inline constexpr Stamped(const Self& other) = default;
  inline constexpr Stamped(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename TimeCtor, typename... DataCtors>
  inline constexpr Stamped(const TimeCtor& time_init, const DataCtors&... data_init)
    : DataType(data_init...), time(time_init)
  {
  }

  static constexpr Self Neutral = Self(TimestampType(), DataType::Neutral);
};

}  // namespace duatic::geometry

// streaming
template <typename DataT, typename TimeStampT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Stamped<DataT, TimeStampT>& stamped)
{
  os << "Stamped:" << std::endl
     << " - Time: " << stamped.time << std::endl
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}
