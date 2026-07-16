#pragma once

#include <rclcpp/time.hpp>

namespace duatic::geometry
{

template <typename DataT, typename TimeStampT = rclcpp::Time>
class Timed : public DataT
{
public:
  using DataType = DataT;
  using TimestampType = TimeStampT;
  using Self = Timed<DataType, TimestampType>;

  using msg_stamped = DataT::msg_stamped;
  using msg = msg_stamped;

  TimestampType time;

  inline constexpr Timed() = default;
  inline explicit constexpr Timed(const Self& other) = default;
  inline explicit constexpr Timed(Self&& other) = default;

  inline explicit Timed(const msg_stamped& msg) : DataType(msg), time(msg.header.stamp)
  {
  }

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename TimeCtor, typename... DataCtors>
  inline constexpr Timed(const TimeCtor& time_init, const DataCtors&... data_init)
    : DataType(data_init...), time(time_init)
  {
  }

  Self& setNeutral()
  {
    DataType::setNeutral();
    time = TimestampType();
    return *this;
  }
  static const Self Neutral;
};

// static initializations
template <typename DataT, typename TimeStampT>
const Timed<DataT, TimeStampT> Timed<DataT, TimeStampT>::Neutral = Timed<DataT, TimeStampT>().setNeutral();

}  // namespace duatic::geometry

// streaming
template <typename DataT, typename TimeStampT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Timed<DataT, TimeStampT>& stamped)
{
  os << "Timed:" << std::endl
     << " - Time: " << stamped.time << std::endl
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}
