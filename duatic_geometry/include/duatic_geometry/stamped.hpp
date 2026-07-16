#pragma once

#include <duatic_geometry/timed.hpp>
#include <rclcpp/time.hpp>

namespace duatic::geometry
{

template <typename DataT, typename TimeStampT = rclcpp::Time>
class Stamped : public Timed<DataT, TimeStampT>
{
public:
  using DataType = DataT;
  using TimestampType = TimeStampT;
  using Self = Stamped<DataType, TimestampType>;

  using msg_stamped = DataT::msg_stamped;
  using msg = msg_stamped;

  std::string reference_frame;

  inline constexpr Stamped() = default;
  inline explicit constexpr Stamped(const Self& other) = default;
  inline explicit constexpr Stamped(Self&& other) = default;

  inline explicit Stamped(const msg_stamped& msg)
    : Timed<DataType, TimestampType>(msg), reference_frame(msg.header.frame_id)
  {
  }

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename TimeCtor, typename... DataCtors>
  inline constexpr Stamped(const TimeCtor& time_init, const std::string& frame_init, const DataCtors&... data_init)
    : Timed<DataType, TimestampType>(time_init, data_init...), reference_frame(frame_init)
  {
  }

  Self& setNeutral()
  {
    Timed<DataType, TimestampType>::setNeutral();
    reference_frame.clear();
    return *this;
  }
  static const Self Neutral;
};

// static initializations
template <typename DataT, typename TimeStampT>
const Stamped<DataT, TimeStampT> Stamped<DataT, TimeStampT>::Neutral = Stamped<DataT, TimeStampT>().setNeutral();

}  // namespace duatic::geometry

// streaming
template <typename DataT, typename TimeStampT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Stamped<DataT, TimeStampT>& stamped)
{
  os << "Stamped:" << std::endl
     << " - Time: " << stamped.time << std::endl
     << " - Frame: " << stamped.reference_frame << std::endl
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}
