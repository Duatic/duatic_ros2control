#pragma once

#include <duatic_geometry/pose_3d.hpp>
#include <duatic_geometry/twist_3d.hpp>
#include <duatic_controller_msgs/msg/state.hpp>
#include <duatic_controller_msgs/msg/state_stamped.hpp>
#include <duatic_geometry/timed.hpp>
#include <duatic_geometry/stamped.hpp>

namespace duatic::geometry
{

template <typename ScalarT = double>
class State3D
{
public:
  using Scalar = ScalarT;
  using Self = State3D<Scalar>;

  using msg = duatic_controller_msgs::msg::State;
  using msg_stamped = duatic_controller_msgs::msg::StateStamped;

  using PoseType = Pose3D<Scalar>;
  using TwistType = Twist3D<Scalar>;

  PoseType pose;
  TwistType twist;

  inline constexpr State3D() = default;
  inline constexpr State3D(const Self& other) = default;
  inline constexpr State3D(Self&& other) = default;

  inline explicit constexpr State3D(const msg& msg) : pose(msg.pose), twist(msg.twist)
  {
  }
  inline explicit constexpr State3D(const msg_stamped& msg) : State3D(msg.state)  // delegate
  {
  }

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename PoseCtor, typename TwistCtor>
  inline constexpr State3D(const PoseCtor& pose_init, const TwistCtor& twist_init) : pose(pose_init), twist(twist_init)
  {
  }

  Self& setNeutral()
  {
    pose.setNeutral();
    twist.setNeutral();
    return *this;
  }
  static const Self Neutral;
};

// static initializations
template <typename ScalarT>
const State3D<ScalarT> State3D<ScalarT>::Neutral = State3D<ScalarT>().setNeutral();

// Type Defs
template <typename ScalarT = double>
using TimedState3D = Timed<State3D<ScalarT>>;
template <typename ScalarT = double>
using StampedState3D = Stamped<State3D<ScalarT>>;

// Explicit Types
using State3Dd = State3D<double>;
using TimedState3Dd = TimedState3D<double>;
using StampedState3Dd = StampedState3D<double>;

}  // namespace duatic::geometry

// streaming
template <typename ScalarT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::State3D<ScalarT>& state)
{
  os << "State:" << std::endl << " - Pose: " << state.pose << std::endl << " - Twist: " << state.twist;
  return os;
}
