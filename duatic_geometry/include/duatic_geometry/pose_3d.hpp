#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ostream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <duatic_geometry/twist_3d.hpp>
#include <duatic_geometry/timed.hpp>
#include <duatic_geometry/stamped.hpp>

namespace duatic::geometry
{

template <typename ScalarT = double>
class Pose3D
{
public:
  using Scalar = ScalarT;
  using Self = Pose3D<Scalar>;

  using msg = geometry_msgs::msg::Pose;
  using msg_stamped = geometry_msgs::msg::PoseStamped;

  using LinearType = Eigen::Vector<Scalar, 3>;
  using RotationType = Eigen::Quaternion<Scalar>;

  LinearType position;
  RotationType orientation;

  inline constexpr Pose3D() = default;
  inline constexpr Pose3D(const Self& other) = default;
  inline constexpr Pose3D(Self&& other) = default;

  inline explicit constexpr Pose3D(const msg& msg)
  {
    position.x() = msg.position.x;
    position.y() = msg.position.y;
    position.z() = msg.position.z;
    orientation.w() = msg.orientation.w;
    orientation.x() = msg.orientation.x;
    orientation.y() = msg.orientation.y;
    orientation.z() = msg.orientation.z;
  }
  inline explicit constexpr Pose3D(const msg_stamped& msg) : Pose3D(msg.pose)  // delegate
  {
  }

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename LinearCtor, typename RotationalCtor>
  inline constexpr Pose3D(const LinearCtor& position_init, const RotationalCtor& orientation_init)
    : position(position_init), orientation(orientation_init)
  {
  }

  Self& setNeutral()
  {
    position.setZero();
    orientation.setIdentity();
    return *this;
  }
  static const Self Neutral;

  // Arithmetics
  Twist3D<Scalar> operator-(const Self& other) const
  {
    const Eigen::AngleAxis<Scalar> orientation_axis(orientation * other.orientation.conjugate());
    return Twist3D<Scalar>(position - other.position, orientation_axis.angle() * orientation_axis.axis());
  }
};

// static initializations
template <typename ScalarT>
const Pose3D<ScalarT> Pose3D<ScalarT>::Neutral = Pose3D<ScalarT>().setNeutral();

// Type Defs
template <typename ScalarT = double>
using TimedPose3D = Timed<Pose3D<ScalarT>>;
template <typename ScalarT = double>
using StampedPose3D = Stamped<Pose3D<ScalarT>>;

// Explicit Types
using Pose3Dd = Pose3D<double>;
using TimedPose3Dd = TimedPose3D<double>;
using StampedPose3Dd = StampedPose3D<double>;

}  // namespace duatic::geometry

// streaming
template <typename ScalarT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Pose3D<ScalarT>& pose)
{
  os << "Pose3D(position: " << pose.position.transpose() << ", orientation: " << pose.orientation << ")";
  return os;
}
