#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ostream>
#include <duatic_geometry/twist_3d.hpp>

namespace duatic::geometry
{

template <typename ScalarT = double>
class Pose3D
{
public:
  using Scalar = ScalarT;
  using Self = Pose3D<Scalar>;

  using LinearType = Eigen::Vector<Scalar, 3>;
  using RotationType = Eigen::Quaternion<Scalar>;

  LinearType position;
  RotationType orientation;

  inline constexpr Pose3D() = default;
  inline constexpr Pose3D(const Self& other) = default;

  inline Self& operator=(const Self& other) = default;

  template <typename LinearCtor, typename RotationalCtor>
  inline constexpr Pose3D(const LinearCtor& position_init, const RotationalCtor& orientation_init)
    : position(position_init), orientation(orientation_init)
  {
  }

  static constexpr Self Neutral = Self(LinearType::Zero(), RotationType::Identity());

  // Arithmetics
  Twist3D<Scalar> operator-(const Self& other) const
  {
    const Eigen::AngleAxis<Scalar> orientation_axis(orientation * other.orientation.conjugate());
    return Twist3D<Scalar>(position - other.position, orientation_axis.angle() * orientation_axis.axis());
  }
};

// Explicit Types
using Pose3Dd = Pose3D<double>;

}  // namespace duatic::geometry

// streaming
template <typename ScalarT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Pose3D<ScalarT>& pose)
{
  os << "Pose3D(position: " << pose.position.transpose() << ", orientation: " << pose.orientation << ")";
  return os;
}
