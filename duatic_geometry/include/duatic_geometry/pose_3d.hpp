#pragma once

#include <Eigen/Dense>

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
};

// Explicit Types
using Pose3Dd = Pose3D<double>;

}  // namespace duatic::geometry
