#pragma once

#include <duatic_geometry/pose_3d.hpp>
#include <duatic_geometry/twist_3d.hpp>

namespace duatic::geometry
{

template <typename ScalarT = double>
class State3D
{
public:
  using Scalar = ScalarT;
  using Self = State3D<Scalar>;

  using PoseType = Pose3D<Scalar>;
  using TwistType = Twist3D<Scalar>;

  PoseType pose;
  TwistType twist;

  inline constexpr State3D() = default;
  inline constexpr State3D(const Self& other) = default;

  inline Self& operator=(const Self& other) = default;

  template <typename PoseCtor, typename TwistCtor>
  inline constexpr State3D(const PoseCtor& pose_init, const TwistCtor& twist_init) : pose(pose_init), twist(twist_init)
  {
  }

  static constexpr Self Neutral = Self(PoseType::Neutral(), TwistType::Neutral());
};

// Explicit Types
using State3Dd = State3D<double>;

}  // namespace duatic::geometry
