#pragma once

#include <utility>
#include <ostream>

#include <Eigen/Dense>

namespace duatic::geometry
{

template <typename ScalarT = double>
class Twist3D
{
public:
  using Scalar = ScalarT;
  using Self = Twist3D<Scalar>;

  using VectorType = Eigen::Vector<Scalar, 6>;
  using TranslationType = decltype(std::declval<VectorType>().segment(0, 3));
  using TranslationTypeConst = decltype(std::declval<const VectorType>().segment(0, 3));
  using RotationType = decltype(std::declval<VectorType>().segment(3, 3));
  using RotationTypeConst = decltype(std::declval<const VectorType>().segment(3, 3));

  VectorType vector;

  inline constexpr Twist3D() = default;
  inline constexpr Twist3D(const Self& other) = default;

  inline Self& operator=(const Self& other) = default;

  template <typename VectorCtor>
  inline constexpr Twist3D(const VectorCtor& init) : vector(init)
  {
  }

  template <typename TranslationCtor, typename RotationCtor>
  inline constexpr Twist3D(const TranslationCtor& translation_init, const RotationCtor& rotation_init) : vector()
  {
    Translation() = translation_init;
    Rotation() = rotation_init;
  }

  static constexpr Self Neutral = Self(VectorType::Zero());

  inline TranslationType Translation()
  {
    return vector.segment(0, 3);
  }
  inline TranslationTypeConst Translation() const
  {
    return vector.segment(0, 3);
  }

  inline RotationType Rotation()
  {
    return vector.segment(3, 3);
  }
  inline const RotationTypeConst Rotation() const
  {
    return vector.segment(3, 3);
  }
};

// Explicit Types
using Twist3Dd = Twist3D<double>;

}  // namespace duatic::geometry

// streaming
template <typename ScalarT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Twist3D<ScalarT>& twist)
{
  os << "Twist3D(translation: " << twist.Translation().transpose() << ", rotation: " << twist.Rotation().transpose()
     << ")";
  return os;
}
