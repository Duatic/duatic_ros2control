#pragma once

#include <utility>
#include <ostream>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Dense>
#include <duatic_geometry/timed.hpp>
#include <duatic_geometry/stamped.hpp>

namespace duatic::geometry
{

template <typename ScalarT = double>
class Twist3D
{
public:
  using Scalar = ScalarT;
  using Self = Twist3D<Scalar>;

  using msg = geometry_msgs::msg::Twist;
  using msg_stamped = geometry_msgs::msg::TwistStamped;

  using VectorType = Eigen::Vector<Scalar, 6>;
  using TranslationType = decltype(std::declval<VectorType>().segment(0, 3));
  using TranslationTypeConst = decltype(std::declval<const VectorType>().segment(0, 3));
  using RotationType = decltype(std::declval<VectorType>().segment(3, 3));
  using RotationTypeConst = decltype(std::declval<const VectorType>().segment(3, 3));

  VectorType vector;

  inline constexpr Twist3D() = default;
  inline constexpr Twist3D(const Self& other) = default;
  inline constexpr Twist3D(Self&& other) = default;

  inline explicit constexpr Twist3D(const msg& msg)
    : vector(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
  {
  }
  inline explicit constexpr Twist3D(const msg_stamped& msg) : Twist3D(msg.twist)  // delegate
  {
  }

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename VectorCtor>
  inline constexpr Twist3D(const VectorCtor& init) : vector(init)
  {
  }

  template <typename TranslationCtor, typename RotationCtor>
  inline constexpr Twist3D(const TranslationCtor& translation_init, const RotationCtor& rotation_init) : vector()
  {
    translation() = translation_init;
    rotation() = rotation_init;
  }

  Self& setNeutral()
  {
    vector.setZero();
    return *this;
  }
  static const Self Neutral;

  inline TranslationType translation()
  {
    return vector.segment(0, 3);
  }
  inline TranslationTypeConst translation() const
  {
    return vector.segment(0, 3);
  }

  inline RotationType rotation()
  {
    return vector.segment(3, 3);
  }
  inline RotationTypeConst rotation() const
  {
    return vector.segment(3, 3);
  }

  inline Self& operator+=(const Self& other)
  {
    vector += other.vector;
    return *this;
  }

  inline Self operator+(const Self& other) const
  {
    return Self(vector + other.vector);
  }

  inline Self& operator-=(const Self& other)
  {
    vector -= other.vector;
    return *this;
  }

  inline Self operator-(const Self& other) const
  {
    return Self(vector - other.vector);
  }

  inline Self& operator*=(const Scalar& scalar)
  {
    vector *= scalar;
    return *this;
  }

  inline Self operator*(const Scalar& scalar) const
  {
    return Self(vector * scalar);
  }
};

// static initializations
template <typename ScalarT>
const Twist3D<ScalarT> Twist3D<ScalarT>::Neutral = Twist3D<ScalarT>().setNeutral();

// Type Defs
template <typename ScalarT = double>
using TimedTwist3D = Timed<Twist3D<ScalarT>>;
template <typename ScalarT = double>
using StampedTwist3D = Stamped<Twist3D<ScalarT>>;

// Explicit Types
using Twist3Dd = Twist3D<double>;
using TimedTwist3Dd = TimedTwist3D<double>;
using StampedTwist3Dd = StampedTwist3D<double>;

}  // namespace duatic::geometry

// streaming
template <typename ScalarT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Twist3D<ScalarT>& twist)
{
  os << "Twist3D(translation: " << twist.translation().transpose() << ", rotation: " << twist.rotation().transpose()
     << ")";
  return os;
}
