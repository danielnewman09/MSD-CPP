#ifndef ANGLE_HPP
#define ANGLE_HPP

#include <cmath>
#include <cstdint>

namespace msd_sim
{

// Conversion constants
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double TWO_PI = 2.0 * M_PI;

/**
 * @brief A class for representing angles with automatic normalization
 *
 * This class stores angles internally in radians and automatically normalizes
 * them based on the specified normalization mode. All arithmetic operations
 * preserve normalization automatically.
 */
class Angle
{
public:
  enum class Norm : uint8_t
  {
    PI,     // (-pi, pi]
    TWO_PI  // [0, 2pi)
  };

  // Default constructor - initializes to 0 radians with PI normalization
  Angle() : rad_{0.0}, normalization_{Norm::PI}
  {
  }

  // Constructor from radians with normalization
  explicit Angle(double radians, Norm norm = Norm::PI)
    : rad_{radians}, normalization_{norm}
  {
  }

  // Factory methods for clarity
  static Angle fromRadians(double radians, Norm norm = Norm::PI)
  {
    return Angle{radians, norm};
  }

  static Angle fromDegrees(double degrees, Norm norm = Norm::PI)
  {
    return Angle{degrees * DEG_TO_RAD, norm};
  }

  /**
   * Get the angle in radians
   */
  double getRad() const
  {
    return normalize();
  }

  void setRad(double rad)
  {
    rad_ = rad;
  }

  /**
   * Get the angle in degrees
   */
  double toDeg() const
  {
    return normalize() * RAD_TO_DEG;
  }

  Norm getNormalization() const
  {
    return normalization_;
  }

  // Arithmetic operators - return new Angle objects
  Angle operator+(const Angle& other) const
  {
    return Angle(rad_ + other.rad_, normalization_);
  }

  Angle operator-(const Angle& other) const
  {
    return Angle(rad_ - other.rad_, normalization_);
  }

  Angle operator*(double scalar) const
  {
    return Angle(rad_ * scalar, normalization_);
  }

  /**
   * Divide an angle by a scalar
   */
  Angle operator/(double scalar) const
  {
    return Angle(rad_ / scalar, normalization_);
  }

  /**
   * unary minus
   */
  Angle operator-() const
  {
    return Angle(-rad_, normalization_);
  }

  Angle& operator+=(const Angle& other)
  {
    rad_ += other.rad_;
    return *this;
  }

  Angle& operator-=(const Angle& other)
  {
    rad_ -= other.rad_;
    return *this;
  }

  Angle& operator*=(double scalar)
  {
    rad_ *= scalar;
    return *this;
  }

  Angle& operator/=(double scalar)
  {
    rad_ /= scalar;
    return *this;
  }

  // Assignment operator
  Angle& operator=(const Angle& other)
  {
    if (this != &other)
    {
      rad_ = other.rad_;
      normalization_ = other.normalization_;
    }
    return *this;
  }

  // Comparison operators
  bool operator==(const Angle& other) const
  {
    return std::abs(rad_ - other.rad_) < 1e-10;
  }

  bool operator!=(const Angle& other) const
  {
    return !(*this == other);
  }

  bool operator<(const Angle& other) const
  {
    return rad_ < other.rad_;
  }

  bool operator>(const Angle& other) const
  {
    return rad_ > other.rad_;
  }

  bool operator<=(const Angle& other) const
  {
    return rad_ <= other.rad_;
  }

  bool operator>=(const Angle& other) const
  {
    return rad_ >= other.rad_;
  }

  // Change normalization mode
  void setNormalization(Norm norm)
  {
    normalization_ = norm;
  }

private:
  // The ground truth for all angles is stored in radians
  double rad_;

  // The normalization for this angle
  Norm normalization_;

  // Normalize the internal angle
  double normalize() const
  {
    double normAngle = rad_;
    switch (normalization_)
    {
      case Norm::PI:
        if (std::abs(normAngle) > M_PI)
        {
          normAngle = std::fmod(rad_ + M_PI, TWO_PI);
          if (normAngle < 0.)
          {
            normAngle += 2 * M_PI;
          }
          normAngle -= M_PI;
        }
        break;
      case Norm::TWO_PI:
        normAngle = std::fmod(rad_, TWO_PI);
        if (normAngle < 0.0)
        {
          normAngle += TWO_PI;
        }
        break;
      default:
        break;
    }
    return normAngle;
  }
};

// Allow scalar * Angle
inline Angle operator*(double scalar, const Angle& angle)
{
  return angle * scalar;
}

}  // namespace msd_sim

#endif  // ANGLE_HPP