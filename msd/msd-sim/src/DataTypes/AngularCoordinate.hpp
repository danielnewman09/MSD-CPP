// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#ifndef ANGULAR_COORDINATE_HPP
#define ANGULAR_COORDINATE_HPP

#include "Vec3FormatterBase.hpp"

#include <Eigen/Dense>
#include <cmath>

#include "msd-transfer/src/AngularCoordinateRecord.hpp"

namespace msd_sim
{

/**
 * @brief Orientation angles with deferred normalization
 *
 * Inherits from Eigen::Vector3d for full matrix operation support.
 * Provides semantic pitch/roll/yaw accessors.
 *
 * Normalization strategy:
 * - Values are normalized to (-π, π] only when |value| exceeds
 * kNormalizationThreshold
 * - Check occurs in assignment and compound operators (+=, -=, *=, /=)
 * - Accessors return raw values (fast, no normalization check)
 * - Use normalized() or normalize() for explicit normalization to (-π, π]
 *
 * LIMITATION: Direct access via operator[] bypasses normalization.
 * Use semantic accessors (pitch(), roll(), yaw()) and setters for normal use.
 *
 * Axis convention (ZYX intrinsic rotation):
 * - pitch: Rotation around Y-axis (component 0)
 * - roll:  Rotation around X-axis (component 1)
 * - yaw:   Rotation around Z-axis (component 2)
 *
 * Memory footprint: 24 bytes (same as Eigen::Vector3d)
 *
 * @see docs/designs/0024_angular_coordinate/0024_angular_coordinate.puml
 * @ticket 0024_angular_coordinate
 */
class AngularCoordinate final : public Eigen::Vector3d
{
public:
  /// Normalization threshold (~50 revolutions). Values exceeding this trigger
  /// normalization.
  static constexpr double kNormalizationThreshold = 100.0 * M_PI;

  // Default constructor - initializes to (0, 0, 0)
  AngularCoordinate() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  // Constructor with pitch, roll, yaw values (in radians)
  AngularCoordinate(double pitch, double roll, double yaw)
    : Eigen::Vector3d{pitch, roll, yaw}
  {
    normalizeIfNeeded();
  }

  // Template constructor for Eigen expressions
  template <typename OtherDerived>
  AngularCoordinate(const Eigen::MatrixBase<OtherDerived>&
                      other)  // NOLINT(google-explicit-constructor)
    : Eigen::Vector3d{other}
  {
    normalizeIfNeeded();
  }

  // Template assignment for Eigen expressions
  template <typename OtherDerived>
  AngularCoordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    normalizeIfNeeded();
    return *this;
  }

  // Override compound operators for complete normalization coverage
  template <typename OtherDerived>
  AngularCoordinate& operator+=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator+=(other);
    normalizeIfNeeded();
    return *this;
  }

  template <typename OtherDerived>
  AngularCoordinate& operator-=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator-=(other);
    normalizeIfNeeded();
    return *this;
  }

  AngularCoordinate& operator*=(double scalar)
  {
    this->Eigen::Vector3d::operator*=(scalar);
    normalizeIfNeeded();
    return *this;
  }

  AngularCoordinate& operator/=(double scalar)
  {
    this->Eigen::Vector3d::operator/=(scalar);
    normalizeIfNeeded();
    return *this;
  }

  // Fast accessors (no normalization check - returns raw value)
  [[nodiscard]] double pitch() const
  {
    return (*this)[0];
  }

  [[nodiscard]] double roll() const
  {
    return (*this)[1];
  }

  [[nodiscard]] double yaw() const
  {
    return (*this)[2];
  }

  // Degree accessors (convenience)
  [[nodiscard]] double pitchDeg() const
  {
    return pitch() * 180.0 / M_PI;
  }

  [[nodiscard]] double rollDeg() const
  {
    return roll() * 180.0 / M_PI;
  }

  [[nodiscard]] double yawDeg() const
  {
    return yaw() * 180.0 / M_PI;
  }

  // Setters with normalization check
  void setPitch(double radians)
  {
    (*this)[0] = radians;
    if (std::abs((*this)[0]) > kNormalizationThreshold)
    {
      (*this)[0] = normalizeAngle((*this)[0]);
    }
  }

  void setRoll(double radians)
  {
    (*this)[1] = radians;
    if (std::abs((*this)[1]) > kNormalizationThreshold)
    {
      (*this)[1] = normalizeAngle((*this)[1]);
    }
  }

  void setYaw(double radians)
  {
    (*this)[2] = radians;
    if (std::abs((*this)[2]) > kNormalizationThreshold)
    {
      (*this)[2] = normalizeAngle((*this)[2]);
    }
  }

  // Explicit full normalization to (-π, π]
  [[nodiscard]] AngularCoordinate normalized() const
  {
    AngularCoordinate result{*this};
    result.normalize();
    return result;
  }

  void normalize()
  {
    (*this)[0] = normalizeAngle((*this)[0]);
    (*this)[1] = normalizeAngle((*this)[1]);
    (*this)[2] = normalizeAngle((*this)[2]);
  }

  // Transfer methods
  static AngularCoordinate fromRecord(
    const msd_transfer::AngularCoordinateRecord& record)
  {
    return AngularCoordinate{record.pitch, record.roll, record.yaw};
  }

  [[nodiscard]] msd_transfer::AngularCoordinateRecord toRecord() const
  {
    msd_transfer::AngularCoordinateRecord record;
    record.pitch = pitch();
    record.roll = roll();
    record.yaw = yaw();
    return record;
  }

  // Rule of Zero
  AngularCoordinate(const AngularCoordinate&) = default;
  AngularCoordinate(AngularCoordinate&&) noexcept = default;
  AngularCoordinate& operator=(const AngularCoordinate&) = default;
  AngularCoordinate& operator=(AngularCoordinate&&) noexcept = default;
  ~AngularCoordinate() = default;

private:
  /// Normalize only if any component exceeds threshold
  void normalizeIfNeeded()
  {
    if (std::abs((*this)[0]) > kNormalizationThreshold)
    {
      (*this)[0] = normalizeAngle((*this)[0]);
    }
    if (std::abs((*this)[1]) > kNormalizationThreshold)
    {
      (*this)[1] = normalizeAngle((*this)[1]);
    }
    if (std::abs((*this)[2]) > kNormalizationThreshold)
    {
      (*this)[2] = normalizeAngle((*this)[2]);
    }
  }

  /// Normalize angle to (-π, π]
  static double normalizeAngle(double rad)
  {
    double const result = std::fmod(rad + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
    {
      return result + M_PI;
    }
    return result - M_PI;
  }
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::AngularCoordinate>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::AngularCoordinate>
{
  auto format(const msd_sim::AngularCoordinate& coord,
              std::format_context& ctx) const
  {
    return formatComponents(
      coord,
      [](const auto& c) { return std::tuple{c.pitch(), c.roll(), c.yaw()}; },
      ctx);
  }
};

#endif  // ANGULAR_COORDINATE_HPP
