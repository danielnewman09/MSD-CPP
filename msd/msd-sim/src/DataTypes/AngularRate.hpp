// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#ifndef ANGULAR_RATE_HPP
#define ANGULAR_RATE_HPP

#include "Vec3FormatterBase.hpp"

#include <Eigen/Dense>

namespace msd_sim
{

/**
 * @brief Angular rate vector (velocity or acceleration) without normalization
 *
 * Inherits from Eigen::Vector3d for full matrix operation support.
 * Provides semantic pitch/roll/yaw accessors without any normalization.
 * Rates can exceed 2π rad/s and should not be normalized.
 *
 * Units:
 * - Angular velocity: rad/s
 * - Angular acceleration: rad/s²
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
class AngularRate final : public Eigen::Vector3d
{
public:
  // Default constructor - initializes to (0, 0, 0)
  AngularRate() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  // Constructor with pitch, roll, yaw rates
  AngularRate(double pitchRate, double rollRate, double yawRate)
    : Eigen::Vector3d{pitchRate, rollRate, yawRate}
  {
  }

  // Constructor from Eigen::Vector3d
  AngularRate(const Eigen::Vector3d& vec)  // NOLINT(google-explicit-constructor)
    : Eigen::Vector3d{vec}
  {
  }

  // Template constructor for Eigen expressions
  template <typename OtherDerived>
  AngularRate(const Eigen::MatrixBase<OtherDerived>& other)  // NOLINT(google-explicit-constructor)
    : Eigen::Vector3d{other}
  {
  }

  // Template assignment for Eigen expressions
  template <typename OtherDerived>
  AngularRate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }

  // Semantic accessors (no normalization)
  double& pitch()
  {
    return (*this)[0];
  }

  double& roll()
  {
    return (*this)[1];
  }

  double& yaw()
  {
    return (*this)[2];
  }

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

  // Rule of Zero
  AngularRate(const AngularRate&) = default;
  AngularRate(AngularRate&&) noexcept = default;
  AngularRate& operator=(const AngularRate&) = default;
  AngularRate& operator=(AngularRate&&) noexcept = default;
  ~AngularRate() = default;
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::AngularRate>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::AngularRate>
{
  auto format(const msd_sim::AngularRate& rate,
              std::format_context& ctx) const
  {
    return formatComponents(
      rate,
      [](const auto& r)
      { return std::tuple{r.pitch(), r.roll(), r.yaw()}; },
      ctx);
  }
};

#endif  // ANGULAR_RATE_HPP
