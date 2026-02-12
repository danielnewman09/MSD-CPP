#ifndef ANGULAR_VELOCITY_HPP
#define ANGULAR_VELOCITY_HPP

#include <Eigen/Dense>

#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"

#include "msd-transfer/src/AngularVelocityRecord.hpp"

namespace msd_sim
{

/**
 * @brief Angular velocity vector [rad/s] without normalization
 *
 * Inherits from Eigen::Vector3d for full matrix operation support.
 * Provides semantic pitch/roll/yaw accessors without any normalization.
 * Rates can exceed 2pi rad/s and should not be normalized.
 *
 * Axis convention (ZYX intrinsic rotation):
 * - pitch: Rotation around Y-axis (component 0)
 * - roll:  Rotation around X-axis (component 1)
 * - yaw:   Rotation around Z-axis (component 2)
 *
 * Memory footprint: 24 bytes (same as Eigen::Vector3d)
 */
class AngularVelocity final : public Eigen::Vector3d
{
public:
  // Default constructor - initializes to (0, 0, 0)
  AngularVelocity() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  // Constructor with pitch, roll, yaw rates
  AngularVelocity(double pitchRate, double rollRate, double yawRate)
    : Eigen::Vector3d{pitchRate, rollRate, yawRate}
  {
  }

  // Constructor from Eigen::Vector3d
  // NOLINTNEXTLINE(google-explicit-constructor)
  AngularVelocity(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec}
  {
  }

  // Template constructor for Eigen expressions
  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  AngularVelocity(const Eigen::MatrixBase<OtherDerived>&
                    other)  // NOLINT(google-explicit-constructor)
    : Eigen::Vector3d{other}
  {
  }

  // Template assignment for Eigen expressions
  template <typename OtherDerived>
  AngularVelocity& operator=(const Eigen::MatrixBase<OtherDerived>& other)
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

  // Transfer methods
  static AngularVelocity fromRecord(
    const msd_transfer::AngularVelocityRecord& record)
  {
    return AngularVelocity{record.pitch, record.roll, record.yaw};
  }

  [[nodiscard]] msd_transfer::AngularVelocityRecord toRecord() const
  {
    msd_transfer::AngularVelocityRecord record;
    record.pitch = pitch();
    record.roll = roll();
    record.yaw = yaw();
    return record;
  }

  // Rule of Zero
  AngularVelocity(const AngularVelocity&) = default;
  AngularVelocity(AngularVelocity&&) noexcept = default;
  AngularVelocity& operator=(const AngularVelocity&) = default;
  AngularVelocity& operator=(AngularVelocity&&) noexcept = default;
  ~AngularVelocity() = default;
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::AngularVelocity>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::AngularVelocity>
{
  auto format(const msd_sim::AngularVelocity& rate,
              std::format_context& ctx) const
  {
    return formatComponents(
      rate,
      [](const auto& r) { return std::tuple{r.pitch(), r.roll(), r.yaw()}; },
      ctx);
  }
};

#endif  // ANGULAR_VELOCITY_HPP
