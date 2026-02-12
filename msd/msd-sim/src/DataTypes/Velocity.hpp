#ifndef VELOCITY_HPP
#define VELOCITY_HPP

#include "msd-sim/src/DataTypes/Vec3DBase.hpp"
#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"
#include "msd-transfer/src/VelocityRecord.hpp"

namespace msd_sim
{

/**
 * @brief 3D velocity vector type [m/s] with transfer object support
 *
 * Thin wrapper around Vec3DBase providing:
 * - Full Eigen matrix operation compatibility
 * - fromRecord/toRecord for database serialization
 * - std::format support
 *
 * Memory footprint: 24 bytes (same as Eigen::Vector3d)
 */
struct Velocity final : detail::Vec3DBase<Velocity>
{
  using Vec3DBase::Vec3DBase;
  using Vec3DBase::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Velocity(const Eigen::MatrixBase<OtherDerived>& other) : Vec3DBase{other}
  {
  }

  // Transfer methods
  static Velocity fromRecord(const msd_transfer::VelocityRecord& record)
  {
    return Velocity{record.x, record.y, record.z};
  }

  [[nodiscard]] msd_transfer::VelocityRecord toRecord() const
  {
    msd_transfer::VelocityRecord record;
    record.x = x();
    record.y = y();
    record.z = z();
    return record;
  }
};

}  // namespace msd_sim

template <>
struct std::formatter<msd_sim::Velocity>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::Velocity>
{
  auto format(const msd_sim::Velocity& vec, std::format_context& ctx) const
  {
    return formatComponents(
      vec, [](const auto& v) { return std::tuple{v.x(), v.y(), v.z()}; }, ctx);
  }
};

#endif  // VELOCITY_HPP
