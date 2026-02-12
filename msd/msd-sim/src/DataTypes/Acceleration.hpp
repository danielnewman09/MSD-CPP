#ifndef ACCELERATION_HPP
#define ACCELERATION_HPP

#include "msd-sim/src/DataTypes/Vec3DBase.hpp"
#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"
#include "msd-transfer/src/AccelerationRecord.hpp"

namespace msd_sim
{

/**
 * @brief 3D acceleration vector type [m/s^2] with transfer object support
 *
 * Thin wrapper around Vec3DBase providing:
 * - Full Eigen matrix operation compatibility
 * - fromRecord/toRecord for database serialization
 * - std::format support
 *
 * Memory footprint: 24 bytes (same as Eigen::Vector3d)
 */
struct Acceleration final : detail::Vec3DBase<Acceleration>
{
  using Vec3DBase::Vec3DBase;
  using Vec3DBase::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Acceleration(const Eigen::MatrixBase<OtherDerived>& other) : Vec3DBase{other}
  {
  }

  // Transfer methods
  static Acceleration fromRecord(
    const msd_transfer::AccelerationRecord& record)
  {
    return Acceleration{record.x, record.y, record.z};
  }

  [[nodiscard]] msd_transfer::AccelerationRecord toRecord() const
  {
    msd_transfer::AccelerationRecord record;
    record.x = x();
    record.y = y();
    record.z = z();
    return record;
  }
};

}  // namespace msd_sim

template <>
struct std::formatter<msd_sim::Acceleration>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::Acceleration>
{
  auto format(const msd_sim::Acceleration& vec, std::format_context& ctx) const
  {
    return formatComponents(
      vec, [](const auto& v) { return std::tuple{v.x(), v.y(), v.z()}; }, ctx);
  }
};

#endif  // ACCELERATION_HPP
