#ifndef FORCE_VECTOR_HPP
#define FORCE_VECTOR_HPP

#include "msd-sim/src/DataTypes/Vec3DBase.hpp"
#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"
#include "msd-transfer/src/ForceVectorRecord.hpp"

namespace msd_sim
{

/**
 * @brief 3D force vector type [N] with transfer object support
 *
 * Thin wrapper around Vec3DBase providing:
 * - Full Eigen matrix operation compatibility
 * - fromRecord/toRecord for database serialization
 * - std::format support
 *
 * Memory footprint: 24 bytes (same as Eigen::Vector3d)
 */
struct ForceVector final : detail::Vec3DBase<ForceVector>
{
  using Vec3DBase::Vec3DBase;
  using Vec3DBase::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  ForceVector(const Eigen::MatrixBase<OtherDerived>& other) : Vec3DBase{other}
  {
  }

  // Transfer methods
  static ForceVector fromRecord(const msd_transfer::ForceVectorRecord& record)
  {
    return ForceVector{record.x, record.y, record.z};
  }

  [[nodiscard]] msd_transfer::ForceVectorRecord toRecord() const
  {
    msd_transfer::ForceVectorRecord record;
    record.x = x();
    record.y = y();
    record.z = z();
    return record;
  }
};

}  // namespace msd_sim

template <>
struct std::formatter<msd_sim::ForceVector>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::ForceVector>
{
  auto format(const msd_sim::ForceVector& vec, std::format_context& ctx) const
  {
    return formatComponents(
      vec, [](const auto& v) { return std::tuple{v.x(), v.y(), v.z()}; }, ctx);
  }
};

#endif  // FORCE_VECTOR_HPP
