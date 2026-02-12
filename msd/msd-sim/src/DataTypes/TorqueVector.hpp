#ifndef TORQUE_VECTOR_HPP
#define TORQUE_VECTOR_HPP

#include "msd-sim/src/DataTypes/Vec3DBase.hpp"
#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"
#include "msd-transfer/src/TorqueVectorRecord.hpp"

namespace msd_sim
{

/**
 * @brief 3D torque vector type [N*m] with transfer object support
 *
 * Thin wrapper around Vec3DBase providing:
 * - Full Eigen matrix operation compatibility
 * - fromRecord/toRecord for database serialization
 * - std::format support
 *
 * Memory footprint: 24 bytes (same as Eigen::Vector3d)
 */
struct TorqueVector final : detail::Vec3DBase<TorqueVector>
{
  using Vec3DBase::Vec3DBase;
  using Vec3DBase::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  TorqueVector(const Eigen::MatrixBase<OtherDerived>& other) : Vec3DBase{other}
  {
  }

  // Transfer methods
  static TorqueVector fromRecord(const msd_transfer::TorqueVectorRecord& record)
  {
    return TorqueVector{record.x, record.y, record.z};
  }

  [[nodiscard]] msd_transfer::TorqueVectorRecord toRecord() const
  {
    msd_transfer::TorqueVectorRecord record;
    record.x = x();
    record.y = y();
    record.z = z();
    return record;
  }
};

}  // namespace msd_sim

template <>
struct std::formatter<msd_sim::TorqueVector>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::TorqueVector>
{
  auto format(const msd_sim::TorqueVector& vec, std::format_context& ctx) const
  {
    return formatComponents(
      vec, [](const auto& v) { return std::tuple{v.x(), v.y(), v.z()}; }, ctx);
  }
};

#endif  // TORQUE_VECTOR_HPP
