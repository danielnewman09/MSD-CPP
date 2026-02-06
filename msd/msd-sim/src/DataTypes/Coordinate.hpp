#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include "msd-sim/src/DataTypes/Vec3DBase.hpp"
#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"

#include "msd-transfer/src/CoordinateRecord.hpp"

namespace msd_sim
{

/**
 * @brief 3D spatial coordinate (position in space)
 *
 * Inherits from msd_sim::Vector3D via Vec3DBase for full matrix operation
 * support. Provides semantic x/y/z accessors and transfer object support.
 *
 * Memory footprint: 24 bytes (same as msd_sim::Vector3D)
 */
struct Coordinate final : detail::Vec3DBase<Coordinate>
{
  using Vec3DBase::Vec3DBase;
  using Vec3DBase::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Coordinate(const Eigen::MatrixBase<OtherDerived>& other) : Vec3DBase{other}
  {
  }

  // Transfer methods
  static Coordinate fromRecord(const msd_transfer::CoordinateRecord& record)
  {
    return Coordinate{record.x, record.y, record.z};
  }

  [[nodiscard]] msd_transfer::CoordinateRecord toRecord() const
  {
    msd_transfer::CoordinateRecord record;
    record.x = x();
    record.y = y();
    record.z = z();
    return record;
  }

  // Rule of Zero
  Coordinate(const Coordinate&) = default;
  Coordinate(Coordinate&&) noexcept = default;
  Coordinate& operator=(const Coordinate&) = default;
  Coordinate& operator=(Coordinate&&) noexcept = default;
  ~Coordinate() = default;
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::Coordinate>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::Coordinate>
{
  auto format(const msd_sim::Coordinate& coord, std::format_context& ctx) const
  {
    return formatComponents(
      coord,
      [](const auto& c) { return std::tuple{c.x(), c.y(), c.z()}; },
      ctx);
  }
};

#endif  // COORDINATE_HPP
