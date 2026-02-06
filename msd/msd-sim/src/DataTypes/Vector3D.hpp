// Ticket: vector_datatype_refactor
// Generic 3D vector wrapper with transfer object support

#ifndef VECTOR3D_HPP
#define VECTOR3D_HPP

#include "msd-sim/src/DataTypes/Vec3DBase.hpp"
#include "msd-sim/src/DataTypes/Vec3FormatterBase.hpp"
#include "msd-transfer/src/Vector3DRecord.hpp"

namespace msd_sim
{

/**
 * @brief Generic 3D vector type with transfer object support
 *
 * Thin wrapper around msd_sim::Vector3D providing:
 * - Full Eigen matrix operation compatibility
 * - fromRecord/toRecord for database serialization
 * - std::format support
 *
 * Use this type for generic 3D vector data that needs to be
 * recorded/transferred. For semantic types (positions, velocities),
 * prefer the domain-specific types like Coordinate.
 * 
 * Memory footprint: 24 bytes (same as msd_sim::Vector3D)
 */
struct Vector3D final : detail::Vec3DBase<Vector3D>
{
  using Vec3DBase::Vec3DBase;
  using Vec3DBase::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Vector3D(const Eigen::MatrixBase<OtherDerived>& other) : Vec3DBase{other}
  {
  }

  // Transfer methods
  static Vector3D fromRecord(const msd_transfer::Vector3DRecord& record)
  {
    return Vector3D{record.x, record.y, record.z};
  }

  [[nodiscard]] msd_transfer::Vector3DRecord toRecord() const
  {
    msd_transfer::Vector3DRecord record;
    record.x = x();
    record.y = y();
    record.z = z();
    return record;
  }

  // Rule of Zero
  Vector3D(const Vector3D&) = default;
  Vector3D(Vector3D&&) noexcept = default;
  Vector3D& operator=(const Vector3D&) = default;
  Vector3D& operator=(Vector3D&&) noexcept = default;
  ~Vector3D() = default;
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::Vector3D>
  : msd_sim::detail::Vec3FormatterBase<msd_sim::Vector3D>
{
  auto format(const msd_sim::Vector3D& vec, std::format_context& ctx) const
  {
    return formatComponents(
      vec, [](const auto& v) { return std::tuple{v.x(), v.y(), v.z()}; }, ctx);
  }
};

#endif  // VECTOR3D_HPP
