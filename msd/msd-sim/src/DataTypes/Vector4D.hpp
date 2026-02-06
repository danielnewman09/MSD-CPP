// Ticket: vector_datatype_refactor
// Generic 4D vector wrapper with transfer object support

#ifndef VECTOR4D_HPP
#define VECTOR4D_HPP

#include "msd-sim/src/DataTypes/Vec4DBase.hpp"
#include "msd-sim/src/DataTypes/Vec4FormatterBase.hpp"

#include "msd-transfer/src/Vector4DRecord.hpp"

namespace msd_sim
{

/**
 * @brief Generic 4D vector type with transfer object support
 *
 * Thin wrapper around Eigen::Vector4d providing:
 * - Full Eigen matrix operation compatibility
 * - fromRecord/toRecord for database serialization
 * - std::format support
 *
 * Commonly used for:
 * - Quaternions (w, x, y, z)
 * - Homogeneous coordinates (x, y, z, w)
 * - RGBA colors
 *
 * Memory footprint: 32 bytes (same as Eigen::Vector4d)
 */
struct Vector4D final : detail::Vec4DBase<Vector4D>
{
  using Vec4DBase::Vec4DBase;
  using Vec4DBase::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Vector4D(const Eigen::MatrixBase<OtherDerived>& other) : Vec4DBase{other}
  {
  }

  // Transfer methods
  static Vector4D fromRecord(const msd_transfer::Vector4DRecord& record)
  {
    return Vector4D{record.x, record.y, record.z, record.w};
  }

  [[nodiscard]] msd_transfer::Vector4DRecord toRecord() const
  {
    msd_transfer::Vector4DRecord record;
    record.x = x();
    record.y = y();
    record.z = z();
    record.w = w();
    return record;
  }

  // Rule of Zero
  Vector4D(const Vector4D&) = default;
  Vector4D(Vector4D&&) noexcept = default;
  Vector4D& operator=(const Vector4D&) = default;
  Vector4D& operator=(Vector4D&&) noexcept = default;
  ~Vector4D() = default;
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::Vector4D>
  : msd_sim::detail::Vec4FormatterBase<msd_sim::Vector4D>
{
  auto format(const msd_sim::Vector4D& vec, std::format_context& ctx) const
  {
    return formatComponents(
      vec,
      [](const auto& v) { return std::tuple{v.x(), v.y(), v.z(), v.w()}; },
      ctx);
  }
};

#endif  // VECTOR4D_HPP
