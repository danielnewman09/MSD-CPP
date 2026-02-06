// Ticket: vector_datatype_refactor
// Quaternion wrapper with transfer object support

#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include "msd-sim/src/DataTypes/QuatDBase.hpp"
#include "msd-sim/src/DataTypes/Vec4FormatterBase.hpp"

#include "msd-transfer/src/QuaternionDRecord.hpp"

namespace msd_sim
{

/**
 * @brief Quaternion type with transfer object support
 *
 * Thin wrapper around Eigen::Quaterniond providing:
 * - Access to underlying Eigen quaternion via eigen()
 * - fromRecord/toRecord for database serialization
 * - std::format support
 *
 * Uses Eigen/Hamilton convention: q = w + xi + yj + zk
 *
 * Memory footprint: 32 bytes (same as Eigen::Quaterniond)
 */
struct QuaternionD final : detail::QuatDBase<QuaternionD>
{
  using QuatDBase::QuatDBase;
  using QuatDBase::operator=;

  // Transfer methods
  static QuaternionD fromRecord(const msd_transfer::QuaternionDRecord& record)
  {
    return QuaternionD{record.w, record.x, record.y, record.z};
  }

  [[nodiscard]] msd_transfer::QuaternionDRecord toRecord() const
  {
    msd_transfer::QuaternionDRecord record;
    record.w = w();
    record.x = x();
    record.y = y();
    record.z = z();
    return record;
  }

  // Rule of Zero
  QuaternionD(const QuaternionD&) = default;
  QuaternionD(QuaternionD&&) noexcept = default;
  QuaternionD& operator=(const QuaternionD&) = default;
  QuaternionD& operator=(QuaternionD&&) noexcept = default;
  ~QuaternionD() = default;
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::QuaternionD>
  : msd_sim::detail::Vec4FormatterBase<msd_sim::QuaternionD>
{
  auto format(const msd_sim::QuaternionD& quat, std::format_context& ctx) const
  {
    return formatComponents(
      quat,
      [](const auto& q) { return std::tuple{q.w(), q.x(), q.y(), q.z()}; },
      ctx);
  }
};

#endif  // QUATERNION_HPP
