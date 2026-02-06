// Ticket: vector_datatype_refactor
// Base CRTP template for 4D vector types

#ifndef VEC4D_BASE_HPP
#define VEC4D_BASE_HPP

// NOLINTBEGIN(bugprone-crtp-constructor-accessibility)

#include <Eigen/Dense>

namespace msd_sim::detail
{

/**
 * @brief CRTP base class for 4D vector types
 *
 * Provides common functionality for all 4D vector types by inheriting
 * from Eigen::Vector4d. Derived types should use this as:
 *
 *   struct MyVec4Type final : Vec4DBase<MyVec4Type> { ... };
 *
 * @tparam Derived The derived type (CRTP pattern)
 */
template <typename Derived>
class Vec4DBase : public Eigen::Vector4d
{
public:
  static constexpr Eigen::Index X = 0;
  static constexpr Eigen::Index Y = 1;
  static constexpr Eigen::Index Z = 2;
  static constexpr Eigen::Index W = 3;

  Vec4DBase() : Eigen::Vector4d{0.0, 0.0, 0.0, 0.0}
  {
  }

  Vec4DBase(double x, double y, double z, double w)
    : Eigen::Vector4d{x, y, z, w}
  {
  }

  // NOLINTNEXTLINE(google-explicit-constructor)
  Vec4DBase(const Eigen::Vector4d& vec) : Eigen::Vector4d{vec}
  {
  }

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Vec4DBase(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector4d{other}
  {
  }

  template <typename OtherDerived>
  Vec4DBase& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector4d::operator=(other);
    return *this;
  }

  // Rule of Zero - use compiler-generated special members
  Vec4DBase(const Vec4DBase&) = default;
  Vec4DBase(Vec4DBase&&) noexcept = default;
  Vec4DBase& operator=(const Vec4DBase&) = default;
  Vec4DBase& operator=(Vec4DBase&&) noexcept = default;
  ~Vec4DBase() = default;
};

}  // namespace msd_sim::detail

// NOLINTEND(bugprone-crtp-constructor-accessibility)

#endif  // VEC4D_BASE_HPP
