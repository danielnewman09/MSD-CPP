// Ticket: vector_datatype_refactor
// Base CRTP template for 3D vector types

#ifndef VEC3D_BASE_HPP
#define VEC3D_BASE_HPP

// NOLINTBEGIN(bugprone-crtp-constructor-accessibility)

#include <Eigen/Dense>

namespace msd_sim::detail
{

/**
 * @brief CRTP base class for 3D vector types
 *
 * Provides common functionality for all 3D vector types by inheriting
 * from Eigen::Vector3d. Derived types should use this as:
 *
 *   struct MyVec3Type final : Vec3DBase<MyVec3Type> { ... };
 *
 * @tparam Derived The derived type (CRTP pattern)
 */
template <typename Derived>
class Vec3DBase : public Eigen::Vector3d
{
public:
  static constexpr Eigen::Index X = 0;
  static constexpr Eigen::Index Y = 1;
  static constexpr Eigen::Index Z = 2;

  Vec3DBase() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  Vec3DBase(double x, double y, double z) : Eigen::Vector3d{x, y, z}
  {
  }

  // NOLINTNEXTLINE(google-explicit-constructor)
  Vec3DBase(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec}
  {
  }

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Vec3DBase(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  template <typename OtherDerived>
  Vec3DBase& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }

  // Rule of Zero - use compiler-generated special members
  Vec3DBase(const Vec3DBase&) = default;
  Vec3DBase(Vec3DBase&&) noexcept = default;
  Vec3DBase& operator=(const Vec3DBase&) = default;
  Vec3DBase& operator=(Vec3DBase&&) noexcept = default;
  ~Vec3DBase() = default;
};

}  // namespace msd_sim::detail

// NOLINTEND(bugprone-crtp-constructor-accessibility)

#endif  // VEC3D_BASE_HPP
