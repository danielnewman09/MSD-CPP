// Ticket: vector_datatype_refactor
// Base CRTP template for quaternion types

#ifndef QUATD_BASE_HPP
#define QUATD_BASE_HPP

// NOLINTBEGIN(bugprone-crtp-constructor-accessibility)

#include <Eigen/Geometry>

namespace msd_sim::detail
{

/**
 * @brief CRTP base class for quaternion types
 *
 * Wraps Eigen::Quaterniond via composition (not inheritance, since
 * Eigen::Quaterniond is not a matrix type).
 *
 * Uses Eigen/Hamilton convention: q = w + xi + yj + zk
 *
 * @tparam Derived The derived type (CRTP pattern)
 */
template <typename Derived>
class QuatDBase
{
public:
  // Identity quaternion (w=1, x=y=z=0)
  QuatDBase() : quat_{Eigen::Quaterniond::Identity()}
  {
  }

  // Construct from components (w, x, y, z) - Eigen convention
  QuatDBase(double w, double x, double y, double z) : quat_{w, x, y, z}
  {
  }

  // NOLINTNEXTLINE(google-explicit-constructor)
  QuatDBase(Eigen::Quaterniond quat) : quat_{std::move(quat)}
  {
  }

  QuatDBase& operator=(const Eigen::Quaterniond& other)
  {
    quat_ = other;
    return *this;
  }

  // Component accessors (const)
  [[nodiscard]] double w() const
  {
    return quat_.w();
  }
  [[nodiscard]] double x() const
  {
    return quat_.x();
  }
  [[nodiscard]] double y() const
  {
    return quat_.y();
  }
  [[nodiscard]] double z() const
  {
    return quat_.z();
  }

  // Component accessors (mutable)
  double& w()
  {
    return quat_.w();
  }
  double& x()
  {
    return quat_.x();
  }
  double& y()
  {
    return quat_.y();
  }
  double& z()
  {
    return quat_.z();
  }

  // Access underlying Eigen quaternion
  [[nodiscard]] const Eigen::Quaterniond& eigen() const
  {
    return quat_;
  }

  [[nodiscard]] Eigen::Quaterniond& eigen()
  {
    return quat_;
  }

  // Quaternion multiplication
  [[nodiscard]] Derived operator*(const QuatDBase& other) const
  {
    return Derived{quat_ * other.quat_};
  }

  // Rotate a vector
  [[nodiscard]] Eigen::Vector3d operator*(const Eigen::Vector3d& v) const
  {
    return quat_ * v;
  }

  // Conversion to rotation matrix
  [[nodiscard]] Eigen::Matrix3d toRotationMatrix() const
  {
    return quat_.toRotationMatrix();
  }

  // Return normalized quaternion
  [[nodiscard]] Derived normalized() const
  {
    return Derived{quat_.normalized()};
  }

  // Normalize in place
  void normalize()
  {
    quat_.normalize();
  }

  // Quaternion norm (magnitude)
  [[nodiscard]] double norm() const
  {
    return quat_.norm();
  }

  // Squared norm (avoids sqrt)
  [[nodiscard]] double squaredNorm() const
  {
    return quat_.squaredNorm();
  }

  // Coefficient array access (x, y, z, w order in Eigen storage)
  [[nodiscard]] const Eigen::Quaterniond::Coefficients& coeffs() const
  {
    return quat_.coeffs();
  }

  [[nodiscard]] Eigen::Quaterniond::Coefficients& coeffs()
  {
    return quat_.coeffs();
  }

  // Rule of Zero
  QuatDBase(const QuatDBase&) = default;
  QuatDBase(QuatDBase&&) noexcept = default;
  QuatDBase& operator=(const QuatDBase&) = default;
  QuatDBase& operator=(QuatDBase&&) noexcept = default;
  ~QuatDBase() = default;

protected:
  Eigen::Quaterniond quat_;
};

}  // namespace msd_sim::detail

// NOLINTEND(bugprone-crtp-constructor-accessibility)

#endif  // QUATD_BASE_HPP
