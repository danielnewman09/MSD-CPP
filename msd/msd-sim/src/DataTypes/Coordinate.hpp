#ifndef COORDINATE_HPP
#define COORDINATE_HPP

// NOLINTBEGIN(bugprone-crtp-constructor-accessibility)

#include "Vec3FormatterBase.hpp"

#include <Eigen/Dense>

namespace msd_sim
{
namespace detail
{

template <typename Derived>
class Vec3Base : public Eigen::Vector3d
{
public:
  static constexpr Eigen::Index X = 0;
  static constexpr Eigen::Index Y = 1;
  static constexpr Eigen::Index Z = 2;

  Vec3Base() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  Vec3Base(double x, double y, double z) : Eigen::Vector3d{x, y, z}
  {
  }

  // NOLINTNEXTLINE(google-explicit-constructor)
  Vec3Base(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec}
  {
  }

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Vec3Base(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  template <typename OtherDerived>
  Vec3Base& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }
};

}  // namespace detail

struct Coordinate final : detail::Vec3Base<Coordinate>
{
  using Vec3Base::Vec3Base;
  using Vec3Base::operator=;

  template <typename OtherDerived>
  // NOLINTNEXTLINE(google-explicit-constructor)
  Coordinate(const Eigen::MatrixBase<OtherDerived>& other) : Vec3Base{other}
  {
  }
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

// NOLINTEND(bugprone-crtp-constructor-accessibility)

#endif  // COORDINATE_HPP