#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <Eigen/Dense>
#include <format>

namespace msd_sim
{
namespace detail
{

template <typename Derived>
class Vec3Base : public Eigen::Vector3d
{
public:
  static inline constexpr Eigen::Index X = 0;
  static inline constexpr Eigen::Index Y = 1;
  static inline constexpr Eigen::Index Z = 2;

  Vec3Base() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  Vec3Base(double x, double y, double z) : Eigen::Vector3d{x, y, z}
  {
  }

  Vec3Base(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec}
  {
  }

  template <typename OtherDerived>
  Vec3Base(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  template <typename OtherDerived>
  Derived& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    return static_cast<Derived&>(*this);
  }
};


}  // namespace detail

struct Coordinate : detail::Vec3Base<Coordinate>
{
  using Vec3Base::Vec3Base;
  using Vec3Base::operator=;

  template <typename OtherDerived>
  Coordinate(const Eigen::MatrixBase<OtherDerived>& other) : Vec3Base{other}
  {
  }
};

struct CoordinateRate : detail::Vec3Base<CoordinateRate>
{
  using Vec3Base::Vec3Base;
  using Vec3Base::operator=;

  template <typename OtherDerived>
  CoordinateRate(const Eigen::MatrixBase<OtherDerived>& other) : Vec3Base{other}
  {
  }
};


}  // namespace msd_sim

// Formatter specialization for std::format support
// This must be in the std namespace
template <>
struct std::formatter<msd_sim::Coordinate>
{
  // Format specification: can be empty or contain floating-point format specs
  // Examples: "{}", "{:.2f}", "{:10.3f}"
  char presentation = 'f';
  int precision = 6;
  int width = 0;

  // Parse the format specification
  constexpr auto parse(std::format_parse_context& ctx)
  {
    auto it = ctx.begin();
    auto end = ctx.end();

    // If no format spec, use defaults
    if (it == end || *it == '}')
      return it;

    // Simple parser for optional width and precision
    // Format: [width][.precision][type]
    // Example: "10.2f" means width=10, precision=2, type='f'

    // Parse optional width
    if (it != end && *it >= '0' && *it <= '9')
    {
      width = 0;
      while (it != end && *it >= '0' && *it <= '9')
      {
        width = width * 10 + (*it - '0');
        ++it;
      }
    }

    // Parse optional precision
    if (it != end && *it == '.')
    {
      ++it;
      precision = 0;
      while (it != end && *it >= '0' && *it <= '9')
      {
        precision = precision * 10 + (*it - '0');
        ++it;
      }
    }

    // Parse optional presentation type
    if (it != end && (*it == 'f' || *it == 'e' || *it == 'g'))
    {
      presentation = *it;
      ++it;
    }

    return it;
  }

  // Format the Coordinate object
  auto format(const msd_sim::Coordinate& coord, std::format_context& ctx) const
  {
    // Build format string for individual components
    std::string component_fmt = "{:";
    if (width > 0)
      component_fmt += std::to_string(width);
    component_fmt += '.';
    component_fmt += std::to_string(precision);
    component_fmt += presentation;
    component_fmt += '}';

    // Format as "(x, y, z)"
    return std::format_to(
      ctx.out(),
      "({}, {}, {})",
      std::vformat(component_fmt, std::make_format_args(coord.x())),
      std::vformat(component_fmt, std::make_format_args(coord.y())),
      std::vformat(component_fmt, std::make_format_args(coord.z())));
  }
};

#endif  // COORDINATE_HPP