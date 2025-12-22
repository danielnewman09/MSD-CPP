#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <Eigen/Dense>
#include <format>

namespace msd_sim
{

/**
 * @brief A 3D coordinate class that inherits from Eigen::Vector3d
 *
 * This class provides a convenient wrapper around Eigen::Vector3d with
 * additional constructors and methods specific to coordinate operations.
 * Uses double precision for numerical stability in physics simulations.
 */
class Coordinate : public Eigen::Vector3d
{
public:
  // Default constructor - initializes to (0, 0, 0)
  Coordinate() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  // Constructor with x, y, z values
  Coordinate(double x, double y, double z) : Eigen::Vector3d{x, y, z}
  {
  }

  // Constructor from Eigen::Vector3d
  Coordinate(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec}
  {
  }

  // This constructor allows you to construct Coordinate from Eigen expressions
  template <typename OtherDerived>
  Coordinate(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  // This method allows you to assign Eigen expressions to Coordinate
  template <typename OtherDerived>
  Coordinate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    return *this;
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
    return std::format_to(ctx.out(),
                          "({}, {}, {})",
                          std::vformat(component_fmt,
                                       std::make_format_args(coord.x())),
                          std::vformat(component_fmt,
                                       std::make_format_args(coord.y())),
                          std::vformat(component_fmt,
                                       std::make_format_args(coord.z())));
  }
};

#endif  // COORDINATE_HPP