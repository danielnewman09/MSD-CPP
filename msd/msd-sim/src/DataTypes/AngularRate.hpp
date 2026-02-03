// Ticket: 0024_angular_coordinate
// Design: docs/designs/0024_angular_coordinate/design.md

#ifndef ANGULAR_RATE_HPP
#define ANGULAR_RATE_HPP

#include <Eigen/Dense>
#include <format>

namespace msd_sim
{

/**
 * @brief Angular rate vector (velocity or acceleration) without normalization
 *
 * Inherits from Eigen::Vector3d for full matrix operation support.
 * Provides semantic pitch/roll/yaw accessors without any normalization.
 * Rates can exceed 2π rad/s and should not be normalized.
 *
 * Units:
 * - Angular velocity: rad/s
 * - Angular acceleration: rad/s²
 *
 * Axis convention (ZYX intrinsic rotation):
 * - pitch: Rotation around Y-axis (component 0)
 * - roll:  Rotation around X-axis (component 1)
 * - yaw:   Rotation around Z-axis (component 2)
 *
 * Memory footprint: 24 bytes (same as Eigen::Vector3d)
 *
 * @see docs/designs/0024_angular_coordinate/0024_angular_coordinate.puml
 * @ticket 0024_angular_coordinate
 */
class AngularRate : public Eigen::Vector3d
{
public:
  // Default constructor - initializes to (0, 0, 0)
  AngularRate() : Eigen::Vector3d{0.0, 0.0, 0.0}
  {
  }

  // Constructor with pitch, roll, yaw rates
  AngularRate(double pitchRate, double rollRate, double yawRate)
    : Eigen::Vector3d{pitchRate, rollRate, yawRate}
  {
  }

  // Constructor from Eigen::Vector3d
  AngularRate(const Eigen::Vector3d& vec) : Eigen::Vector3d{vec}
  {
  }

  // Template constructor for Eigen expressions
  template <typename OtherDerived>
  AngularRate(const Eigen::MatrixBase<OtherDerived>& other)
    : Eigen::Vector3d{other}
  {
  }

  // Template assignment for Eigen expressions
  template <typename OtherDerived>
  AngularRate& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    this->Eigen::Vector3d::operator=(other);
    return *this;
  }

  // Semantic accessors (no normalization)
  double& pitch()
  {
    return (*this)[0];
  }

  double& roll()
  {
    return (*this)[1];
  }

  double& yaw()
  {
    return (*this)[2];
  }

  double pitch() const
  {
    return (*this)[0];
  }

  double roll() const
  {
    return (*this)[1];
  }

  double yaw() const
  {
    return (*this)[2];
  }

  // Rule of Zero
  AngularRate(const AngularRate&) = default;
  AngularRate(AngularRate&&) noexcept = default;
  AngularRate& operator=(const AngularRate&) = default;
  AngularRate& operator=(AngularRate&&) noexcept = default;
  ~AngularRate() = default;
};

}  // namespace msd_sim

// Formatter specialization for std::format support
template <>
struct std::formatter<msd_sim::AngularRate>
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

  // Format the AngularRate object
  auto format(const msd_sim::AngularRate& rate, std::format_context& ctx) const
  {
    // Build format string for individual components
    std::string component_fmt = "{:";
    if (width > 0)
      component_fmt += std::to_string(width);
    component_fmt += '.';
    component_fmt += std::to_string(precision);
    component_fmt += presentation;
    component_fmt += '}';

    // Get values as lvalues for std::make_format_args
    double pitchVal = rate.pitch();
    double rollVal = rate.roll();
    double yawVal = rate.yaw();

    // Format as "(pitch, roll, yaw)"
    return std::format_to(ctx.out(),
                          "({}, {}, {})",
                          std::vformat(component_fmt,
                                       std::make_format_args(pitchVal)),
                          std::vformat(component_fmt,
                                       std::make_format_args(rollVal)),
                          std::vformat(component_fmt,
                                       std::make_format_args(yawVal)));
  }
};

#endif  // ANGULAR_RATE_HPP
