// Ticket: vector_datatype_refactor
// Formatter base for 4D vector types

#ifndef VEC4_FORMATTER_BASE_HPP
#define VEC4_FORMATTER_BASE_HPP

#include <format>
#include <string>

namespace msd_sim::detail
{

/**
 * @brief Shared formatter base for 4-component vector types
 *
 * Provides parse() for [width][.precision][type] format specs
 * and formatComponents() to emit "(c0, c1, c2, c3)" output.
 */
template <typename T>
struct Vec4FormatterBase
{
  char presentation = 'f';
  int precision = 6;
  int width = 0;

  constexpr auto parse(std::format_parse_context& ctx)
  {
    const auto* it = ctx.begin();
    const auto* end = ctx.end();

    if (it == end || *it == '}')
    {
      return it;
    }

    // Parse optional width
    if (it != end && *it >= '0' && *it <= '9')
    {
      width = 0;
      while (it != end && *it >= '0' && *it <= '9')
      {
        width = (width * 10) + (*it - '0');
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
        precision = (precision * 10) + (*it - '0');
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

protected:
  [[nodiscard]] std::string buildComponentFormat() const
  {
    std::string componentFmt = "{:";
    if (width > 0)
    {
      componentFmt += std::to_string(width);
    }
    componentFmt += '.';
    componentFmt += std::to_string(precision);
    componentFmt += presentation;
    componentFmt += '}';
    return componentFmt;
  }

  template <typename F>
  auto formatComponents(const T& vec,
                        F&& accessor,
                        std::format_context& ctx) const
  {
    auto [c0, c1, c2, c3] = std::forward<F>(accessor)(vec);
    const std::string componentFmt = buildComponentFormat();
    return std::format_to(
      ctx.out(),
      "({}, {}, {}, {})",
      std::vformat(componentFmt, std::make_format_args(c0)),
      std::vformat(componentFmt, std::make_format_args(c1)),
      std::vformat(componentFmt, std::make_format_args(c2)),
      std::vformat(componentFmt, std::make_format_args(c3)));
  }
};

}  // namespace msd_sim::detail

#endif  // VEC4_FORMATTER_BASE_HPP
