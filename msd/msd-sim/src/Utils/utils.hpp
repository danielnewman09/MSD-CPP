#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>
#include <type_traits>

namespace msd_sim
{

// Helper function for comparing doubles with tolerance
constexpr double TOLERANCE = 1e-10;

template <std::floating_point T>
bool almostEqual(T a, T b, double tolerance = TOLERANCE)
{
  return std::abs(a - b) < tolerance;
}

}  // namespace msd_sim

#endif  // UTILS_HPP