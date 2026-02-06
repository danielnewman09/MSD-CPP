#ifndef GEOMETRY_TRAITS_HPP
#define GEOMETRY_TRAITS_HPP

#include <concepts>
#include <type_traits>

namespace msd_assets
{

/**
 * @brief Concept requiring a 3D vector type with x(), y(), z() accessors
 *
 * This concept matches types like msd_sim::Vector3D or msd_sim::Coordinate
 * that provide coordinate access methods returning floating-point values.
 */
template <typename T>
concept IsVector3 = requires(const T v) {
  { v.x() } -> std::convertible_to<double>;
  { v.y() } -> std::convertible_to<double>;
  { v.z() } -> std::convertible_to<double>;
};

}  // namespace msd_assets

#endif  // GEOMETRY_TRAITS_HPP
