#ifndef GEOMETRY_TRAITS_HPP
#define GEOMETRY_TRAITS_HPP

#include <concepts>
#include <type_traits>

namespace msd_assets
{

// Forward declarations
class VisualGeometry;
class CollisionGeometry;

/**
 * @brief Concept requiring a type to be either VisualGeometry or CollisionGeometry
 *
 * This concept ensures compile-time type safety for geometry factory
 * methods and other templated geometry operations.
 */
template <typename T>
concept IsGeometry = std::is_same_v<T, VisualGeometry> || std::is_same_v<T, CollisionGeometry>;

/**
 * @brief Concept requiring a 3D vector type with x(), y(), z() accessors
 *
 * This concept matches types like Eigen::Vector3d or msd_sim::Coordinate
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
