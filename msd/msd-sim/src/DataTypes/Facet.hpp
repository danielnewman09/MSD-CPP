#ifndef FACET_HPP
#define FACET_HPP

#include <array>
#include <utility>

#include "msd-sim/src/DataTypes/Coordinate.hpp"

namespace msd_sim
{


/**
 * @brief Represents a triangular facet of the convex hull.
 *
 * Each facet stores indices into the hull's vertex array,
 * along with the outward-facing normal vector.
 */
struct Facet
{
  static constexpr size_t kFacetSize = 3;

  static size_t vertexSize()
  {
    return kFacetSize;
  }

  std::array<size_t,
             kFacetSize>
    vertexIndices{};    // Indices of
                      // triangle vertices
  Coordinate normal;  // Outward-facing unit normal
  double offset{};      // Distance from origin (for half-space)

  Facet() = default;
  Facet(size_t v0, size_t v1, size_t v2, Coordinate  n, double d)
    : vertexIndices{v0, v1, v2}, normal{std::move(n)}, offset{d}
  {
  }
};


}  // namespace msd_sim

#endif