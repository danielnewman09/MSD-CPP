// Ticket: 0027a_expanding_polytope_algorithm (refactoring)
// Extracted from GJK.cpp and EPA.cpp to eliminate duplication

#ifndef MSD_SIM_PHYSICS_SUPPORT_FUNCTION_HPP
#define MSD_SIM_PHYSICS_SUPPORT_FUNCTION_HPP

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{

/**
 * @brief Result of Minkowski support query with witness tracking.
 *
 * Contains both the Minkowski difference point and the original
 * support points on each object's surface that contributed to it.
 * All coordinates are in world space.
 *
 * @ticket 0028_epa_witness_points
 */
struct SupportResult
{
  Coordinate minkowski;  // supportA - supportB (Minkowski space)
  Coordinate witnessA;   // Support point on A's surface (world space)
  Coordinate witnessB;   // Support point on B's surface (world space)

  SupportResult() = default;
  SupportResult(const Coordinate& m, const Coordinate& wA, const Coordinate& wB)
    : minkowski{m}, witnessA{wA}, witnessB{wB}
  {
  }
};

/**
 * @brief Support function utilities for collision detection algorithms.
 *
 * Provides shared support function implementations used by both GJK and EPA
 * for Minkowski difference support queries. Extracting these to a common
 * location eliminates code duplication and ensures consistent behavior.
 *
 * @see GJK for collision detection
 * @see EPA for contact information extraction
 */
namespace SupportFunction
{

/**
 * @brief Find vertex furthest in given direction (local space).
 *
 * Searches for the vertex with maximum dot product in the given direction
 * within the hull's local coordinate system. This is the fundamental
 * operation for both GJK and EPA algorithms.
 *
 * @param hull Convex hull to query
 * @param dir Search direction in hull's local coordinate system
 * @return Vertex with maximum dot product with direction
 */
Coordinate support(const ConvexHull& hull, const Coordinate& dir);

/**
 * @brief Minkowski difference support with world-space transformations.
 *
 * Computes support(A, dir) - support(B, -dir) in world space,
 * applying ReferenceFrame transformations on-the-fly:
 *
 * 1. Transform search direction from world space to local space (rotation only)
 * 2. Find support vertex in local hull geometry
 * 3. Transform support vertex from local space to world space (rotation +
 *    translation)
 * 4. Compute Minkowski difference in world space
 *
 * @param assetA First physical asset (includes collision hull and reference
 * frame)
 * @param assetB Second physical asset (includes collision hull and reference
 * frame)
 * @param dir Search direction in world space
 * @return Support point in world space (supportA - supportB)
 */
Coordinate supportMinkowski(const AssetPhysical& assetA,
                            const AssetPhysical& assetB,
                            const Eigen::Vector3d& dir);

/**
 * @brief Minkowski difference support with witness point tracking.
 *
 * Computes support(A, dir) - support(B, -dir) in world space while
 * tracking the original witness points on each object's surface.
 * This enables accurate contact point extraction for collision response.
 *
 * Applies ReferenceFrame transformations on-the-fly:
 * 1. Transform search direction from world space to local space (rotation only)
 * 2. Find support vertex in local hull geometry
 * 3. Transform support vertex from local space to world space (rotation +
 *    translation)
 * 4. Compute Minkowski difference in world space
 *
 * @param assetA First physical asset (includes collision hull and reference
 * frame)
 * @param assetB Second physical asset (includes collision hull and reference
 * frame)
 * @param dir Search direction in world space
 * @return SupportResult with Minkowski point and witness points (all world
 * space)
 * @ticket 0028_epa_witness_points
 */
SupportResult supportMinkowskiWithWitness(const AssetPhysical& assetA,
                                          const AssetPhysical& assetB,
                                          const Eigen::Vector3d& dir);

}  // namespace SupportFunction

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_SUPPORT_FUNCTION_HPP
