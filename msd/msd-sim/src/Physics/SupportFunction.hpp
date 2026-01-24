// Ticket: 0027a_expanding_polytope_algorithm (refactoring)
// Extracted from GJK.cpp and EPA.cpp to eliminate duplication

#ifndef MSD_SIM_PHYSICS_SUPPORT_FUNCTION_HPP
#define MSD_SIM_PHYSICS_SUPPORT_FUNCTION_HPP

#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{

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
                            const CoordinateRate& dir);

}  // namespace SupportFunction

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_SUPPORT_FUNCTION_HPP
