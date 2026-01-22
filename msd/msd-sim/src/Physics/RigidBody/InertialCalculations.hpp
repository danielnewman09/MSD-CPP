#ifndef MSD_SIM_PHYSICS_INERTIAL_CALCULATIONS_HPP
#define MSD_SIM_PHYSICS_INERTIAL_CALCULATIONS_HPP

#include <Eigen/Dense>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

class ConvexHull;

/**
 * @brief Utility functions for computing inertial properties from convex hulls.
 *
 * These functions compute mass properties (moment of inertia) from convex hull
 * geometry. They are used by AssetInertial to derive rigid body properties from
 * collision geometry.
 *
 * Note: For centroid calculation, use ConvexHull::getCentroid() directly.
 */
namespace InertialCalculations
{
/**
 * @brief Compute the moment of inertia tensor about the centroid.
 *
 * Computes the inertia tensor about the hull's center of mass assuming
 * uniform density. This is the standard form for rigid body dynamics.
 *
 * Uses Brian Mirtich's algorithm from "Fast and Accurate Computation of
 * Polyhedral Mass Properties" (1996). The algorithm applies the divergence
 * theorem to convert volume integrals to surface integrals through three
 * layers: projection integrals → face integrals → volume integrals.
 *
 * This produces results within machine precision of analytical solutions
 * (< 1e-10 error).
 *
 * Ticket: 0026_mirtich_inertia_tensor
 * Design: docs/designs/0026_mirtich_inertia_tensor/design.md
 *
 * @param hull The convex hull geometry
 * @param density Density in kilograms [kg/m^3]
 * @return 3x3 inertia tensor about centroid [kg⋅m²]
 * @throws std::invalid_argument if density <= 0
 * @throws std::runtime_error if hull is invalid or degenerate
 */
Eigen::Matrix3d computeInertiaTensorAboutCentroid(const ConvexHull& hull,
                                                  double density);

}  // namespace InertialCalculations

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INERTIAL_CALCULATIONS_HPP
