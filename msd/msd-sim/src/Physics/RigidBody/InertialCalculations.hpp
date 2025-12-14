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
 * @brief Compute the moment of inertia tensor about the origin.
 *
 * Computes the inertia tensor for a uniform-density solid with the given
 * mass, with the tensor computed about the coordinate system origin.
 *
 * @param hull The convex hull geometry
 * @param mass Mass in kilograms [kg]
 * @return 3x3 inertia tensor about origin [kg⋅m²]
 * @throws std::invalid_argument if mass <= 0
 * @throws std::runtime_error if hull is degenerate (volume <= 0)
 */
Eigen::Matrix3d computeInertiaTensorAboutOrigin(const ConvexHull& hull,
                                                double mass);

/**
 * @brief Compute the moment of inertia tensor about the centroid.
 *
 * Computes the inertia tensor about the hull's center of mass assuming
 * uniform density. This is the most useful form for rigid body dynamics.
 *
 * Uses the parallel axis theorem to transform from the origin-based tensor.
 *
 * @param hull The convex hull geometry
 * @param mass Mass in kilograms [kg]
 * @return 3x3 inertia tensor about centroid [kg⋅m²]
 * @throws std::invalid_argument if mass <= 0
 * @throws std::runtime_error if hull is degenerate (volume <= 0)
 */
Eigen::Matrix3d computeInertiaTensorAboutCentroid(const ConvexHull& hull,
                                                  double mass);

}  // namespace InertialCalculations

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_INERTIAL_CALCULATIONS_HPP
