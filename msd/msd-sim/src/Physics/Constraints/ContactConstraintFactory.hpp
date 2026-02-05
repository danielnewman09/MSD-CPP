// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#ifndef MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_FACTORY_HPP
#define MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_FACTORY_HPP

#include <memory>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim::contact_constraint_factory
{

/**
 * @brief Factory for creating ContactConstraint instances from CollisionResult
 *
 * Provides stateless utility functions to construct contact constraints from
 * collision detection output. Computes pre-impact velocities and lever arms
 * needed for ContactConstraint construction.
 *
 * Design rationale: Separates construction logic (velocity computation, lever
 * arm calculation) from the constraint itself. This keeps ContactConstraint
 * focused on the mathematical interface and makes testing easier.
 *
 * Thread safety: Stateless functions, thread-safe
 * Error handling: Returns empty vector if collision result has contactCount ==
 * 0
 *
 * @see
 * docs/designs/0032_contact_constraint_refactor/0032_contact_constraint_refactor.puml
 * @see
 * prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md
 * @ticket 0032_contact_constraint_refactor
 */

/**
 * @brief Rest velocity threshold [m/s]
 *
 * Below this threshold, restitution is disabled (e_effective = 0) to prevent
 * jitter for resting contacts. The constraint formulation handles resting
 * contacts via Baumgarte stabilization.
 *
 * Value from math formulation Section 6.4: 0.5-1.0 m/s recommended.
 */
constexpr double kRestVelocityThreshold = 0.5;

/**
 * @brief Default restitution for environment objects
 *
 * Used when one body is an AssetEnvironment (static object).
 */
constexpr double kEnvironmentRestitution = 0.5;

/**
 * @brief Create contact constraints from a collision result
 *
 * Generates one ContactConstraint per contact point in the collision manifold.
 * For a CollisionResult with contactCount = 4, this produces 4 constraints.
 *
 * CRITICAL: Uses correct restitution formula v_target = -e * v_pre (not
 * -(1+e)*v_pre). See P2 Debug Findings for explanation of the difference
 * between constraint RHS and target velocity formulations.
 *
 * @param bodyAIndex Index of body A in the solver body list
 * @param bodyBIndex Index of body B in the solver body list
 * @param result Collision result with contact manifold
 * @param stateA Inertial state of body A
 * @param stateB Inertial state of body B
 * @param comA Center of mass of body A (world space) [m]
 * @param comB Center of mass of body B (world space) [m]
 * @param restitution Combined coefficient of restitution [0, 1]
 * @return Vector of contact constraints (one per contact point)
 */
std::vector<std::unique_ptr<ContactConstraint>> createFromCollision(
  size_t bodyAIndex,
  size_t bodyBIndex,
  const CollisionResult& result,
  const InertialState& stateA,
  const InertialState& stateB,
  const Coordinate& comA,
  const Coordinate& comB,
  double restitution);

/**
 * @brief Combine two coefficients of restitution using geometric mean
 *
 * Formula: e_combined = sqrt(eA * eB)
 *
 * Replaces CollisionResponse::combineRestitution().
 *
 * @param eA Coefficient of restitution for body A [0, 1]
 * @param eB Coefficient of restitution for body B [0, 1]
 * @return Combined coefficient of restitution [0, 1]
 */
double combineRestitution(double eA, double eB);

/**
 * @brief Compute relative normal velocity at a contact point
 *
 * Formula: v_rel_n = (v_B + ω_B × r_B - v_A - ω_A × r_A) · n
 *
 * Positive value: bodies approaching (collision)
 * Negative value: bodies separating
 *
 * @param stateA Inertial state of body A
 * @param stateB Inertial state of body B
 * @param leverArmA Lever arm from A's center of mass to contact point [m]
 * @param leverArmB Lever arm from B's center of mass to contact point [m]
 * @param normal Contact normal (A → B, unit length)
 * @return Relative normal velocity [m/s]
 */
double computeRelativeNormalVelocity(const InertialState& stateA,
                                     const InertialState& stateB,
                                     const Coordinate& leverArmA,
                                     const Coordinate& leverArmB,
                                     const Coordinate& normal);


}  // namespace msd_sim::contact_constraint_factory

#endif  // MSD_SIM_PHYSICS_CONTACT_CONSTRAINT_FACTORY_HPP
