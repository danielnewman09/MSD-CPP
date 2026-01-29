// Ticket: 0027_collision_response_system
// Refactored: Lagrangian mechanics with frictionless constraints
// Design: docs/designs/0027_collision_response_system/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_RESPONSE_HPP
#define MSD_SIM_PHYSICS_COLLISION_RESPONSE_HPP

#include "msd-sim/src/Physics/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"

namespace msd_sim
{

/**
 * @brief Stateless utility namespace for Lagrangian constraint-based collision
 * response.
 *
 * Implements frictionless collision response using Lagrangian mechanics with
 * Lagrange multipliers. The approach formulates collision as a velocity-level
 * constraint and solves for the constraint force magnitude (Lagrange
 * multiplier).
 *
 * ## Lagrangian Formulation
 *
 * For a frictionless contact, the non-penetration constraint is:
 *   C(q) = (x_B - x_A) · n ≥ 0
 *
 * At the velocity level:
 *   Ċ = v_rel · n ≥ 0
 *
 * The constraint Jacobian is:
 *   J = [n^T, (r × n)^T] for each body
 *
 * The Lagrange multiplier λ (constraint force magnitude) is found by solving:
 *   (J * M^-1 * J^T) * λ = -J * v
 *
 * For frictionless contacts, the constraint force acts only in the normal
 * direction:
 *   F_constraint = λ * n
 *
 * This is equivalent to applying a normal impulse J_n = λ * n to each body.
 *
 * ## No Friction
 *
 * This implementation is frictionless - constraint forces act only along the
 * contact normal. Tangential (friction) constraints will be added in a future
 * refactoring step.
 *
 * @see
 * docs/designs/0027_collision_response_system/0027_collision_response_system.puml
 * @ticket 0027_collision_response_system
 */
namespace CollisionResponse
{

// ========== Constants ==========

/**
 * @brief Penetration slop tolerance [m].
 *
 * Objects penetrating less than this depth are not corrected.
 * Prevents jitter from floating-point precision issues.
 * This is a form of constraint stabilization (Baumgarte stabilization).
 */
constexpr double kSlop = 0.01;  // 1cm

/**
 * @brief Position correction factor [0, 1].
 *
 * Fraction of penetration to correct per frame (Baumgarte parameter β).
 * - 1.0: Full correction (can cause instability)
 * - 0.8: Recommended value (balances stability and responsiveness)
 * - 0.0: No correction
 */
constexpr double kCorrectionFactor = 0.8;

/**
 * @brief Rest velocity threshold [m/s].
 *
 * Contact point velocities below this threshold are considered "at rest".
 * Used to prevent constraint solving for near-zero velocities.
 */
constexpr double kRestVelocityThreshold = 0.0001;

/**
 * @brief Default coefficient of restitution for static environment objects.
 *
 * Used when computing combined restitution for dynamic-static collisions.
 * Represents moderately elastic surfaces like concrete or wood.
 */
constexpr double kEnvironmentRestitution = 0.5;

// ========== Lagrangian Constraint Functions ==========

/**
 * @brief Compute Lagrange multiplier for non-penetration constraint.
 *
 * Solves the constraint equation to find λ (constraint force magnitude):
 *
 *   λ = (1 + e) * (v_rel · n) / (J * M^-1 * J^T)
 *
 * Where:
 *   v_rel = relative velocity at contact point
 *   n = contact normal (A → B)
 *   e = coefficient of restitution
 *   J = constraint Jacobian
 *   M = generalized mass matrix
 *
 * The denominator J * M^-1 * J^T expands to:
 *   (1/m_A + 1/m_B) + (I_A^-1 * (r_A × n)) · (r_A × n)
 *                   + (I_B^-1 * (r_B × n)) · (r_B × n)
 *
 * @param assetA First colliding object
 * @param assetB Second colliding object
 * @param result Collision information (normal, contact points)
 * @param restitution Coefficient of restitution [0, 1]
 * @return Lagrange multiplier λ (constraint force magnitude) [N·s]
 *
 * @pre assetA and assetB have positive mass
 * @pre result contains valid contact information
 *
 * @note Returns 0 if objects are already separating (constraint satisfied).
 */
double computeLagrangeMultiplier(const AssetInertial& assetA,
                                 const AssetInertial& assetB,
                                 const CollisionResult& result,
                                 double restitution);

/**
 * @brief Apply Lagrangian constraint response (frictionless).
 *
 * Applies the constraint force to satisfy the non-penetration constraint.
 * The constraint force acts only in the normal direction (frictionless).
 *
 * For each body:
 *   Δv_linear = ±λ * n / m
 *   Δω = I^-1 * (r × (±λ * n))
 *
 * Where the sign depends on which body (A gets negative, B gets positive).
 *
 * @param assetA First colliding object (modified)
 * @param assetB Second colliding object (modified)
 * @param result Collision information (normal, contacts array)
 * @param restitution Coefficient of restitution [0, 1]
 *
 * @pre assetA and assetB have positive mass
 * @pre result contains valid manifold (contactCount ∈ [1, 4])
 *
 * @note Modifies linear and angular velocities in InertialState.
 * @note No friction - constraint forces act only along contact normal.
 */
void applyConstraintResponse(AssetInertial& assetA,
                             AssetInertial& assetB,
                             const CollisionResult& result,
                             double restitution);

/**
 * @brief Apply position-level constraint stabilization (Baumgarte).
 *
 * Corrects position drift from numerical integration using Baumgarte
 * stabilization. Moves objects apart along the contact normal to reduce
 * penetration depth.
 *
 * Correction formula:
 *   correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
 *   separationVector = normal * correction
 *
 * Each object is moved by weighted fraction based on inverse mass:
 *   weight_A = (1/m_A) / (1/m_A + 1/m_B)
 *   weight_B = (1/m_B) / (1/m_A + 1/m_B)
 *
 * @param assetA First colliding object (modified)
 * @param assetB Second colliding object (modified)
 * @param result Collision information (normal, penetration depth)
 *
 * @pre assetA and assetB have positive mass
 * @pre result contains valid penetration depth and normal
 *
 * @note Modifies InertialState.position directly.
 */
void applyPositionStabilization(AssetInertial& assetA,
                                AssetInertial& assetB,
                                const CollisionResult& result);

// ========== Dynamic-Static Collision (Inertial vs Environment) ==========

/**
 * @brief Compute Lagrange multiplier for dynamic-static collision.
 *
 * Simplified formula for collision with infinite-mass static object:
 *   λ = (1 + e) * (v_rel · n) / (1/m + angular_term)
 *
 * The static object contributes:
 * - No mass term (1/m_static = 0, effectively infinite mass)
 * - No angular terms (static object doesn't rotate)
 * - No velocity (static object is stationary)
 *
 * @param dynamic The dynamic (inertial) object
 * @param staticObj The static (environment) object
 * @param result Collision information (normal points from dynamic toward
 * static)
 * @param restitution Coefficient of restitution [0, 1]
 * @return Lagrange multiplier λ (constraint force magnitude) [N·s]
 *
 * @pre dynamic has positive mass
 * @pre result contains valid contact information
 *
 * @note Returns 0 if objects are already separating.
 */
double computeLagrangeMultiplierStatic(const AssetInertial& dynamic,
                                       const AssetEnvironment& staticObj,
                                       const CollisionResult& result,
                                       double restitution);

/**
 * @brief Apply Lagrangian constraint response for dynamic-static collision.
 *
 * Applies constraint force to dynamic object only (static has infinite mass).
 * The constraint force acts only in the normal direction (frictionless).
 *
 * @param dynamic The dynamic (inertial) object (modified)
 * @param staticObj The static (environment) object (unchanged)
 * @param result Collision information (normal, contacts array)
 * @param restitution Coefficient of restitution [0, 1]
 *
 * @pre dynamic has positive mass
 * @pre result contains valid manifold (contactCount in [1, 4])
 *
 * @note Modifies linear and angular velocities in dynamic object's
 * InertialState.
 * @note No friction - constraint forces act only along contact normal.
 */
void applyConstraintResponseStatic(AssetInertial& dynamic,
                                   const AssetEnvironment& staticObj,
                                   const CollisionResult& result,
                                   double restitution);

/**
 * @brief Apply position stabilization for dynamic-static collision.
 *
 * Only the dynamic object is moved; static object remains fixed.
 *
 * @param dynamic The dynamic object (modified)
 * @param staticObj The static object (unchanged)
 * @param result Collision information (normal, penetration depth)
 *
 * @pre dynamic has positive mass
 * @pre result contains valid penetration depth and normal
 */
void applyPositionStabilizationStatic(AssetInertial& dynamic,
                                      const AssetEnvironment& staticObj,
                                      const CollisionResult& result);

// ========== Utility Functions ==========

/**
 * @brief Combine two coefficients of restitution using geometric mean.
 *
 * Formula: e_combined = sqrt(e_A * e_B)
 *
 * This ensures:
 * - If either object is fully inelastic (e=0), collision is inelastic
 * - If both are fully elastic (e=1), collision is fully elastic
 * - Symmetric: e(A,B) = e(B,A)
 *
 * @param eA Coefficient of restitution for object A [0, 1]
 * @param eB Coefficient of restitution for object B [0, 1]
 * @return Combined coefficient [0, 1]
 */
double combineRestitution(double eA, double eB);

}  // namespace CollisionResponse

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_RESPONSE_HPP
