// Ticket: 0027_collision_response_system
// Design: docs/designs/0027_collision_response_system/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_RESPONSE_HPP
#define MSD_SIM_PHYSICS_COLLISION_RESPONSE_HPP

#include "msd-sim/src/Physics/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"

namespace msd_sim
{

/**
 * @brief Stateless utility namespace for collision impulse and position
 * correction.
 *
 * Provides impulse-based physics response for rigid body collisions using:
 * - Coefficient of restitution for elastic/inelastic behavior
 * - Full rigid body impulse formula with angular terms
 * - Position correction with slop tolerance to prevent jitter
 *
 * All functions are stateless and thread-safe when called with different object
 * instances.
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
 */
constexpr double kSlop = 0.01;  // 1cm

/**
 * @brief Position correction factor [0, 1].
 *
 * Fraction of penetration to correct per frame.
 * - 1.0: Full correction (can cause instability)
 * - 0.8: Recommended value (balances stability and responsiveness)
 * - 0.0: No correction
 */
constexpr double kCorrectionFactor = 0.8;

/**
 * @brief Rest velocity threshold [m/s].
 *
 * Contact point velocities below this threshold are considered "at rest".
 * Velocities are clamped to zero to prevent micro-oscillations.
 * - Too small: Objects oscillate indefinitely
 * - Too large: Objects stop unnaturally early
 */
constexpr double kRestVelocityThreshold = 0.0001;  // 5 cm/s

/**
 * @brief Rest angular velocity threshold [rad/s].
 *
 * Angular velocities below this threshold are clamped to zero.
 */
constexpr double kRestAngularThreshold = 0.0001;  // ~3 deg/s

/**
 * @brief Restitution velocity threshold [m/s].
 *
 * Approach velocities below this threshold use zero restitution
 * (perfectly inelastic collision) to prevent micro-bouncing.
 * This is critical for objects to settle at rest on surfaces.
 */
constexpr double kRestitutionVelocityThreshold = 0.0001;  // 50 cm/s

// ========== Functions (Ticket: 0029_contact_manifold_generation) ==========

/**
 * @brief Apply impulse across all contact points in manifold.
 *
 * Distributes total impulse equally across all contacts:
 *   j_per_contact = j_total / contactCount
 *
 * For each contact point:
 * - Applies linear impulse (same magnitude for all contacts, additive)
 * - Computes lever arm from contact point to center of mass
 * - Applies angular impulse: Δω = I^-1 * (r × J)
 *
 * This improves stability for face-face contacts by balancing torques.
 *
 * @param assetA First colliding object (modified)
 * @param assetB Second colliding object (modified)
 * @param result Collision information (normal, contacts array)
 * @param combinedRestitution Combined coefficient of restitution [0, 1]
 *
 * @pre assetA and assetB have positive mass
 * @pre result contains valid manifold (contactCount ∈ [1, 4])
 * @pre combinedRestitution ∈ [0, 1]
 *
 * @note Modifies linear and angular velocities in InertialState.
 *
 * @see
 * docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml
 * @ticket 0029_contact_manifold_generation
 */
void applyImpulseManifold(AssetInertial& assetA,
                          AssetInertial& assetB,
                          const CollisionResult& result,
                          double combinedRestitution);

/**
 * @brief Apply position correction using average of all contact points.
 *
 * Uses linear projection with slop tolerance to prevent jitter.
 * Position correction uses average contact point for stable direction.
 *
 * Correction formula:
 *   correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
 *   separationVector = normal * correction
 *
 * Each object is moved by weighted fraction based on inverse mass.
 *
 * @param assetA First colliding object (modified)
 * @param assetB Second colliding object (modified)
 * @param result Collision information (normal, penetration depth, contacts)
 *
 * @pre assetA and assetB have positive mass
 * @pre result contains valid penetration depth and normal
 *
 * @note Modifies InertialState.position directly.
 *       ReferenceFrame synchronization happens in WorldModel::updatePhysics().
 *
 * @see
 * docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml
 * @ticket 0029_contact_manifold_generation
 */
void applyPositionCorrectionManifold(AssetInertial& assetA,
                                     AssetInertial& assetB,
                                     const CollisionResult& result);

// ========== Legacy Functions (Deprecated, use manifold versions) ==========

/**
 * @brief Compute scalar impulse magnitude for collision resolution.
 *
 * Uses the impulse-based collision response formula:
 *   j = -(1 + e) * (v_rel · n) / denominator
 *
 * Where:
 *   v_rel = relative velocity at contact point
 *   n = contact normal (A → B)
 *   e = combined coefficient of restitution
 *   denominator = (1/m_A + 1/m_B) + angular terms
 *
 * Angular terms account for rotational effects:
 *   denominator += (I_A^-1 * (r_A × n)) × r_A · n
 *                + (I_B^-1 * (r_B × n)) × r_B · n
 *
 * Where:
 *   I_A^-1, I_B^-1 = inverse inertia tensors (world space)
 *   r_A, r_B = lever arms (contact point to center of mass)
 *
 * @param assetA First colliding object
 * @param assetB Second colliding object
 * @param result Collision information (normal, contact points)
 * @param combinedRestitution Combined coefficient of restitution [0, 1]
 * @return Scalar impulse magnitude (non-negative value) [N·s]
 *
 * @pre assetA and assetB have positive mass
 * @pre result contains valid contact information
 * @pre combinedRestitution ∈ [0, 1]
 *
 * @note Assumes objects are separating if relative velocity is positive.
 *       Returns 0 if objects are already separating.
 */
double computeImpulseMagnitude(const AssetInertial& assetA,
                               const AssetInertial& assetB,
                               const CollisionResult& result,
                               double combinedRestitution);

/**
 * @brief Apply position correction to separate overlapping objects.
 *
 * Uses linear projection with slop tolerance to prevent jitter.
 * Only corrects when penetration exceeds slop threshold.
 *
 * Correction formula:
 *   correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
 *   separationVector = normal * correction
 *
 * Each object is moved by weighted fraction based on inverse mass:
 *   weight_A = (1/m_A) / (1/m_A + 1/m_B)
 *   weight_B = (1/m_B) / (1/m_A + 1/m_B)
 *
 * Heavier objects move less than lighter objects.
 *
 * @param assetA First colliding object (modified)
 * @param assetB Second colliding object (modified)
 * @param result Collision information (normal, penetration depth)
 *
 * @pre assetA and assetB have positive mass
 * @pre result contains valid penetration depth and normal
 *
 * @note Modifies InertialState.position directly.
 *       ReferenceFrame synchronization happens in WorldModel::updatePhysics().
 */
void applyPositionCorrection(AssetInertial& assetA,
                             AssetInertial& assetB,
                             const CollisionResult& result);

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
 *
 * @pre eA ∈ [0, 1]
 * @pre eB ∈ [0, 1]
 */
double combineRestitution(double eA, double eB);

// ========== Dynamic-Static Collision (Inertial vs Environment) ==========

/**
 * @brief Default coefficient of restitution for static environment objects.
 *
 * Used when computing combined restitution for dynamic-static collisions.
 * Represents moderately elastic surfaces like concrete or wood.
 */
constexpr double kEnvironmentRestitution = 0.5;

/**
 * @brief Default coefficient of friction for collisions.
 *
 * Coulomb friction coefficient (μ) used for tangential impulse calculation.
 * Represents moderately rough surfaces.
 * - 0.0: Frictionless (ice on ice)
 * - 0.3: Smooth surfaces (wood on wood)
 * - 0.5: Moderate friction (rubber on concrete)
 * - 1.0+: High friction (rubber on rubber)
 */
constexpr double kFrictionCoefficient = .5;

/**
 * @brief Compute scalar impulse magnitude for dynamic-static collision.
 *
 * Simplified formula for collision with infinite-mass static object:
 *   j = -(1 + e) * (v_rel · n) / (1/m + angular_term)
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
 * @return Scalar impulse magnitude (non-negative value) [N·s]
 *
 * @pre dynamic has positive mass
 * @pre result contains valid contact information
 * @pre restitution ∈ [0, 1]
 *
 * @note Returns 0 if objects are already separating.
 */
double computeImpulseMagnitudeStatic(const AssetInertial& dynamic,
                                     const AssetEnvironment& staticObj,
                                     const CollisionResult& result,
                                     double restitution);

/**
 * @brief Apply position correction for dynamic-static collision.
 *
 * Only the dynamic object is moved; static object remains fixed.
 *
 * Correction formula:
 *   correction = max(penetrationDepth - kSlop, 0.0) * kCorrectionFactor
 *   dynamic.position -= normal * correction
 *
 * @param dynamic The dynamic object (modified)
 * @param staticObj The static object (unchanged)
 * @param result Collision information (normal, penetration depth)
 *
 * @pre dynamic has positive mass
 * @pre result contains valid penetration depth and normal
 *
 * @note Normal should point from dynamic toward static.
 */
void applyPositionCorrectionStatic(AssetInertial& dynamic,
                                   const AssetEnvironment& staticObj,
                                   const CollisionResult& result);

/**
 * @brief Apply impulse across all contact points for dynamic-static collision.
 *
 * Manifold-aware version of static collision impulse application.
 * Distributes total impulse equally across all contacts:
 *   j_per_contact = j_total / contactCount
 *
 * For each contact point:
 * - Applies linear impulse to dynamic object only (static has infinite mass)
 * - Computes lever arm from contact point to dynamic object's center of mass
 * - Applies angular impulse to dynamic object only
 *
 * This improves stability for face-face contacts between dynamic objects
 * and static environment (e.g., box resting on ground plane).
 *
 * @param dynamic The dynamic (inertial) object (modified)
 * @param staticObj The static (environment) object (unchanged)
 * @param result Collision information (normal, contacts array)
 * @param restitution Coefficient of restitution [0, 1]
 *
 * @pre dynamic has positive mass
 * @pre result contains valid manifold (contactCount in [1, 4])
 * @pre restitution in [0, 1]
 *
 * @note Modifies linear and angular velocities in dynamic object's
 * InertialState.
 *
 * @see
 * docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml
 * @ticket 0029_contact_manifold_generation
 */
void applyImpulseManifoldStatic(AssetInertial& dynamic,
                                const AssetEnvironment& staticObj,
                                const CollisionResult& result,
                                double restitution);

}  // namespace CollisionResponse

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_RESPONSE_HPP
