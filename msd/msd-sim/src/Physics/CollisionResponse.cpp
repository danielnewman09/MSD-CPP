// Ticket: 0027_collision_response_system
// Design: docs/designs/0027_collision_response_system/design.md

#include "msd-sim/src/Physics/CollisionResponse.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace msd_sim
{
namespace CollisionResponse
{

double combineRestitution(double eA, double eB)
{
  // Geometric mean: e_combined = sqrt(e_A * e_B)
  // Ensures:
  // - If either is 0, result is 0 (inelastic)
  // - If both are 1, result is 1 (elastic)
  // - Symmetric: combineRestitution(A,B) = combineRestitution(B,A)
  return std::sqrt(eA * eB);
}

double computeImpulseMagnitude(const AssetInertial& assetA,
                               const AssetInertial& assetB,
                               const CollisionResult& result,
                               double combinedRestitution)
{
  // ===== Compute Relative Velocity at Contact Point =====

  // Get linear velocities
  const Coordinate& vA = assetA.getInertialState().velocity;
  const Coordinate& vB = assetB.getInertialState().velocity;

  // Get angular velocities
  const AngularRate& omegaA = assetA.getInertialState().angularVelocity;
  const AngularRate& omegaB = assetB.getInertialState().angularVelocity;

  // Compute lever arms (contact point to center of mass) in world space
  // Center of mass is at ReferenceFrame origin (world space)
  // Use first contact point for backward compatibility (Ticket: 0029)
  Coordinate rA =
    result.contacts[0].pointA - assetA.getReferenceFrame().getOrigin();
  Coordinate rB =
    result.contacts[0].pointB - assetB.getReferenceFrame().getOrigin();

  // Velocity at contact point: v_contact = v_linear + ω × r
  Coordinate vContactA = vA + omegaA.cross(rA);
  Coordinate vContactB = vB + omegaB.cross(rB);

  // Relative velocity: v_rel = v_A - v_B
  Coordinate vRel = vContactA - vContactB;

  // ===== Check if Objects are Separating =====

  // Velocity along normal direction
  // With normal pointing from A to B:
  // - vRelNormal > 0: A catching up to B (approaching)
  // - vRelNormal < 0: A falling behind B (separating)
  double vRelNormal = vRel.dot(result.normal);

  // If objects are already separating, no impulse needed
  if (vRelNormal < 0.0)
  {
    return 0.0;
  }

  // ===== Compute Impulse Denominator =====

  // Linear terms: (1/m_A + 1/m_B)
  double invMassSum = (1.0 / assetA.getMass()) + (1.0 / assetB.getMass());

  // IMPORTANT: Inertia tensors are in body frame, so we must transform
  // the world-frame vectors to body frame for the calculation.

  // Angular terms for object A: (I_A^-1 * (r_A × n)) × r_A · n
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  CoordinateRate rA_local = frameA.globalToLocal(CoordinateRate{rA});
  CoordinateRate n_localA = frameA.globalToLocal(CoordinateRate{result.normal});
  Coordinate rA_cross_n_local = rA_local.cross(n_localA);
  Coordinate angularTermA = assetA.getInverseInertiaTensor() * rA_cross_n_local;
  double angularContribA = angularTermA.cross(rA_local).dot(n_localA);

  // Angular terms for object B: (I_B^-1 * (r_B × n)) × r_B · n
  const ReferenceFrame& frameB = assetB.getReferenceFrame();
  CoordinateRate rB_local = frameB.globalToLocal(CoordinateRate{rB});
  CoordinateRate n_localB = frameB.globalToLocal(CoordinateRate{result.normal});
  Coordinate rB_cross_n_local = rB_local.cross(n_localB);
  Coordinate angularTermB = assetB.getInverseInertiaTensor() * rB_cross_n_local;
  double angularContribB = angularTermB.cross(rB_local).dot(n_localB);

  // Total denominator
  double denominator = invMassSum + angularContribA + angularContribB;

  // ===== Compute Impulse Magnitude =====

  // Use zero restitution for low-velocity impacts to prevent micro-bouncing
  double effectiveRestitution =
    (vRelNormal < kRestitutionVelocityThreshold) ? 0.0 : combinedRestitution;

  // j = (1 + e) * v_rel · n / denominator
  // With our convention (normal A→B, vRelNormal > 0 for approaching),
  // this directly gives a positive impulse magnitude
  double impulseMagnitude =
    (1.0 + effectiveRestitution) * vRelNormal / denominator;

  // Ensure non-negative (should be guaranteed by separating check above)
  return impulseMagnitude;
}

void applyPositionCorrection(AssetInertial& assetA,
                             AssetInertial& assetB,
                             const CollisionResult& result)
{
  // ===== Compute Correction Amount =====

  // Only correct if penetration exceeds slop tolerance
  double correction = std::max(0.0, result.penetrationDepth - kSlop);

  // Apply correction factor
  correction *= kCorrectionFactor;

  // If no correction needed, early exit
  if (correction <= 0.0)
  {
    return;
  }

  // ===== Compute Mass-Weighted Separation =====

  // Total inverse mass
  double invMassSum = (1.0 / assetA.getMass()) + (1.0 / assetB.getMass());

  // Weight by inverse mass (heavier objects move less)
  double weightA = (1.0 / assetA.getMass()) / invMassSum;
  double weightB = (1.0 / assetB.getMass()) / invMassSum;

  // Separation vector along contact normal
  Coordinate separationVector = result.normal * correction;

  // ===== Apply Position Correction =====

  // Move A away from B (opposite to normal direction)
  assetA.getInertialState().position -= separationVector * weightA;

  // Move B away from A (along normal direction)
  assetB.getInertialState().position += separationVector * weightB;

  // Note: ReferenceFrame synchronization happens in WorldModel::updatePhysics()
  // after collision response is complete
}

// ========== Manifold-Aware Functions (Ticket:
// 0029_contact_manifold_generation) ==========

void applyImpulseManifold(AssetInertial& assetA,
                          AssetInertial& assetB,
                          const CollisionResult& result,
                          double combinedRestitution)
{
  // Compute total impulse magnitude using centroid-based formula
  double j =
    computeImpulseMagnitude(assetA, assetB, result, combinedRestitution);

  if (j <= 0.0)
  {
    return;  // No impulse needed (objects separating)
  }

  // ===== Compute Contact Centroids =====
  Coordinate centroidA{0, 0, 0};
  Coordinate centroidB{0, 0, 0};
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    centroidA += result.contacts[i].pointA;
    centroidB += result.contacts[i].pointB;
  }
  centroidA /= static_cast<double>(result.contactCount);
  centroidB /= static_cast<double>(result.contactCount);

  // Compute lever arms
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  const ReferenceFrame& frameB = assetB.getReferenceFrame();
  Coordinate leverArmA_world = centroidA - assetA.getInertialState().position;
  Coordinate leverArmB_world = centroidB - assetB.getInertialState().position;
  CoordinateRate leverArmA_local =
    frameA.globalToLocal(CoordinateRate{leverArmA_world});
  CoordinateRate leverArmB_local =
    frameB.globalToLocal(CoordinateRate{leverArmB_world});

  // ===== Apply Normal Impulse =====
  CoordinateRate normalImpulse_world = result.normal * j;
  assetA.getInertialState().velocity -= normalImpulse_world / assetA.getMass();
  assetB.getInertialState().velocity += normalImpulse_world / assetB.getMass();

  // Apply angular impulse for normal component
  CoordinateRate impulseA_local = frameA.globalToLocal(normalImpulse_world);
  AngularRate angularImpulseA_local =
    assetA.getInverseInertiaTensor() * leverArmA_local.cross(-impulseA_local);
  assetA.getInertialState().angularVelocity +=
    frameA.localToGlobal(angularImpulseA_local);

  CoordinateRate impulseB_local = frameB.globalToLocal(normalImpulse_world);
  AngularRate angularImpulseB_local =
    assetB.getInverseInertiaTensor() * leverArmB_local.cross(impulseB_local);
  assetB.getInertialState().angularVelocity +=
    frameB.localToGlobal(angularImpulseB_local);

  // ===== Apply Friction Impulse (Coulomb Friction) =====
  // Compute relative tangential velocity at contact point after normal impulse
  Coordinate& velA = assetA.getInertialState().velocity;
  Coordinate& velB = assetB.getInertialState().velocity;
  AngularRate& omegaA = assetA.getInertialState().angularVelocity;
  AngularRate& omegaB = assetB.getInertialState().angularVelocity;

  Coordinate vContactA = velA + omegaA.cross(leverArmA_world);
  Coordinate vContactB = velB + omegaB.cross(leverArmB_world);
  Coordinate vRel = vContactA - vContactB;

  // Tangential velocity = relative velocity - normal component
  double vNormalMag = vRel.dot(result.normal);
  Coordinate vTangent = vRel - result.normal * vNormalMag;
  double vTangentMag = vTangent.norm();

  // Apply friction if there's any tangential motion
  constexpr double kTangentEpsilon = 1e-8;
  if (vTangentMag > kTangentEpsilon)
  {
    // Friction direction opposes relative tangential velocity
    Coordinate frictionDir = -vTangent / vTangentMag;

    // Maximum friction impulse (Coulomb's law): j_friction <= μ * j_normal
    double maxFrictionImpulse = kFrictionCoefficient * j;

    // Compute effective mass at contact point for friction direction
    // This accounts for both linear and angular contributions from both objects
    // m_eff = 1 / (1/m_A + 1/m_B + angular_A + angular_B)
    CoordinateRate frictionDir_localA =
      frameA.globalToLocal(CoordinateRate{frictionDir});
    Coordinate r_cross_t_A = leverArmA_local.cross(frictionDir_localA);
    Coordinate angular_term_A = assetA.getInverseInertiaTensor() * r_cross_t_A;
    double angular_contrib_A = angular_term_A.dot(r_cross_t_A);

    CoordinateRate frictionDir_localB =
      frameB.globalToLocal(CoordinateRate{frictionDir});
    Coordinate r_cross_t_B = leverArmB_local.cross(frictionDir_localB);
    Coordinate angular_term_B = assetB.getInverseInertiaTensor() * r_cross_t_B;
    double angular_contrib_B = angular_term_B.dot(r_cross_t_B);

    double effectiveMass =
      1.0 / ((1.0 / assetA.getMass()) + (1.0 / assetB.getMass()) +
             angular_contrib_A + angular_contrib_B);
    double stoppingImpulse = vTangentMag * effectiveMass;

    // Check if friction can fully stop the relative motion (static friction)
    bool canFullyStop = (stoppingImpulse <= maxFrictionImpulse);

    // Apply the smaller of max friction or stopping impulse
    double frictionImpulse = std::min(maxFrictionImpulse, stoppingImpulse);

    // Apply friction linear impulse
    CoordinateRate frictionImpulse_world = frictionDir * frictionImpulse;
    velA += frictionImpulse_world / assetA.getMass();
    velB -= frictionImpulse_world / assetB.getMass();

    // Apply friction angular impulse to A
    CoordinateRate frictionA_local =
      frameA.globalToLocal(frictionImpulse_world);
    AngularRate frictionAngularA =
      assetA.getInverseInertiaTensor() * leverArmA_local.cross(frictionA_local);
    omegaA += frameA.localToGlobal(frictionAngularA);

    // Apply friction angular impulse to B (opposite direction)
    CoordinateRate frictionB_local =
      frameB.globalToLocal(frictionImpulse_world);
    AngularRate frictionAngularB = assetB.getInverseInertiaTensor() *
                                   leverArmB_local.cross(-frictionB_local);
    omegaB += frameB.localToGlobal(frictionAngularB);

    // If friction could fully stop relative motion, ensure we hit exactly zero
    // (prevents numerical overshoot causing oscillation)
    if (canFullyStop)
    {
      // Zero both objects' tangential motion
      velA = Coordinate{0, 0, 0};
      velB = Coordinate{0, 0, 0};
      omegaA = AngularRate{0, 0, 0};
      omegaB = AngularRate{0, 0, 0};
    }
  }
}

void applyPositionCorrectionManifold(AssetInertial& assetA,
                                     AssetInertial& assetB,
                                     const CollisionResult& result)
{
  // ===== Compute Correction Amount =====

  // Only correct if penetration exceeds slop tolerance
  double correction = std::max(0.0, result.penetrationDepth - kSlop);

  // Apply correction factor
  correction *= kCorrectionFactor;

  // If no correction needed, early exit
  if (correction <= 0.0)
  {
    return;
  }

  // ===== Compute Mass-Weighted Separation =====

  // Total inverse mass
  double invMassSum = (1.0 / assetA.getMass()) + (1.0 / assetB.getMass());

  // Weight by inverse mass (heavier objects move less)
  double weightA = (1.0 / assetA.getMass()) / invMassSum;
  double weightB = (1.0 / assetB.getMass()) / invMassSum;

  // Separation vector along contact normal
  Coordinate separationVector = result.normal * correction;

  // ===== Apply Position Correction =====

  // Move A away from B (opposite to normal direction)
  assetA.getInertialState().position -= separationVector * weightA;

  // Move B away from A (along normal direction)
  assetB.getInertialState().position += separationVector * weightB;

  // Note: ReferenceFrame synchronization happens in WorldModel::updatePhysics()
  // after collision response is complete
}

// ========== Dynamic-Static Collision (Inertial vs Environment) ==========

double computeImpulseMagnitudeStatic(
  const AssetInertial& dynamic,
  [[maybe_unused]] const AssetEnvironment& staticObj,
  const CollisionResult& result,
  double restitution)
{
  // ===== Compute Contact Centroid =====
  // Using the centroid of all contact points gives correct behavior for
  // symmetric collisions (e.g., cube landing flat). Individual corner
  // lever arms would over-estimate the angular contribution to the
  // denominator, but the net torque is actually zero due to symmetry.
  // The centroid captures this by having r × n ≈ 0 when the contact
  // centroid is directly below the center of mass.

  Coordinate contactCentroid{0, 0, 0};
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    contactCentroid += result.contacts[i].pointA;
  }
  contactCentroid /= static_cast<double>(result.contactCount);

  // ===== Compute Relative Velocity at Contact Centroid =====

  // Get dynamic object's velocities
  const Coordinate& vDynamic = dynamic.getInertialState().velocity;
  const AngularRate& omegaDynamic = dynamic.getInertialState().angularVelocity;

  // Static object has zero velocity (both linear and angular)

  // Compute lever arm from center of mass to contact centroid (world frame)
  Coordinate rDynamic =
    contactCentroid - dynamic.getReferenceFrame().getOrigin();

  // Velocity at contact centroid: v_contact = v_linear + ω × r
  Coordinate vContact = vDynamic + omegaDynamic.cross(rDynamic);

  // Relative velocity (static object has zero velocity)
  Coordinate vRel = vContact;

  // ===== Check if Objects are Separating =====

  // Velocity along normal direction
  // Normal points from dynamic (A) toward static (B)
  // vRelNormal > 0: approaching, vRelNormal < 0: separating
  double vRelNormal = vRel.dot(result.normal);

  if (vRelNormal < 0.0)
  {
    return 0.0;  // Already separating
  }

  // ===== Compute Impulse Denominator =====

  // Linear term: only dynamic object contributes (1/m_static = 0)
  double invMass = 1.0 / dynamic.getMass();

  // Angular term: only dynamic object contributes
  // IMPORTANT: Inertia tensor is in body frame, so we must transform
  // the world-frame vectors to body frame for the calculation.
  const ReferenceFrame& frame = dynamic.getReferenceFrame();

  // Transform lever arm and normal to body frame
  CoordinateRate rLocal = frame.globalToLocal(CoordinateRate{rDynamic});
  CoordinateRate nLocal = frame.globalToLocal(CoordinateRate{result.normal});

  // Compute angular contribution in body frame: (I^-1 * (r × n)) × r · n
  Coordinate r_cross_n_local = rLocal.cross(nLocal);
  Coordinate angularTermLocal =
    dynamic.getInverseInertiaTensor() * r_cross_n_local;
  double angularContrib = angularTermLocal.cross(rLocal).dot(nLocal);

  double denominator = invMass + angularContrib;

  // ===== Compute Impulse Magnitude =====

  // Use zero restitution for low-velocity impacts to prevent micro-bouncing
  // This allows objects to settle at rest on surfaces
  double effectiveRestitution =
    (std::abs(vRelNormal) < kRestitutionVelocityThreshold) ? 0.0 : restitution;

  double impulseMagnitude =
    (1.0 + effectiveRestitution) * vRelNormal / denominator;

  return impulseMagnitude;
}

void applyPositionCorrectionStatic(
  AssetInertial& dynamic,
  [[maybe_unused]] const AssetEnvironment& staticObj,
  const CollisionResult& result)
{
  // ===== Compute Correction Amount =====

  double correction = std::max(0.0, result.penetrationDepth - kSlop);
  correction *= kCorrectionFactor;

  if (correction <= 0.0)
  {
    return;
  }

  // ===== Apply Full Correction to Dynamic Object =====

  // Static object has infinite mass, so weight_static = 0, weight_dynamic = 1
  // All correction is applied to the dynamic object
  Coordinate separationVector = result.normal * correction;

  // Move dynamic object away from static (opposite to normal direction)
  dynamic.getInertialState().position -= separationVector;

  // Note: ReferenceFrame synchronization happens in WorldModel::updatePhysics()
}

void applyImpulseManifoldStatic(AssetInertial& dynamic,
                                const AssetEnvironment& staticObj,
                                const CollisionResult& result,
                                double restitution)
{
  // Compute total impulse magnitude using centroid-based formula
  double j =
    computeImpulseMagnitudeStatic(dynamic, staticObj, result, restitution);

  if (j <= 0.0)
  {
    return;  // No impulse needed (objects separating)
  }

  // ===== Compute Contact Centroid =====
  Coordinate contactCentroid{0, 0, 0};
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    contactCentroid += result.contacts[i].pointA;
  }
  contactCentroid /= static_cast<double>(result.contactCount);

  // ===== Apply Normal Impulse =====
  // Linear impulse directly changes velocity, independent of application point
  // Dynamic object receives impulse in -normal direction (pushed away from
  // static)
  CoordinateRate normalImpulse_world = result.normal * j;
  dynamic.getInertialState().velocity -=
    normalImpulse_world / dynamic.getMass();

  // Compute lever arm from center of mass to contact centroid
  const ReferenceFrame& frame = dynamic.getReferenceFrame();
  Coordinate leverArm_world =
    contactCentroid - dynamic.getInertialState().position;
  CoordinateRate leverArm_local =
    frame.globalToLocal(CoordinateRate{leverArm_world});

  // Transform normal impulse to body frame and apply angular impulse
  CoordinateRate normalImpulse_local = frame.globalToLocal(normalImpulse_world);
  AngularRate angularImpulse_local = dynamic.getInverseInertiaTensor() *
                                     leverArm_local.cross(-normalImpulse_local);
  dynamic.getInertialState().angularVelocity +=
    frame.localToGlobal(angularImpulse_local);

  // ===== Apply Friction Impulse (Coulomb Friction) =====
  // Compute tangential velocity at contact point after normal impulse
  Coordinate& vel = dynamic.getInertialState().velocity;
  AngularRate& omega = dynamic.getInertialState().angularVelocity;
  Coordinate vContact = vel + omega.cross(leverArm_world);

  // Tangential velocity = total velocity - normal component
  double vNormal = vContact.dot(result.normal);
  Coordinate vTangent = vContact - result.normal * vNormal;
  double vTangentMag = vTangent.norm();

  // Apply friction if there's any tangential motion
  constexpr double kTangentEpsilon = 1e-8;
  if (vTangentMag > kTangentEpsilon)
  {
    // Friction direction opposes tangential velocity
    Coordinate frictionDir = -vTangent / vTangentMag;

    // Maximum friction impulse (Coulomb's law): j_friction <= μ * j_normal
    double maxFrictionImpulse = kFrictionCoefficient * j;

    // Compute effective mass at contact point for friction direction
    // m_eff = 1 / (1/m + (r × t)ᵀ · I⁻¹ · (r × t))
    // Where t is the friction direction (tangent)
    CoordinateRate frictionDir_local =
      frame.globalToLocal(CoordinateRate{frictionDir});
    Coordinate r_cross_t = leverArm_local.cross(frictionDir_local);
    Coordinate angular_term = dynamic.getInverseInertiaTensor() * r_cross_t;
    double angular_contrib = angular_term.dot(r_cross_t);
    double effectiveMass = 1.0 / (1.0 / dynamic.getMass() + angular_contrib);

    // Compute impulse needed to stop tangential motion at contact point
    double stoppingImpulse = vTangentMag * effectiveMass;

    // Check if friction can fully stop the motion (static friction case)
    bool canFullyStop = (stoppingImpulse <= maxFrictionImpulse);

    // Apply the smaller of max friction or stopping impulse
    double frictionImpulse = std::min(maxFrictionImpulse, stoppingImpulse);

    // Apply friction linear impulse
    CoordinateRate frictionImpulse_world = frictionDir * frictionImpulse;
    vel += frictionImpulse_world / dynamic.getMass();

    // Apply friction angular impulse
    CoordinateRate frictionImpulse_local =
      frame.globalToLocal(frictionImpulse_world);
    AngularRate frictionAngularImpulse_local =
      dynamic.getInverseInertiaTensor() *
      leverArm_local.cross(frictionImpulse_local);
    omega += frame.localToGlobal(frictionAngularImpulse_local);

    // If friction could fully stop motion, ensure we hit exactly zero
    // (prevents numerical overshoot causing oscillation)
    // if (canFullyStop)
    // {
    //   vel = Coordinate{0, 0, 0};
    //   omega = AngularRate{0, 0, 0};
    // }
  }
}

}  // namespace CollisionResponse
}  // namespace msd_sim
