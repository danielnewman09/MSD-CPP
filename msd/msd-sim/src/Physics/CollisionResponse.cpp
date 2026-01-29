// Ticket: 0027_collision_response_system
// Refactored: Lagrangian mechanics with frictionless constraints
// Design: docs/designs/0027_collision_response_system/design.md

#include "msd-sim/src/Physics/CollisionResponse.hpp"
#include <algorithm>
#include <cmath>

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

// ========== Lagrangian Constraint Functions ==========

double computeLagrangeMultiplier(const AssetInertial& assetA,
                                 const AssetInertial& assetB,
                                 const CollisionResult& result,
                                 double restitution)
{
  // ===== Compute Relative Velocity at Contact Point =====
  //
  // The velocity-level non-penetration constraint is:
  //   Ċ = v_rel · n ≥ 0
  //
  // Where v_rel is the relative velocity at the contact point.

  // Get linear velocities
  const Coordinate& vA = assetA.getInertialState().velocity;
  const Coordinate& vB = assetB.getInertialState().velocity;

  // Get angular velocities (quaternion conversion)
  AngularRate omegaA = assetA.getInertialState().getAngularVelocity();
  AngularRate omegaB = assetB.getInertialState().getAngularVelocity();

  // Compute lever arms (contact point to center of mass) in world space
  // Using centroid of contact manifold for stability
  Coordinate contactCentroid{0, 0, 0};
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    contactCentroid += result.contacts[i].pointA;
  }
  contactCentroid /= static_cast<double>(result.contactCount);

  Coordinate rA = contactCentroid - assetA.getReferenceFrame().getOrigin();
  Coordinate rB = contactCentroid - assetB.getReferenceFrame().getOrigin();

  // Velocity at contact point: v_contact = v_linear + ω × r
  Coordinate vContactA = vA + omegaA.cross(rA);
  Coordinate vContactB = vB + omegaB.cross(rB);

  // Relative velocity: v_rel = v_A - v_B
  Coordinate vRel = vContactA - vContactB;

  // ===== Check Constraint Violation =====
  //
  // The constraint Ċ = v_rel · n should be ≥ 0 for separation.
  // If vRelNormal < 0, objects are separating (constraint satisfied).
  // If vRelNormal > 0, objects are approaching (constraint violated).

  double vRelNormal = vRel.dot(result.normal);

  // If objects are already separating, constraint is satisfied
  if (vRelNormal < 0.0)
  {
    return 0.0;
  }

  // ===== Compute Effective Mass (J * M^-1 * J^T) =====
  //
  // The constraint Jacobian for body A is:
  //   J_A = [-n^T, -(r_A × n)^T]
  //
  // The effective mass is:
  //   m_eff = J * M^-1 * J^T
  //         = (1/m_A + 1/m_B) + angular_A + angular_B
  //
  // Where angular terms account for rotational inertia:
  //   angular_A = (I_A^-1 * (r_A × n)) · (r_A × n)

  // Linear terms: (1/m_A + 1/m_B)
  double invMassSum = (1.0 / assetA.getMass()) + (1.0 / assetB.getMass());

  // Angular terms require transforming to body frame since inertia tensors
  // are stored in body frame coordinates.
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  CoordinateRate rA_local = frameA.globalToLocal(CoordinateRate{rA});
  CoordinateRate n_localA = frameA.globalToLocal(CoordinateRate{result.normal});
  Coordinate rA_cross_n_local = rA_local.cross(n_localA);
  Coordinate angularTermA = assetA.getInverseInertiaTensor() * rA_cross_n_local;
  double angularContribA = angularTermA.cross(rA_local).dot(n_localA);

  const ReferenceFrame& frameB = assetB.getReferenceFrame();
  CoordinateRate rB_local = frameB.globalToLocal(CoordinateRate{rB});
  CoordinateRate n_localB = frameB.globalToLocal(CoordinateRate{result.normal});
  Coordinate rB_cross_n_local = rB_local.cross(n_localB);
  Coordinate angularTermB = assetB.getInverseInertiaTensor() * rB_cross_n_local;
  double angularContribB = angularTermB.cross(rB_local).dot(n_localB);

  // Total effective mass (denominator)
  double effectiveMass = invMassSum + angularContribA + angularContribB;

  // ===== Solve for Lagrange Multiplier =====
  //
  // The Lagrange multiplier equation is:
  //   (J * M^-1 * J^T) * λ = -(1 + e) * (v_rel · n)
  //
  // Solving for λ:
  //   λ = (1 + e) * (v_rel · n) / (J * M^-1 * J^T)
  //
  // Note: We use (1 + e) to incorporate restitution. For perfectly
  // inelastic collisions (e=0), we get the minimum impulse needed
  // to prevent interpenetration. For elastic collisions (e=1), we
  // get an impulse that reverses the normal velocity component.

  double lambda = (1.0 + restitution) * vRelNormal / effectiveMass;

  return lambda;
}

void applyConstraintResponse(AssetInertial& assetA,
                             AssetInertial& assetB,
                             const CollisionResult& result,
                             double restitution)
{
  // Compute Lagrange multiplier (constraint force magnitude)
  double lambda = computeLagrangeMultiplier(assetA, assetB, result, restitution);

  if (lambda <= 0.0)
  {
    return;  // Constraint already satisfied
  }

  // Get angular velocities for modification (quaternion conversion)
  AngularRate omegaA = assetA.getInertialState().getAngularVelocity();
  AngularRate omegaB = assetB.getInertialState().getAngularVelocity();

  // ===== Compute Contact Centroid =====
  Coordinate centroidA{0, 0, 0};
  Coordinate centroidB{0, 0, 0};
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    centroidA += result.contacts[i].pointA;
    centroidB += result.contacts[i].pointB;
  }
  centroidA /= static_cast<double>(result.contactCount);
  centroidB /= static_cast<double>(result.contactCount);

  // Compute lever arms for torque calculation
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  const ReferenceFrame& frameB = assetB.getReferenceFrame();
  Coordinate leverArmA_world = centroidA - assetA.getInertialState().position;
  Coordinate leverArmB_world = centroidB - assetB.getInertialState().position;
  CoordinateRate leverArmA_local =
    frameA.globalToLocal(CoordinateRate{leverArmA_world});
  CoordinateRate leverArmB_local =
    frameB.globalToLocal(CoordinateRate{leverArmB_world});

  // ===== Apply Constraint Force (Normal Direction Only) =====
  //
  // The constraint force is:
  //   F_constraint = λ * n
  //
  // This acts as an impulse applied to each body:
  //   Body A: J = -λ * n (pushed away from B)
  //   Body B: J = +λ * n (pushed away from A)
  //
  // Linear velocity change:
  //   Δv = J / m
  //
  // Angular velocity change:
  //   Δω = I^-1 * (r × J)

  // Constraint impulse in world space (normal direction only - frictionless)
  CoordinateRate constraintImpulse_world = result.normal * lambda;

  // Apply linear velocity change
  assetA.getInertialState().velocity -= constraintImpulse_world / assetA.getMass();
  assetB.getInertialState().velocity += constraintImpulse_world / assetB.getMass();

  // Apply angular velocity change to A
  // Torque = r × F, so angular impulse = r × J
  CoordinateRate impulseA_local = frameA.globalToLocal(constraintImpulse_world);
  AngularRate angularImpulseA_local =
    assetA.getInverseInertiaTensor() * leverArmA_local.cross(-impulseA_local);
  AngularRate omegaA_new = omegaA + frameA.localToGlobal(angularImpulseA_local);
  assetA.getInertialState().setAngularVelocity(omegaA_new);

  // Apply angular velocity change to B
  CoordinateRate impulseB_local = frameB.globalToLocal(constraintImpulse_world);
  AngularRate angularImpulseB_local =
    assetB.getInverseInertiaTensor() * leverArmB_local.cross(impulseB_local);
  AngularRate omegaB_new = omegaB + frameB.localToGlobal(angularImpulseB_local);
  assetB.getInertialState().setAngularVelocity(omegaB_new);

  // NOTE: No friction applied - this is a frictionless collision.
  // Tangential constraint forces will be added in a future refactoring step
  // using Coulomb friction cones as inequality constraints.
}

void applyPositionStabilization(AssetInertial& assetA,
                                AssetInertial& assetB,
                                const CollisionResult& result)
{
  // ===== Baumgarte Position Stabilization =====
  //
  // Numerical integration causes position drift over time. Baumgarte
  // stabilization corrects this by adding a position-level correction
  // that reduces penetration depth.
  //
  // The correction is:
  //   correction = max(depth - slop, 0) * β
  //
  // Where β (kCorrectionFactor) controls correction speed.
  // Slop tolerance prevents jitter for small penetrations.

  double correction = std::max(0.0, result.penetrationDepth - kSlop);
  correction *= kCorrectionFactor;

  if (correction <= 0.0)
  {
    return;
  }

  // Mass-weighted separation (heavier objects move less)
  double invMassSum = (1.0 / assetA.getMass()) + (1.0 / assetB.getMass());
  double weightA = (1.0 / assetA.getMass()) / invMassSum;
  double weightB = (1.0 / assetB.getMass()) / invMassSum;

  // Separation vector along contact normal
  Coordinate separationVector = result.normal * correction;

  // Move objects apart
  assetA.getInertialState().position -= separationVector * weightA;
  assetB.getInertialState().position += separationVector * weightB;
}

// ========== Dynamic-Static Collision (Inertial vs Environment) ==========

double computeLagrangeMultiplierStatic(const AssetInertial& dynamic,
                                       const AssetEnvironment& staticObj,
                                       const CollisionResult& result,
                                       double restitution)
{
  (void)staticObj;  // Static object contributes no terms

  // Compute contact centroid
  Coordinate contactCentroid{0, 0, 0};
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    contactCentroid += result.contacts[i].pointA;
  }
  contactCentroid /= static_cast<double>(result.contactCount);

  // Get dynamic object's velocities
  const CoordinateRate& vDynamic = dynamic.getInertialState().velocity;
  AngularRate omegaDynamic = dynamic.getInertialState().getAngularVelocity();

  // Lever arm from center of mass to contact centroid
  Coordinate rDynamic = contactCentroid - dynamic.getReferenceFrame().getOrigin();

  // Velocity at contact point
  CoordinateRate vContact = vDynamic + omegaDynamic.cross(rDynamic);

  // Relative velocity (static object has zero velocity)
  double vRelNormal = vContact.dot(result.normal);

  // Check if already separating
  if (vRelNormal < 0.0)
  {
    return 0.0;
  }

  // Effective mass (only dynamic object contributes)
  double invMass = 1.0 / dynamic.getMass();

  // Angular contribution
  const ReferenceFrame& frame = dynamic.getReferenceFrame();
  CoordinateRate rLocal = frame.globalToLocal(CoordinateRate{rDynamic});
  CoordinateRate nLocal = frame.globalToLocal(CoordinateRate{result.normal});
  Coordinate r_cross_n_local = rLocal.cross(nLocal);
  Coordinate angularTermLocal = dynamic.getInverseInertiaTensor() * r_cross_n_local;
  double angularContrib = angularTermLocal.cross(rLocal).dot(nLocal);

  double effectiveMass = invMass + angularContrib;

  // Solve for Lagrange multiplier
  double lambda = (1.0 + restitution) * vRelNormal / effectiveMass;

  return lambda;
}

void applyConstraintResponseStatic(AssetInertial& dynamic,
                                   const AssetEnvironment& staticObj,
                                   const CollisionResult& result,
                                   double restitution)
{
  // Compute Lagrange multiplier
  double lambda =
    computeLagrangeMultiplierStatic(dynamic, staticObj, result, restitution);

  if (lambda <= 0.0)
  {
    return;
  }

  // Get angular velocity for modification (quaternion conversion)
  AngularRate omegaDynamic = dynamic.getInertialState().getAngularVelocity();

  // Compute contact centroid
  Coordinate contactCentroid{0, 0, 0};
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    contactCentroid += result.contacts[i].pointA;
  }
  contactCentroid /= static_cast<double>(result.contactCount);

  // Constraint impulse (normal direction only - frictionless)
  CoordinateRate constraintImpulse_world = result.normal * lambda;

  // Apply linear velocity change (dynamic object pushed away from static)
  dynamic.getInertialState().velocity -=
    constraintImpulse_world / dynamic.getMass();

  // Apply angular velocity change
  const ReferenceFrame& frame = dynamic.getReferenceFrame();
  Coordinate leverArm_world =
    contactCentroid - dynamic.getInertialState().position;
  CoordinateRate leverArm_local =
    frame.globalToLocal(CoordinateRate{leverArm_world});
  CoordinateRate impulse_local = frame.globalToLocal(constraintImpulse_world);
  AngularRate angularImpulse_local =
    dynamic.getInverseInertiaTensor() * leverArm_local.cross(-impulse_local);
  AngularRate omegaDynamic_new =
    omegaDynamic + frame.localToGlobal(angularImpulse_local);
  dynamic.getInertialState().setAngularVelocity(omegaDynamic_new);

  // NOTE: No friction applied - this is a frictionless collision.
}

void applyPositionStabilizationStatic(AssetInertial& dynamic,
                                      const AssetEnvironment& staticObj,
                                      const CollisionResult& result)
{
  (void)staticObj;

  double correction = std::max(0.0, result.penetrationDepth - kSlop);
  correction *= kCorrectionFactor;

  if (correction <= 0.0)
  {
    return;
  }

  // All correction applied to dynamic object (static has infinite mass)
  Coordinate separationVector = result.normal * correction;
  dynamic.getInertialState().position -= separationVector;
}

}  // namespace CollisionResponse
}  // namespace msd_sim
