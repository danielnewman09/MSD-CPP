// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include <cmath>

#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
namespace msd_sim::contact_constraint_factory
{

std::vector<std::unique_ptr<ContactConstraint>> createFromCollision(
  size_t bodyAIndex,
  size_t bodyBIndex,
  const CollisionResult& result,
  const InertialState& stateA,
  const InertialState& stateB,
  const Coordinate& comA,
  const Coordinate& comB,
  double restitution)
{
  std::vector<std::unique_ptr<ContactConstraint>> constraints;

  // Return empty vector if no contacts
  if (result.contactCount == 0)
  {
    return constraints;
  }

  // Create one constraint per contact point
  for (size_t i = 0; i < result.contactCount; ++i)
  {
    const auto& contactPair = result.contacts[i];

    // Compute lever arms (world space)
    Coordinate const leverArmA = contactPair.pointA - comA;
    Coordinate const leverArmB = contactPair.pointB - comB;

    // Compute pre-impact relative normal velocity
    double const relVelNormal = computeRelativeNormalVelocity(
      stateA, stateB, leverArmA, leverArmB, result.normal);

    // Disable restitution below rest velocity threshold
    double effectiveRestitution = restitution;
    if (std::abs(relVelNormal) < kRestVelocityThreshold)
    {
      effectiveRestitution = 0.0;
    }

    // Create contact constraint
    // Ticket: 0040a — use per-contact depth instead of shared penetrationDepth
    constraints.push_back(
      std::make_unique<ContactConstraint>(bodyAIndex,
                                          bodyBIndex,
                                          result.normal,
                                          contactPair.pointA,
                                          contactPair.pointB,
                                          contactPair.depth,
                                          comA,
                                          comB,
                                          effectiveRestitution,
                                          relVelNormal));
  }

  return constraints;
}

double combineRestitution(double eA, double eB)
{
  // Geometric mean combination (standard in physics engines)
  return std::sqrt(eA * eB);
}

double computeRelativeNormalVelocity(const InertialState& stateA,
                                     const InertialState& stateB,
                                     const Coordinate& leverArmA,
                                     const Coordinate& leverArmB,
                                     const Coordinate& normal)
{
  // Get angular velocities
  const Coordinate omegaA = stateA.getAngularVelocity();
  const Coordinate omegaB = stateB.getAngularVelocity();

  // Velocity at contact point A: v_A + ω_A × r_A
  const Coordinate velAtContactA = stateA.velocity + omegaA.cross(leverArmA);

  // Velocity at contact point B: v_B + ω_B × r_B
  const Coordinate velAtContactB = stateB.velocity + omegaB.cross(leverArmB);

  // Relative velocity: v_B - v_A
  const Coordinate relVel = velAtContactB - velAtContactA;

  // Project onto normal: v_rel · n
  return relVel.dot(normal);
}

}  // namespace msd_sim::contact_constraint_factory
