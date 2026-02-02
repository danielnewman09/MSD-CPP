// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include <cmath>
#include <stdexcept>

namespace msd_sim
{

namespace ContactConstraintFactory
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
  if (result.contactCount == 0) {
    return constraints;
  }

  // Create one constraint per contact point
  for (size_t i = 0; i < result.contactCount; ++i) {
    const auto& contactPair = result.contacts[i];

    // Compute lever arms (world space)
    Coordinate leverArmA = contactPair.pointA - comA;
    Coordinate leverArmB = contactPair.pointB - comB;

    // Compute pre-impact relative normal velocity
    double relVelNormal = computeRelativeNormalVelocity(
        stateA, stateB, leverArmA, leverArmB, result.normal);

    // Disable restitution below rest velocity threshold
    double effectiveRestitution = restitution;
    if (std::abs(relVelNormal) < kRestVelocityThreshold) {
      effectiveRestitution = 0.0;
    }

    // Create contact constraint
    constraints.push_back(std::make_unique<ContactConstraint>(
        bodyAIndex,
        bodyBIndex,
        result.normal,
        contactPair.pointA,
        contactPair.pointB,
        result.penetrationDepth,
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

double computeRelativeNormalVelocity(
    const InertialState& stateA,
    const InertialState& stateB,
    const Coordinate& leverArmA,
    const Coordinate& leverArmB,
    const Coordinate& normal)
{
  // Get angular velocities
  Coordinate omegaA = stateA.getAngularVelocity();
  Coordinate omegaB = stateB.getAngularVelocity();

  // Velocity at contact point A: v_A + ω_A × r_A
  Coordinate velAtContactA = stateA.velocity + omegaA.cross(leverArmA);

  // Velocity at contact point B: v_B + ω_B × r_B
  Coordinate velAtContactB = stateB.velocity + omegaB.cross(leverArmB);

  // Relative velocity: v_B - v_A
  Coordinate relVel = velAtContactB - velAtContactA;

  // Project onto normal: v_rel · n
  return relVel.dot(normal);
}

// ========== Friction Support (ticket 0035c) ==========

double combineFrictionCoefficient(double muA, double muB)
{
  // Geometric mean combination (matches restitution pattern)
  // Property: if either is zero, result is zero (frictionless)
  return std::sqrt(muA * muB);
}

// ========== Centroid Reduction (ticket 0035d4) ==========

namespace {

/**
 * @brief Compute geometric centroid of contact points on both bodies
 *
 * Centroid computed as arithmetic mean:
 *   centroidA = (1/N) * Σ contacts[i].pointA
 *   centroidB = (1/N) * Σ contacts[i].pointB
 *
 * @param result Collision result with contact manifold
 * @return Pair of (centroidA, centroidB)
 */
std::pair<Coordinate, Coordinate> computeContactCentroid(const CollisionResult& result)
{
  Coordinate centroidA{0.0, 0.0, 0.0};
  Coordinate centroidB{0.0, 0.0, 0.0};

  for (size_t i = 0; i < result.contactCount; ++i) {
    centroidA = centroidA + result.contacts[i].pointA;
    centroidB = centroidB + result.contacts[i].pointB;
  }

  double invN = 1.0 / static_cast<double>(result.contactCount);
  centroidA = centroidA * invN;
  centroidB = centroidB * invN;

  return {centroidA, centroidB};
}

}  // anonymous namespace

std::unique_ptr<FrictionConstraint> createCentroidFrictionConstraint(
    size_t bodyAIndex,
    size_t bodyBIndex,
    const CollisionResult& result,
    const Coordinate& comA,
    const Coordinate& comB,
    double frictionCoefficientA,
    double frictionCoefficientB)
{
  // Validate friction coefficients
  if (frictionCoefficientA < 0.0) {
    throw std::invalid_argument(
        "Friction coefficient A must be non-negative, got: " +
        std::to_string(frictionCoefficientA));
  }
  if (frictionCoefficientB < 0.0) {
    throw std::invalid_argument(
        "Friction coefficient B must be non-negative, got: " +
        std::to_string(frictionCoefficientB));
  }

  // Combine friction coefficients
  double mu = combineFrictionCoefficient(frictionCoefficientA, frictionCoefficientB);

  // Return nullptr if μ = 0.0 exactly (frictionless contact)
  if (mu == 0.0) {
    return nullptr;
  }

  // Return nullptr if no contacts
  if (result.contactCount == 0) {
    return nullptr;
  }

  // Compute centroid of contact points
  auto [centroidA, centroidB] = computeContactCentroid(result);

  // Create single friction constraint at centroid
  return std::make_unique<FrictionConstraint>(
      bodyAIndex,
      bodyBIndex,
      result.normal,
      centroidA,
      centroidB,
      comA,
      comB,
      mu);
}

std::unique_ptr<ContactConstraint> createCentroidContactConstraint(
    size_t bodyAIndex,
    size_t bodyBIndex,
    const CollisionResult& result,
    const InertialState& stateA,
    const InertialState& stateB,
    const Coordinate& comA,
    const Coordinate& comB,
    double restitution)
{
  // Return nullptr if no contacts
  if (result.contactCount == 0) {
    return nullptr;
  }

  // Compute centroid of contact points
  auto [centroidA, centroidB] = computeContactCentroid(result);

  // Compute lever arms from centroid
  Coordinate leverArmA = centroidA - comA;
  Coordinate leverArmB = centroidB - comB;

  // Compute pre-impact relative normal velocity at centroid
  double relVelNormal = computeRelativeNormalVelocity(
      stateA, stateB, leverArmA, leverArmB, result.normal);

  // Disable restitution below rest velocity threshold
  double effectiveRestitution = restitution;
  if (std::abs(relVelNormal) < kRestVelocityThreshold) {
    effectiveRestitution = 0.0;
  }

  // Create single contact constraint at centroid
  return std::make_unique<ContactConstraint>(
      bodyAIndex,
      bodyBIndex,
      result.normal,
      centroidA,
      centroidB,
      result.penetrationDepth,
      comA,
      comB,
      effectiveRestitution,
      relVelNormal);
}

}  // namespace ContactConstraintFactory

}  // namespace msd_sim
