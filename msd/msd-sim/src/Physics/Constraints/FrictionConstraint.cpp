// Ticket: 0035a_tangent_basis_and_friction_constraint
// Design: docs/designs/0035a_tangent_basis_and_friction_constraint/design.md

#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace msd_sim
{

FrictionConstraint::FrictionConstraint(size_t bodyAIndex,
                                       size_t bodyBIndex,
                                       const Coordinate& normal,
                                       const Coordinate& contactPointA,
                                       const Coordinate& contactPointB,
                                       const Coordinate& comA,
                                       const Coordinate& comB,
                                       double frictionCoefficient)
  : TwoBodyConstraint{bodyAIndex, bodyBIndex},
    contact_normal_{normal},
    friction_coefficient_{frictionCoefficient}
{
  // Validate friction coefficient
  if (frictionCoefficient < 0.0)
  {
    throw std::invalid_argument(
      "FrictionConstraint: friction coefficient must be non-negative (got " +
      std::to_string(frictionCoefficient) + ")");
  }

  // Compute tangent basis from normal (validates normal unit length internally)
  const TangentFrame frame = tangent_basis::computeTangentBasis(normal);
  tangent1_ = frame.t1;
  tangent2_ = frame.t2;

  // Compute lever arms (contactPoint - centerOfMass)
  lever_arm_a_ = contactPointA - comA;
  lever_arm_b_ = contactPointB - comB;
}

Eigen::VectorXd FrictionConstraint::evaluateTwoBody(const InertialState& stateA,
                                                    const InertialState& stateB,
                                                    double /* time */) const
{
  // Compute relative velocity at contact point
  // v_rel = (vA + ωA × rA) - (vB + ωB × rB)

  const Coordinate& vA = stateA.velocity;
  const Coordinate omegaA = stateA.getAngularVelocity();
  const Coordinate& vB = stateB.velocity;
  const Coordinate omegaB = stateB.getAngularVelocity();

  // Contact point velocity for body A
  const Coordinate vContactA = vA + omegaA.cross(lever_arm_a_);

  // Contact point velocity for body B
  const Coordinate vContactB = vB + omegaB.cross(lever_arm_b_);

  // Relative velocity (A relative to B)
  const Coordinate vRel = vContactA - vContactB;

  // Project onto tangent directions
  Eigen::VectorXd c{2};
  c(0) = vRel.dot(tangent1_);  // Relative velocity in t1 direction
  c(1) = vRel.dot(tangent2_);  // Relative velocity in t2 direction

  return c;
}

Eigen::MatrixXd FrictionConstraint::jacobianTwoBody(
  const InertialState& /* stateA */,
  const InertialState& /* stateB */,
  double /* time */) const
{
  // Jacobian structure (2×12):
  // Row 0: J_t1 = [t1^T, (rA×t1)^T, -t1^T, -(rB×t1)^T]
  // Row 1: J_t2 = [t2^T, (rA×t2)^T, -t2^T, -(rB×t2)^T]
  //
  // Block structure: [v_A (3), ω_A (3), v_B (3), ω_B (3)]

  Eigen::MatrixXd j = Eigen::MatrixXd::Zero(2, 12);

  // Row 0: t1 direction
  {
    // Lever arm cross products
    Coordinate rACrossT1 = lever_arm_a_.cross(tangent1_);
    Coordinate rBCrossT1 = lever_arm_b_.cross(tangent1_);

    // Block 1: vA contribution (columns 0-2)
    j(0, 0) = tangent1_.x();
    j(0, 1) = tangent1_.y();
    j(0, 2) = tangent1_.z();

    // Block 2: ωA contribution (columns 3-5)
    j(0, 3) = rACrossT1.x();
    j(0, 4) = rACrossT1.y();
    j(0, 5) = rACrossT1.z();

    // Block 3: vB contribution (columns 6-8)
    j(0, 6) = -tangent1_.x();
    j(0, 7) = -tangent1_.y();
    j(0, 8) = -tangent1_.z();

    // Block 4: ωB contribution (columns 9-11)
    j(0, 9) = -rBCrossT1.x();
    j(0, 10) = -rBCrossT1.y();
    j(0, 11) = -rBCrossT1.z();
  }

  // Row 1: t2 direction
  {
    // Lever arm cross products
    Coordinate rACrossT2 = lever_arm_a_.cross(tangent2_);
    Coordinate rBCrossT2 = lever_arm_b_.cross(tangent2_);

    // Block 1: vA contribution (columns 0-2)
    j(1, 0) = tangent2_.x();
    j(1, 1) = tangent2_.y();
    j(1, 2) = tangent2_.z();

    // Block 2: ωA contribution (columns 3-5)
    j(1, 3) = rACrossT2.x();
    j(1, 4) = rACrossT2.y();
    j(1, 5) = rACrossT2.z();

    // Block 3: vB contribution (columns 6-8)
    j(1, 6) = -tangent2_.x();
    j(1, 7) = -tangent2_.y();
    j(1, 8) = -tangent2_.z();

    // Block 4: ωB contribution (columns 9-11)
    j(1, 9) = -rBCrossT2.x();
    j(1, 10) = -rBCrossT2.y();
    j(1, 11) = -rBCrossT2.z();
  }

  return j;
}

bool FrictionConstraint::isActiveTwoBody(const InertialState& /* stateA */,
                                         const InertialState& /* stateB */,
                                         double /* time */) const
{
  // Friction is active when:
  // 1. Friction coefficient is non-zero (μ > 0)
  // 2. Normal contact force is positive (λn > 0)
  return (friction_coefficient_ > 0.0) && (normal_lambda_ > 0.0);
}

void FrictionConstraint::setNormalLambda(double normalLambda)
{
  normal_lambda_ = normalLambda;
}

std::pair<double, double> FrictionConstraint::getFrictionBounds() const
{
  // Coulomb friction cone: ||λ_t|| ≤ μ · λn
  // Box constraint approximation (inscribed square): |λ_ti| ≤ μ/√2 · λn
  // See M3-coulomb-cone.md for derivation

  constexpr double kSqrt2 = std::numbers::sqrt2;  // √2
  const double bound = (friction_coefficient_ / kSqrt2) * normal_lambda_;

  return {-bound, bound};
}

}  // namespace msd_sim
