// Ticket: 0035a_tangent_basis_and_friction_constraint
// Design: docs/designs/0035a_tangent_basis_and_friction_constraint/design.md
// Ticket: 0075a_unified_constraint_data_structure
// Note: FrictionConstraint is deprecated as of 0075a. Friction data is now
// embedded in ContactConstraint. This class is retained to avoid breaking
// ConstraintPool::allocateFriction() which exists for backward compatibility
// but is no longer called by the pipeline.

#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-transfer/src/ConstraintRecordVisitor.hpp"
#include "msd-transfer/src/UnifiedContactConstraintRecord.hpp"
#include <cmath>
#include <limits>
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
  : Constraint{bodyAIndex, bodyBIndex, /*alpha=*/0.0, /*beta=*/0.0},
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

Eigen::VectorXd FrictionConstraint::evaluate(const InertialState& stateA,
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

Eigen::MatrixXd FrictionConstraint::jacobian(
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

bool FrictionConstraint::isActive(const InertialState& /* stateA */,
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

void FrictionConstraint::setTangentLambdas(double t1Lambda, double t2Lambda)
{
  tangent1_lambda_ = t1Lambda;
  tangent2_lambda_ = t2Lambda;
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

void FrictionConstraint::setSlidingMode(const Vector3D& slidingDirection)
{
  // Align tangent basis with sliding direction:
  // t1 = -slidingDirection (opposing motion, so positive lambda_t1 decelerates)
  // t2 = normal × t1 (perpendicular to both normal and t1)

  tangent1_ = Coordinate{-slidingDirection.x(), -slidingDirection.y(), -slidingDirection.z()};

  // Compute t2 = normal × t1 (already unit since both inputs are unit)
  Vector3D const normalVec{contact_normal_.x(), contact_normal_.y(), contact_normal_.z()};
  Vector3D const t1Vec{tangent1_.x(), tangent1_.y(), tangent1_.z()};
  Vector3D const t2Vec = normalVec.cross(t1Vec);

  tangent2_ = Coordinate{t2Vec.x(), t2Vec.y(), t2Vec.z()};

  is_sliding_mode_ = true;
}

void FrictionConstraint::recordState(msd_transfer::ConstraintRecordVisitor& visitor,
                                      uint32_t bodyAId,
                                      uint32_t bodyBId) const
{
  // Ticket: 0075a_unified_constraint_data_structure
  // FrictionConstraint is deprecated — friction is now embedded in
  // ContactConstraint. This recordState() builds a UnifiedContactConstraintRecord
  // using only the friction fields available here; contact-specific fields
  // (penetration_depth, restitution, pre_impact_rel_vel_normal) are NaN because
  // FrictionConstraint does not store them.
  //
  // This method should not be called during normal operation (the pipeline no
  // longer creates FrictionConstraint instances as of 0075a).
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  msd_transfer::UnifiedContactConstraintRecord record;
  record.body_a_id = bodyAId;
  record.body_b_id = bodyBId;

  record.normal = Vector3D{contact_normal_.x(), contact_normal_.y(), contact_normal_.z()}.toRecord();
  record.lever_arm_a = Vector3D{lever_arm_a_.x(), lever_arm_a_.y(), lever_arm_a_.z()}.toRecord();
  record.lever_arm_b = Vector3D{lever_arm_b_.x(), lever_arm_b_.y(), lever_arm_b_.z()}.toRecord();

  // Contact-only fields not available in FrictionConstraint
  record.penetration_depth = kNaN;
  record.restitution = kNaN;
  record.pre_impact_rel_vel_normal = kNaN;

  record.tangent1 = Vector3D{tangent1_.x(), tangent1_.y(), tangent1_.z()}.toRecord();
  record.tangent2 = Vector3D{tangent2_.x(), tangent2_.y(), tangent2_.z()}.toRecord();
  record.friction_coefficient = friction_coefficient_;
  record.normal_lambda = normal_lambda_;
  record.tangent1_lambda = tangent1_lambda_;
  record.tangent2_lambda = tangent2_lambda_;

  visitor.visit(record);
}

}  // namespace msd_sim
