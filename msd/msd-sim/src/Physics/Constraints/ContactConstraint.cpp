// Ticket: 0032_contact_constraint_refactor
// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Collision/TangentBasis.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-transfer/src/ConstraintRecordVisitor.hpp"
#include "msd-transfer/src/UnifiedContactConstraintRecord.hpp"
#include <stdexcept>
#include <cmath>

namespace msd_sim
{

ContactConstraint::ContactConstraint(size_t bodyAIndex,
                                     size_t bodyBIndex,
                                     const Coordinate& normal,
                                     const Coordinate& contactPointA,
                                     const Coordinate& contactPointB,
                                     double penetrationDepth,
                                     const Coordinate& comA,
                                     const Coordinate& comB,
                                     double restitution,
                                     double preImpactRelVelNormal,
                                     double frictionCoefficient)
  : Constraint{bodyAIndex, bodyBIndex, /*alpha=*/0.2, /*beta=*/0.0},
    contact_normal_{normal},
    lever_arm_a_{contactPointA - comA},
    lever_arm_b_{contactPointB - comB},
    penetration_depth_{penetrationDepth},
    restitution_{restitution},
    pre_impact_rel_vel_normal_{preImpactRelVelNormal},
    friction_coefficient_{frictionCoefficient}
{
  // Validate normal is unit length (within tolerance)
  double const normalLength = normal.norm();
  if (std::abs(normalLength - 1.0) > 1e-6) {
    throw std::invalid_argument(
        "ContactConstraint: normal must be unit length (|n| = " +
        std::to_string(normalLength) + ")");
  }

  // Validate penetration depth
  if (penetrationDepth < 0.0) {
    throw std::invalid_argument(
        "ContactConstraint: penetration depth must be non-negative (d = " +
        std::to_string(penetrationDepth) + ")");
  }

  // Validate restitution coefficient
  if (restitution < 0.0 || restitution > 1.0) {
    throw std::invalid_argument(
        "ContactConstraint: restitution must be in [0, 1] (e = " +
        std::to_string(restitution) + ")");
  }

  // Validate friction coefficient
  if (frictionCoefficient < 0.0) {
    throw std::invalid_argument(
        "ContactConstraint: friction coefficient must be non-negative (mu = " +
        std::to_string(frictionCoefficient) + ")");
  }

  // Compute tangent basis when friction is active
  if (frictionCoefficient > 0.0) {
    const TangentFrame frame = tangent_basis::computeTangentBasis(normal);
    tangent1_ = frame.t1;
    tangent2_ = frame.t2;
  }
  // When frictionless: tangent1_ and tangent2_ remain zero-initialized
}

Eigen::VectorXd ContactConstraint::evaluate(
    const InertialState& stateA,
    const InertialState& stateB,
    double /* time */) const
{
  // Constraint function: C = (x_B - x_A) · n
  // where x_A = posA + r_A, x_B = posB + r_B
  //
  // For penetrating bodies, C < 0 (constraint violated)
  // For separated bodies, C > 0 (constraint satisfied)

  Coordinate const contactPointA = stateA.position + lever_arm_a_;
  Coordinate const contactPointB = stateB.position + lever_arm_b_;

  Eigen::VectorXd c(dimension());
  c(0) = (contactPointB - contactPointA).dot(contact_normal_);

  // Friction rows evaluate tangential relative velocity
  if (hasFriction()) {
    const Coordinate& vA = stateA.velocity;
    const Coordinate omegaA = stateA.getAngularVelocity();
    const Coordinate& vB = stateB.velocity;
    const Coordinate omegaB = stateB.getAngularVelocity();

    const Coordinate vContactA = vA + omegaA.cross(lever_arm_a_);
    const Coordinate vContactB = vB + omegaB.cross(lever_arm_b_);
    const Coordinate vRel = vContactA - vContactB;

    c(1) = vRel.dot(tangent1_);
    c(2) = vRel.dot(tangent2_);
  }

  return c;
}

Eigen::MatrixXd ContactConstraint::jacobian(
    const InertialState& /* stateA */,
    const InertialState& /* stateB */,
    double /* time */) const
{
  const int dim = dimension();
  Eigen::MatrixXd j = Eigen::MatrixXd::Zero(dim, 12);

  // Row 0: Normal constraint
  // J_n = [-n^T, -(rA×n)^T, n^T, (rB×n)^T]
  Coordinate const rACrossN = lever_arm_a_.cross(contact_normal_);
  Coordinate const rBCrossN = lever_arm_b_.cross(contact_normal_);

  j.block<1, 3>(0, 0) = -contact_normal_.transpose();  // -n^T
  j.block<1, 3>(0, 3) = -rACrossN.transpose();          // -(rA×n)^T
  j.block<1, 3>(0, 6) = contact_normal_.transpose();    // n^T
  j.block<1, 3>(0, 9) = rBCrossN.transpose();           // (rB×n)^T

  if (!hasFriction()) {
    return j;
  }

  // Row 1: t1 direction
  // J_t1 = [t1^T, (rA×t1)^T, -t1^T, -(rB×t1)^T]
  {
    Coordinate const rACrossT1 = lever_arm_a_.cross(tangent1_);
    Coordinate const rBCrossT1 = lever_arm_b_.cross(tangent1_);

    j.block<1, 3>(1, 0) = tangent1_.transpose();    //  t1^T
    j.block<1, 3>(1, 3) = rACrossT1.transpose();    //  (rA×t1)^T
    j.block<1, 3>(1, 6) = -tangent1_.transpose();   // -t1^T
    j.block<1, 3>(1, 9) = -rBCrossT1.transpose();   // -(rB×t1)^T
  }

  // Row 2: t2 direction
  // J_t2 = [t2^T, (rA×t2)^T, -t2^T, -(rB×t2)^T]
  {
    Coordinate const rACrossT2 = lever_arm_a_.cross(tangent2_);
    Coordinate const rBCrossT2 = lever_arm_b_.cross(tangent2_);

    j.block<1, 3>(2, 0) = tangent2_.transpose();    //  t2^T
    j.block<1, 3>(2, 3) = rACrossT2.transpose();    //  (rA×t2)^T
    j.block<1, 3>(2, 6) = -tangent2_.transpose();   // -t2^T
    j.block<1, 3>(2, 9) = -rBCrossT2.transpose();   // -(rB×t2)^T
  }

  return j;
}

bool ContactConstraint::isActive(
    const InertialState& stateA,
    const InertialState& stateB,
    double time) const
{
  // Contact is active if bodies are penetrating or very close (C <= threshold)
  // Threshold chosen to match typical slop tolerance (0.01m)
  const double kActivationThreshold = 0.01;  // [m]

  Eigen::VectorXd c = evaluate(stateA, stateB, time);
  return c(0) <= kActivationThreshold;
}

void ContactConstraint::setSlidingMode(const Vector3D& slidingDirection)
{
  if (!hasFriction()) {
    return;  // No-op for frictionless contacts
  }

  // Align tangent basis with sliding direction:
  // t1 = -slidingDirection (opposing motion, so positive lambda_t1 decelerates)
  // t2 = normal × t1 (perpendicular to both normal and t1)

  tangent1_ = Coordinate{-slidingDirection.x(),
                          -slidingDirection.y(),
                          -slidingDirection.z()};

  // Compute t2 = normal × t1 (already unit since both inputs are unit)
  Vector3D const normalVec{contact_normal_.x(),
                            contact_normal_.y(),
                            contact_normal_.z()};
  Vector3D const t1Vec{tangent1_.x(), tangent1_.y(), tangent1_.z()};
  Vector3D const t2Vec = normalVec.cross(t1Vec);

  tangent2_ = Coordinate{t2Vec.x(), t2Vec.y(), t2Vec.z()};

  is_sliding_mode_ = true;
}

void ContactConstraint::setTangentLambdas(double t1Lambda, double t2Lambda)
{
  tangent1_lambda_ = t1Lambda;
  tangent2_lambda_ = t2Lambda;
}

void ContactConstraint::setNormalLambda(double normalLambda)
{
  normal_lambda_ = normalLambda;
}

void ContactConstraint::recordState(msd_transfer::ConstraintRecordVisitor& visitor,
                                     uint32_t bodyAId,
                                     uint32_t bodyBId) const
{
  msd_transfer::UnifiedContactConstraintRecord record;
  record.body_a_id = bodyAId;
  record.body_b_id = bodyBId;

  // Serialize contact geometry (convert Coordinate to Vector3D to avoid
  // point semantics)
  record.normal = Vector3D{contact_normal_.x(),
                            contact_normal_.y(),
                            contact_normal_.z()}.toRecord();
  record.lever_arm_a = Vector3D{lever_arm_a_.x(),
                                  lever_arm_a_.y(),
                                  lever_arm_a_.z()}.toRecord();
  record.lever_arm_b = Vector3D{lever_arm_b_.x(),
                                  lever_arm_b_.y(),
                                  lever_arm_b_.z()}.toRecord();
  record.penetration_depth = penetration_depth_;
  record.restitution = restitution_;
  record.pre_impact_rel_vel_normal = pre_impact_rel_vel_normal_;

  // Serialize friction fields
  record.tangent1 = Vector3D{tangent1_.x(),
                               tangent1_.y(),
                               tangent1_.z()}.toRecord();
  record.tangent2 = Vector3D{tangent2_.x(),
                               tangent2_.y(),
                               tangent2_.z()}.toRecord();
  record.friction_coefficient = friction_coefficient_;
  record.normal_lambda = normal_lambda_;
  record.tangent1_lambda = tangent1_lambda_;
  record.tangent2_lambda = tangent2_lambda_;

  // Dispatch to visitor via overload resolution
  visitor.visit(record);
}

}  // namespace msd_sim
