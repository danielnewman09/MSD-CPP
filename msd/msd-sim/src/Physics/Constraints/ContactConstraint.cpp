// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
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
                                     double preImpactRelVelNormal)
  : TwoBodyConstraint{bodyAIndex, bodyBIndex},
    contact_normal_{normal},
    lever_arm_a_{contactPointA - comA},
    lever_arm_b_{contactPointB - comB},
    penetration_depth_{penetrationDepth},
    restitution_{restitution},
    pre_impact_rel_vel_normal_{preImpactRelVelNormal}
{
  // Validate normal is unit length (within tolerance)
  double normalLength = normal.norm();
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
}

Eigen::VectorXd ContactConstraint::evaluateTwoBody(
    const InertialState& stateA,
    const InertialState& stateB,
    double /* time */) const
{
  // Constraint function: C = (x_B - x_A) · n
  // where x_A = posA + r_A, x_B = posB + r_B
  //
  // For penetrating bodies, C < 0 (constraint violated)
  // For separated bodies, C > 0 (constraint satisfied)

  Coordinate contactPointA = stateA.position + lever_arm_a_;
  Coordinate contactPointB = stateB.position + lever_arm_b_;

  Eigen::VectorXd C(1);
  C(0) = (contactPointB - contactPointA).dot(contact_normal_);

  return C;
}

Eigen::MatrixXd ContactConstraint::jacobianTwoBody(
    const InertialState& /* stateA */,
    const InertialState& /* stateB */,
    double /* time */) const
{
  // Jacobian J = [∂C/∂v_A, ∂C/∂ω_A, ∂C/∂v_B, ∂C/∂ω_B]
  //
  // From math formulation Section 3.6:
  // J = [-n^T, -(r_A × n)^T, n^T, (r_B × n)^T]
  //
  // This is the velocity-level Jacobian (12 columns):
  // - Columns 0-2: ∂C/∂v_A = -n^T
  // - Columns 3-5: ∂C/∂ω_A = -(r_A × n)^T
  // - Columns 6-8: ∂C/∂v_B = n^T
  // - Columns 9-11: ∂C/∂ω_B = (r_B × n)^T

  Eigen::MatrixXd J(1, 12);

  // Linear components
  J.block<1, 3>(0, 0) = -contact_normal_.transpose();  // -n^T
  J.block<1, 3>(0, 6) = contact_normal_.transpose();   // n^T

  // Angular components (cross product: r × n)
  Coordinate rA_cross_n = lever_arm_a_.cross(contact_normal_);
  Coordinate rB_cross_n = lever_arm_b_.cross(contact_normal_);

  J.block<1, 3>(0, 3) = -rA_cross_n.transpose();  // -(r_A × n)^T
  J.block<1, 3>(0, 9) = rB_cross_n.transpose();   // (r_B × n)^T

  return J;
}

bool ContactConstraint::isActiveTwoBody(
    const InertialState& stateA,
    const InertialState& stateB,
    double time) const
{
  // Contact is active if bodies are penetrating or very close (C <= threshold)
  // Threshold chosen to match typical slop tolerance (0.01m)
  const double kActivationThreshold = 0.01;  // [m]

  Eigen::VectorXd C = evaluateTwoBody(stateA, stateB, time);
  return C(0) <= kActivationThreshold;
}

}  // namespace msd_sim
