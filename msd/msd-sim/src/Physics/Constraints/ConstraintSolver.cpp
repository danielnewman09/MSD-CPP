// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>

namespace msd_sim
{

ConstraintSolver::SolveResult ConstraintSolver::solve(
    const std::vector<Constraint*>& constraints,
    const InertialState& state,
    const Coordinate& externalForce,
    const Coordinate& externalTorque,
    double mass,
    const Eigen::Matrix3d& inverseInertia,
    double dt)
{
  // Handle empty constraint set
  if (constraints.empty())
  {
    return SolveResult{Eigen::VectorXd{},
                       Coordinate{0.0, 0.0, 0.0},
                       Coordinate{0.0, 0.0, 0.0},
                       true,  // converged (vacuously)
                       1.0};  // condition number
  }

  // Current simulation time (not used by current constraints, but part of interface)
  double time = 0.0;  // WorldModel doesn't track absolute time yet

  // Assemble constraint matrix A = J·M^-1·J^T
  Eigen::MatrixXd A = assembleConstraintMatrix(constraints, state, time, mass, inverseInertia);

  // Assemble RHS b = -J·M^-1·F_ext - α·C - β·Ċ
  Eigen::VectorXd b = assembleRHS(constraints, state, externalForce, externalTorque,
                                   mass, inverseInertia, time, dt);

  // Compute condition number for diagnostics
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
  double conditionNumber = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // Solve A·λ = b using LLT decomposition (assumes positive definite)
  Eigen::LLT<Eigen::MatrixXd> llt(A);
  if (llt.info() != Eigen::Success)
  {
    // Matrix is not positive definite (singular or ill-conditioned)
    return SolveResult{Eigen::VectorXd{},
                       Coordinate{0.0, 0.0, 0.0},
                       Coordinate{0.0, 0.0, 0.0},
                       false,  // not converged
                       conditionNumber};
  }

  Eigen::VectorXd lambdas = llt.solve(b);

  // Extract constraint forces F_c = J^T·λ
  auto [linearForce, angularTorque] = extractConstraintForces(lambdas, constraints, state, time);

  return SolveResult{lambdas,
                     linearForce,
                     angularTorque,
                     true,  // converged
                     conditionNumber};
}

Eigen::MatrixXd ConstraintSolver::assembleConstraintMatrix(
    const std::vector<Constraint*>& constraints,
    const InertialState& state,
    double time,
    double mass,
    const Eigen::Matrix3d& inverseInertia) const
{
  // Compute total constraint dimension
  int totalDim = 0;
  for (const auto* constraint : constraints)
  {
    totalDim += constraint->dimension();
  }

  // Assemble stacked Jacobian J (totalDim × 7)
  Eigen::MatrixXd J(totalDim, 7);
  int rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int dim = constraint->dimension();
    J.block(rowOffset, 0, dim, 7) = constraint->jacobian(state, time);
    rowOffset += dim;
  }

  // Construct mass matrix inverse M^-1 (7 × 7 block diagonal)
  Eigen::MatrixXd M_inv = Eigen::MatrixXd::Zero(7, 7);

  // Linear mass inverse: (1/m)·I₃
  M_inv.block<3, 3>(0, 0) = (1.0 / mass) * Eigen::Matrix3d::Identity();

  // Angular mass inverse: I^-1 (inverse inertia tensor)
  M_inv.block<3, 3>(3, 3) = inverseInertia;

  // Note: Quaternion components don't have a direct "mass" in configuration space,
  // but the inertia tensor relates angular velocity to quaternion rate via
  // ω = 2·Q̄⊗Q̇. The 4×4 block should technically be derived from the quaternion
  // kinetic energy metric, but for the quaternion constraint (which only cares
  // about normalization), we use the identity for the remaining component.
  M_inv(6, 6) = 1.0;  // Quaternion scalar component (simplified)

  // Compute constraint matrix A = J·M^-1·J^T
  Eigen::MatrixXd A = J * M_inv * J.transpose();

  return A;
}

Eigen::VectorXd ConstraintSolver::assembleRHS(
    const std::vector<Constraint*>& constraints,
    const InertialState& state,
    const Coordinate& externalForce,
    const Coordinate& externalTorque,
    double mass,
    const Eigen::Matrix3d& inverseInertia,
    double time,
    double dt) const
{
  // Compute total constraint dimension
  int totalDim = 0;
  for (const auto* constraint : constraints)
  {
    totalDim += constraint->dimension();
  }

  // Assemble stacked Jacobian J (totalDim × 7)
  Eigen::MatrixXd J(totalDim, 7);
  int rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int dim = constraint->dimension();
    J.block(rowOffset, 0, dim, 7) = constraint->jacobian(state, time);
    rowOffset += dim;
  }

  // Construct mass matrix inverse M^-1 (7 × 7 block diagonal)
  Eigen::MatrixXd M_inv = Eigen::MatrixXd::Zero(7, 7);
  M_inv.block<3, 3>(0, 0) = (1.0 / mass) * Eigen::Matrix3d::Identity();
  M_inv.block<3, 3>(3, 3) = inverseInertia;
  M_inv(6, 6) = 1.0;

  // External forces in generalized coordinates (7 × 1)
  Eigen::VectorXd F_ext(7);
  F_ext.segment<3>(0) = Eigen::Vector3d{externalForce.x(), externalForce.y(), externalForce.z()};
  F_ext.segment<3>(3) = Eigen::Vector3d{externalTorque.x(), externalTorque.y(), externalTorque.z()};
  F_ext(6) = 0.0;  // No external force on quaternion scalar component

  // Assemble constraint violation C and time derivative Ċ
  Eigen::VectorXd C(totalDim);
  Eigen::VectorXd C_dot(totalDim);
  Eigen::VectorXd alpha_vec(totalDim);
  Eigen::VectorXd beta_vec(totalDim);
  rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int dim = constraint->dimension();
    C.segment(rowOffset, dim) = constraint->evaluate(state, time);
    C_dot.segment(rowOffset, dim) = constraint->partialTimeDerivative(state, time);

    // Baumgarte parameters (per-constraint)
    for (int i = 0; i < dim; ++i)
    {
      alpha_vec(rowOffset + i) = constraint->alpha();
      beta_vec(rowOffset + i) = constraint->beta();
    }

    rowOffset += dim;
  }

  // Compute velocity-level constraint violation Ċ = J·q̇
  // For holonomic constraints, Ċ = J·q̇ + ∂C/∂t
  // State velocity vector (7 × 1)
  Eigen::VectorXd q_dot(7);
  q_dot.segment<3>(0) = Eigen::Vector3d{state.velocity.x(), state.velocity.y(), state.velocity.z()};

  // Quaternion rate Q̇
  q_dot.segment<4>(3) = state.quaternionRate;

  Eigen::VectorXd J_q_dot = J * q_dot;
  Eigen::VectorXd C_dot_total = J_q_dot + C_dot;

  // RHS: b = -J·M^-1·F_ext - α·C - β·Ċ
  Eigen::VectorXd b = -J * M_inv * F_ext - alpha_vec.cwiseProduct(C) - beta_vec.cwiseProduct(C_dot_total);

  return b;
}

std::pair<Coordinate, Coordinate> ConstraintSolver::extractConstraintForces(
    const Eigen::VectorXd& lambdas,
    const std::vector<Constraint*>& constraints,
    const InertialState& state,
    double time) const
{
  // Compute total constraint dimension
  int totalDim = 0;
  for (const auto* constraint : constraints)
  {
    totalDim += constraint->dimension();
  }

  // Assemble stacked Jacobian J (totalDim × 7)
  Eigen::MatrixXd J(totalDim, 7);
  int rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int dim = constraint->dimension();
    J.block(rowOffset, 0, dim, 7) = constraint->jacobian(state, time);
    rowOffset += dim;
  }

  // Constraint forces: F_c = J^T·λ (7 × 1)
  Eigen::VectorXd F_c = J.transpose() * lambdas;

  // Extract linear force (first 3 components)
  Coordinate linearForce{F_c(0), F_c(1), F_c(2)};

  // Extract angular torque (next 3 components)
  // Note: Component 6 is the quaternion scalar force, which we ignore
  // (it contributes to quaternion normalization but not angular dynamics)
  Coordinate angularTorque{F_c(3), F_c(4), F_c(5)};

  return {linearForce, angularTorque};
}

}  // namespace msd_sim
