// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

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
    double /* dt */) const
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

// ===== Multi-Body Contact Constraint Solver (Ticket 0032, 0034) =====
//
// Implements Active Set Method (ASM) for contact constraints with λ ≥ 0.
// Replaced PGS (Ticket 0032b) with ASM (Ticket 0034) for exact convergence.
//
// CRITICAL notes from prototype debugging:
// - Baumgarte uses ERP formulation: b += (ERP/dt) · penetration_depth
// - Restitution RHS: b = -(1+e) · J·v_minus (for system A·λ = b)
// - DO NOT use -(1+e)·v as target velocity directly (causes energy injection)
// See: prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md

ConstraintSolver::MultiBodySolveResult ConstraintSolver::solveWithContacts(
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies,
    double dt)
{
  MultiBodySolveResult result;
  result.bodyForces.resize(numBodies, BodyForces{Coordinate{0.0, 0.0, 0.0},
                                                  Coordinate{0.0, 0.0, 0.0}});

  if (contactConstraints.empty())
  {
    result.converged = true;
    result.iterations = 0;
    result.lambdas = Eigen::VectorXd{};
    result.residual = 0.0;
    return result;
  }

  // Step 1: Compute per-contact Jacobians
  auto jacobians = assembleContactJacobians(contactConstraints, states);

  // Step 2+3: Build effective mass matrix A = J·M⁻¹·Jᵀ
  auto A = assembleContactEffectiveMass(contactConstraints, jacobians,
                                         inverseMasses, inverseInertias, numBodies);

  // Step 4: Assemble RHS vector
  auto b = assembleContactRHS(contactConstraints, jacobians, states, dt);

  // Step 5: Active Set Method solve with λ ≥ 0 enforcement
  const int numContacts = static_cast<int>(contactConstraints.size());
  auto [lambda, converged, iterations, activeSetSize] = solveActiveSet(A, b, numContacts);

  // Step 6: Extract per-body forces
  result.bodyForces = extractContactBodyForces(contactConstraints, jacobians,
                                                lambda, numBodies, dt);

  result.lambdas = lambda;
  result.converged = converged;
  result.iterations = iterations;

  // Compute residual: ||A·λ - b||
  Eigen::VectorXd residualVec = A * lambda - b;
  result.residual = residualVec.norm();

  return result;
}

// ===== Contact solver helper implementations =====

std::vector<Eigen::MatrixXd> ConstraintSolver::assembleContactJacobians(
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    const std::vector<std::reference_wrapper<const InertialState>>& states) const
{
  const size_t C = contactConstraints.size();
  std::vector<Eigen::MatrixXd> jacobians(C);

  for (size_t i = 0; i < C; ++i)
  {
    const auto* contact = contactConstraints[i];
    size_t bodyA = contact->getBodyAIndex();
    size_t bodyB = contact->getBodyBIndex();
    jacobians[i] = contact->jacobianTwoBody(
        states[bodyA].get(), states[bodyB].get(), 0.0);
  }

  return jacobians;
}

Eigen::MatrixXd ConstraintSolver::assembleContactEffectiveMass(
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies) const
{
  const size_t C = contactConstraints.size();

  // Build per-body inverse mass matrices (6×6)
  // M_k_inv = diag(inverseMass_k · I_3, inverseInertia_k)
  std::vector<Eigen::Matrix<double, 6, 6>> bodyMInv(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    bodyMInv[k] = Eigen::Matrix<double, 6, 6>::Zero();
    bodyMInv[k].block<3, 3>(0, 0) = inverseMasses[k] * Eigen::Matrix3d::Identity();
    bodyMInv[k].block<3, 3>(3, 3) = inverseInertias[k];
  }

  // Assemble effective mass matrix A (C×C)
  // A_ij = sum over shared bodies k of: J_i_k · M_k_inv · J_j_k^T
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(C), static_cast<Eigen::Index>(C));
  for (size_t i = 0; i < C; ++i)
  {
    size_t bodyA_i = contactConstraints[i]->getBodyAIndex();
    size_t bodyB_i = contactConstraints[i]->getBodyBIndex();

    // J_i is 1×12: [J_i_A(1×6) | J_i_B(1×6)]
    Eigen::Matrix<double, 1, 6> J_i_A = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> J_i_B = jacobians[i].block<1, 6>(0, 6);

    for (size_t j = i; j < C; ++j)
    {
      size_t bodyA_j = contactConstraints[j]->getBodyAIndex();
      size_t bodyB_j = contactConstraints[j]->getBodyBIndex();

      Eigen::Matrix<double, 1, 6> J_j_A = jacobians[j].block<1, 6>(0, 0);
      Eigen::Matrix<double, 1, 6> J_j_B = jacobians[j].block<1, 6>(0, 6);

      double a_ij = 0.0;

      // Check all body sharing combinations
      if (bodyA_i == bodyA_j)
        a_ij += (J_i_A * bodyMInv[bodyA_i] * J_j_A.transpose())(0);
      if (bodyA_i == bodyB_j)
        a_ij += (J_i_A * bodyMInv[bodyA_i] * J_j_B.transpose())(0);
      if (bodyB_i == bodyA_j)
        a_ij += (J_i_B * bodyMInv[bodyB_i] * J_j_A.transpose())(0);
      if (bodyB_i == bodyB_j)
        a_ij += (J_i_B * bodyMInv[bodyB_i] * J_j_B.transpose())(0);

      A(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = a_ij;
      A(static_cast<Eigen::Index>(j), static_cast<Eigen::Index>(i)) = a_ij;  // Symmetric
    }
  }

  // Add regularization to diagonal for numerical stability
  for (size_t i = 0; i < C; ++i)
  {
    A(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) += kRegularizationEpsilon;
  }

  return A;
}

Eigen::VectorXd ConstraintSolver::assembleContactRHS(
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const std::vector<std::reference_wrapper<const InertialState>>& states,
    double dt) const
{
  const size_t C = contactConstraints.size();

  // b_i = -(1 + e_i) · (J_i · v_minus) + (ERP_i / dt) · penetration_i
  //
  // CRITICAL: The -(1+e) factor is correct for the PGS system A·λ = b.
  // This differs from the target velocity formulation v_target = -e · v_pre.
  // See P2 Debug Findings for explanation.
  Eigen::VectorXd b(static_cast<Eigen::Index>(C));
  for (size_t i = 0; i < C; ++i)
  {
    const auto* contact = dynamic_cast<const ContactConstraint*>(contactConstraints[i]);
    size_t bodyA = contactConstraints[i]->getBodyAIndex();
    size_t bodyB = contactConstraints[i]->getBodyBIndex();

    // Compute pre-impact relative velocity along constraint: J_i · v_minus
    const InertialState& stateA = states[bodyA].get();
    const InertialState& stateB = states[bodyB].get();

    // Velocity vector v = [v_A, omega_A, v_B, omega_B] (12×1)
    Eigen::VectorXd v(12);
    v.segment<3>(0) = Eigen::Vector3d{stateA.velocity.x(), stateA.velocity.y(), stateA.velocity.z()};
    AngularRate omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Eigen::Vector3d{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) = Eigen::Vector3d{stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularRate omegaB = stateB.getAngularVelocity();
    v.segment<3>(9) = Eigen::Vector3d{omegaB.x(), omegaB.y(), omegaB.z()};

    double Jv = (jacobians[i] * v)(0);  // Scalar: relative velocity along constraint

    // Restitution and Baumgarte terms
    if (contact != nullptr)
    {
      double e = contact->getRestitution();
      double erp = contact->alpha();  // alpha() returns ERP for ContactConstraint
      double penetration = contact->getPenetrationDepth();

      // RHS: -(1+e) · J·v⁻ + (ERP/dt) · penetration
      b(static_cast<Eigen::Index>(i)) = -(1.0 + e) * Jv + (erp / dt) * penetration;
    }
    else
    {
      // Generic two-body constraint (no restitution)
      b(static_cast<Eigen::Index>(i)) = -Jv;
    }
  }

  return b;
}

// Ticket: 0034_active_set_method_contact_solver
// Design: docs/designs/0034_active_set_method_contact_solver/design.md
ConstraintSolver::ActiveSetResult ConstraintSolver::solveActiveSet(
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b, int numContacts) const
{
  const int C = numContacts;
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(C);

  if (C == 0)
  {
    return ActiveSetResult{lambda, true, 0, 0};
  }

  // Initialize working set: all contacts active (optimal for resting contacts)
  std::vector<int> activeIndices;
  activeIndices.reserve(static_cast<size_t>(C));
  for (int i = 0; i < C; ++i)
  {
    activeIndices.push_back(i);
  }

  const int effectiveMaxIter = std::min(2 * C, max_safety_iterations_);

  for (int iter = 1; iter <= effectiveMaxIter; ++iter)
  {
    const int activeSize = static_cast<int>(activeIndices.size());

    // Step 1: Solve equality subproblem for active set
    if (activeSize == 0)
    {
      // All contacts inactive — lambda is already zero
      lambda.setZero();
    }
    else
    {
      // Extract active submatrix and subvector
      Eigen::MatrixXd A_W(activeSize, activeSize);
      Eigen::VectorXd b_W(activeSize);

      for (int i = 0; i < activeSize; ++i)
      {
        b_W(i) = b(activeIndices[static_cast<size_t>(i)]);
        for (int j = 0; j < activeSize; ++j)
        {
          A_W(i, j) = A(activeIndices[static_cast<size_t>(i)],
                        activeIndices[static_cast<size_t>(j)]);
        }
      }

      // Direct LLT solve
      Eigen::LLT<Eigen::MatrixXd> llt{A_W};
      if (llt.info() != Eigen::Success)
      {
        // Subproblem is singular despite regularization (extremely rare)
        return ActiveSetResult{lambda, false, iter, activeSize};
      }

      Eigen::VectorXd lambda_W = llt.solve(b_W);

      // Assign solution back to full lambda vector
      lambda.setZero();
      for (int i = 0; i < activeSize; ++i)
      {
        lambda(activeIndices[static_cast<size_t>(i)]) = lambda_W(i);
      }
    }

    // Step 2: Check primal feasibility (lambda >= 0 for active set)
    double minLambda = std::numeric_limits<double>::infinity();
    int minIndex = -1;
    for (int i = 0; i < activeSize; ++i)
    {
      int idx = activeIndices[static_cast<size_t>(i)];
      double val = lambda(idx);
      if (val < minLambda || (val == minLambda && idx < minIndex))
      {
        minLambda = val;
        minIndex = idx;
      }
    }

    if (minLambda < 0.0)
    {
      // Remove most negative lambda from active set (Bland's rule for ties)
      activeIndices.erase(
          std::remove(activeIndices.begin(), activeIndices.end(), minIndex),
          activeIndices.end());
      continue;
    }

    // Step 3: Check dual feasibility (w >= 0 for inactive set)
    Eigen::VectorXd w = A * lambda - b;

    double maxViolation = 0.0;
    int maxViolationIndex = -1;
    for (int i = 0; i < C; ++i)
    {
      // Skip active contacts
      bool isActive = std::find(activeIndices.begin(), activeIndices.end(), i) != activeIndices.end();
      if (isActive)
      {
        continue;
      }

      if (w(i) < -convergence_tolerance_)
      {
        if (maxViolationIndex == -1 ||
            w(i) < maxViolation ||
            (w(i) == maxViolation && i < maxViolationIndex))
        {
          maxViolation = w(i);
          maxViolationIndex = i;
        }
      }
    }

    if (maxViolationIndex == -1)
    {
      // All KKT conditions satisfied — exact solution found
      return ActiveSetResult{lambda, true, iter, activeSize};
    }

    // Step 4: Add most violated constraint to active set (sorted insertion)
    auto insertPos = std::lower_bound(activeIndices.begin(), activeIndices.end(), maxViolationIndex);
    activeIndices.insert(insertPos, maxViolationIndex);
  }

  // Safety cap reached
  return ActiveSetResult{lambda, false, effectiveMaxIter, static_cast<int>(activeIndices.size())};
}

std::vector<ConstraintSolver::BodyForces> ConstraintSolver::extractContactBodyForces(
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    const std::vector<Eigen::MatrixXd>& jacobians,
    const Eigen::VectorXd& lambda,
    size_t numBodies,
    double dt) const
{
  const size_t C = contactConstraints.size();
  std::vector<BodyForces> bodyForces(numBodies, BodyForces{Coordinate{0.0, 0.0, 0.0},
                                                            Coordinate{0.0, 0.0, 0.0}});

  // F_body_k = sum over contacts touching body k: J_i_k^T · lambda_i / dt
  // Dividing by dt converts impulse to force (since integrator multiplies by dt)
  for (size_t i = 0; i < C; ++i)
  {
    if (lambda(static_cast<Eigen::Index>(i)) <= 0.0)
      continue;

    size_t bodyA = contactConstraints[i]->getBodyAIndex();
    size_t bodyB = contactConstraints[i]->getBodyBIndex();

    Eigen::Matrix<double, 1, 6> J_i_A = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> J_i_B = jacobians[i].block<1, 6>(0, 6);

    // Force = J^T · lambda / dt (convert impulse to force)
    Eigen::Matrix<double, 6, 1> forceA = J_i_A.transpose() * lambda(static_cast<Eigen::Index>(i)) / dt;
    Eigen::Matrix<double, 6, 1> forceB = J_i_B.transpose() * lambda(static_cast<Eigen::Index>(i)) / dt;

    bodyForces[bodyA].linearForce += Coordinate{forceA(0), forceA(1), forceA(2)};
    bodyForces[bodyA].angularTorque += Coordinate{forceA(3), forceA(4), forceA(5)};

    bodyForces[bodyB].linearForce += Coordinate{forceB(0), forceB(1), forceB(2)};
    bodyForces[bodyB].angularTorque += Coordinate{forceB(3), forceB(4), forceB(5)};
  }

  return bodyForces;
}

}  // namespace msd_sim
