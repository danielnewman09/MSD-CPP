// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Ticket: 0035b4_ecos_solve_integration
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include <Eigen/src/Cholesky/LLT.h>
#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <ecos/ecos.h>
#include <ecos/glblopts.h>
#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

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

  // Current simulation time (not used by current constraints, but part of
  // interface)
  double const time = 0.0;  // WorldModel doesn't track absolute time yet

  // Assemble constraint matrix A = J·M^-1·J^T
  Eigen::MatrixXd a =
    assembleConstraintMatrix(constraints, state, time, mass, inverseInertia);

  // Assemble RHS b = -J·M^-1·F_ext - α·C - β·Ċ
  Eigen::VectorXd const b = assembleRHS(constraints,
                                        state,
                                        externalForce,
                                        externalTorque,
                                        mass,
                                        inverseInertia,
                                        time,
                                        dt);

  // Compute condition number for diagnostics
  Eigen::JacobiSVD<Eigen::MatrixXd> const svd(a);
  double const conditionNumber =
    svd.singularValues()(0) /
    svd.singularValues()(svd.singularValues().size() - 1);

  // Solve A·λ = b using LLT decomposition (assumes positive definite)
  Eigen::LLT<Eigen::MatrixXd> const llt(a);
  if (llt.info() != Eigen::Success)
  {
    // Matrix is not positive definite (singular or ill-conditioned)
    return SolveResult{Eigen::VectorXd{},
                       Coordinate{0.0, 0.0, 0.0},
                       Coordinate{0.0, 0.0, 0.0},
                       false,  // not converged
                       conditionNumber};
  }

  Eigen::VectorXd const lambdas = llt.solve(b);

  // Extract constraint forces F_c = J^T·λ
  auto [linearForce, angularTorque] =
    extractConstraintForces(lambdas, constraints, state, time);

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
  const Eigen::Matrix3d& inverseInertia)
{
  // Compute total constraint dimension
  int totalDim = 0;
  for (const auto* constraint : constraints)
  {
    totalDim += constraint->dimension();
  }

  // Assemble stacked Jacobian J (totalDim × 7)
  Eigen::MatrixXd j(totalDim, kNumStates);
  int rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int const dim = constraint->dimension();
    j.block(rowOffset, 0, dim, kNumStates) = constraint->jacobian(state, time);
    rowOffset += dim;
  }

  // Construct mass matrix inverse M^-1 (7 × 7 block diagonal)
  Eigen::MatrixXd mInv = Eigen::MatrixXd::Zero(kNumStates, kNumStates);

  // Linear mass inverse: (1/m)·I₃
  mInv.block<3, 3>(0, 0) = (1.0 / mass) * Eigen::Matrix3d::Identity();

  // Angular mass inverse: I^-1 (inverse inertia tensor)
  mInv.block<3, 3>(3, 3) = inverseInertia;

  // Note: Quaternion components don't have a direct "mass" in configuration
  // space, but the inertia tensor relates angular velocity to quaternion rate
  // via ω = 2·Q̄⊗Q̇. The 4×4 block should technically be derived from the
  // quaternion kinetic energy metric, but for the quaternion constraint (which
  // only cares about normalization), we use the identity for the remaining
  // component.
  mInv(6, 6) = 1.0;  // Quaternion scalar component (simplified)

  // Compute constraint matrix A = J·M^-1·J^T
  Eigen::MatrixXd a = j * mInv * j.transpose();

  return a;
}

Eigen::VectorXd ConstraintSolver::assembleRHS(
  const std::vector<Constraint*>& constraints,
  const InertialState& state,
  const Coordinate& externalForce,
  const Coordinate& externalTorque,
  double mass,
  const Eigen::Matrix3d& inverseInertia,
  double time,
  double /* dt */)
{
  // Compute total constraint dimension
  int totalDim = 0;
  for (const auto* constraint : constraints)
  {
    totalDim += constraint->dimension();
  }

  // Assemble stacked Jacobian J (totalDim × 7)
  Eigen::MatrixXd j(totalDim, kNumStates);
  int rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int const dim = constraint->dimension();
    j.block(rowOffset, 0, dim, kNumStates) = constraint->jacobian(state, time);
    rowOffset += dim;
  }

  // Construct mass matrix inverse M^-1 (7 × 7 block diagonal)
  Eigen::MatrixXd mInv = Eigen::MatrixXd::Zero(kNumStates, kNumStates);
  mInv.block<3, 3>(0, 0) = (1.0 / mass) * Eigen::Matrix3d::Identity();
  mInv.block<3, 3>(3, 3) = inverseInertia;
  mInv(kNumStates - 1, kNumStates - 1) = 1.0;

  // External forces in generalized coordinates (7 × 1)
  Eigen::VectorXd fExt(kNumStates);
  fExt.segment<3>(0) =
    Eigen::Vector3d{externalForce.x(), externalForce.y(), externalForce.z()};
  fExt.segment<3>(3) =
    Eigen::Vector3d{externalTorque.x(), externalTorque.y(), externalTorque.z()};
  fExt(6) = 0.0;  // No external force on quaternion scalar component

  // Assemble constraint violation C and time derivative Ċ
  Eigen::VectorXd c(totalDim);
  Eigen::VectorXd cDot(totalDim);
  Eigen::VectorXd alphaVec(totalDim);
  Eigen::VectorXd betaVec(totalDim);
  rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int const dim = constraint->dimension();
    c.segment(rowOffset, dim) = constraint->evaluate(state, time);
    cDot.segment(rowOffset, dim) =
      constraint->partialTimeDerivative(state, time);

    // Baumgarte parameters (per-constraint)
    for (int i = 0; i < dim; ++i)
    {
      alphaVec(rowOffset + i) = constraint->alpha();
      betaVec(rowOffset + i) = constraint->beta();
    }

    rowOffset += dim;
  }

  // Compute velocity-level constraint violation Ċ = J·q̇
  // For holonomic constraints, Ċ = J·q̇ + ∂C/∂t
  // State velocity vector (7 × 1)
  Eigen::VectorXd qDot(kNumStates);
  qDot.segment<3>(0) =
    Eigen::Vector3d{state.velocity.x(), state.velocity.y(), state.velocity.z()};

  // Quaternion rate Q̇
  qDot.segment<4>(3) = state.quaternionRate;

  Eigen::VectorXd const jQDot = j * qDot;
  Eigen::VectorXd const cDotTotal = jQDot + cDot;

  // RHS: b = -J·M^-1·F_ext - α·C - β·Ċ
  Eigen::VectorXd b = -j * mInv * fExt - alphaVec.cwiseProduct(c) -
                      betaVec.cwiseProduct(cDotTotal);

  return b;
}

std::pair<Coordinate, Coordinate> ConstraintSolver::extractConstraintForces(
  const Eigen::VectorXd& lambdas,
  const std::vector<Constraint*>& constraints,
  const InertialState& state,
  double time)
{
  // Compute total constraint dimension
  int totalDim = 0;
  for (const auto* constraint : constraints)
  {
    totalDim += constraint->dimension();
  }

  // Assemble stacked Jacobian J (totalDim × 7)
  Eigen::MatrixXd j(totalDim, kNumStates);
  int rowOffset = 0;
  for (const auto* constraint : constraints)
  {
    int const dim = constraint->dimension();
    j.block(rowOffset, 0, dim, kNumStates) = constraint->jacobian(state, time);
    rowOffset += dim;
  }

  // Constraint forces: F_c = J^T·λ (7 × 1)
  Eigen::VectorXd fC = j.transpose() * lambdas;

  // Extract linear force (first 3 components)
  Coordinate const linearForce{fC(0), fC(1), fC(2)};

  // Extract angular torque (next 3 components)
  // Note: Component 6 is the quaternion scalar force, which we ignore
  // (it contributes to quaternion normalization but not angular dynamics)
  Coordinate const angularTorque{fC(3), fC(4), fC(5)};

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
// See:
// prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md

ConstraintSolver::MultiBodySolveResult ConstraintSolver::solveWithContacts(
  const std::vector<TwoBodyConstraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies,
  double dt)
{
  MultiBodySolveResult result;
  result.bodyForces.resize(
    numBodies,
    BodyForces{Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0}});

  if (contactConstraints.empty())
  {
    result.converged = true;
    result.iterations = 0;
    result.lambdas = Eigen::VectorXd{};
    result.residual = 0.0;
    return result;
  }

  // Ticket 0035b4: Detect friction constraints via dynamic_cast
  // If any FrictionConstraint is present, route to ECOS SOCP solver
  bool hasFriction = false;
  for (const auto* constraint : contactConstraints)
  {
    if (dynamic_cast<const FrictionConstraint*>(constraint) != nullptr)
    {
      hasFriction = true;
      break;
    }
  }

  // Step 1: Compute per-contact Jacobians
  auto jacobians = assembleContactJacobians(contactConstraints, states);

  // Step 2+3: Build effective mass matrix A = J·M⁻¹·Jᵀ
  auto a = assembleContactEffectiveMass(
    contactConstraints, jacobians, inverseMasses, inverseInertias, numBodies);

  // Step 4: Assemble RHS vector
  auto b = assembleContactRHS(contactConstraints, jacobians, states, dt);

  // Step 5: Solve — dispatch based on friction presence (Ticket 0035b4)
  const int numConstraintRows = static_cast<int>(contactConstraints.size());
  ActiveSetResult asmResult;

  if (hasFriction)
  {
    // Count actual contacts (each contact = 1 normal + 1 friction with dim=2,
    // so constraint count = C normals + C frictions = 2C constraints, but
    // the Jacobian rows are: C*1 (normals) + C*2 (frictions) = 3C rows)
    // However, contactConstraints is a flat list of TwoBodyConstraint*.
    // We need to count the number of ContactConstraint (normals) to get C.
    int numContacts = 0;
    for (const auto* constraint : contactConstraints)
    {
      if (dynamic_cast<const ContactConstraint*>(constraint) != nullptr)
      {
        ++numContacts;
      }
    }

    // Build friction cone specification from constraint list
    FrictionConeSpec const coneSpec =
      buildFrictionConeSpec(contactConstraints, numContacts);

    // Route to ECOS SOCP solver
    asmResult = solveWithECOS(a, b, coneSpec, numContacts);
  }
  else
  {
    // No friction — use existing Active Set Method (zero-regression path)
    asmResult = solveActiveSet(a, b, numConstraintRows);
  }

  // Step 6: Extract per-body forces
  result.bodyForces = extractContactBodyForces(
    contactConstraints, jacobians, asmResult.lambda, numBodies, dt);

  result.lambdas = asmResult.lambda;
  result.converged = asmResult.converged;
  result.iterations = asmResult.iterations;

  // Compute residual: ||A·λ - b||
  Eigen::VectorXd const residualVec = a * asmResult.lambda - b;
  result.residual = residualVec.norm();

  return result;
}

// ===== Contact solver helper implementations =====

std::vector<Eigen::MatrixXd> ConstraintSolver::assembleContactJacobians(
  const std::vector<TwoBodyConstraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states)
{
  const size_t c = contactConstraints.size();
  std::vector<Eigen::MatrixXd> jacobians(c);

  for (size_t i = 0; i < c; ++i)
  {
    const auto* contact = contactConstraints[i];
    size_t const bodyA = contact->getBodyAIndex();
    size_t const bodyB = contact->getBodyBIndex();
    jacobians[i] =
      contact->jacobianTwoBody(states[bodyA].get(), states[bodyB].get(), 0.0);
  }

  return jacobians;
}

Eigen::MatrixXd ConstraintSolver::assembleContactEffectiveMass(
  const std::vector<TwoBodyConstraint*>& contactConstraints,
  const std::vector<Eigen::MatrixXd>& jacobians,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies)
{
  const size_t c = contactConstraints.size();

  // Build per-body inverse mass matrices (6×6)
  // M_k_inv = diag(inverseMass_k · I_3, inverseInertia_k)
  std::vector<Eigen::Matrix<double, 6, 6>> bodyMInv(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    bodyMInv[k] = Eigen::Matrix<double, 6, 6>::Zero();
    bodyMInv[k].block<3, 3>(0, 0) =
      inverseMasses[k] * Eigen::Matrix3d::Identity();
    bodyMInv[k].block<3, 3>(3, 3) = inverseInertias[k];
  }

  // Assemble effective mass matrix A (C×C)
  // A_ij = sum over shared bodies k of: J_i_k · M_k_inv · J_j_k^T
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(c),
                                            static_cast<Eigen::Index>(c));
  for (size_t i = 0; i < c; ++i)
  {
    size_t const bodyAI = contactConstraints[i]->getBodyAIndex();
    size_t const bodyBI = contactConstraints[i]->getBodyBIndex();

    // J_i is 1×12: [J_i_A(1×6) | J_i_B(1×6)]
    Eigen::Matrix<double, 1, 6> const jIA = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> const jIB = jacobians[i].block<1, 6>(0, 6);

    for (size_t j = i; j < c; ++j)
    {
      size_t const bodyAJ = contactConstraints[j]->getBodyAIndex();
      size_t const bodyBJ = contactConstraints[j]->getBodyBIndex();

      Eigen::Matrix<double, 1, 6> jJA = jacobians[j].block<1, 6>(0, 0);
      Eigen::Matrix<double, 1, 6> jJB = jacobians[j].block<1, 6>(0, 6);

      double aIj = 0.0;

      // Check all body sharing combinations
      if (bodyAI == bodyAJ)
      {
        aIj += (jIA * bodyMInv[bodyAI] * jJA.transpose())(0);
      }
      if (bodyAI == bodyBJ)
      {
        aIj += (jIA * bodyMInv[bodyAI] * jJB.transpose())(0);
      }
      if (bodyBI == bodyAJ)
      {
        aIj += (jIB * bodyMInv[bodyBI] * jJA.transpose())(0);
      }
      if (bodyBI == bodyBJ)
      {
        aIj += (jIB * bodyMInv[bodyBI] * jJB.transpose())(0);
      }

      a(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = aIj;
      a(static_cast<Eigen::Index>(j), static_cast<Eigen::Index>(i)) =
        aIj;  // Symmetric
    }
  }

  // Add regularization to diagonal for numerical stability
  for (size_t i = 0; i < c; ++i)
  {
    a(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) +=
      kRegularizationEpsilon;
  }

  return a;
}

Eigen::VectorXd ConstraintSolver::assembleContactRHS(
  const std::vector<TwoBodyConstraint*>& contactConstraints,
  const std::vector<Eigen::MatrixXd>& jacobians,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  double dt)
{
  const size_t c = contactConstraints.size();

  // b_i = -(1 + e_i) · (J_i · v_minus) + (ERP_i / dt) · penetration_i
  //
  // CRITICAL: The -(1+e) factor is correct for the PGS system A·λ = b.
  // This differs from the target velocity formulation v_target = -e · v_pre.
  // See P2 Debug Findings for explanation.
  Eigen::VectorXd b(static_cast<Eigen::Index>(c));
  for (size_t i = 0; i < c; ++i)
  {
    const auto* contact =
      dynamic_cast<const ContactConstraint*>(contactConstraints[i]);
    size_t const bodyA = contactConstraints[i]->getBodyAIndex();
    size_t const bodyB = contactConstraints[i]->getBodyBIndex();

    // Compute pre-impact relative velocity along constraint: J_i · v_minus
    const InertialState& stateA = states[bodyA].get();
    const InertialState& stateB = states[bodyB].get();

    // Velocity vector v = [v_A, omega_A, v_B, omega_B] (12×1)
    Eigen::VectorXd v(12);
    v.segment<3>(0) = Eigen::Vector3d{
      stateA.velocity.x(), stateA.velocity.y(), stateA.velocity.z()};
    AngularRate omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Eigen::Vector3d{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) = Eigen::Vector3d{
      stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularRate omegaB = stateB.getAngularVelocity();
    v.segment<3>(9) = Eigen::Vector3d{omegaB.x(), omegaB.y(), omegaB.z()};

    double const jv =
      (jacobians[i] * v)(0);  // Scalar: relative velocity along constraint

    // Restitution and Baumgarte terms
    if (contact != nullptr)
    {
      double const e = contact->getRestitution();
      double const erp =
        contact->alpha();  // alpha() returns ERP for ContactConstraint
      double const penetration = contact->getPenetrationDepth();

      // RHS: -(1+e) · J·v⁻ + (ERP/dt) · penetration
      b(static_cast<Eigen::Index>(i)) =
        (-(1.0 + e) * jv) + ((erp / dt) * penetration);
    }
    else
    {
      // Generic two-body constraint (no restitution)
      b(static_cast<Eigen::Index>(i)) = -jv;
    }
  }

  return b;
}

// Ticket: 0034_active_set_method_contact_solver
// Design: docs/designs/0034_active_set_method_contact_solver/design.md
ConstraintSolver::ActiveSetResult ConstraintSolver::solveActiveSet(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  int numContacts) const
{
  const int c = numContacts;
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(c);

  if (c == 0)
  {
    return ActiveSetResult{.lambda = lambda,
                           .converged = true,
                           .iterations = 0,
                           .active_set_size = 0};
  }

  // Initialize working set: all contacts active (optimal for resting contacts)
  std::vector<int> activeIndices;
  activeIndices.reserve(static_cast<size_t>(c));
  for (int i = 0; i < c; ++i)
  {
    activeIndices.push_back(i);
  }

  const int effectiveMaxIter = std::min(2 * c, max_safety_iterations_);

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
      Eigen::MatrixXd aW(activeSize, activeSize);
      Eigen::VectorXd bW(activeSize);

      for (int i = 0; i < activeSize; ++i)
      {
        bW(i) = b(activeIndices[static_cast<size_t>(i)]);
        for (int j = 0; j < activeSize; ++j)
        {
          aW(i, j) = A(activeIndices[static_cast<size_t>(i)],
                       activeIndices[static_cast<size_t>(j)]);
        }
      }

      // Direct LLT solve
      Eigen::LLT<Eigen::MatrixXd> const llt{aW};
      if (llt.info() != Eigen::Success)
      {
        // Subproblem is singular despite regularization (extremely rare)
        return ActiveSetResult{.lambda = lambda,
                               .converged = false,
                               .iterations = iter,
                               .active_set_size = activeSize};
      }

      Eigen::VectorXd lambdaW = llt.solve(bW);

      // Assign solution back to full lambda vector
      lambda.setZero();
      for (int i = 0; i < activeSize; ++i)
      {
        lambda(activeIndices[static_cast<size_t>(i)]) = lambdaW(i);
      }
    }

    // Step 2: Check primal feasibility (lambda >= 0 for active set)
    double minLambda = std::numeric_limits<double>::infinity();
    int minIndex = -1;
    for (int i = 0; i < activeSize; ++i)
    {
      int const idx = activeIndices[static_cast<size_t>(i)];
      double const val = lambda(idx);
      if (val < minLambda || (val == minLambda && idx < minIndex))
      {
        minLambda = val;
        minIndex = idx;
      }
    }

    if (minLambda < 0.0)
    {
      // Remove most negative lambda from active set (Bland's rule for ties)
      std::erase(activeIndices, minIndex);
      continue;
    }

    // Step 3: Check dual feasibility (w >= 0 for inactive set)
    Eigen::VectorXd w = A * lambda - b;

    double maxViolation = 0.0;
    int maxViolationIndex = -1;
    for (int i = 0; i < c; ++i)
    {
      // Skip active contacts
      bool const isActive =
        std::ranges::find(activeIndices, i) != activeIndices.end();
      if (isActive)
      {
        continue;
      }

      if (w(i) < -convergence_tolerance_)
      {
        if (maxViolationIndex == -1 || w(i) < maxViolation ||
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
      return ActiveSetResult{.lambda = lambda,
                             .converged = true,
                             .iterations = iter,
                             .active_set_size = activeSize};
    }

    // Step 4: Add most violated constraint to active set (sorted insertion)
    auto insertPos = std::ranges::lower_bound(activeIndices, maxViolationIndex);
    activeIndices.insert(insertPos, maxViolationIndex);
  }

  // Safety cap reached
  return ActiveSetResult{
    .lambda = lambda,
    .converged = false,
    .iterations = effectiveMaxIter,
    .active_set_size = static_cast<int>(activeIndices.size())};
}

std::vector<ConstraintSolver::BodyForces>
ConstraintSolver::extractContactBodyForces(
  const std::vector<TwoBodyConstraint*>& contactConstraints,
  const std::vector<Eigen::MatrixXd>& jacobians,
  const Eigen::VectorXd& lambda,
  size_t numBodies,
  double dt)
{
  const size_t c = contactConstraints.size();
  std::vector<BodyForces> bodyForces(
    numBodies,
    BodyForces{Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0}});

  // F_body_k = sum over contacts touching body k: J_i_k^T · lambda_i / dt
  // Dividing by dt converts impulse to force (since integrator multiplies by
  // dt)
  for (size_t i = 0; i < c; ++i)
  {
    if (lambda(static_cast<Eigen::Index>(i)) <= 0.0)
    {
      continue;
    }

    size_t const bodyA = contactConstraints[i]->getBodyAIndex();
    size_t const bodyB = contactConstraints[i]->getBodyBIndex();

    Eigen::Matrix<double, 1, 6> jIA = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> jIB = jacobians[i].block<1, 6>(0, 6);

    // Force = J^T · lambda / dt (convert impulse to force)
    Eigen::Matrix<double, 6, 1> forceA =
      jIA.transpose() * lambda(static_cast<Eigen::Index>(i)) / dt;
    Eigen::Matrix<double, 6, 1> forceB =
      jIB.transpose() * lambda(static_cast<Eigen::Index>(i)) / dt;

    bodyForces[bodyA].linearForce +=
      Coordinate{forceA(0), forceA(1), forceA(2)};
    bodyForces[bodyA].angularTorque +=
      Coordinate{forceA(3), forceA(4), forceA(5)};

    bodyForces[bodyB].linearForce +=
      Coordinate{forceB(0), forceB(1), forceB(2)};
    bodyForces[bodyB].angularTorque +=
      Coordinate{forceB(3), forceB(4), forceB(5)};
  }

  return bodyForces;
}

// ===== ECOS Solver Integration (Ticket 0035b4) =====
//
// Implements friction cone constraints via ECOS SOCP solver.
// The contact LCP with friction is reformulated as:
//   min c^T·x  s.t.  A_eq·x = b_eq,  G·x + s = h,  s ∈ K_SOC
// where K_SOC is a product of second-order cones (one 3D cone per contact).

FrictionConeSpec ConstraintSolver::buildFrictionConeSpec(
  const std::vector<TwoBodyConstraint*>& contactConstraints,
  int numContacts)
{
  FrictionConeSpec coneSpec{numContacts};

  // Constraint ordering convention:
  // For each contact i, the constraints are interleaved as:
  //   ContactConstraint (normal, dim=1) at some index
  //   FrictionConstraint (tangential, dim=2) at some index
  //
  // In the 3C-dimensional lambda vector, the ordering is:
  //   [λ_n0, λ_t1_0, λ_t2_0, λ_n1, λ_t1_1, λ_t2_1, ...]
  //
  // We scan for FrictionConstraint instances and extract μ values.
  // The normal constraint index for contact i is 3*i.
  int contactIdx = 0;
  for (const auto* constraint : contactConstraints)
  {
    const auto* friction = dynamic_cast<const FrictionConstraint*>(constraint);
    if (friction != nullptr)
    {
      coneSpec.setFriction(
        contactIdx, friction->getFrictionCoefficient(), 3 * contactIdx);
      ++contactIdx;
    }
  }

  return coneSpec;
}

ConstraintSolver::ActiveSetResult ConstraintSolver::solveWithECOS(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  const FrictionConeSpec& coneSpec,
  int numContacts) const
{
  const int n = 3 * numContacts;  // Decision variables

  // Build ECOS problem data (G matrix, h, c, cone sizes, A_eq, b_eq)
  ECOSData ecosData = ECOSProblemBuilder::build(A, b, coneSpec);

  // Setup ECOS workspace
  ecosData.setup();

  // Configure ECOS solver settings (Ticket 0035b4)
  pwork* workspace = ecosData.workspace_.get();
  workspace->stgs->maxit = static_cast<idxint>(ecos_max_iters_);
  workspace->stgs->abstol = ecos_abs_tol_;
  workspace->stgs->reltol = ecos_rel_tol_;
  workspace->stgs->feastol = ecos_abs_tol_;

  // Suppress ECOS stdout output
  workspace->stgs->verbose = 0;

  // Solve
  idxint const exitFlag = ECOS_solve(workspace);

  // Extract solution
  Eigen::VectorXd const lambda = Eigen::Map<Eigen::VectorXd>(workspace->x, n);

  // Determine convergence from exit code
  bool const converged = (exitFlag == ECOS_OPTIMAL);

  // Extract diagnostics
  int const iterations = static_cast<int>(workspace->info->iter);
  double const primalRes = workspace->info->pres;
  double const dualRes = workspace->info->dres;
  double const gapVal = workspace->info->gap;

  // Build result with ECOS-specific fields
  ActiveSetResult result;
  result.lambda = lambda;
  result.converged = converged;
  result.iterations = iterations;
  result.active_set_size = 0;  // Not applicable for ECOS
  result.solver_type = "ECOS";
  result.ecos_exit_flag = static_cast<int>(exitFlag);
  result.primal_residual = primalRes;
  result.dual_residual = dualRes;
  result.gap = gapVal;

  // ECOSData destructor handles ECOS_cleanup via unique_ptr

  return result;
}

}  // namespace msd_sim
