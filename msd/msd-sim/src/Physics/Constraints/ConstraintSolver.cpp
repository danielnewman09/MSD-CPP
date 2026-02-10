// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Ticket: 0035b4_ecos_solve_integration
// Ticket: 0045_constraint_solver_unification
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md
// Design: docs/designs/0045_constraint_solver_unification/design.md

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
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// ===== Multi-Body Contact Constraint Solver (Ticket 0032, 0034, 0045) =====
//
// Implements Active Set Method (ASM) for contact constraints with λ ≥ 0.
// Replaced PGS (Ticket 0032b) with ASM (Ticket 0034) for exact convergence.
//
// CRITICAL notes from prototype debugging:
// - Velocity-level RHS: b = -(1+e) · J·v_minus (for system A·λ = b)
// - DO NOT use -(1+e)·v as target velocity directly (causes energy injection)
// - DO NOT add position-correction terms to RHS (causes energy injection, Ticket 0046)
// - Penetration correction handled by PositionCorrector using pseudo-velocities
// See:
// prototypes/0032_contact_constraint_refactor/p2_energy_conservation/Debug_Findings.md

ConstraintSolver::SolveResult ConstraintSolver::solve(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies,
  double dt,
  const std::optional<Eigen::VectorXd>& initialLambda,
  const std::optional<std::vector<InertialState>>& velocityBias)
{
  SolveResult result;
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

  // Ticket 0035b4: Detect friction constraints via lambdaBounds()
  // If any box-constrained constraint is present, route to ECOS SOCP solver
  bool hasFriction = false;
  for (const auto* constraint : contactConstraints)
  {
    if (constraint->lambdaBounds().isBoxConstrained())
    {
      hasFriction = true;
      break;
    }
  }

  // Step 1: Compute per-contact Jacobians
  auto jacobians = assembleJacobians(contactConstraints, states);

  // Step 2+3: Build effective mass matrix A = J·M⁻¹·Jᵀ
  auto a = assembleEffectiveMass(
    contactConstraints, jacobians, inverseMasses, inverseInertias, numBodies);

  // Step 4: Assemble RHS vector
  auto b = assembleRHS(contactConstraints, jacobians, states, dt, velocityBias);

  // Step 5: Solve — dispatch based on friction presence (Ticket 0035b4)
  const int numConstraintRows = static_cast<int>(contactConstraints.size());
  ActiveSetResult asmResult;

  if (hasFriction)
  {
    // Count actual contacts (each contact = 1 normal + 1 friction with dim=2,
    // so constraint count = C normals + C frictions = 2C constraints, but
    // the Jacobian rows are: C*1 (normals) + C*2 (frictions) = 3C rows)
    // However, contactConstraints is a flat list of Constraint&.
    // We need to count the number of unilateral (normal) constraints to get C.
    int numContacts = 0;
    for (const auto* constraint : contactConstraints)
    {
      if (constraint->lambdaBounds().isUnilateral())
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
    // Ticket 0040d: Pass initial lambda for warm-starting
    asmResult = solveActiveSet(a, b, numConstraintRows, initialLambda);
  }

  // Step 6: Extract per-body forces
  result.bodyForces = extractBodyForces(
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

std::vector<Eigen::MatrixXd> ConstraintSolver::assembleJacobians(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states)
{
  const size_t c = contactConstraints.size();
  std::vector<Eigen::MatrixXd> jacobians(c);

  for (size_t i = 0; i < c; ++i)
  {
    const auto* contact = contactConstraints[i];
    size_t const bodyA = contact->bodyAIndex();
    size_t const bodyB = contact->bodyBIndex();
    jacobians[i] =
      contact->jacobian(states[bodyA].get(), states[bodyB].get(), 0.0);
  }

  return jacobians;
}

Eigen::MatrixXd ConstraintSolver::assembleEffectiveMass(
  const std::vector<Constraint*>& contactConstraints,
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
    size_t const bodyAI = contactConstraints[i]->bodyAIndex();
    size_t const bodyBI = contactConstraints[i]->bodyBIndex();

    // J_i is 1×12: [J_i_A(1×6) | J_i_B(1×6)]
    Eigen::Matrix<double, 1, 6> const jIA = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> const jIB = jacobians[i].block<1, 6>(0, 6);

    for (size_t j = i; j < c; ++j)
    {
      size_t const bodyAJ = contactConstraints[j]->bodyAIndex();
      size_t const bodyBJ = contactConstraints[j]->bodyBIndex();

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

Eigen::VectorXd ConstraintSolver::assembleRHS(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<Eigen::MatrixXd>& jacobians,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  double dt,
  const std::optional<std::vector<InertialState>>& velocityBias)
{
  const size_t c = contactConstraints.size();

  // Ticket 0051: Velocity-bias approach to decouple restitution from gravity.
  //
  // RHS formula: b = -(1+e) * J * v_current  -  J * v_bias
  //
  // Where v_bias represents pre-applied gravity (g*dt). The bias term is NOT
  // multiplied by (1+e), preventing restitution-gravity coupling.
  //
  // Ticket 0046: No position-correction terms in velocity-level RHS.
  // Penetration resolved by PositionCorrector (pseudo-velocities, no KE injection).
  //
  // CRITICAL: The -(1+e) factor is correct for the system A·λ = b.
  // This differs from the target velocity formulation v_target = -e · v_pre.
  // See P2 Debug Findings for explanation.
  Eigen::VectorXd b(static_cast<Eigen::Index>(c));
  for (size_t i = 0; i < c; ++i)
  {
    const auto* contact =
      dynamic_cast<const ContactConstraint*>(contactConstraints[i]);
    size_t const bodyA = contactConstraints[i]->bodyAIndex();
    size_t const bodyB = contactConstraints[i]->bodyBIndex();

    // Compute pre-impact relative velocity along constraint: J_i · v_minus
    const InertialState& stateA = states[bodyA].get();
    const InertialState& stateB = states[bodyB].get();

    // Velocity vector v = [v_A, omega_A, v_B, omega_B] (12×1)
    Eigen::VectorXd v(12);
    v.segment<3>(0) =
      Vector3D{stateA.velocity.x(), stateA.velocity.y(), stateA.velocity.z()};
    AngularRate omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Vector3D{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) =
      Vector3D{stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularRate omegaB = stateB.getAngularVelocity();
    v.segment<3>(9) = Vector3D{omegaB.x(), omegaB.y(), omegaB.z()};

    double const jv =
      (jacobians[i] * v)(0);  // Scalar: relative velocity along constraint

    // Ticket 0051: Compute bias term J_i · v_bias (if bias provided)
    double jvBias = 0.0;
    if (velocityBias.has_value())
    {
      const InertialState& biasA = (*velocityBias)[bodyA];
      const InertialState& biasB = (*velocityBias)[bodyB];

      Eigen::VectorXd vBias(12);
      vBias.segment<3>(0) =
        Vector3D{biasA.velocity.x(), biasA.velocity.y(), biasA.velocity.z()};
      AngularRate omegaBiasA = biasA.getAngularVelocity();
      vBias.segment<3>(3) = Vector3D{omegaBiasA.x(), omegaBiasA.y(), omegaBiasA.z()};
      vBias.segment<3>(6) =
        Vector3D{biasB.velocity.x(), biasB.velocity.y(), biasB.velocity.z()};
      AngularRate omegaBiasB = biasB.getAngularVelocity();
      vBias.segment<3>(9) = Vector3D{omegaBiasB.x(), omegaBiasB.y(), omegaBiasB.z()};

      jvBias = (jacobians[i] * vBias)(0);
    }

    // Restitution term (Ticket 0046: no velocity-level position correction)
    if (contact != nullptr)
    {
      double const e = contact->getRestitution();

      // Ticket 0051: RHS = -(1+e)*J*v - J*v_bias
      // The bias term is NOT multiplied by (1+e), preventing restitution coupling
      b(static_cast<Eigen::Index>(i)) = -(1.0 + e) * jv - jvBias;
    }
    else
    {
      // Generic two-body constraint (no restitution)
      b(static_cast<Eigen::Index>(i)) = -jv - jvBias;
    }
  }

  return b;
}

// Ticket: 0034_active_set_method_contact_solver
// Ticket: 0040d_contact_persistence_warm_starting
// Design: docs/designs/0034_active_set_method_contact_solver/design.md
ConstraintSolver::ActiveSetResult ConstraintSolver::solveActiveSet(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  int numContacts,
  const std::optional<Eigen::VectorXd>& initialLambda) const
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

  // Ticket 0040d: Warm-start active set from previous frame's solution.
  // If initial lambda is provided and correctly sized, initialize the active
  // set to only those contacts with positive cached lambda. This skips the
  // iterations needed to remove inactive contacts from the working set.
  const bool warmStart = initialLambda.has_value() &&
                         initialLambda->size() == c &&
                         initialLambda->maxCoeff() > 0.0;

  std::vector<int> activeIndices;
  activeIndices.reserve(static_cast<size_t>(c));

  if (warmStart)
  {
    for (int i = 0; i < c; ++i)
    {
      if ((*initialLambda)(i) > 0.0)
      {
        activeIndices.push_back(i);
      }
    }
    // If warm-start produced an empty active set, fall back to all active
    if (activeIndices.empty())
    {
      for (int i = 0; i < c; ++i)
      {
        activeIndices.push_back(i);
      }
    }
  }
  else
  {
    // Default: all contacts active (optimal for resting contacts)
    for (int i = 0; i < c; ++i)
    {
      activeIndices.push_back(i);
    }
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
ConstraintSolver::extractBodyForces(
  const std::vector<Constraint*>& contactConstraints,
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

    size_t const bodyA = contactConstraints[i]->bodyAIndex();
    size_t const bodyB = contactConstraints[i]->bodyBIndex();

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
  const std::vector<Constraint*>& contactConstraints,
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
