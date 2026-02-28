// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Ticket: 0045_constraint_solver_unification
// Ticket: 0052d_solver_integration_ecos_removal
// Ticket: 0068c_constraint_solver_integration
// Ticket: 0070_nlopt_convergence_energy_injection
// Ticket: 0073_hybrid_pgs_large_islands
// Ticket: 0071c_eigen_fixed_size_matrices
// Ticket: 0071e_trivial_allocation_elimination
// Ticket: 0071f_solver_workspace_reuse
// Ticket: 0075b_block_pgs_solver
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0045_constraint_solver_unification/design.md
// Design: docs/designs/0052_custom_friction_cone_solver/design.md
// Design: docs/designs/0068_nlopt_friction_cone_solver/design.md

#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include <Eigen/src/Cholesky/LLT.h>
#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// ===== Multi-Body Contact Constraint Solver =====
//
// Dispatches between ASM (no friction) and NLoptFrictionSolver (with friction).
// For friction path, constraints are flattened to per-row entries before assembly.
// Ticket: 0068c

ConstraintSolver::SolveResult ConstraintSolver::solve(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies,
  double dt,
  const std::optional<Eigen::VectorXd>& initialLambda)
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

  // ===== Ticket 0073: Hybrid PGS dispatch for large islands =====
  // Count total Jacobian rows = sum of constraint->dimension().
  // If numRows > kASMThreshold, route to PGS (O(n) per sweep).
  // Otherwise, continue with existing exact ASM/decoupled-friction path.
  {
    size_t numRows = 0;
    for (const auto* c : contactConstraints)
    {
      numRows += static_cast<size_t>(c->dimension());
    }

    if (numRows > kASMThreshold)
    {
      // PGS path — handles both friction and non-friction cases
      auto pgsResult = pgsSolver_.solve(
        contactConstraints, states, inverseMasses, inverseInertias, numBodies,
        dt, initialLambda);

      // Convert ProjectedGaussSeidel::SolveResult to ConstraintSolver::SolveResult
      result.lambdas = std::move(pgsResult.lambdas);
      result.converged = pgsResult.converged;
      result.iterations = pgsResult.iterations;
      result.residual = pgsResult.residual;
      result.bodyForces.resize(numBodies,
        BodyForces{Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0}});
      for (size_t k = 0; k < numBodies; ++k)
      {
        result.bodyForces[k].linearForce = pgsResult.bodyForces[k].linearForce;
        result.bodyForces[k].angularTorque =
          pgsResult.bodyForces[k].angularTorque;
      }
      return result;
    }
  }

  // Ticket 0075b: Detect friction contacts via ContactConstraint::hasFriction().
  // As of Phase 1 (0075a), all constraints in the friction path are unified
  // ContactConstraints with hasFriction() == true. lambdaBounds() is always
  // unilateral() for ContactConstraint (block PGS handles cone projection internally).
  bool hasFriction = false;
  for (const auto* constraint : contactConstraints)
  {
    const auto* cc = dynamic_cast<const ContactConstraint*>(constraint);
    if (cc != nullptr && cc->hasFriction())
    {
      hasFriction = true;
      break;
    }
  }

  if (hasFriction)
  {
    // ===== Block PGS Solver — Ticket 0075b_block_pgs_solver =====
    //
    // Two-phase solve eliminates energy injection (DD-0070-H2 mechanism):
    //   Phase A: Restitution bounce (normal-only, sequential, provably energy-correct)
    //   Phase B: Dissipative 3x3 block PGS with Coulomb cone projection
    //            (RHS = -v_err, provably dissipative)
    //
    // Replaces the decoupled normal-then-friction approach (ticket 0070).
    // The 3x3 block captures K_nt coupling that the decoupled solver ignores.

    auto blockResult = blockPgsSolver_.solve(
      contactConstraints, states, inverseMasses, inverseInertias, numBodies,
      dt, initialLambda);

    // Convert BlockPGSSolver::SolveResult to ConstraintSolver::SolveResult
    result.lambdas = std::move(blockResult.lambdas);
    result.converged = blockResult.converged;
    result.iterations = blockResult.iterations;
    result.residual = blockResult.residual;
    for (size_t k = 0; k < numBodies && k < blockResult.bodyForces.size(); ++k)
    {
      result.bodyForces[k].linearForce =
        Coordinate{blockResult.bodyForces[k].linearForce.x(),
                   blockResult.bodyForces[k].linearForce.y(),
                   blockResult.bodyForces[k].linearForce.z()};
      result.bodyForces[k].angularTorque =
        Coordinate{blockResult.bodyForces[k].angularTorque.x(),
                   blockResult.bodyForces[k].angularTorque.y(),
                   blockResult.bodyForces[k].angularTorque.z()};
    }
  }
  else
  {
    // ===== No-friction path: unchanged ASM =====

    // Step 1: Compute per-contact Jacobians
    auto jacobians = assembleJacobians(contactConstraints, states);

    // Step 2+3: Build effective mass matrix A = J*M^-1*J^T
    auto a = assembleEffectiveMass(
      contactConstraints, jacobians, inverseMasses, inverseInertias, numBodies);

    // Step 4: Assemble RHS vector
    auto b = assembleRHS(contactConstraints, jacobians, states, dt);

    // Step 5: Solve using Active Set Method
    const int numConstraintRows =
      static_cast<int>(contactConstraints.size());
    auto asmResult = solveActiveSet(a, b, numConstraintRows, initialLambda);

    // Step 6: Extract per-body forces
    result.bodyForces = extractBodyForces(
      contactConstraints, jacobians, asmResult.lambda, numBodies, dt);

    result.lambdas = asmResult.lambda;
    result.converged = asmResult.converged;
    result.iterations = asmResult.iterations;

    // Compute residual: ||A*lambda - b||
    Eigen::VectorXd const residualVec = a * asmResult.lambda - b;
    result.residual = residualVec.norm();
  }

  return result;
}

// ===== Contact solver helper implementations =====

// Ticket 0071c: assembleJacobians returns fixed-size JacobianRow (Matrix<double,1,12>).
// All contact constraints in the no-friction path are ContactConstraint (dim=1, 12 columns).
// The virtual jacobian() still returns MatrixXd — we copy row 0 into the fixed-size type
// at this boundary, eliminating per-constraint heap allocations downstream.
std::vector<ConstraintSolver::JacobianRow> ConstraintSolver::assembleJacobians(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states)
{
  const size_t c = contactConstraints.size();
  std::vector<JacobianRow> jacobians;
  jacobians.reserve(c);

  for (size_t i = 0; i < c; ++i)
  {
    const auto* contact = contactConstraints[i];
    size_t const bodyA = contact->bodyAIndex();
    size_t const bodyB = contact->bodyBIndex();
    // Virtual call returns MatrixXd; row 0 is always 1x12 for ContactConstraint
    const Eigen::MatrixXd j =
      contact->jacobian(states[bodyA].get(), states[bodyB].get(), 0.0);
    jacobians.push_back(j.row(0));
  }

  return jacobians;
}

// Ticket 0071c: jacobians parameter is now vector<JacobianRow> (fixed-size 1x12).
// Block extractions are compile-time sized: leftCols<6>() / rightCols<6>()
// instead of runtime block<1,6>(0,0) / block<1,6>(0,6).
Eigen::MatrixXd ConstraintSolver::assembleEffectiveMass(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<JacobianRow>& jacobians,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies)
{
  const size_t c = contactConstraints.size();

  // Build per-body inverse mass matrices (6x6)
  std::vector<Eigen::Matrix<double, 6, 6>> bodyMInv(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    bodyMInv[k] = Eigen::Matrix<double, 6, 6>::Zero();
    bodyMInv[k].block<3, 3>(0, 0) =
      inverseMasses[k] * Eigen::Matrix3d::Identity();
    bodyMInv[k].block<3, 3>(3, 3) = inverseInertias[k];
  }

  // Assemble effective mass matrix A (CxC)
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(c),
                                            static_cast<Eigen::Index>(c));
  for (size_t i = 0; i < c; ++i)
  {
    size_t const bodyAI = contactConstraints[i]->bodyAIndex();
    size_t const bodyBI = contactConstraints[i]->bodyBIndex();

    // Ticket 0071c: leftCols<6>() / rightCols<6>() are compile-time sized
    Eigen::Matrix<double, 1, 6> const jIA = jacobians[i].leftCols<6>();
    Eigen::Matrix<double, 1, 6> const jIB = jacobians[i].rightCols<6>();

    for (size_t j = i; j < c; ++j)
    {
      size_t const bodyAJ = contactConstraints[j]->bodyAIndex();
      size_t const bodyBJ = contactConstraints[j]->bodyBIndex();

      Eigen::Matrix<double, 1, 6> const jJA = jacobians[j].leftCols<6>();
      Eigen::Matrix<double, 1, 6> const jJB = jacobians[j].rightCols<6>();

      double aIj = 0.0;

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
      a(static_cast<Eigen::Index>(j), static_cast<Eigen::Index>(i)) = aIj;
    }
  }

  // Add regularization to diagonal
  for (size_t i = 0; i < c; ++i)
  {
    a(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) +=
      kRegularizationEpsilon;
  }

  return a;
}

// Ticket 0071c: jacobians parameter is now vector<JacobianRow> (fixed-size 1x12).
// The (1x12)*(12x1) dot product is fully compile-time-sized — Eigen resolves it
// to a scalar without heap allocation.
Eigen::VectorXd ConstraintSolver::assembleRHS(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<JacobianRow>& jacobians,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  double /* dt */)
{
  const size_t c = contactConstraints.size();

  Eigen::VectorXd b(static_cast<Eigen::Index>(c));
  for (size_t i = 0; i < c; ++i)
  {
    const auto* contact =
      dynamic_cast<const ContactConstraint*>(contactConstraints[i]);
    size_t const bodyA = contactConstraints[i]->bodyAIndex();
    size_t const bodyB = contactConstraints[i]->bodyBIndex();

    const InertialState& stateA = states[bodyA].get();
    const InertialState& stateB = states[bodyB].get();

    // Ticket 0053f: Stack-allocated fixed-size velocity vector (zero heap cost)
    // Ticket 0071c: (JacobianRow 1x12) * (v 12x1) is a compile-time scalar product
    Eigen::Matrix<double, 12, 1> v;
    v.segment<3>(0) =
      Vector3D{stateA.velocity.x(), stateA.velocity.y(), stateA.velocity.z()};
    AngularVelocity omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Vector3D{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) =
      Vector3D{stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularVelocity omegaB = stateB.getAngularVelocity();
    v.segment<3>(9) = Vector3D{omegaB.x(), omegaB.y(), omegaB.z()};

    double const jv = jacobians[i].dot(v);

    if (contact != nullptr)
    {
      double const e = contact->getRestitution();
      b(static_cast<Eigen::Index>(i)) = -(1.0 + e) * jv;
    }
    else
    {
      b(static_cast<Eigen::Index>(i)) = -jv;
    }
  }

  return b;
}

// Ticket: 0034_active_set_method_contact_solver
// Ticket: 0040d_contact_persistence_warm_starting
ConstraintSolver::ActiveSetResult ConstraintSolver::solveActiveSet(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  int numContacts,
  const std::optional<Eigen::VectorXd>& initialLambda)
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
    for (int i = 0; i < c; ++i)
    {
      activeIndices.push_back(i);
    }
  }

  const int effectiveMaxIter = std::min(2 * c, max_safety_iterations_);

  for (int iter = 1; iter <= effectiveMaxIter; ++iter)
  {
    const int activeSize = static_cast<int>(activeIndices.size());

    if (activeSize == 0)
    {
      lambda.setZero();
    }
    else
    {
      // Ticket 0053f: Reuse workspace instead of per-iteration allocation
      asmAw_.resize(activeSize, activeSize);
      asmBw_.resize(activeSize);

      for (int i = 0; i < activeSize; ++i)
      {
        asmBw_(i) = b(activeIndices[static_cast<size_t>(i)]);
        for (int j = 0; j < activeSize; ++j)
        {
          asmAw_(i, j) = A(activeIndices[static_cast<size_t>(i)],
                           activeIndices[static_cast<size_t>(j)]);
        }
      }

      Eigen::LLT<Eigen::MatrixXd> const llt{asmAw_};
      if (llt.info() != Eigen::Success)
      {
        return ActiveSetResult{.lambda = lambda,
                               .converged = false,
                               .iterations = iter,
                               .active_set_size = activeSize};
      }

      asmBw_ = llt.solve(asmBw_);

      lambda.setZero();
      for (int i = 0; i < activeSize; ++i)
      {
        lambda(activeIndices[static_cast<size_t>(i)]) = asmBw_(i);
      }
    }

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
      std::erase(activeIndices, minIndex);
      continue;
    }

    // Ticket 0053f: Reuse workspace for dual residual
    asmW_.resize(c);
    asmW_.noalias() = A * lambda - b;

    double maxViolation = 0.0;
    int maxViolationIndex = -1;
    for (int i = 0; i < c; ++i)
    {
      bool const isActive =
        std::ranges::find(activeIndices, i) != activeIndices.end();
      if (isActive)
      {
        continue;
      }

      if (asmW_(i) < -convergence_tolerance_)
      {
        if (maxViolationIndex == -1 || asmW_(i) < maxViolation ||
            (asmW_(i) == maxViolation && i < maxViolationIndex))
        {
          maxViolation = asmW_(i);
          maxViolationIndex = i;
        }
      }
    }

    if (maxViolationIndex == -1)
    {
      return ActiveSetResult{.lambda = lambda,
                             .converged = true,
                             .iterations = iter,
                             .active_set_size = activeSize};
    }

    auto insertPos = std::ranges::lower_bound(activeIndices, maxViolationIndex);
    activeIndices.insert(insertPos, maxViolationIndex);
  }

  return ActiveSetResult{
    .lambda = lambda,
    .converged = false,
    .iterations = effectiveMaxIter,
    .active_set_size = static_cast<int>(activeIndices.size())};
}

// Ticket 0071c: jacobians parameter is now vector<JacobianRow> (fixed-size 1x12).
// leftCols<6>() / rightCols<6>() are compile-time sized block extractions.
std::vector<ConstraintSolver::BodyForces>
ConstraintSolver::extractBodyForces(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<JacobianRow>& jacobians,
  const Eigen::VectorXd& lambda,
  size_t numBodies,
  double dt)
{
  const size_t c = contactConstraints.size();
  std::vector<BodyForces> bodyForces(
    numBodies,
    BodyForces{Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0}});

  for (size_t i = 0; i < c; ++i)
  {
    if (lambda(static_cast<Eigen::Index>(i)) <= 0.0)
    {
      continue;
    }

    size_t const bodyA = contactConstraints[i]->bodyAIndex();
    size_t const bodyB = contactConstraints[i]->bodyBIndex();

    // Ticket 0071c: compile-time sized sub-block extractions
    Eigen::Matrix<double, 1, 6> const jIA = jacobians[i].leftCols<6>();
    Eigen::Matrix<double, 1, 6> const jIB = jacobians[i].rightCols<6>();

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

// ===== Friction Solver Integration (Ticket 0052d) =====

// ===== In-place workspace population (Ticket 0071f_solver_workspace_reuse) =====

void ConstraintSolver::populateFlatConstraints_(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states)
{
  // Clear vector contents but retain allocated capacity, so push_back() below
  // will reuse the existing heap buffers rather than reallocating.
  flat_.jacobianRows.clear();
  flat_.bodyAIndices.clear();
  flat_.bodyBIndices.clear();
  flat_.rowTypes.clear();
  flat_.restitutions.clear();
  flat_.numContacts = 0;

  // Reserve based on current constraint set (same two-pass strategy as
  // flattenConstraints() + Ticket 0071e: compute total rows first).
  size_t totalRows = 0;
  for (const auto* constraint : contactConstraints)
  {
    totalRows += static_cast<size_t>(constraint->dimension());
  }
  flat_.jacobianRows.reserve(totalRows);
  flat_.bodyAIndices.reserve(totalRows);
  flat_.bodyBIndices.reserve(totalRows);
  flat_.rowTypes.reserve(totalRows);
  flat_.restitutions.reserve(totalRows);

  int numContacts = 0;
  for (const auto* constraint : contactConstraints)
  {
    size_t const bodyA = constraint->bodyAIndex();
    size_t const bodyB = constraint->bodyBIndex();
    auto j = constraint->jacobian(states[bodyA].get(), states[bodyB].get(), 0.0);
    int const dim = constraint->dimension();

    const auto* contact = dynamic_cast<const ContactConstraint*>(constraint);

    for (int row = 0; row < dim; ++row)
    {
      flat_.jacobianRows.push_back(j.row(row));
      flat_.bodyAIndices.push_back(bodyA);
      flat_.bodyBIndices.push_back(bodyB);

      if (contact != nullptr)
      {
        flat_.rowTypes.push_back(RowType::Normal);
        flat_.restitutions.push_back(contact->getRestitution());
        ++numContacts;
      }
      else
      {
        flat_.rowTypes.push_back(RowType::Tangent);
        flat_.restitutions.push_back(0.0);
      }
    }
  }

  flat_.numContacts = numContacts;
}

void ConstraintSolver::assembleFlatEffectiveMassInPlace_(
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies)
{
  const auto numRows = static_cast<Eigen::Index>(flat_.jacobianRows.size());

  // Build per-body inverse mass matrices (6x6)
  std::vector<Eigen::Matrix<double, 6, 6>> bodyMInv(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    bodyMInv[k] = Eigen::Matrix<double, 6, 6>::Zero();
    bodyMInv[k].block<3, 3>(0, 0) =
      inverseMasses[k] * Eigen::Matrix3d::Identity();
    bodyMInv[k].block<3, 3>(3, 3) = inverseInertias[k];
  }

  // Resize-only: Eigen::MatrixXd::resize() does NOT reallocate when dimensions
  // are the same as current (Eigen caches the allocated size). This avoids the
  // 720KB allocation for 300-row problems identified in the ticket.
  flatEffectiveMass_.resize(numRows, numRows);
  flatEffectiveMass_.setZero();

  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    size_t const bodyAI = flat_.bodyAIndices[static_cast<size_t>(i)];
    size_t const bodyBI = flat_.bodyBIndices[static_cast<size_t>(i)];

    Eigen::Matrix<double, 1, 6> const jIA =
      flat_.jacobianRows[static_cast<size_t>(i)].leftCols<6>();
    Eigen::Matrix<double, 1, 6> const jIB =
      flat_.jacobianRows[static_cast<size_t>(i)].rightCols<6>();

    for (Eigen::Index j = i; j < numRows; ++j)
    {
      size_t const bodyAJ = flat_.bodyAIndices[static_cast<size_t>(j)];
      size_t const bodyBJ = flat_.bodyBIndices[static_cast<size_t>(j)];

      Eigen::Matrix<double, 1, 6> const jJA =
        flat_.jacobianRows[static_cast<size_t>(j)].leftCols<6>();
      Eigen::Matrix<double, 1, 6> const jJB =
        flat_.jacobianRows[static_cast<size_t>(j)].rightCols<6>();

      double aIj = 0.0;

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

      flatEffectiveMass_(i, j) = aIj;
      flatEffectiveMass_(j, i) = aIj;
    }
  }

  // Add regularization to diagonal
  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    flatEffectiveMass_(i, i) += kRegularizationEpsilon;
  }
}

ConstraintSolver::FlattenedConstraints ConstraintSolver::flattenConstraints(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states)
{
  FlattenedConstraints flat;
  int numContacts = 0;

  // Ticket: 0071e_trivial_allocation_elimination
  // Compute total row count before the loop so all 5 FlattenedConstraints
  // vectors can be reserved up-front, eliminating reallocation growth.
  size_t totalRows = 0;
  for (const auto* constraint : contactConstraints)
  {
    totalRows += static_cast<size_t>(constraint->dimension());
  }
  flat.jacobianRows.reserve(totalRows);
  flat.bodyAIndices.reserve(totalRows);
  flat.bodyBIndices.reserve(totalRows);
  flat.rowTypes.reserve(totalRows);
  flat.restitutions.reserve(totalRows);

  for (const auto* constraint : contactConstraints)
  {
    size_t const bodyA = constraint->bodyAIndex();
    size_t const bodyB = constraint->bodyBIndex();
    auto j = constraint->jacobian(states[bodyA].get(), states[bodyB].get(), 0.0);
    int const dim = constraint->dimension();

    const auto* contact = dynamic_cast<const ContactConstraint*>(constraint);

    for (int row = 0; row < dim; ++row)
    {
      flat.jacobianRows.push_back(j.row(row));
      flat.bodyAIndices.push_back(bodyA);
      flat.bodyBIndices.push_back(bodyB);

      if (contact != nullptr)
      {
        flat.rowTypes.push_back(RowType::Normal);
        flat.restitutions.push_back(contact->getRestitution());
        ++numContacts;
      }
      else
      {
        flat.rowTypes.push_back(RowType::Tangent);
        flat.restitutions.push_back(0.0);
      }
    }
  }

  flat.numContacts = numContacts;
  return flat;
}

FrictionSpec ConstraintSolver::buildFrictionSpec(
  const std::vector<Constraint*>& contactConstraints)
{
  FrictionSpec spec;
  int numContacts = 0;

  for (const auto* constraint : contactConstraints)
  {
    const auto* friction =
      dynamic_cast<const FrictionConstraint*>(constraint);
    if (friction != nullptr)
    {
      spec.frictionCoefficients.push_back(friction->getFrictionCoefficient());
      ++numContacts;
    }
  }

  spec.numContacts = numContacts;
  return spec;
}

Eigen::MatrixXd ConstraintSolver::assembleFlatEffectiveMass(
  const FlattenedConstraints& flat,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies)
{
  const auto numRows = static_cast<Eigen::Index>(flat.jacobianRows.size());

  // Build per-body inverse mass matrices (6x6)
  std::vector<Eigen::Matrix<double, 6, 6>> bodyMInv(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    bodyMInv[k] = Eigen::Matrix<double, 6, 6>::Zero();
    bodyMInv[k].block<3, 3>(0, 0) =
      inverseMasses[k] * Eigen::Matrix3d::Identity();
    bodyMInv[k].block<3, 3>(3, 3) = inverseInertias[k];
  }

  // Assemble effective mass matrix A (numRows x numRows)
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(numRows, numRows);
  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    size_t const bodyAI = flat.bodyAIndices[static_cast<size_t>(i)];
    size_t const bodyBI = flat.bodyBIndices[static_cast<size_t>(i)];

    Eigen::Matrix<double, 1, 6> const jIA =
      flat.jacobianRows[static_cast<size_t>(i)].leftCols<6>();
    Eigen::Matrix<double, 1, 6> const jIB =
      flat.jacobianRows[static_cast<size_t>(i)].rightCols<6>();

    for (Eigen::Index j = i; j < numRows; ++j)
    {
      size_t const bodyAJ = flat.bodyAIndices[static_cast<size_t>(j)];
      size_t const bodyBJ = flat.bodyBIndices[static_cast<size_t>(j)];

      Eigen::Matrix<double, 1, 6> const jJA =
        flat.jacobianRows[static_cast<size_t>(j)].leftCols<6>();
      Eigen::Matrix<double, 1, 6> const jJB =
        flat.jacobianRows[static_cast<size_t>(j)].rightCols<6>();

      double aIj = 0.0;

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

      a(i, j) = aIj;
      a(j, i) = aIj;
    }
  }

  // Add regularization to diagonal
  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    a(i, i) += kRegularizationEpsilon;
  }

  return a;
}

Eigen::VectorXd ConstraintSolver::assembleFlatRHS(
  const FlattenedConstraints& flat,
  const std::vector<std::reference_wrapper<const InertialState>>& states)
{
  const auto numRows = static_cast<Eigen::Index>(flat.jacobianRows.size());
  Eigen::VectorXd b(numRows);

  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    const auto idx = static_cast<size_t>(i);
    size_t const bodyA = flat.bodyAIndices[idx];
    size_t const bodyB = flat.bodyBIndices[idx];

    const InertialState& stateA = states[bodyA].get();
    const InertialState& stateB = states[bodyB].get();

    // Ticket 0053f: Stack-allocated fixed-size velocity vector (zero heap cost)
    Eigen::Matrix<double, 12, 1> v;
    v.segment<3>(0) =
      Vector3D{stateA.velocity.x(), stateA.velocity.y(), stateA.velocity.z()};
    AngularVelocity omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Vector3D{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) =
      Vector3D{stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularVelocity omegaB = stateB.getAngularVelocity();
    v.segment<3>(9) = Vector3D{omegaB.x(), omegaB.y(), omegaB.z()};

    double const jv = (flat.jacobianRows[idx] * v)(0);

    if (flat.rowTypes[idx] == RowType::Normal)
    {
      // Normal row: b = -(1+e) * jv
      double const e = flat.restitutions[idx];
      b(i) = -(1.0 + e) * jv;
    }
    else
    {
      // Tangent row: b = -jv (no restitution for friction)
      b(i) = -jv;
    }
  }

  return b;
}

// Removed functions (Ticket: 0070_nlopt_convergence_energy_injection):
// - solveWithFriction(): Replaced by decoupled normal-then-friction approach
// - clampPositiveWorkFriction(): No longer needed with corrected formulation
// - clampImpulseEnergy(): No longer needed with corrected formulation

std::vector<ConstraintSolver::BodyForces>
ConstraintSolver::extractBodyForcesFlat(
  const FlattenedConstraints& flat,
  const Eigen::VectorXd& lambda,
  size_t numBodies,
  double dt)
{
  const auto numRows = static_cast<size_t>(flat.jacobianRows.size());
  std::vector<BodyForces> bodyForces(
    numBodies,
    BodyForces{Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0}});

  for (size_t i = 0; i < numRows; ++i)
  {
    // No negative lambda skip — friction forces can be negative
    size_t const bodyA = flat.bodyAIndices[i];
    size_t const bodyB = flat.bodyBIndices[i];

    Eigen::Matrix<double, 1, 6> jIA = flat.jacobianRows[i].leftCols<6>();
    Eigen::Matrix<double, 1, 6> jIB = flat.jacobianRows[i].rightCols<6>();

    double const lam = lambda(static_cast<Eigen::Index>(i));

    Eigen::Matrix<double, 6, 1> forceA = jIA.transpose() * lam / dt;
    Eigen::Matrix<double, 6, 1> forceB = jIB.transpose() * lam / dt;

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

}  // namespace msd_sim
