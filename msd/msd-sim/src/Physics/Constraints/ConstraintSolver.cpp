// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Ticket: 0045_constraint_solver_unification
// Ticket: 0052d_solver_integration_ecos_removal
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md
// Design: docs/designs/0032_contact_constraint_refactor/design.md
// Design: docs/designs/0045_constraint_solver_unification/design.md
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include <Eigen/src/Cholesky/LLT.h>
#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace
{

// Ticket 0055c: Projected Gauss-Seidel solver for disk-constrained friction QP.
// Each contact has 2 friction rows (t1, t2) with constraint:
//   ||(λ_t1, λ_t2)|| ≤ bound[contact]
// where bound = μ · λ_n from the decoupled normal solve.
Eigen::VectorXd solveFrictionPGS(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  const std::vector<double>& bounds,
  int numContacts)
{
  const int n = 2 * numContacts;
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(n);

  constexpr int kMaxIterations = 50;
  constexpr double kTolerance = 1e-8;

  for (int iter = 0; iter < kMaxIterations; ++iter)
  {
    double maxChange = 0.0;

    for (int c = 0; c < numContacts; ++c)
    {
      int const r0 = 2 * c;
      int const r1 = 2 * c + 1;

      // Gauss-Seidel update for t1
      double rhs0 = b(r0);
      for (int j = 0; j < n; ++j)
      {
        if (j != r0)
        {
          rhs0 -= A(r0, j) * lambda(j);
        }
      }
      double newL0 = rhs0 / A(r0, r0);

      // Gauss-Seidel update for t2
      double rhs1 = b(r1);
      for (int j = 0; j < n; ++j)
      {
        if (j != r1)
        {
          rhs1 -= A(r1, j) * lambda(j);
        }
      }
      double newL1 = rhs1 / A(r1, r1);

      // Disk clamp: ||(λ_t1, λ_t2)|| ≤ bound
      double const bound = bounds[static_cast<size_t>(c)];
      double const norm = std::sqrt(newL0 * newL0 + newL1 * newL1);
      if (bound <= 0.0)
      {
        newL0 = 0.0;
        newL1 = 0.0;
      }
      else if (norm > bound)
      {
        double const scale = bound / norm;
        newL0 *= scale;
        newL1 *= scale;
      }

      maxChange = std::max(maxChange, std::abs(newL0 - lambda(r0)));
      maxChange = std::max(maxChange, std::abs(newL1 - lambda(r1)));
      lambda(r0) = newL0;
      lambda(r1) = newL1;
    }

    if (maxChange < kTolerance)
    {
      break;
    }
  }

  return lambda;
}

}  // anonymous namespace

namespace msd_sim
{

// ===== Multi-Body Contact Constraint Solver =====
//
// Dispatches between ASM (no friction) and decoupled normal+friction solve.
// Normal constraints solved first with ASM, then friction solved with PGS
// using post-normal velocities and per-contact disk bounds.

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

  // Ticket 0052d: Detect friction constraints via lambdaBounds()
  bool hasFriction = false;
  for (const auto* constraint : contactConstraints)
  {
    if (constraint->lambdaBounds().isBoxConstrained())
    {
      hasFriction = true;
      break;
    }
  }

  if (hasFriction)
  {
    // ===== Capped coupled solve (Ticket 0055c) =====
    // Solve the coupled friction cone QP (FrictionConeSolver), then check
    // if the normal impulse was inflated beyond the friction-free value.
    // If inflated, clamp λ_n to the friction-free value and proportionally
    // scale λ_t to maintain the cone constraint.
    //
    // This prevents A-matrix off-diagonal coupling from inflating λ_n
    // (root cause of energy injection) while preserving the coupled
    // solution's accuracy for well-behaved cases.

    // --- Step 1: Solve coupled system (existing FrictionConeSolver) ---
    auto flat = flattenConstraints(contactConstraints, states);
    auto a = assembleFlatEffectiveMass(
      flat, inverseMasses, inverseInertias, numBodies);
    auto b = assembleFlatRHS(flat, states);
    auto spec = buildFrictionSpec(contactConstraints);
    auto coupledResult = solveWithFriction(a, b, spec, initialLambda);

    // --- Step 2: Partition and solve normal-only for reference ---
    std::vector<Constraint*> normalOnly;
    std::vector<Constraint*> frictionOnly;
    normalOnly.reserve(contactConstraints.size() / 2);
    frictionOnly.reserve(contactConstraints.size() / 2);

    for (auto* c : contactConstraints)
    {
      if (c->lambdaBounds().isBoxConstrained())
      {
        frictionOnly.push_back(c);
      }
      else
      {
        normalOnly.push_back(c);
      }
    }

    const int numContacts = static_cast<int>(normalOnly.size());

    auto normalJacobians = assembleJacobians(normalOnly, states);
    auto normalA = assembleEffectiveMass(
      normalOnly, normalJacobians, inverseMasses, inverseInertias, numBodies);
    auto normalB = assembleRHS(normalOnly, normalJacobians, states, dt);

    // Extract normal-only warm-start from interleaved format
    std::optional<Eigen::VectorXd> normalInitLambda;
    if (initialLambda.has_value() &&
        initialLambda->size() == 3 * numContacts)
    {
      Eigen::VectorXd nLam(numContacts);
      for (int i = 0; i < numContacts; ++i)
      {
        nLam(i) = (*initialLambda)(3 * i);
      }
      normalInitLambda = nLam;
    }

    auto normalResult =
      solveActiveSet(normalA, normalB, numContacts, normalInitLambda);

    // --- Step 3: Clamp inflated normal impulses ---
    Eigen::VectorXd adjustedLambda = coupledResult.lambda;
    bool needsAdjustment = false;

    for (int c = 0; c < numContacts; ++c)
    {
      double const coupledN = adjustedLambda(3 * c);
      double const cleanN = std::max(normalResult.lambda(c), 0.0);

      // Detect inflation: coupled λ_n exceeds friction-free λ_n by multiplicative threshold
      // Ticket 0055d: Use 1.5× threshold to allow moderate inflation for elastic bounces
      // while catching extreme 3.75× cases from tilted cube corner contacts
      constexpr double kInflationThreshold = 1.5;
      if (coupledN > kInflationThreshold * cleanN && coupledN > cleanN + 1e-10)
      {
        needsAdjustment = true;
        adjustedLambda(3 * c) = cleanN;

        // Scale friction to maintain cone: ||λ_t|| ≤ μ · λ_n_new
        double const lt1 = adjustedLambda(3 * c + 1);
        double const lt2 = adjustedLambda(3 * c + 2);
        double const ltNorm = std::sqrt(lt1 * lt1 + lt2 * lt2);

        const auto* friction = dynamic_cast<const FrictionConstraint*>(
          frictionOnly[static_cast<size_t>(c)]);
        double const mu =
          (friction != nullptr) ? friction->getFrictionCoefficient() : 0.0;
        double const maxLt = mu * cleanN;

        if (ltNorm > maxLt && ltNorm > 1e-15)
        {
          double const scale = maxLt / ltNorm;
          adjustedLambda(3 * c + 1) *= scale;
          adjustedLambda(3 * c + 2) *= scale;
        }
      }
    }

    // Use adjusted lambda if inflation detected, otherwise coupled as-is
    const Eigen::VectorXd& finalLambda =
      needsAdjustment ? adjustedLambda : coupledResult.lambda;

    result.bodyForces =
      extractBodyForcesFlat(flat, finalLambda, numBodies, dt);
    result.lambdas = finalLambda;
    result.converged = coupledResult.converged;
    result.iterations = coupledResult.iterations;

    Eigen::VectorXd const residualVec = a * finalLambda - b;
    result.residual = residualVec.norm();
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

    Eigen::Matrix<double, 1, 6> const jIA = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> const jIB = jacobians[i].block<1, 6>(0, 6);

    for (size_t j = i; j < c; ++j)
    {
      size_t const bodyAJ = contactConstraints[j]->bodyAIndex();
      size_t const bodyBJ = contactConstraints[j]->bodyBIndex();

      Eigen::Matrix<double, 1, 6> jJA = jacobians[j].block<1, 6>(0, 0);
      Eigen::Matrix<double, 1, 6> jJB = jacobians[j].block<1, 6>(0, 6);

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

Eigen::VectorXd ConstraintSolver::assembleRHS(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<Eigen::MatrixXd>& jacobians,
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
    Eigen::Matrix<double, 12, 1> v;
    v.segment<3>(0) =
      Vector3D{stateA.velocity.x(), stateA.velocity.y(), stateA.velocity.z()};
    AngularRate omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Vector3D{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) =
      Vector3D{stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularRate omegaB = stateB.getAngularVelocity();
    v.segment<3>(9) = Vector3D{omegaB.x(), omegaB.y(), omegaB.z()};

    double const jv = (jacobians[i] * v)(0);

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

ConstraintSolver::FlattenedConstraints ConstraintSolver::flattenConstraints(
  const std::vector<Constraint*>& contactConstraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states)
{
  FlattenedConstraints flat;
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
    AngularRate omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Vector3D{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) =
      Vector3D{stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularRate omegaB = stateB.getAngularVelocity();
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

ConstraintSolver::ActiveSetResult ConstraintSolver::solveWithFriction(
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  const FrictionSpec& spec,
  const std::optional<Eigen::VectorXd>& initialLambda)
{
  // Determine warm-start vector
  Eigen::VectorXd lambda0;
  if (initialLambda.has_value() && initialLambda->size() == b.size())
  {
    lambda0 = *initialLambda;
  }

  // Delegate to FrictionConeSolver
  auto solveResult = frictionConeSolver_.solve(
    A, b, spec.frictionCoefficients, lambda0);

  ActiveSetResult result;
  result.lambda = std::move(solveResult.lambda);
  result.converged = solveResult.converged;
  result.iterations = solveResult.iterations;
  result.active_set_size = 0;

  return result;
}

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
