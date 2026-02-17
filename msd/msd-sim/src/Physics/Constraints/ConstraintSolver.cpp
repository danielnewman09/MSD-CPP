// Ticket: 0031_generalized_lagrange_constraints
// Ticket: 0032_contact_constraint_refactor
// Ticket: 0045_constraint_solver_unification
// Ticket: 0052d_solver_integration_ecos_removal
// Ticket: 0068c_constraint_solver_integration
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
    // ===== Friction path: flatten + NLoptFrictionSolver =====
    // Ticket: 0068c

    // Step 1: Flatten constraints into per-row entries
    auto flat = flattenConstraints(contactConstraints, states);

    // Step 2: Assemble 3C x 3C effective mass matrix
    auto a = assembleFlatEffectiveMass(
      flat, inverseMasses, inverseInertias, numBodies);

    // Step 3: Assemble 3C x 1 RHS vector
    auto b = assembleFlatRHS(flat, states);

    // Step 4: Build friction specification
    auto spec = buildFrictionSpec(contactConstraints);

    // Step 5: Solve with NLoptFrictionSolver
    auto asmResult = solveWithFriction(a, b, spec, initialLambda);

    // Step 5b: Energy-conserving impulse clamp (Ticket: 0068)
    // Compute ΔKE from impulse. If positive (energy injection), scale λ to
    // make ΔKE = 0. This prevents Newton's restitution from injecting energy
    // on rotating/tumbling contacts.
    clampImpulseEnergy(asmResult.lambda, a, b, flat.restitutions);

    // Step 6: Extract per-body forces (no negative lambda skip)
    result.bodyForces =
      extractBodyForcesFlat(flat, asmResult.lambda, numBodies, dt);

    result.lambdas = asmResult.lambda;
    result.converged = asmResult.converged;
    result.iterations = asmResult.iterations;

    // Compute residual: ||A*lambda - b||
    Eigen::VectorXd const residualVec = a * asmResult.lambda - b;
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
    AngularVelocity omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Vector3D{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) =
      Vector3D{stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularVelocity omegaB = stateB.getAngularVelocity();
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

  // Delegate to NLoptFrictionSolver (Ticket: 0068c)
  auto nloptResult = nloptSolver_.solve(
    A, b, spec.frictionCoefficients, lambda0);

  // Post-solve: clamp friction that reverses velocity or does positive work
  int const numContacts =
    static_cast<int>(spec.frictionCoefficients.size());
  clampPositiveWorkFriction(nloptResult.lambda, A, b, numContacts);

  // Map NLoptFrictionSolver::SolveResult to ActiveSetResult
  ActiveSetResult result;
  result.lambda = std::move(nloptResult.lambda);
  result.converged = nloptResult.converged;
  result.iterations = nloptResult.iterations;
  result.active_set_size = 0;  // Not applicable for NLopt

  return result;
}

void ConstraintSolver::clampPositiveWorkFriction(
  Eigen::VectorXd& lambda,
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  int numContacts)
{
  // Ticket: 0068_nlopt_friction_cone_solver
  //
  // Each contact contributes 3 rows: [normal, tangent1, tangent2].
  //
  // Check each active contact for friction doing positive work (accelerating
  // instead of decelerating). If detected, zero the tangent lambdas.
  //
  // Also check for velocity reversal: if friction would push a tangent velocity
  // past zero by a significant amount, cap it to arrest the motion instead.

  Eigen::VectorXd const dv = A * lambda;

  for (int i = 0; i < numContacts; ++i)
  {
    int const base = 3 * i;

    // Skip inactive contacts
    if (lambda[base] <= 0.0)
    {
      continue;
    }

    // Pre-solve tangent velocities: jv = -b (tangent rows store -jv)
    double const preT1 = -b[base + 1];
    double const preT2 = -b[base + 2];

    // Check 1: Positive work (friction accelerating)
    double const work =
      lambda[base + 1] * preT1 + lambda[base + 2] * preT2;

    if (work > 0.0)
    {
      lambda[base + 1] = 0.0;
      lambda[base + 2] = 0.0;
      continue;
    }

    // Check 2: Velocity reversal — cap tangent impulse per-row
    // Only trigger when post-solve velocity has reversed AND the reversal
    // is significant (> 10% of pre-solve magnitude). Small overshoot at
    // arrest is acceptable; large reversal is not.
    for (int t = 1; t <= 2; ++t)
    {
      int const row = base + t;
      double const pre = -b[row];
      double const post = pre + dv[row];

      // Reversal: signs differ and reversal is > 10% of original speed
      if (std::abs(pre) > 1e-6 && pre * post < 0.0 &&
          std::abs(post) > 0.1 * std::abs(pre))
      {
        // Scale this tangent lambda to exactly arrest the velocity
        // post = pre + dv[row] = pre + scale * dv[row] = 0
        // scale = -pre / dv[row] = pre / (pre - post)
        double const dvRow = dv[row];
        if (std::abs(dvRow) > 1e-12)
        {
          double const scale = -pre / dvRow;
          lambda[row] *= scale;
        }
      }
    }
  }
}

void ConstraintSolver::clampImpulseEnergy(
  Eigen::VectorXd& lambda,
  const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b,
  const std::vector<double>& restitutions)
{
  // Ticket: 0068_nlopt_friction_cone_solver
  //
  // Reconstruct pre-solve velocity in constraint space from b:
  //   Normal rows: b = -(1+e)*Jv  →  Jv = -b/(1+e)
  //   Tangent rows: b = -Jv       →  Jv = -b
  //
  // Then ΔKE = λ·Jv + 0.5·λ·A·λ
  // If ΔKE > 0, solve for α: α·λ·Jv + 0.5·α²·λ·A·λ = 0
  //   → α = -2·(λ·Jv) / (λ·A·λ)

  Eigen::Index const n = b.size();
  Eigen::VectorXd jv(n);

  for (Eigen::Index i = 0; i < n; ++i)
  {
    double const e = restitutions[static_cast<size_t>(i)];
    if (e > 0.0)
    {
      // Normal row with restitution
      jv[i] = -b[i] / (1.0 + e);
    }
    else
    {
      // Tangent row (e=0 stored for tangent rows)
      jv[i] = -b[i];
    }
  }

  double const c1 = lambda.dot(jv);       // λ·Jv
  double const c2 = lambda.dot(A * lambda); // λ·A·λ (always ≥ 0)

  // ΔKE = c1 + 0.5*c2
  double const deltaKE = c1 + 0.5 * c2;

  if (deltaKE > 0.0 && c2 > 1e-12)
  {
    double const alpha = -2.0 * c1 / c2;

    // Only clamp if α is a valid scaling (0 < α < 1)
    if (alpha > 0.0 && alpha < 1.0)
    {
      lambda *= alpha;
    }
  }
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
