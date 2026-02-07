// Ticket: 0040b_split_impulse_position_correction
// Design: docs/designs/0040b-split-impulse-position-correction/design.md

#include "msd-sim/src/Physics/Constraints/PositionCorrector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

#include <Eigen/Cholesky>

#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

void PositionCorrector::correctPositions(
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    std::vector<InertialState*>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies,
    size_t numInertial,
    double dt)
{
  Config const config{};
  correctPositions(contactConstraints, states, inverseMasses,
                   inverseInertias, numBodies, numInertial, dt, config);
}

void PositionCorrector::correctPositions(
    const std::vector<TwoBodyConstraint*>& contactConstraints,
    std::vector<InertialState*>& states,
    const std::vector<double>& inverseMasses,
    const std::vector<Eigen::Matrix3d>& inverseInertias,
    size_t numBodies,
    size_t numInertial,
    double dt,
    const Config& config)
{
  if (contactConstraints.empty() || dt <= 0.0)
  {
    return;
  }

  const size_t c = contactConstraints.size();

  // ===== Step 1: Build position-level RHS =====
  // b_pos_i = (beta / dt) * max(depth_i - slop, 0)
  // Only process ContactConstraint instances (skip FrictionConstraint etc.)
  Eigen::VectorXd bPos{Eigen::VectorXd::Zero(static_cast<Eigen::Index>(c))};
  bool anyCorrection = false;

  for (size_t i = 0; i < c; ++i)
  {
    const auto* contact =
        dynamic_cast<const ContactConstraint*>(contactConstraints[i]);
    if (contact == nullptr)
    {
      continue;
    }

    double const depth = contact->getPenetrationDepth();
    double const correctedDepth = std::max(depth - config.slop, 0.0);

    if (correctedDepth > 0.0)
    {
      bPos(static_cast<Eigen::Index>(i)) =
          (config.beta / dt) * correctedDepth;
      anyCorrection = true;
    }
  }

  // Early exit if all penetrations within slop
  if (!anyCorrection)
  {
    return;
  }

  // ===== Step 2: Assemble Jacobians =====
  // Same algorithm as ConstraintSolver::assembleContactJacobians()
  std::vector<Eigen::MatrixXd> jacobians(c);
  for (size_t i = 0; i < c; ++i)
  {
    const auto* constraint = contactConstraints[i];
    size_t const bodyA = constraint->getBodyAIndex();
    size_t const bodyB = constraint->getBodyBIndex();
    jacobians[i] = constraint->jacobianTwoBody(
        *states[bodyA], *states[bodyB], 0.0);
  }

  // ===== Step 3: Build effective mass matrix A = J * M^-1 * J^T =====
  // Same algorithm as ConstraintSolver::assembleContactEffectiveMass()
  std::vector<Eigen::Matrix<double, 6, 6>> bodyMInv(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    bodyMInv[k] = Eigen::Matrix<double, 6, 6>::Zero();
    bodyMInv[k].block<3, 3>(0, 0) =
        inverseMasses[k] * Eigen::Matrix3d::Identity();
    bodyMInv[k].block<3, 3>(3, 3) = inverseInertias[k];
  }

  Eigen::MatrixXd a{Eigen::MatrixXd::Zero(
      static_cast<Eigen::Index>(c), static_cast<Eigen::Index>(c))};

  for (size_t i = 0; i < c; ++i)
  {
    size_t const bodyAI = contactConstraints[i]->getBodyAIndex();
    size_t const bodyBI = contactConstraints[i]->getBodyBIndex();

    Eigen::Matrix<double, 1, 6> const jIA = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> const jIB = jacobians[i].block<1, 6>(0, 6);

    for (size_t j = i; j < c; ++j)
    {
      size_t const bodyAJ = contactConstraints[j]->getBodyAIndex();
      size_t const bodyBJ = contactConstraints[j]->getBodyBIndex();

      Eigen::Matrix<double, 1, 6> const jJA =
          jacobians[j].block<1, 6>(0, 0);
      Eigen::Matrix<double, 1, 6> const jJB =
          jacobians[j].block<1, 6>(0, 6);

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

  // Regularization
  for (size_t i = 0; i < c; ++i)
  {
    a(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) +=
        kRegularizationEpsilon;
  }

  // ===== Step 4: Solve with ASM (lambda_pos >= 0) =====
  // Lightweight Active Set Method — same algorithm as ConstraintSolver
  const int numConstraints = static_cast<int>(c);
  Eigen::VectorXd lambdaPos{Eigen::VectorXd::Zero(numConstraints)};

  // Initialize active set: all constraints
  std::vector<int> activeIndices;
  activeIndices.reserve(static_cast<size_t>(numConstraints));
  for (int i = 0; i < numConstraints; ++i)
  {
    activeIndices.push_back(i);
  }

  int const maxIter = std::min(2 * numConstraints, 100);

  for (int iter = 1; iter <= maxIter; ++iter)
  {
    int const activeSize = static_cast<int>(activeIndices.size());

    if (activeSize == 0)
    {
      lambdaPos.setZero();
    }
    else
    {
      // Extract active submatrix and solve
      Eigen::MatrixXd aW{activeSize, activeSize};
      Eigen::VectorXd bW{activeSize};

      for (int i = 0; i < activeSize; ++i)
      {
        bW(i) = bPos(activeIndices[static_cast<size_t>(i)]);
        for (int j = 0; j < activeSize; ++j)
        {
          aW(i, j) = a(activeIndices[static_cast<size_t>(i)],
                       activeIndices[static_cast<size_t>(j)]);
        }
      }

      Eigen::LLT<Eigen::MatrixXd> const llt{aW};
      if (llt.info() != Eigen::Success)
      {
        return;  // Singular — skip correction
      }

      Eigen::VectorXd lambdaW = llt.solve(bW);

      lambdaPos.setZero();
      for (int i = 0; i < activeSize; ++i)
      {
        lambdaPos(activeIndices[static_cast<size_t>(i)]) = lambdaW(i);
      }
    }

    // Check primal feasibility (lambda >= 0)
    double minLambda = std::numeric_limits<double>::infinity();
    int minIndex = -1;
    for (int i = 0; i < activeSize; ++i)
    {
      int const idx = activeIndices[static_cast<size_t>(i)];
      double const val = lambdaPos(idx);
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

    // Check dual feasibility (w >= 0 for inactive set)
    Eigen::VectorXd w = a * lambdaPos - bPos;

    double maxViolation = 0.0;
    int maxViolationIndex = -1;
    constexpr double kTolerance = 1e-6;

    for (int i = 0; i < numConstraints; ++i)
    {
      bool const isActive =
          std::ranges::find(activeIndices, i) != activeIndices.end();
      if (isActive)
      {
        continue;
      }

      if (w(i) < -kTolerance)
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
      break;  // All KKT conditions satisfied
    }

    auto insertPos =
        std::ranges::lower_bound(activeIndices, maxViolationIndex);
    activeIndices.insert(insertPos, maxViolationIndex);
  }

  // ===== Step 5: Compute pseudo-velocity and apply as position change =====
  // dv_pseudo_k = M_k^-1 * sum(J_i_k^T * lambda_pos_i)
  // dx_k = dv_pseudo_k * dt
  for (size_t i = 0; i < c; ++i)
  {
    if (lambdaPos(static_cast<Eigen::Index>(i)) <= 0.0)
    {
      continue;
    }

    size_t const bodyA = contactConstraints[i]->getBodyAIndex();
    size_t const bodyB = contactConstraints[i]->getBodyBIndex();

    Eigen::Matrix<double, 1, 6> const jIA = jacobians[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> const jIB = jacobians[i].block<1, 6>(0, 6);

    double const lambda = lambdaPos(static_cast<Eigen::Index>(i));

    // Body A pseudo-velocity contribution
    if (bodyA < numInertial && inverseMasses[bodyA] > 0.0)
    {
      // Pseudo-velocity = M^-1 * J^T * lambda
      Eigen::Matrix<double, 6, 1> pseudoImpulseA = jIA.transpose() * lambda;
      Eigen::Vector3d dvLinearA =
          inverseMasses[bodyA] * pseudoImpulseA.head<3>();
      Eigen::Vector3d dwAngularA =
          inverseInertias[bodyA] * pseudoImpulseA.tail<3>();

      // Position change = pseudo-velocity * dt
      Eigen::Vector3d dxA = dvLinearA * dt;
      Eigen::Vector3d dthetaA = dwAngularA * dt;

      // Apply linear position correction
      states[bodyA]->position +=
          Coordinate{dxA(0), dxA(1), dxA(2)};

      // Apply angular orientation correction via quaternion update
      double const halfAngle = dthetaA.norm() * 0.5;
      if (halfAngle > 1e-12)
      {
        Eigen::Vector3d axis = dthetaA.normalized();
        QuaternionD dq{Eigen::Quaterniond{Eigen::AngleAxisd{dthetaA.norm(), axis}}};
        states[bodyA]->orientation = dq * states[bodyA]->orientation;
        states[bodyA]->orientation.normalize();
      }
    }

    // Body B pseudo-velocity contribution
    if (bodyB < numInertial && inverseMasses[bodyB] > 0.0)
    {
      Eigen::Matrix<double, 6, 1> pseudoImpulseB = jIB.transpose() * lambda;
      Eigen::Vector3d dvLinearB =
          inverseMasses[bodyB] * pseudoImpulseB.head<3>();
      Eigen::Vector3d dwAngularB =
          inverseInertias[bodyB] * pseudoImpulseB.tail<3>();

      Eigen::Vector3d dxB = dvLinearB * dt;
      Eigen::Vector3d dthetaB = dwAngularB * dt;

      states[bodyB]->position +=
          Coordinate{dxB(0), dxB(1), dxB(2)};

      double const halfAngleB = dthetaB.norm() * 0.5;
      if (halfAngleB > 1e-12)
      {
        Eigen::Vector3d axisB = dthetaB.normalized();
        QuaternionD dqB{Eigen::Quaterniond{Eigen::AngleAxisd{dthetaB.norm(), axisB}}};
        states[bodyB]->orientation = dqB * states[bodyB]->orientation;
        states[bodyB]->orientation.normalize();
      }
    }
  }
  // Pseudo-velocities are local variables — discarded here, never become KE
}

} // namespace msd_sim
