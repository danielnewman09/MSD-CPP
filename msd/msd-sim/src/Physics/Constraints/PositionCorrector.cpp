// Ticket: 0040b_split_impulse_position_correction
// Ticket: 0053f_wire_solver_workspace
// Ticket: 0071f_solver_workspace_reuse
// Design: docs/designs/0040b-split-impulse-position-correction/design.md

#include "msd-sim/src/Physics/Constraints/PositionCorrector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

#include <Eigen/Cholesky>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

void PositionCorrector::correctPositions(
  const std::vector<Constraint*>& contactConstraints,
  std::vector<InertialState*>& states,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies,
  size_t numInertial,
  double dt)
{
  Config const config{};
  correctPositions(contactConstraints,
                   states,
                   inverseMasses,
                   inverseInertias,
                   numBodies,
                   numInertial,
                   dt,
                   config);
}

void PositionCorrector::correctPositions(
  const std::vector<Constraint*>& contactConstraints,
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

  const auto c = contactConstraints.size();
  const auto cIdx = static_cast<Eigen::Index>(c);

  // ===== Step 1: Build position-level RHS =====
  // b_pos_i = (beta / dt) * max(depth_i - slop, 0)
  // Ticket 0053f: Reuse workspace
  bPos_.resize(cIdx);
  bPos_.setZero();
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
      bPos_(static_cast<Eigen::Index>(i)) = (config.beta / dt) * correctedDepth;
      anyCorrection = true;
    }
  }

  // Early exit if all penetrations within slop
  if (!anyCorrection)
  {
    return;
  }

  // ===== Step 2: Assemble Jacobians =====
  // Ticket 0053f: Reuse workspace vector
  jacobians_.resize(c);
  for (size_t i = 0; i < c; ++i)
  {
    const auto* constraint = contactConstraints[i];
    size_t const bodyA = constraint->bodyAIndex();
    size_t const bodyB = constraint->bodyBIndex();
    jacobians_[i] = constraint->jacobian(*states[bodyA], *states[bodyB], 0.0);
  }

  // ===== Step 3: Build effective mass matrix A = J * M^-1 * J^T =====
  // Ticket 0071f: bodyMInv_ is a member workspace. resize() reuses the existing
  // buffer when numBodies is unchanged; per-element assignment overwrites stale data.
  bodyMInv_.resize(numBodies);
  for (size_t k = 0; k < numBodies; ++k)
  {
    bodyMInv_[k] = Eigen::Matrix<double, 6, 6>::Zero();
    bodyMInv_[k].block<3, 3>(0, 0) =
      inverseMasses[k] * Eigen::Matrix3d::Identity();
    bodyMInv_[k].block<3, 3>(3, 3) = inverseInertias[k];
  }

  // Ticket 0053f: Reuse workspace
  effectiveMass_.resize(cIdx, cIdx);
  effectiveMass_.setZero();

  for (size_t i = 0; i < c; ++i)
  {
    size_t const bodyAI = contactConstraints[i]->bodyAIndex();
    size_t const bodyBI = contactConstraints[i]->bodyBIndex();

    Eigen::Matrix<double, 1, 6> const jIA = jacobians_[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> const jIB = jacobians_[i].block<1, 6>(0, 6);

    for (size_t j = i; j < c; ++j)
    {
      size_t const bodyAJ = contactConstraints[j]->bodyAIndex();
      size_t const bodyBJ = contactConstraints[j]->bodyBIndex();

      Eigen::Matrix<double, 1, 6> const jJA = jacobians_[j].block<1, 6>(0, 0);
      Eigen::Matrix<double, 1, 6> const jJB = jacobians_[j].block<1, 6>(0, 6);

      double aIj = 0.0;

      if (bodyAI == bodyAJ)
      {
        aIj += (jIA * bodyMInv_[bodyAI] * jJA.transpose())(0);
      }
      if (bodyAI == bodyBJ)
      {
        aIj += (jIA * bodyMInv_[bodyAI] * jJB.transpose())(0);
      }
      if (bodyBI == bodyAJ)
      {
        aIj += (jIB * bodyMInv_[bodyBI] * jJA.transpose())(0);
      }
      if (bodyBI == bodyBJ)
      {
        aIj += (jIB * bodyMInv_[bodyBI] * jJB.transpose())(0);
      }

      effectiveMass_(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = aIj;
      effectiveMass_(static_cast<Eigen::Index>(j), static_cast<Eigen::Index>(i)) = aIj;
    }
  }

  // Regularization
  for (size_t i = 0; i < c; ++i)
  {
    effectiveMass_(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) +=
      kRegularizationEpsilon;
  }

  // ===== Step 4: Solve with ASM (lambda_pos >= 0) =====
  const int numConstraints = static_cast<int>(c);
  lambdaPos_.resize(numConstraints);
  lambdaPos_.setZero();

  // Initialize active set: all constraints
  // Ticket 0071f: activeIndices_ is a member workspace. clear() + reserve() reuses
  // the existing buffer rather than heap-allocating a new vector each call.
  activeIndices_.clear();
  activeIndices_.reserve(static_cast<size_t>(numConstraints));
  for (int i = 0; i < numConstraints; ++i)
  {
    activeIndices_.push_back(i);
  }

  int const maxIter = std::min(2 * numConstraints, 100);

  for (int iter = 1; iter <= maxIter; ++iter)
  {
    int const activeSize = static_cast<int>(activeIndices_.size());

    if (activeSize == 0)
    {
      lambdaPos_.setZero();
    }
    else
    {
      // Ticket 0053f: Reuse ASM workspace
      asmAw_.resize(activeSize, activeSize);
      asmBw_.resize(activeSize);

      for (int i = 0; i < activeSize; ++i)
      {
        asmBw_(i) = bPos_(activeIndices_[static_cast<size_t>(i)]);
        for (int j = 0; j < activeSize; ++j)
        {
          asmAw_(i, j) = effectiveMass_(activeIndices_[static_cast<size_t>(i)],
                                        activeIndices_[static_cast<size_t>(j)]);
        }
      }

      Eigen::LLT<Eigen::MatrixXd> const llt{asmAw_};
      if (llt.info() != Eigen::Success)
      {
        return;  // Singular — skip correction
      }

      asmBw_ = llt.solve(asmBw_);

      lambdaPos_.setZero();
      for (int i = 0; i < activeSize; ++i)
      {
        lambdaPos_(activeIndices_[static_cast<size_t>(i)]) = asmBw_(i);
      }
    }

    // Check primal feasibility (lambda >= 0)
    double minLambda = std::numeric_limits<double>::infinity();
    int minIndex = -1;
    for (int i = 0; i < activeSize; ++i)
    {
      int const idx = activeIndices_[static_cast<size_t>(i)];
      double const val = lambdaPos_(idx);
      if (val < minLambda || (val == minLambda && idx < minIndex))
      {
        minLambda = val;
        minIndex = idx;
      }
    }

    if (minLambda < 0.0)
    {
      std::erase(activeIndices_, minIndex);
      continue;
    }

    // Check dual feasibility (w >= 0 for inactive set)
    // Ticket 0053f: Reuse workspace
    asmW_.resize(numConstraints);
    asmW_.noalias() = effectiveMass_ * lambdaPos_ - bPos_;

    double maxViolation = 0.0;
    int maxViolationIndex = -1;
    constexpr double kTolerance = 1e-6;

    for (int i = 0; i < numConstraints; ++i)
    {
      bool const isActive =
        std::ranges::find(activeIndices_, i) != activeIndices_.end();
      if (isActive)
      {
        continue;
      }

      if (asmW_(i) < -kTolerance)
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
      break;  // All KKT conditions satisfied
    }

    auto insertPos = std::ranges::lower_bound(activeIndices_, maxViolationIndex);
    activeIndices_.insert(insertPos, maxViolationIndex);
  }

  // ===== Step 5: Compute pseudo-velocity and apply as position change =====
  // dv_pseudo_k = M_k^-1 * sum(J_i_k^T * lambda_pos_i)
  // dx_k = dv_pseudo_k * dt
  for (size_t i = 0; i < c; ++i)
  {
    if (lambdaPos_(static_cast<Eigen::Index>(i)) <= 0.0)
    {
      continue;
    }

    size_t const bodyA = contactConstraints[i]->bodyAIndex();
    size_t const bodyB = contactConstraints[i]->bodyBIndex();

    Eigen::Matrix<double, 1, 6> const jIA = jacobians_[i].block<1, 6>(0, 0);
    Eigen::Matrix<double, 1, 6> const jIB = jacobians_[i].block<1, 6>(0, 6);

    double const lambda = lambdaPos_(static_cast<Eigen::Index>(i));

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
      states[bodyA]->position += Coordinate{dxA(0), dxA(1), dxA(2)};

      // Apply angular orientation correction via quaternion update
      double const halfAngle = dthetaA.norm() * 0.5;
      if (halfAngle > 1e-12)
      {
        Eigen::Vector3d axis = dthetaA.normalized();
        QuaternionD dq{
          Eigen::Quaterniond{Eigen::AngleAxisd{dthetaA.norm(), axis}}};
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

      states[bodyB]->position += Coordinate{dxB(0), dxB(1), dxB(2)};

      double const halfAngleB = dthetaB.norm() * 0.5;
      if (halfAngleB > 1e-12)
      {
        Eigen::Vector3d axisB = dthetaB.normalized();
        QuaternionD dqB{
          Eigen::Quaterniond{Eigen::AngleAxisd{dthetaB.norm(), axisB}}};
        states[bodyB]->orientation = dqB * states[bodyB]->orientation;
        states[bodyB]->orientation.normalize();
      }
    }
  }
  // Pseudo-velocities are local variables — discarded here, never become KE
}

}  // namespace msd_sim
