// Ticket: 0073_hybrid_pgs_large_islands
// Design: docs/designs/0073_hybrid_pgs_large_islands/design.md

#include "msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// ============================================================================
// Public solve()
// ============================================================================

ProjectedGaussSeidel::SolveResult ProjectedGaussSeidel::solve(
  const std::vector<Constraint*>& constraints,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias,
  size_t numBodies,
  double dt,
  const std::optional<Eigen::VectorXd>& initialLambda)
{
  SolveResult result;
  result.bodyForces.resize(
    numBodies, BodyForces{Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0}});

  if (constraints.empty())
  {
    result.converged = true;
    result.iterations = 0;
    result.lambdas = Eigen::VectorXd{};
    result.residual = 0.0;
    return result;
  }

  // ===== Flatten constraints into per-row entries =====
  // R3 from design review: replicate the flattenConstraints() logic using our
  // internal FlatRow type (avoids circular include with ConstraintSolver.hpp).
  // Order: [CC, FC, CC, FC, ...] -> [n_0, t1_0, t2_0, n_1, t1_1, t2_1, ...]
  std::vector<FlatRow> rows;
  rows.reserve(constraints.size() * 3);  // Upper bound: 3 rows per constraint

  // Per-contact friction coefficients, indexed by contact (Normal row) order
  std::vector<double> muPerContact;

  for (const auto* constraint : constraints)
  {
    size_t const bodyA = constraint->bodyAIndex();
    size_t const bodyB = constraint->bodyBIndex();
    auto j = constraint->jacobian(states[bodyA].get(), states[bodyB].get(), 0.0);
    int const dim = constraint->dimension();

    const auto* cc = dynamic_cast<const ContactConstraint*>(constraint);
    const auto* fc = dynamic_cast<const FrictionConstraint*>(constraint);

    for (int row = 0; row < dim; ++row)
    {
      FlatRow flatRow;
      flatRow.jacobianRow = j.row(row);
      flatRow.bodyAIndex = bodyA;
      flatRow.bodyBIndex = bodyB;

      if (cc != nullptr)
      {
        flatRow.rowType = RowType::Normal;
        flatRow.restitution = cc->getRestitution();
      }
      else
      {
        flatRow.rowType = RowType::Tangent;
        flatRow.restitution = 0.0;
      }

      rows.push_back(flatRow);
    }

    if (fc != nullptr)
    {
      muPerContact.push_back(fc->getFrictionCoefficient());
    }
  }

  const auto numRows = static_cast<Eigen::Index>(rows.size());
  if (numRows == 0)
  {
    result.converged = true;
    result.iterations = 0;
    result.lambdas = Eigen::VectorXd{};
    result.residual = 0.0;
    return result;
  }

  // ===== Assemble diagonal effective-mass elements A_ii =====
  // A_ii = J_i * M^{-1} * J_i^T + regularization
  Eigen::VectorXd diag(numRows);
  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    const FlatRow& row = rows[static_cast<size_t>(i)];
    size_t const bodyA = row.bodyAIndex;
    size_t const bodyB = row.bodyBIndex;

    Eigen::Matrix<double, 1, 6> const jA = row.jacobianRow.leftCols<6>();
    Eigen::Matrix<double, 1, 6> const jB = row.jacobianRow.rightCols<6>();

    double aii = 0.0;
    aii += inverseMasses[bodyA] * jA.leftCols<3>().squaredNorm();
    aii += (jA.rightCols<3>() * inverseInertias[bodyA] *
            jA.rightCols<3>().transpose())(0);
    aii += inverseMasses[bodyB] * jB.leftCols<3>().squaredNorm();
    aii += (jB.rightCols<3>() * inverseInertias[bodyB] *
            jB.rightCols<3>().transpose())(0);

    diag(i) = aii + kRegularizationEpsilon;
  }

  // ===== Assemble RHS b_i =====
  // Normal rows: b_i = -(1+e) * J_i * v
  // Tangent rows: b_i = -J_i * v
  Eigen::VectorXd b(numRows);
  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    const FlatRow& row = rows[static_cast<size_t>(i)];
    size_t const bodyA = row.bodyAIndex;
    size_t const bodyB = row.bodyBIndex;

    const InertialState& stateA = states[bodyA].get();
    const InertialState& stateB = states[bodyB].get();

    Eigen::Matrix<double, 12, 1> v;
    v.segment<3>(0) = Eigen::Vector3d{
      stateA.velocity.x(), stateA.velocity.y(), stateA.velocity.z()};
    AngularVelocity omegaA = stateA.getAngularVelocity();
    v.segment<3>(3) = Eigen::Vector3d{omegaA.x(), omegaA.y(), omegaA.z()};
    v.segment<3>(6) = Eigen::Vector3d{
      stateB.velocity.x(), stateB.velocity.y(), stateB.velocity.z()};
    AngularVelocity omegaB = stateB.getAngularVelocity();
    v.segment<3>(9) = Eigen::Vector3d{omegaB.x(), omegaB.y(), omegaB.z()};

    double const jv = (row.jacobianRow * v)(0);

    if (row.rowType == RowType::Normal)
    {
      b(i) = -(1.0 + row.restitution) * jv;
    }
    else
    {
      b(i) = -jv;
    }
  }

  // ===== Initialize lambda =====
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(numRows);
  if (initialLambda.has_value() && initialLambda->size() == numRows)
  {
    lambda = *initialLambda;
    // Clamp warm-start normals >= 0 (unilateral)
    for (Eigen::Index i = 0; i < numRows; ++i)
    {
      if (rows[static_cast<size_t>(i)].rowType == RowType::Normal)
      {
        lambda(i) = std::max(0.0, lambda(i));
      }
    }
  }

  // ===== Initialize velocity residual v_res =====
  // v_res[6k..6k+5] = M^{-1} * J^T * lambda, accumulated over all rows.
  // Layout: [vLin_k(3), omega_k(3)] for body k.
  Eigen::VectorXd vRes =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(6 * numBodies));

  for (Eigen::Index i = 0; i < numRows; ++i)
  {
    if (lambda(i) != 0.0)
    {
      updateVRes(rows, static_cast<size_t>(i), lambda(i), vRes, inverseMasses,
                 inverseInertias);
    }
  }

  // ===== PGS sweep loop =====
  double lastMaxDelta = std::numeric_limits<double>::quiet_NaN();
  bool converged = false;
  int sweep = 0;

  for (sweep = 0; sweep < maxSweeps_; ++sweep)
  {
    lastMaxDelta = sweepOnce(
      rows, muPerContact, diag, b, lambda, vRes, inverseMasses, inverseInertias);

    if (lastMaxDelta < convergenceTolerance_)
    {
      converged = true;
      ++sweep;  // Report sweep count (1-based on exit)
      break;
    }
  }

  // ===== Extract per-body forces from final lambda =====
  // No negative lambda skip — friction forces can be negative.
  for (size_t i = 0; i < static_cast<size_t>(numRows); ++i)
  {
    const FlatRow& row = rows[i];
    size_t const bodyA = row.bodyAIndex;
    size_t const bodyB = row.bodyBIndex;

    Eigen::Matrix<double, 1, 6> jA = row.jacobianRow.leftCols<6>();
    Eigen::Matrix<double, 1, 6> jB = row.jacobianRow.rightCols<6>();
    double const lam = lambda(static_cast<Eigen::Index>(i));

    Eigen::Matrix<double, 6, 1> forceA = jA.transpose() * lam / dt;
    Eigen::Matrix<double, 6, 1> forceB = jB.transpose() * lam / dt;

    result.bodyForces[bodyA].linearForce +=
      Coordinate{forceA(0), forceA(1), forceA(2)};
    result.bodyForces[bodyA].angularTorque +=
      Coordinate{forceA(3), forceA(4), forceA(5)};

    result.bodyForces[bodyB].linearForce +=
      Coordinate{forceB(0), forceB(1), forceB(2)};
    result.bodyForces[bodyB].angularTorque +=
      Coordinate{forceB(3), forceB(4), forceB(5)};
  }

  result.lambdas = lambda;
  result.converged = converged;
  result.iterations = sweep;
  result.residual = lastMaxDelta;

  return result;
}

// ============================================================================
// Private updateVRes()
// ============================================================================

void ProjectedGaussSeidel::updateVRes(
  const std::vector<FlatRow>& rows,
  size_t rowIdx,
  double dLambda,
  Eigen::VectorXd& vRes,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias)
{
  if (dLambda == 0.0)
  {
    return;
  }

  const FlatRow& row = rows[rowIdx];
  size_t const bodyA = row.bodyAIndex;
  size_t const bodyB = row.bodyBIndex;

  Eigen::Matrix<double, 1, 6> const jA = row.jacobianRow.leftCols<6>();
  Eigen::Matrix<double, 1, 6> const jB = row.jacobianRow.rightCols<6>();

  // Body A: v_res += M_A^{-1} * J_A^T * dLambda
  auto const aOff = static_cast<Eigen::Index>(6 * bodyA);
  Eigen::Matrix<double, 6, 1> const dA = jA.transpose() * dLambda;
  vRes.segment<3>(aOff) += inverseMasses[bodyA] * dA.segment<3>(0);
  vRes.segment<3>(aOff + 3) += inverseInertias[bodyA] * dA.segment<3>(3);

  // Body B: v_res += M_B^{-1} * J_B^T * dLambda
  auto const bOff = static_cast<Eigen::Index>(6 * bodyB);
  Eigen::Matrix<double, 6, 1> const dB = jB.transpose() * dLambda;
  vRes.segment<3>(bOff) += inverseMasses[bodyB] * dB.segment<3>(0);
  vRes.segment<3>(bOff + 3) += inverseInertias[bodyB] * dB.segment<3>(3);
}

// ============================================================================
// Private sweepOnce()
// ============================================================================

double ProjectedGaussSeidel::sweepOnce(
  const std::vector<FlatRow>& rows,
  const std::vector<double>& muPerContact,
  const Eigen::VectorXd& diag,
  const Eigen::VectorXd& b,
  Eigen::VectorXd& lambda,
  Eigen::VectorXd& vRes,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias)
{
  const auto numRows = static_cast<Eigen::Index>(rows.size());
  double maxDelta = 0.0;
  int contactIdx = 0;
  Eigen::Index i = 0;

  while (i < numRows)
  {
    const FlatRow& curRow = rows[static_cast<size_t>(i)];
    const bool isNormal = (curRow.rowType == RowType::Normal);

    if (isNormal)
    {
      // ===== Normal row: unilateral, lambda_i >= 0 =====

      size_t const bodyA = curRow.bodyAIndex;
      size_t const bodyB = curRow.bodyBIndex;
      auto const aOff = static_cast<Eigen::Index>(6 * bodyA);
      auto const bOff = static_cast<Eigen::Index>(6 * bodyB);

      Eigen::Matrix<double, 1, 6> const jA = curRow.jacobianRow.leftCols<6>();
      Eigen::Matrix<double, 1, 6> const jB = curRow.jacobianRow.rightCols<6>();

      double const jvRes =
        (jA * vRes.segment<6>(aOff))(0) + (jB * vRes.segment<6>(bOff))(0);

      double const delta = (b(i) - jvRes) / diag(i);
      double const lambdaOld = lambda(i);
      double const lambdaNew = std::max(0.0, lambdaOld + delta);
      double const dLambda = lambdaNew - lambdaOld;
      lambda(i) = lambdaNew;

      maxDelta = std::max(maxDelta, std::abs(dLambda));
      updateVRes(rows, static_cast<size_t>(i), dLambda, vRes, inverseMasses,
                 inverseInertias);

      // Check for following friction pair (t1, t2 rows)
      const bool hasFrictionPair =
        (i + 2 < numRows) &&
        (rows[static_cast<size_t>(i + 1)].rowType == RowType::Tangent) &&
        (rows[static_cast<size_t>(i + 2)].rowType == RowType::Tangent);

      if (hasFrictionPair)
      {
        double const mu =
          (contactIdx < static_cast<int>(muPerContact.size()))
          ? muPerContact[static_cast<size_t>(contactIdx)]
          : 0.0;

        const size_t t1Idx = static_cast<size_t>(i + 1);
        const size_t t2Idx = static_cast<size_t>(i + 2);

        // ===== Solve t1 (unclamped PGS update) =====
        {
          const FlatRow& r = rows[t1Idx];
          auto const taOff = static_cast<Eigen::Index>(6 * r.bodyAIndex);
          auto const tbOff = static_cast<Eigen::Index>(6 * r.bodyBIndex);
          Eigen::Matrix<double, 1, 6> const ja = r.jacobianRow.leftCols<6>();
          Eigen::Matrix<double, 1, 6> const jb = r.jacobianRow.rightCols<6>();

          double const jv =
            (ja * vRes.segment<6>(taOff))(0) +
            (jb * vRes.segment<6>(tbOff))(0);
          lambda(i + 1) += (b(i + 1) - jv) / diag(i + 1);
        }

        // ===== Solve t2 (unclamped PGS update) =====
        {
          const FlatRow& r = rows[t2Idx];
          auto const taOff = static_cast<Eigen::Index>(6 * r.bodyAIndex);
          auto const tbOff = static_cast<Eigen::Index>(6 * r.bodyBIndex);
          Eigen::Matrix<double, 1, 6> const ja = r.jacobianRow.leftCols<6>();
          Eigen::Matrix<double, 1, 6> const jb = r.jacobianRow.rightCols<6>();

          double const jv =
            (ja * vRes.segment<6>(taOff))(0) +
            (jb * vRes.segment<6>(tbOff))(0);
          lambda(i + 2) += (b(i + 2) - jv) / diag(i + 2);
        }

        // ===== Ball-projection onto friction disk =====
        // R1: bypass lambdaBounds() box bounds; use ||lambda_t|| <= mu*lambda_n
        // (same approach as decoupled solver, ticket 0070)
        double const lt1Before = lambda(i + 1);
        double const lt2Before = lambda(i + 2);

        double projLt1 = lt1Before;
        double projLt2 = lt2Before;

        if (lambdaNew <= 0.0)
        {
          // No normal force: no friction
          projLt1 = 0.0;
          projLt2 = 0.0;
        }
        else
        {
          double const radiusSquared = mu * mu * lambdaNew * lambdaNew;
          double const normSquared =
            lt1Before * lt1Before + lt2Before * lt2Before;

          if (normSquared > radiusSquared && radiusSquared > 1e-12)
          {
            double const scale = std::sqrt(radiusSquared / normSquared);
            projLt1 = lt1Before * scale;
            projLt2 = lt2Before * scale;
          }
        }

        double const dLt1 = projLt1 - lt1Before;
        double const dLt2 = projLt2 - lt2Before;
        lambda(i + 1) = projLt1;
        lambda(i + 2) = projLt2;

        // Use the PGS delta (pre-projection) for convergence — these represent
        // the unconstrained increments which drive the system toward rest.
        double const preProjectDelta1 = std::abs(lambda(i + 1) - lt1Before + dLt1);
        double const preProjectDelta2 = std::abs(lambda(i + 2) - lt2Before + dLt2);
        maxDelta = std::max(maxDelta, preProjectDelta1);
        maxDelta = std::max(maxDelta, preProjectDelta2);
        maxDelta = std::max(maxDelta, std::abs(dLt1));
        maxDelta = std::max(maxDelta, std::abs(dLt2));

        // Update velocity residual for projection changes
        updateVRes(rows, t1Idx, dLt1, vRes, inverseMasses, inverseInertias);
        updateVRes(rows, t2Idx, dLt2, vRes, inverseMasses, inverseInertias);

        ++contactIdx;
        i += 3;  // Consumed normal + t1 + t2
      }
      else
      {
        ++contactIdx;
        ++i;
      }
    }
    else
    {
      // Standalone tangent row (no preceding normal in this sweep window)
      size_t const bodyA = curRow.bodyAIndex;
      size_t const bodyB = curRow.bodyBIndex;
      auto const aOff = static_cast<Eigen::Index>(6 * bodyA);
      auto const bOff = static_cast<Eigen::Index>(6 * bodyB);

      Eigen::Matrix<double, 1, 6> const jA = curRow.jacobianRow.leftCols<6>();
      Eigen::Matrix<double, 1, 6> const jB = curRow.jacobianRow.rightCols<6>();

      double const jvRes =
        (jA * vRes.segment<6>(aOff))(0) + (jB * vRes.segment<6>(bOff))(0);
      double const delta = (b(i) - jvRes) / diag(i);
      double const lambdaOld = lambda(i);
      lambda(i) += delta;
      double const dLambda = lambda(i) - lambdaOld;

      maxDelta = std::max(maxDelta, std::abs(dLambda));
      updateVRes(rows, static_cast<size_t>(i), dLambda, vRes, inverseMasses,
                 inverseInertias);

      ++i;
    }
  }

  return maxDelta;
}

}  // namespace msd_sim
