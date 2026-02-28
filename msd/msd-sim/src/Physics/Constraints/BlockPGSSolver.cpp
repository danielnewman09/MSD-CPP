// Ticket: 0075b_block_pgs_solver
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 2)

#include "msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// ============================================================================
// Public solve()
// ============================================================================

BlockPGSSolver::SolveResult BlockPGSSolver::solve(
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
    numBodies,
    BodyForces{ForceVector{0.0, 0.0, 0.0}, TorqueVector{0.0, 0.0, 0.0}});

  if (constraints.empty())
  {
    result.converged = true;
    result.iterations = 0;
    result.lambdas = Eigen::VectorXd{};
    result.residual = 0.0;
    return result;
  }

  // Cast all constraints to ContactConstraint (caller guarantees this)
  std::vector<ContactConstraint*> contacts;
  contacts.reserve(constraints.size());
  for (Constraint* c : constraints)
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
    auto* cc = static_cast<ContactConstraint*>(c);
    contacts.push_back(cc);
  }

  const size_t numContacts = contacts.size();

  // ===== Initialize velocity residual workspace =====
  // vRes_ is 6 * numBodies: [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z] per body.
  // Ticket: 0071f pattern — resize only (does not reallocate if same or smaller size)
  vRes_.resize(static_cast<Eigen::Index>(kBodyDof * numBodies));
  vRes_.setZero();

  // ===== Pre-compute 3x3 block K matrices and their inverses =====
  std::vector<Eigen::Matrix3d> blockKs;
  blockKs.reserve(numContacts);
  std::vector<Eigen::Matrix3d> blockKInvs;
  blockKInvs.reserve(numContacts);

  for (const ContactConstraint* cc : contacts)
  {
    Eigen::Matrix3d K = buildBlockK(*cc, inverseMasses, inverseInertias);
    blockKs.push_back(K);

    // Invert via LDLT (symmetric positive definite; falls back gracefully)
    Eigen::LDLT<Eigen::Matrix3d> ldlt{K};
    if (ldlt.info() == Eigen::Success)
    {
      blockKInvs.push_back(ldlt.solve(Eigen::Matrix3d::Identity()));
    }
    else
    {
      // Singular or ill-conditioned: use zero inverse (no correction for this contact)
      blockKInvs.push_back(Eigen::Matrix3d::Zero());
    }
  }

  // ===== Warm-start: initialize accumulated lambda from previous frame =====
  // Flat layout: [lambda_n0, lambda_t1_0, lambda_t2_0, lambda_n1, ...]
  //
  // Warm-start strategy:
  //   1. Seed lambdaPhaseB = lambda_warm (so Phase B accumulator starts from
  //      previous solution)
  //   2. Initialize vRes_ = M^{-1} * J^T * lambda_warm (so Phase B velocity
  //      error computation accounts for warm-start impulse already applied)
  //
  // For a stable resting contact (v_pre = -g*dt, lambda_warm = m*g*dt):
  //   vRes_A = M_A^{-1} * J_n_A^T * lambda_n = wA * (-n) * lambda_n
  //   vA_eff = v_pre + vRes_A = (-g*dt) + (g*dt) = 0
  //   vErr(0) = n · (vB - vA_eff) = n · (0) = 0
  //   → Phase B sees zero error, makes no change → warm-start preserved.
  //
  // Key: the contact normal n points A→B. For cube(A) on floor(B) with n
  // pointing downward (A→B), J_A_lin = -n points upward. The warm-start impulse
  // cancels gravity's downward velocity, not adds to it.
  const size_t lambdaSize = numContacts * 3;
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(
    static_cast<Eigen::Index>(lambdaSize));

  const bool hasWarmStart = initialLambda.has_value() &&
                            initialLambda->size() == static_cast<Eigen::Index>(lambdaSize) &&
                            initialLambda->maxCoeff() > 0.0;

  if (hasWarmStart)
  {
    lambda = *initialLambda;
    // Initialize vRes_ from warm-start so Phase B velocity error computation
    // accounts for the warm-start impulse already applied to the bodies.
    for (size_t ci = 0; ci < numContacts; ++ci)
    {
      const auto base = static_cast<Eigen::Index>(ci * 3);
      const Eigen::Vector3d warmBlock = lambda.segment<3>(base);
      updateVRes3(*contacts[ci], warmBlock, inverseMasses, inverseInertias);
    }
  }

  // ===== Phase A: Restitution Pre-Solve =====
  // For each contact with e > 0 and approaching normal velocity, apply a
  // restitution bounce impulse (normal direction only) and update vRes_.
  // Sequential so each contact sees updated velocities from prior bounces.
  bounceLambdas_ = applyRestitutionPreSolve(
    contacts, blockKs, states, inverseMasses, inverseInertias);

  // ===== Phase B: Dissipative Block PGS Sweeps =====
  // RHS is always -v_err (no restitution term). This is provably dissipative:
  // the unconstrained optimum drives constraint velocity to zero (max dissipation),
  // and the Coulomb cone projection can only reduce impulse magnitudes.

  double maxDelta = std::numeric_limits<double>::infinity();
  int iter = 0;

  // ===== Phase B accumulator =====
  // Seed Phase B from the warm-start lambda (or zero if no warm-start).
  // vRes_ was initialized from warm-start above so Phase B velocity error
  // computation sees the correct post-warm-start velocity state.
  Eigen::VectorXd lambdaPhaseB = lambda;  // lambda is zero or lambda_warm

  for (iter = 0; iter < maxSweeps_ && maxDelta > convergenceTolerance_; ++iter)
  {
    // Record lambda_n before the sweep for each contact
    std::vector<double> lambdaNBefore(numContacts);
    for (size_t ci = 0; ci < numContacts; ++ci)
    {
      lambdaNBefore[ci] = lambdaPhaseB(static_cast<Eigen::Index>(ci * 3));
    }

    maxDelta = sweepOnce(
      contacts, blockKInvs, lambdaPhaseB, states, inverseMasses, inverseInertias);

    // ===== Post-Sweep Net-Zero Redistribution (Ticket: 0084 Prototype P6) =====
    //
    // After each sweep, the nonlinear Coulomb cone projection may have
    // asymmetrically clipped tangential impulses across contacts sharing a body,
    // breaking the pre-projection cancellation symmetry. This creates a net
    // positive delta_lambda_n across a body's contacts — the actual energy
    // injection mechanism (see math-formulation.md Section 3 and 6).
    //
    // Fix: for each dynamic body (non-zero inverse mass) with 2+ sliding contacts,
    // compute the sum of delta_lambda_n across its contacts. If net positive,
    // subtract the mean from each contact's lambda_n and re-project onto
    // the Coulomb cone to restore feasibility.
    //
    // GUARD: Only redistribute when ALL contacts on the body are non-penetrating
    // (vErr(0) >= 0). If any contact is penetrating, the normal impulse growth
    // is physically correct (resisting penetration) and must not be redistributed.
    //
    // Only dynamic bodies (inverseMasses[k] > 0) are corrected.
    // Only applies when a body has >= 2 contacts.
    //
    // This preserves the per-contact differential (tipping torque) while
    // eliminating spurious net normal impulse growth from cone asymmetry.

    // Compute velocity errors and check penetration status for all contacts
    std::vector<double> vErrNormal(numContacts);
    for (size_t ci = 0; ci < numContacts; ++ci)
    {
      vErrNormal[ci] = computeBlockVelocityError(*contacts[ci], states)(0);
    }

    // Check per-body: is any contact penetrating (vErr_n < 0)?
    // Also accumulate net delta_lambda_n and contact count for non-penetrating bodies.
    std::vector<double> bodyNetDeltaN(numBodies, 0.0);
    std::vector<int> bodyContactCount(numBodies, 0);
    std::vector<bool> bodyHasPenetration(numBodies, false);

    for (size_t ci = 0; ci < numContacts; ++ci)
    {
      const auto base = static_cast<Eigen::Index>(ci * 3);
      const double deltaN = lambdaPhaseB(base) - lambdaNBefore[ci];

      const ContactConstraint& cc = *contacts[ci];
      const size_t idxA = cc.bodyAIndex();
      const size_t idxB = cc.bodyBIndex();

      const bool penetrating = (vErrNormal[ci] < 0.0);

      // Track for dynamic bodies only
      if (inverseMasses[idxA] > 0.0)
      {
        bodyNetDeltaN[idxA] += deltaN;
        bodyContactCount[idxA] += 1;
        if (penetrating)
        {
          bodyHasPenetration[idxA] = true;
        }
      }
      if (inverseMasses[idxB] > 0.0)
      {
        bodyNetDeltaN[idxB] += deltaN;
        bodyContactCount[idxB] += 1;
        if (penetrating)
        {
          bodyHasPenetration[idxB] = true;
        }
      }
    }

    // Redistribute: only for bodies that are non-penetrating, dynamic, and
    // have >= 2 sliding contacts, AND have a net POSITIVE delta_n (energy injection).
    // "Sliding" means the contact is at the cone boundary (||lambda_t|| == mu*lambda_n).
    // Static-friction contacts (inside the cone) are excluded because their delta_n
    // already sums to ~zero by the K_inv symmetry in the linear regime.
    constexpr double kRedistTolerance = 1e-8;
    constexpr double kConeOnSurface = 0.99;  // fraction of cone radius to consider "on surface"

    for (size_t ci = 0; ci < numContacts; ++ci)
    {
      const ContactConstraint& cc = *contacts[ci];
      const size_t idxA = cc.bodyAIndex();
      const auto base = static_cast<Eigen::Index>(ci * 3);

      // Only redistribute if:
      // 1. Body A is dynamic
      // 2. Body A has >= 2 contacts
      // 3. No contact on body A is penetrating
      // 4. Net delta is positive (spurious normal injection)
      if (inverseMasses[idxA] <= 0.0 ||
          bodyContactCount[idxA] < 2 ||
          bodyHasPenetration[idxA])
      {
        continue;
      }

      const double netDelta = bodyNetDeltaN[idxA];
      if (netDelta <= kRedistTolerance)
      {
        continue;  // Net non-positive: no energy injection to correct
      }

      // Only apply correction to contacts that are at the sliding friction boundary.
      // Static friction contacts (inside the cone) should not be redistributed.
      const Eigen::Vector3d lambdaBlock = lambdaPhaseB.segment<3>(base);
      const double lambdaN = lambdaBlock(0);
      const double tangentNorm = lambdaBlock.tail<2>().norm();
      const double mu = cc.getFrictionCoefficient();
      const bool isSliding = (lambdaN > 0.0 && tangentNorm >= kConeOnSurface * mu * lambdaN);

      if (!isSliding)
      {
        continue;
      }

      const double correction = netDelta / bodyContactCount[idxA];
      Eigen::Vector3d lambdaNew = lambdaBlock;
      const Eigen::Vector3d lambdaOld = lambdaBlock;
      lambdaNew(0) -= correction;

      // Re-project onto Coulomb cone after redistributing lambda_n
      const Eigen::Vector3d lambdaProj = projectCoulombCone(lambdaNew, mu);

      const Eigen::Vector3d delta = lambdaProj - lambdaOld;
      if (delta.squaredNorm() > 1e-24)
      {
        updateVRes3(cc, delta, inverseMasses, inverseInertias);
        lambdaPhaseB.segment<3>(base) = lambdaProj;
      }
    }
  }

  result.converged = (maxDelta <= convergenceTolerance_);
  result.iterations = iter;
  result.residual = maxDelta;

  // ===== Assemble final lambdas =====
  // Final normal lambda = Phase A bounce + Phase B correction
  //
  // phaseBLambdas: Phase B lambdas only (no bounce contribution).
  // These are the correct warm-start seed for the next frame:
  //   - Phase A bounce impulses are stateless (recomputed each frame from
  //     current velocity) — including them in the warm-start would cause
  //     Phase A to see an inflated warm-start baseline and compute
  //     incorrect bounce magnitudes on the next bounce frame.
  //   - Phase B lambdas represent the dissipative friction state that
  //     should persist as warm-start.
  // Ticket: 0084 Fix F2
  result.phaseBLambdas = lambdaPhaseB;  // Phase B only (no bounce)

  result.lambdas.resize(static_cast<Eigen::Index>(lambdaSize));
  for (size_t ci = 0; ci < numContacts; ++ci)
  {
    const auto base = static_cast<Eigen::Index>(ci * 3);
    result.lambdas(base)     = lambdaPhaseB(base) + bounceLambdas_[ci];
    result.lambdas(base + 1) = lambdaPhaseB(base + 1);
    result.lambdas(base + 2) = lambdaPhaseB(base + 2);
  }

  // ===== Extract per-body forces from total lambda =====
  // F_body = J^T * lambda_total / dt
  //
  // We accumulate J^T * lambda for each contact directly (not via vRes_).
  // vRes_ stores M^{-1} * J^T * lambda (already divided by mass), so it cannot
  // be used directly — using vRes_/dt would apply an extra 1/m factor when the
  // integrator applies F/m*dt.
  //
  // Instead, compute impulse = J^T * lambda per body, then divide by dt to get force.
  // This matches the ASM path in ConstraintSolver::extractBodyForces().

  // impulse_[6k..6k+5] = sum over contacts of J_k^T * lambda for body k
  Eigen::VectorXd impulse = Eigen::VectorXd::Zero(
    static_cast<Eigen::Index>(kBodyDof * numBodies));

  for (size_t ci = 0; ci < numContacts; ++ci)
  {
    const ContactConstraint& cc = *contacts[ci];
    const size_t idxA = cc.bodyAIndex();
    const size_t idxB = cc.bodyBIndex();

    const Eigen::Vector3d n  = cc.getContactNormal();
    const Eigen::Vector3d t1 = cc.getTangent1();
    const Eigen::Vector3d t2 = cc.getTangent2();
    const Eigen::Vector3d rA = cc.getLeverArmA();
    const Eigen::Vector3d rB = cc.getLeverArmB();

    const auto base = static_cast<Eigen::Index>(ci * 3);
    const double dN  = result.lambdas(base);
    const double dT1 = result.lambdas(base + 1);
    const double dT2 = result.lambdas(base + 2);

    // J_block_A^T * lambda (signs from ContactConstraint Jacobian rows):
    //   Row 0 (n):  J_A_lin = -n,    J_A_ang = -(rA×n)
    //   Row 1 (t1): J_A_lin = +t1,   J_A_ang = +(rA×t1)
    //   Row 2 (t2): J_A_lin = +t2,   J_A_ang = +(rA×t2)
    const Eigen::Vector3d impulseALin = -n * dN + t1 * dT1 + t2 * dT2;
    const Eigen::Vector3d impulseAAng = -rA.cross(n) * dN +
                                         rA.cross(t1) * dT1 +
                                         rA.cross(t2) * dT2;

    // J_block_B^T * lambda:
    //   Row 0 (n):  J_B_lin = +n,    J_B_ang = +(rB×n)
    //   Row 1 (t1): J_B_lin = -t1,   J_B_ang = -(rB×t1)
    //   Row 2 (t2): J_B_lin = -t2,   J_B_ang = -(rB×t2)
    const Eigen::Vector3d impulseBLin = n * dN - t1 * dT1 - t2 * dT2;
    const Eigen::Vector3d impulseBAng = rB.cross(n) * dN -
                                         rB.cross(t1) * dT1 -
                                         rB.cross(t2) * dT2;

    const auto baseA = static_cast<Eigen::Index>(kBodyDof * idxA);
    impulse.segment<3>(baseA)     += impulseALin;
    impulse.segment<3>(baseA + 3) += impulseAAng;

    const auto baseB = static_cast<Eigen::Index>(kBodyDof * idxB);
    impulse.segment<3>(baseB)     += impulseBLin;
    impulse.segment<3>(baseB + 3) += impulseBAng;
  }

  for (size_t k = 0; k < numBodies; ++k)
  {
    const auto base = static_cast<Eigen::Index>(kBodyDof * k);
    const Eigen::Vector3d lin = impulse.segment<3>(base)     / dt;
    const Eigen::Vector3d ang = impulse.segment<3>(base + 3) / dt;

    result.bodyForces[k].linearForce  = ForceVector{lin.x(), lin.y(), lin.z()};
    result.bodyForces[k].angularTorque = TorqueVector{ang.x(), ang.y(), ang.z()};
  }

  return result;
}

// ============================================================================
// buildBlockK
// ============================================================================

Eigen::Matrix3d BlockPGSSolver::buildBlockK(
  const ContactConstraint& c,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias) const
{
  // Get body indices
  const size_t idxA = c.bodyAIndex();
  const size_t idxB = c.bodyBIndex();

  // Lever arms and directions
  const Eigen::Vector3d n  = c.getContactNormal();
  const Eigen::Vector3d t1 = c.getTangent1();
  const Eigen::Vector3d t2 = c.getTangent2();
  const Eigen::Vector3d rA = c.getLeverArmA();
  const Eigen::Vector3d rB = c.getLeverArmB();

  const double wA = inverseMasses[idxA];
  const double wB = inverseMasses[idxB];
  const Eigen::Matrix3d& IA_inv = inverseInertias[idxA];
  const Eigen::Matrix3d& IB_inv = inverseInertias[idxB];

  // Jacobian structure (3x12):
  //   Row 0 (n):  J_n  = [-n^T, -(rA×n)^T,  n^T,  (rB×n)^T ]
  //   Row 1 (t1): J_t1 = [ t1^T, (rA×t1)^T, -t1^T, -(rB×t1)^T]
  //   Row 2 (t2): J_t2 = [ t2^T, (rA×t2)^T, -t2^T, -(rB×t2)^T]
  //
  // K = J_block * M^{-1} * J_block^T
  //
  // K(i,j) = contribution from body A + contribution from body B:
  //   from A: w_A * (J_A_i · J_A_j) + (rA×dir_i)^T * IA_inv * (rA×dir_j)
  //   from B: w_B * (J_B_i · J_B_j) + (rB×dir_i)^T * IB_inv * (rB×dir_j)
  //
  // where dir_i ∈ {n, t1, t2} and J_A_i = ±dir_i, J_B_i = ∓dir_i.

  // Helper: sign of body A's linear contribution for direction d_i:
  //   normal row → body A gets -n  → sign = -1
  //   tangent rows → body A gets +t1, +t2 → sign = +1
  // Body B:
  //   normal row → body B gets +n  → sign = +1
  //   tangent rows → body B gets -t1, -t2 → sign = -1
  // Linear term K_ij from body A: (-signA_i) * (-signA_j) * wA * (dir_i · dir_j)
  // Since dir_i · dir_j = delta_ij (orthonormal basis), off-diagonal linear = 0.
  // Actually K_ij from A = wA * (dirA_i · dirA_j) where dirA_i is the linear
  // Jacobian row for body A.
  //
  // Let us just compute J directly:
  Eigen::Matrix<double, 3, 6> JA;  // columns: [v_A (3), omega_A (3)]
  Eigen::Matrix<double, 3, 6> JB;  // columns: [v_B (3), omega_B (3)]

  const Eigen::Vector3d rACrossN  = rA.cross(n);
  const Eigen::Vector3d rACrossT1 = rA.cross(t1);
  const Eigen::Vector3d rACrossT2 = rA.cross(t2);
  const Eigen::Vector3d rBCrossN  = rB.cross(n);
  const Eigen::Vector3d rBCrossT1 = rB.cross(t1);
  const Eigen::Vector3d rBCrossT2 = rB.cross(t2);

  // Row 0: J_n → linear A part = -n, angular A part = -rA×n
  //              linear B part = +n, angular B part = +rB×n
  JA.row(0) << -n.transpose(), -rACrossN.transpose();
  JB.row(0) << n.transpose(),   rBCrossN.transpose();

  // Row 1: J_t1 → linear A = +t1, angular A = +rA×t1
  //               linear B = -t1, angular B = -(rB×t1)
  JA.row(1) << t1.transpose(),  rACrossT1.transpose();
  JB.row(1) << -t1.transpose(), -rBCrossT1.transpose();

  // Row 2: J_t2 → linear A = +t2, angular A = +rA×t2
  //               linear B = -t2, angular B = -(rB×t2)
  JA.row(2) << t2.transpose(),  rACrossT2.transpose();
  JB.row(2) << -t2.transpose(), -rBCrossT2.transpose();

  // M_A^{-1} block-diagonal (6x6):  [ wA * I3,  0; 0, IA_inv ]
  // K contribution from A: JA * M_A^{-1} * JA^T
  Eigen::Matrix3d KA = wA * (JA.leftCols<3>() * JA.leftCols<3>().transpose()) +
                       JA.rightCols<3>() * IA_inv * JA.rightCols<3>().transpose();

  Eigen::Matrix3d KB = wB * (JB.leftCols<3>() * JB.leftCols<3>().transpose()) +
                       JB.rightCols<3>() * IB_inv * JB.rightCols<3>().transpose();

  Eigen::Matrix3d K = KA + KB;

  // CFM regularization: prevents singularity at extreme mass ratios
  K.diagonal().array() += kCFMEpsilon;

  return K;
}

// ============================================================================
// projectCoulombCone
// ============================================================================

Eigen::Vector3d BlockPGSSolver::projectCoulombCone(
  const Eigen::Vector3d& lambda_block, double mu)
{
  const double lambda_n = lambda_block(0);

  // If normal force is zero or separating: zero impulse
  if (lambda_n <= 0.0)
  {
    return Eigen::Vector3d::Zero();
  }

  const double maxTangent = mu * lambda_n;
  const Eigen::Vector2d lambda_t = lambda_block.tail<2>();
  const double tangentNorm = lambda_t.norm();

  if (tangentNorm <= maxTangent)
  {
    // Static friction: inside cone
    return lambda_block;
  }

  // Sliding friction: project tangent component onto cone surface
  Eigen::Vector3d projected;
  projected(0) = lambda_n;
  projected.tail<2>() = lambda_t * (maxTangent / tangentNorm);
  return projected;
}

// ============================================================================
// Phase A: applyRestitutionPreSolve
// ============================================================================

std::vector<double> BlockPGSSolver::applyRestitutionPreSolve(
  const std::vector<ContactConstraint*>& contacts,
  const std::vector<Eigen::Matrix3d>& blockKs,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias)
{
  const size_t n = contacts.size();
  std::vector<double> bounceLambdas(n, 0.0);

  for (size_t ci = 0; ci < n; ++ci)
  {
    const ContactConstraint& cc = *contacts[ci];
    const double e = cc.getRestitution();

    // Phase A only applies to bouncing contacts (e > 0)
    if (e <= 0.0)
    {
      continue;
    }

    // Compute current normal constraint velocity including any prior bounces
    // Jv_n = J_n * (v_pre[bodyA] + vRes_[bodyA] + v_pre[bodyB] + vRes_[bodyB])
    // This uses computeBlockVelocityError which returns v_err = J_block * v_current
    const Eigen::Vector3d vErr = computeBlockVelocityError(cc, states);
    const double Jv_n = vErr(0);

    // Skip if bodies are already separating (Jv_n >= 0 means separating)
    // Constraint sign: Jv_n < 0 means approaching (contact is active)
    if (Jv_n >= 0.0)
    {
      continue;
    }

    // Compute bounce impulse: lambda_bounce = (1+e) * (-Jv_n) / K_nn
    // K_nn = blockKs[ci](0,0)
    const double K_nn = blockKs[ci](0, 0);
    if (K_nn < 1e-12)
    {
      continue;  // Degenerate K — skip
    }

    const double lambda_bounce = (1.0 + e) * (-Jv_n) / K_nn;

    // Clamp to non-negative (bounce can only push bodies apart)
    if (lambda_bounce <= 0.0)
    {
      continue;
    }

    bounceLambdas[ci] = lambda_bounce;

    // Update vRes_ for normal direction only
    updateVResNormalOnly(cc, lambda_bounce, inverseMasses, inverseInertias);
  }

  return bounceLambdas;
}

// ============================================================================
// updateVResNormalOnly (Phase A helper)
// ============================================================================

void BlockPGSSolver::updateVResNormalOnly(
  const ContactConstraint& c,
  double deltaLambdaNormal,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias)
{
  const size_t idxA = c.bodyAIndex();
  const size_t idxB = c.bodyBIndex();

  const Eigen::Vector3d n  = c.getContactNormal();
  const Eigen::Vector3d rA = c.getLeverArmA();
  const Eigen::Vector3d rB = c.getLeverArmB();

  // J_n row for body A: linear = -n, angular = -(rA×n)
  // vRes_A += M_A^{-1} * J_n_A^T * deltaLambda
  const double wA = inverseMasses[idxA];
  const Eigen::Matrix3d& IA_inv = inverseInertias[idxA];

  const auto baseA = static_cast<Eigen::Index>(kBodyDof * idxA);
  vRes_.segment<3>(baseA)     -= wA * n * deltaLambdaNormal;          // linear A
  vRes_.segment<3>(baseA + 3) -= IA_inv * rA.cross(n) * deltaLambdaNormal;  // angular A

  // J_n row for body B: linear = +n, angular = +(rB×n)
  const double wB = inverseMasses[idxB];
  const Eigen::Matrix3d& IB_inv = inverseInertias[idxB];

  const auto baseB = static_cast<Eigen::Index>(kBodyDof * idxB);
  vRes_.segment<3>(baseB)     += wB * n * deltaLambdaNormal;          // linear B
  vRes_.segment<3>(baseB + 3) += IB_inv * rB.cross(n) * deltaLambdaNormal;  // angular B
}

// ============================================================================
// Phase B: sweepOnce
// ============================================================================

double BlockPGSSolver::sweepOnce(
  const std::vector<ContactConstraint*>& contacts,
  const std::vector<Eigen::Matrix3d>& blockKInvs,
  Eigen::VectorXd& lambda,
  const std::vector<std::reference_wrapper<const InertialState>>& states,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias)
{
  double maxDelta = 0.0;

  for (size_t ci = 0; ci < contacts.size(); ++ci)
  {
    const ContactConstraint& cc = *contacts[ci];
    const auto base = static_cast<Eigen::Index>(ci * 3);

    // Step 1: Compute constraint-space velocity error
    // v_err = J_block * (v_pre + vRes_[bodyA, bodyB])
    const Eigen::Vector3d vErr = computeBlockVelocityError(cc, states);

    // Step 2: Full coupled K_inv solve.
    //
    // Ticket: 0084 Prototype P6 — reverted P5 velocity-gated clamp.
    // The coupled K_inv solve preserves the exact K_inv(0,0) coefficient
    // for the Coulomb cone bound (lambda_n) and the K_inv(1:2, 0:2) rows
    // for correct tangential/angular coupling (tipping torque).
    //
    // The energy injection mechanism is the nonlinear Coulomb cone projection
    // breaking the pre-projection cancellation symmetry (see math-formulation.md
    // Section 3). The fix is a post-sweep net-zero redistribution in solve(),
    // not a per-contact clamp in sweepOnce.
    const Eigen::Vector3d unconstrained = blockKInvs[ci] * (-vErr);

    // Step 3: Accumulate
    Eigen::Vector3d lambdaOld = lambda.segment<3>(base);
    Eigen::Vector3d lambdaTemp = lambdaOld + unconstrained;

    // Step 4: Project onto Coulomb cone
    const double mu = cc.getFrictionCoefficient();
    const Eigen::Vector3d lambdaProj = projectCoulombCone(lambdaTemp, mu);

    // Step 5: Actual change after projection
    const Eigen::Vector3d delta = lambdaProj - lambdaOld;

    // Step 6: Update velocity residual
    if (delta.squaredNorm() > 1e-24)
    {
      updateVRes3(cc, delta, inverseMasses, inverseInertias);
    }

    // Step 7: Update accumulator
    lambda.segment<3>(base) = lambdaProj;

    // Track convergence metric
    maxDelta = std::max(maxDelta, delta.norm());
  }

  return maxDelta;
}

// ============================================================================
// updateVRes3
// ============================================================================

void BlockPGSSolver::updateVRes3(
  const ContactConstraint& c,
  const Eigen::Vector3d& delta3,
  const std::vector<double>& inverseMasses,
  const std::vector<Eigen::Matrix3d>& inverseInertias)
{
  const size_t idxA = c.bodyAIndex();
  const size_t idxB = c.bodyBIndex();

  const Eigen::Vector3d n  = c.getContactNormal();
  const Eigen::Vector3d t1 = c.getTangent1();
  const Eigen::Vector3d t2 = c.getTangent2();
  const Eigen::Vector3d rA = c.getLeverArmA();
  const Eigen::Vector3d rB = c.getLeverArmB();

  const double wA = inverseMasses[idxA];
  const Eigen::Matrix3d& IA_inv = inverseInertias[idxA];
  const double wB = inverseMasses[idxB];
  const Eigen::Matrix3d& IB_inv = inverseInertias[idxB];

  const double dN  = delta3(0);
  const double dT1 = delta3(1);
  const double dT2 = delta3(2);

  // Body A contribution: J_A = [-n, -(rA×n), t1, (rA×t1), t2, (rA×t2)] per row
  // vRes_A += M_A^{-1} * J_block_A^T * delta3
  //
  // Linear A: -n * dN + t1 * dT1 + t2 * dT2
  // Angular A: -(rA×n) * dN + (rA×t1) * dT1 + (rA×t2) * dT2
  const auto baseA = static_cast<Eigen::Index>(kBodyDof * idxA);
  const Eigen::Vector3d linearA  = -n * dN + t1 * dT1 + t2 * dT2;
  const Eigen::Vector3d angularA = -rA.cross(n) * dN +
                                    rA.cross(t1) * dT1 +
                                    rA.cross(t2) * dT2;

  vRes_.segment<3>(baseA)     += wA * linearA;
  vRes_.segment<3>(baseA + 3) += IA_inv * angularA;

  // Body B contribution: J_B = [n, (rB×n), -t1, -(rB×t1), -t2, -(rB×t2)] per row
  // Linear B: n * dN - t1 * dT1 - t2 * dT2
  // Angular B: (rB×n) * dN - (rB×t1) * dT1 - (rB×t2) * dT2
  const auto baseB = static_cast<Eigen::Index>(kBodyDof * idxB);
  const Eigen::Vector3d linearB  = n * dN - t1 * dT1 - t2 * dT2;
  const Eigen::Vector3d angularB = rB.cross(n) * dN -
                                    rB.cross(t1) * dT1 -
                                    rB.cross(t2) * dT2;

  vRes_.segment<3>(baseB)     += wB * linearB;
  vRes_.segment<3>(baseB + 3) += IB_inv * angularB;
}

// ============================================================================
// computeBlockVelocityError
// ============================================================================

Eigen::Vector3d BlockPGSSolver::computeBlockVelocityError(
  const ContactConstraint& c,
  const std::vector<std::reference_wrapper<const InertialState>>& states) const
{
  const size_t idxA = c.bodyAIndex();
  const size_t idxB = c.bodyBIndex();

  const InertialState& sA = states[idxA].get();
  const InertialState& sB = states[idxB].get();

  // Current body A velocity = v_pre_A + vRes_A
  const auto baseA = static_cast<Eigen::Index>(kBodyDof * idxA);
  const Eigen::Vector3d vA = Eigen::Vector3d{
    sA.velocity.x(), sA.velocity.y(), sA.velocity.z()} + vRes_.segment<3>(baseA);
  const AngularVelocity omegaA_pre = sA.getAngularVelocity();
  const Eigen::Vector3d omegaA = Eigen::Vector3d{
    omegaA_pre.x(), omegaA_pre.y(), omegaA_pre.z()} + vRes_.segment<3>(baseA + 3);

  // Current body B velocity = v_pre_B + vRes_B
  const auto baseB = static_cast<Eigen::Index>(kBodyDof * idxB);
  const Eigen::Vector3d vB = Eigen::Vector3d{
    sB.velocity.x(), sB.velocity.y(), sB.velocity.z()} + vRes_.segment<3>(baseB);
  const AngularVelocity omegaB_pre = sB.getAngularVelocity();
  const Eigen::Vector3d omegaB = Eigen::Vector3d{
    omegaB_pre.x(), omegaB_pre.y(), omegaB_pre.z()} + vRes_.segment<3>(baseB + 3);

  const Eigen::Vector3d n  = c.getContactNormal();
  const Eigen::Vector3d t1 = c.getTangent1();
  const Eigen::Vector3d t2 = c.getTangent2();
  const Eigen::Vector3d rA = c.getLeverArmA();
  const Eigen::Vector3d rB = c.getLeverArmB();

  // Relative contact point velocity
  // v_rel_A = vA + omegaA × rA (velocity of contact point on A)
  // v_rel_B = vB + omegaB × rB (velocity of contact point on B)
  const Eigen::Vector3d vContactA = vA + omegaA.cross(rA);
  const Eigen::Vector3d vContactB = vB + omegaB.cross(rB);

  // J_n * v = -n · v_A_contact + n · v_B_contact (sign from constraint convention)
  // J_t1 * v = +t1 · v_A_contact - t1 · v_B_contact
  // J_t2 * v = +t2 · v_A_contact - t2 · v_B_contact

  // Note: sign convention from ContactConstraint::jacobian():
  //   J_n row 0: [-n^T, -(rA×n)^T, n^T, (rB×n)^T]
  //   J_t1 row 1: [t1^T, (rA×t1)^T, -t1^T, -(rB×t1)^T]
  //
  // So v_err(0) = J_n · [vA; omegaA; vB; omegaB]
  //             = -n·vA - (rA×n)·omegaA + n·vB + (rB×n)·omegaB
  //             = n·(vContactB - vContactA)
  // v_err(1) = t1·vA + (rA×t1)·omegaA - t1·vB - (rB×t1)·omegaB
  //           = t1·(vContactA - vContactB)

  Eigen::Vector3d vErr;
  vErr(0) = n.dot(vContactB - vContactA);     // normal: positive = separating
  vErr(1) = t1.dot(vContactA - vContactB);    // tangent1
  vErr(2) = t2.dot(vContactA - vContactB);    // tangent2

  return vErr;
}

}  // namespace msd_sim
