# Design: Block PGS Solver Rework (0084)

## Summary

Rework the friction solver path in `ConstraintSolver` to fix 12 test failures
introduced by the 0082-series expanded test coverage. The failures expose two
categories of bugs in the existing decoupled normal-then-friction solver: (1)
energy injection during oblique sliding from K_nt cross-coupling in Phase B's
RHS construction, and (2) incorrect restitution impulse application for
elastic/inelastic collisions. The fix is a new `BlockPGSSolver` class that
implements a correct two-phase architecture and replaces only the
`hasFriction && numRows <= kASMThreshold` dispatch branch inside
`ConstraintSolver::solve()`.

---

## Architecture Changes

### PlantUML Diagram

See: `./0084_block_pgs_solver_rework.puml`

---

## Root Cause Analysis

### Root Cause 1: Phase B RHS Uses Wrong Velocity (Oblique Sliding Failures)

The current decoupled solver computes `b = assembleFlatRHS(flat_, states)` which
sets:

```
b_normal[i] = -(1+e) * J_i * v_current
b_tangent[i] = -J_i * v_current
```

After solving Phase A (normals), it computes `jvPostNormal` correctly. But for
Phase B friction, the RHS is then:

```
bt = -jvCurrent  where jvCurrent = jvPostNormal + A_tt * lambda_friction_others
```

The problem: `jvPostNormal` includes the full K matrix product
`A_full * lambda_normal`, including the K_nt off-diagonal terms
(`flatEffectiveMass_(tangent_idx, normal_idx) * lambda_n`). This means
`jvPostNormal(tangent)` already encodes the indirect velocity change due to the
normal impulse through the tangential Jacobian rows — but the Phase B friction
solve then tries to drive this residual to zero with an additional friction
impulse, injecting energy.

The correct Phase B RHS should use the **actual post-bounce tangential sliding
velocity** `J_t * v_postBounce`, where `v_postBounce` is the 12D velocity vector
after applying the Phase A normal impulse directly in body space.

### Root Cause 2: Phase A RHS Reconstructed from Post-Regularization Velocity

The current solver assembles `b` from current velocities at the time of
`assembleFlatRHS()`. However, `ContactConstraint` stores
`pre_impact_rel_vel_normal` captured at constraint construction time (before
integration). When there are multiple sub-steps or when the constraint is warm-
started from a previous frame, the current velocity at solve time differs from
the pre-impact velocity. This causes the restitution RHS to be wrong for elastic
collision tests.

The fix: `PhaseAResult` must use `pre_impact_rel_vel_normal` directly from the
`ContactConstraint` to build the normal-only RHS, not the current velocity dot-
product.

### Root Cause 3: Warm-Start Velocity Residual Not Seeded from Phase A

The existing PGS solver (`ProjectedGaussSeidel`) seeds `vRes_` from the warm-
start lambda. When the small-island friction path transitions to Phase B, it
initializes `lambdaFriction = zero` and iterates from scratch, ignoring that
Phase A already changed the body velocities. The velocity residual for Phase B
must be initialized to include Phase A's contribution.

---

## Design Decisions

### DD-0084-001: New `BlockPGSSolver` class, minimal `ConstraintSolver` changes

**Decision**: Create a new class `BlockPGSSolver` rather than modifying the
existing decoupled friction logic in-place. `ConstraintSolver::solve()` will
call `BlockPGSSolver` for the `hasFriction && numRows <= kASMThreshold` branch.

**Rationale**: Isolates the fix; the existing decoupled code remains as a
reference (and can be deleted after validation). The ASM no-friction path and
the large-island PGS path are unaffected.

**Constraint preserved**: The dispatch table structure of `ConstraintSolver` is
unchanged. Only the friction branch is replaced.

**Severity**: required (MSD-ARCH-001: new functionality in dedicated class)

### DD-0084-002: Phase A uses `pre_impact_rel_vel_normal` directly

**Decision**: Phase A assembles the normal-only RHS as:

```
b_normal[c] = -(1 + e_c) * preImpactRelVelNormal_c
```

where `preImpactRelVelNormal_c = contactConstraint->getPreImpactRelVelNormal()`.

This is the relative normal velocity at contact detection time (stored in
`ContactConstraint` at construction via `ContactConstraintFactory`). It is
correct for the restitution impulse regardless of integration-order effects.

**Rationale**: Fixes `InelasticBounce_KEReducedByESquared`,
`PerfectlyElastic_EnergyConserved`, and `EqualMassElastic_VelocitySwap`. The
current solver reconstructs this from `J_n * v_current` which is wrong when
there is any velocity change between contact detection and solve time.

**Severity**: required (physics correctness)

### DD-0084-003: Phase B RHS computed from post-bounce body-space velocity

**Decision**: After Phase A, compute the post-bounce 12D velocity vector for
each constraint pair:

```
v_postBounce[body] = v_current[body] + M_body^{-1} * J_body^T * lambda_A[body]
```

Then Phase B RHS for tangent rows:

```
b_tangent[i] = -J_tangent[i] * v_postBounce
```

For normal rows (Phase B), `b_normal[i] = 0` (the normal was already resolved
in Phase A; Phase B only maintains `lambda_n >= 0` for stability).

**Rationale**: Eliminates K_nt energy injection. The tangential velocity used
for Phase B friction is the actual sliding velocity after the bounce, not a
velocity that includes the normal impulse's effect propagated through the
off-diagonal effective-mass terms.

**Severity**: required (physics correctness — fixes oblique sliding failures)

### DD-0084-004: 3×3 block solve per contact in Phase B

**Decision**: Phase B solves the 3×3 system `[n, t1, t2]` per contact in each
PGS sweep using:

1. Compute `delta = (b_i - J_i * vRes_) / A_ii` for all three rows
2. Update `lambda_n = max(0, lambda_n + delta_n)`
3. Update `lambda_t1, lambda_t2` unconstrainedly
4. Ball-project: if `norm([lambda_t1, lambda_t2]) > mu * lambda_n`, scale to boundary
5. Update `vRes_` for the combined delta of all three components

**Rationale**: Coupling all three components in each sweep iteration prevents
the phase-splitting artifact where the normal update and friction update see
inconsistent velocity residuals within the same sweep. This is the standard
Catto 2005 approach for coupled friction-normal PGS.

**Severity**: required (numerical stability)

### DD-0084-005: vRes_ seeded from Phase A in Phase B initialization

**Decision**: Before starting Phase B sweeps, initialize `vRes_` using
Phase A's `lambda_normal`:

```
vRes_ = M^{-1} * J_normal^T * lambda_A_normal  (summed over all contacts)
```

Then Phase B sweeps update `vRes_` incrementally as each `lambda` changes.

**Rationale**: Without this, Phase B's first sweep computes `b - J * vRes_` but
`vRes_ = 0`, ignoring that Phase A already changed velocities. This causes the
friction solver to overshoot and invert the bounce velocity in some cases.

**Severity**: required (correctness for FrictionWithRestitution_BounceThenSlide)

### DD-0084-006: Phase B normal row update uses stabilized RHS

**Decision**: For normal rows in Phase B, the RHS is not zero but instead
includes only the Baumgarte position correction (no restitution):

```
b_normal_phaseB[c] = ERP/dt * penetration_c
```

This ensures that position correction (ERP) is applied through Phase B rather
than Phase A, keeping Phase A purely a velocity-level bounce solve.

**Rationale**: Fixes `TimestepSensitivity_ERPAmplification`. The current solver
lumps ERP into Phase A's RHS `b = -(1+e)*Jv + ERP_term`. This causes the ERP
correction to interact with the restitution impulse. By separating them, Phase A
is a clean velocity-level impulse and Phase B handles position correction through
the normal row's unilateral constraint.

**Severity**: required (fixes ERP coupling failure)

### DD-0084-007: Body-force extraction uses `J^T * lambda / dt` (unchanged)

**Decision**: The force extraction formula from `extractBodyForcesFlat()` is
correct and unchanged: `F = J^T * lambda / dt`. This satisfies the requirement
in the ticket that body forces remain `J^T * lambda / dt`.

**Rationale**: This is already the correct formula. No change needed.

---

## New Components

### `BlockPGSSolver`

- **Purpose**: Two-phase contact solver for the small-island friction path
  (numRows <= kASMThreshold, hasFriction). Phase A resolves restitution bounce;
  Phase B dissipates kinetic energy via coupled friction-normal PGS.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp`
- **Key interfaces**:

```cpp
namespace msd_sim {

class BlockPGSSolver
{
public:
    BlockPGSSolver() = default;
    ~BlockPGSSolver() = default;

    // Rule of Five (default all — pure value semantics on workspace members)
    BlockPGSSolver(const BlockPGSSolver&) = default;
    BlockPGSSolver& operator=(const BlockPGSSolver&) = default;
    BlockPGSSolver(BlockPGSSolver&&) noexcept = default;
    BlockPGSSolver& operator=(BlockPGSSolver&&) noexcept = default;

    struct PhaseAResult
    {
        Eigen::VectorXd lambdaNormal;  ///< Normal impulses from Phase A (flat-indexed)
        Eigen::VectorXd vPostBounce;   ///< 6*numBodies velocity vector after Phase A
        bool converged{false};
    };

    struct SolveResult
    {
        std::vector<ConstraintSolver::BodyForces> bodyForces;
        Eigen::VectorXd lambdas;
        bool converged{false};
        int iterations{0};
        double residual{std::numeric_limits<double>::quiet_NaN()};
    };

    /// Solve two-phase friction contact system.
    ///
    /// @param flat     Flattened constraint data (from ConstraintSolver::populateFlatConstraints_)
    /// @param states   Per-body kinematic states (size >= max body index in flat)
    /// @param inverseMasses   Per-body inverse mass
    /// @param inverseInertias Per-body inverse inertia tensor
    /// @param numBodies Number of bodies
    /// @param dt       Timestep [s]
    /// @param muPerContact Friction coefficient per contact (indexed by Normal row)
    /// @param initialLambda Warm-start lambda (size = flat rows); std::nullopt = cold start
    [[nodiscard]] SolveResult solve(
        const ConstraintSolver::FlattenedConstraints& flat,
        const std::vector<std::reference_wrapper<const InertialState>>& states,
        const std::vector<double>& inverseMasses,
        const std::vector<Eigen::Matrix3d>& inverseInertias,
        size_t numBodies,
        double dt,
        const std::vector<double>& muPerContact,
        const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt);

    void setMaxSweeps(int n) { maxSweeps_ = n; }
    void setConvergenceTolerance(double tol) { convergenceTol_ = tol; }

private:
    // Phase A: solve normal-only subproblem with restitution RHS
    [[nodiscard]] PhaseAResult solvePhaseA_(
        const ConstraintSolver::FlattenedConstraints& flat,
        const std::vector<std::reference_wrapper<const InertialState>>& states,
        const Eigen::MatrixXd& flatEffectiveMass,
        const std::vector<double>& inverseMasses,
        const std::vector<Eigen::Matrix3d>& inverseInertias,
        size_t numBodies,
        const std::vector<int>& normalIndices);

    // Phase B: coupled 3x3 block PGS with post-bounce RHS
    void initVelocityResidual_(
        const ConstraintSolver::FlattenedConstraints& flat,
        const Eigen::VectorXd& lambdaInit,
        const std::vector<double>& inverseMasses,
        const std::vector<Eigen::Matrix3d>& inverseInertias);

    double sweepOnce_(
        const ConstraintSolver::FlattenedConstraints& flat,
        const Eigen::VectorXd& bPhaseB,
        const Eigen::VectorXd& diag,
        Eigen::VectorXd& lambda,
        const std::vector<double>& muPerContact,
        const std::vector<double>& inverseMasses,
        const std::vector<Eigen::Matrix3d>& inverseInertias);

    void updateVRes_(
        size_t rowIdx,
        double dLambda,
        const ConstraintSolver::FlattenedConstraints& flat,
        const std::vector<double>& inverseMasses,
        const std::vector<Eigen::Matrix3d>& inverseInertias);

    // Workspace members — reused across calls to avoid heap allocation
    Eigen::VectorXd vRes_;    ///< 6*numBodies velocity residual
    Eigen::VectorXd diag_;    ///< Diagonal A_ii effective mass
    Eigen::VectorXd bPhaseB_; ///< Phase B RHS vector

    int maxSweeps_{50};
    double convergenceTol_{1e-6};

    static constexpr double kRegularizationEpsilon = 1e-8;
    static constexpr Eigen::Index kBodyDof = 6;
    static constexpr Eigen::Index kLinearDof = 3;
    static constexpr Eigen::Index kAngularDof = 3;
};

}  // namespace msd_sim
```

- **Dependencies**:
  - `ConstraintSolver::FlattenedConstraints` — reads flat row data
  - `ContactConstraint::getPreImpactRelVelNormal()` — Phase A RHS
  - `ContactConstraint::getPenetrationDepth()` — Phase B ERP term
  - `InertialState` — velocity extraction for vRes_ initialization
  - `Eigen/Dense` — matrix operations (already a project dependency)
- **Thread safety**: Not thread-safe; single-threaded per-frame use
- **Error handling**: Returns `converged = false` if Phase A LLT factorization fails

---

## Modified Components

### `ConstraintSolver`

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp` / `.cpp`
- **Changes required**:
  1. Add `BlockPGSSolver blockPgsSolver_` as a private member (workspace reuse)
  2. In `solve()`, replace the `if (hasFriction)` branch body with a call to
     `blockPgsSolver_.solve()` instead of the existing decoupled normal-then-
     friction logic
  3. Remove or comment-out the old decoupled friction code (keep as reference
     comment block with `// Replaced by BlockPGSSolver (ticket 0084)`)
  4. Pass `flat_` and `flatEffectiveMass_` through the existing workspace
     members to avoid re-assembly inside `BlockPGSSolver`
- **Backward compatibility**: No API change. The `SolveResult` interface is
  unchanged. The no-friction ASM path and large-island PGS path are untouched.

---

## Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---|---|---|---|
| `BlockPGSSolver` | `ConstraintSolver` | Composition (member) | Called only for friction path |
| `BlockPGSSolver` | `ContactConstraint` | Read via `FlattenedConstraints` | Reads `preImpactRelVelNormal` from `flat_.restitutions` — NOTE: needs `flat_` to carry penetration depths too (see Open Questions) |
| `BlockPGSSolver` | `ProjectedGaussSeidel` | None | Large-island path unchanged |
| `BlockPGSSolver` | `CollisionPipeline` | Indirect (through ConstraintSolver) | No change to pipeline |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|---|---|---|---|
| `EnergyAccountingTest.cpp` | `F3_InelasticBounce_KEReducedByESquared` | Fix expected | No test change |
| `EnergyAccountingTest.cpp` | `F2_ElasticBounce_KEConserved` | Fix expected | No test change |
| `LinearCollisionTest.cpp` | `PerfectlyElastic_EnergyConserved` | Fix expected | No test change |
| `LinearCollisionTest.cpp` | `EqualMassElastic_VelocitySwap` | Fix expected | No test change |
| `ParameterIsolationTest.cpp` | `H3_TimestepSensitivity_ERPAmplification` | Fix expected | No test change |
| `RotationDampingTest.cpp` | `C2_RockingCube_AmplitudeDecreases` | Fix expected | No test change |
| `RotationalCollisionTest.cpp` | `B3_SphereDrop_NoRotation` | Fix expected | No test change |
| `RotationalEnergyTest.cpp` | `ZeroGravity_RotationalEnergyTransfer_Conserved` | Fix expected | No test change |
| `FrictionDirectionTest.cpp` | Oblique45, Oblique45_Slow, Oblique45_Medium, HighSpeedOblique | Fix expected | No test change |
| `FrictionDirectionTest.cpp` | `FrictionWithRestitution_BounceThenSlide` | Fix expected | No test change |

All 768 currently-passing tests must remain green.

### New Tests Required

#### Unit Tests for `BlockPGSSolver`

| Component | Test Case | What It Validates |
|---|---|---|
| `BlockPGSSolver` | `PhaseA_PerfectlyElastic_NormalImpulseCorrect` | Phase A produces lambda_n = m * (1+e) * v_pre for single contact |
| `BlockPGSSolver` | `PhaseA_InelasticCollision_NormalImpulseCorrect` | Phase A produces lambda_n = m * (1+0.5) * v_pre for e=0.5 |
| `BlockPGSSolver` | `PhaseB_AxisAlignedSliding_NoZVelocityInjection` | Axis-aligned sliding on flat floor: vz stays bounded |
| `BlockPGSSolver` | `PhaseB_ObliqueSliding_NoCrossTermInjection` | 45-degree slide: vz < 2 m/s threshold |
| `BlockPGSSolver` | `PhaseB_FrictionCone_Kinematic_Correct` | Ball projection satisfies cone constraint |
| `BlockPGSSolver` | `BounceThenSlide_VelocityTransition` | Post-bounce velocity is correct before friction solve |

---

## Constraints Audit

The following project coding conventions (required severity) apply to this design:

- **MSD-INIT-001** (required): All uninitialized `double` members initialized
  with `NaN` or `0.0` in constructors. `BlockPGSSolver` workspace members are
  default-constructed Eigen vectors (zero-initialized via `setZero()`). Scalars
  `maxSweeps_` and `convergenceTol_` are in-class initialized.
- **MSD-MEM-001** (required): No raw pointers in public API. `BlockPGSSolver`
  accepts `const FlattenedConstraints&` by reference. All ownership is via value
  or reference. No `new`/`delete`.
- **MSD-NAME-001** (required): `PascalCase` for class, `camelCase` for methods,
  `snake_case_` for members. Applied throughout.
- **MSD-INIT-002** (required): Brace initialization. All constructors and
  structs use `{}` initializers. Applied throughout.

---

## Open Questions

### Design Decisions (Blocking)

1. **Does `FlattenedConstraints` carry `penetrationDepth` per Normal row?**

   Phase B needs penetration depth per contact for the ERP/Baumgarte term:
   `b_normal_phaseB[c] = (ERP/dt) * penetration_c`

   Currently `FlattenedConstraints` carries only `restitutions[]` for Normal
   rows. The `ContactConstraint::getPenetrationDepth()` accessor exists but
   is not stored in `flat_`.

   - **Option A**: Add `std::vector<double> penetrationDepths` to
     `FlattenedConstraints`. Populated from `ContactConstraint::getPenetrationDepth()`
     during `populateFlatConstraints_()`. Zero for Tangent rows.
     — Pros: self-contained, no runtime cast in BlockPGSSolver
     — Cons: small struct size increase
   - **Option B**: In `BlockPGSSolver`, accept `const std::vector<Constraint*>&`
     directly and cast to `ContactConstraint*` to read depth at solve time.
     — Pros: no struct change
     — Cons: couples BlockPGSSolver to ContactConstraint type, adds dynamic casts

   **Recommendation**: Option A — extend `FlattenedConstraints` with
   `penetrationDepths`. This is consistent with how `restitutions` is already
   stored. One additional `double` per Normal row is negligible overhead.

2. **Should Phase B iterate normals and tangents jointly, or normals-only for
   stability?**

   The design calls for 3×3 block updates. An alternative is to keep Phase B
   tangent-only (skip normal row updates in Phase B) since Phase A already
   resolved normals. The risk: if Phase A's normal impulse slightly overshoots
   (due to solver tolerance), Phase B needs to relax the normal to maintain
   non-penetration.

   - **Option A**: Full 3×3 block — normal updated in Phase B with
     `b_normal_phaseB = ERP/dt * penetration`. Correct normal maintained.
     — Recommended.
   - **Option B**: Tangent-only Phase B — normal fixed from Phase A.
     — Simpler but may drift for multi-contact resting scenarios.

   **Recommendation**: Option A. The RockingCube and resting-contact tests
   require stable normal maintenance.

### Prototype Required

1. **Validate that Phase A using `pre_impact_rel_vel_normal` gives correct
   elastic bounce behavior.**

   The `pre_impact_rel_vel_normal` is the relative normal velocity at
   `ContactConstraintFactory::createFromCollision()` time. Verify this equals
   the velocity at the moment of contact detection, not after an integration
   step. If there is a timestep between detection and solve, the value may be
   stale for fast-moving objects.

   **Prototype**: Write a unit test that creates a `ContactConstraint` with a
   known `preImpactRelVelNormal`, runs `BlockPGSSolver::solvePhaseA_`, and
   checks that the resulting lambda equals the analytical value
   `lambda = m * (1+e) * v_pre`.

2. **Confirm that vRes_ seeded from Phase A eliminates friction overshoot.**

   If the warm-start vRes_ is correct, Phase B's first sweep should produce
   near-zero tangential impulse for a scenario that has already bounced
   (friction during falling, not during contact). Validate with
   `FrictionWithRestitution_BounceThenSlide`.

---

## Revision Notes

_None — initial design._

---

## References

- Catto, E. (2005). "Iterative Dynamics with Temporal Coherence." GDC 2005.
- Baraff, D. (1994). "Fast Contact Force Computation for Nonpenetrating Rigid
  Bodies." SIGGRAPH 1994.
- Ticket 0070: `docs/designs/0070-nlopt-convergence-energy-injection/` —
  energy injection analysis (decoupled solver rationale)
- Ticket 0073: `docs/designs/0073_hybrid_pgs_large_islands/` —
  PGS velocity-residual approach (adapted for BlockPGSSolver)
- `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp` — existing solver
- `msd/msd-sim/src/Physics/Constraints/ProjectedGaussSeidel.cpp` — existing
  velocity-residual PGS (design reference for BlockPGSSolver)
- `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp` — source of
  `pre_impact_rel_vel_normal` and `penetration_depth`
