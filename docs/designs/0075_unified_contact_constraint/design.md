# Design: Unified Contact Constraint (Block PGS)

## Summary

This design merges the separate `ContactConstraint` (dimension=1, normal) and `FrictionConstraint` (dimension=2, tangent) into a single unified `ContactConstraint` with dimension=3 when friction is present. It replaces the decoupled normal-then-friction solve (ticket 0070) and the NLopt SLSQP friction cone solver with a **two-phase Block Projected Gauss-Seidel (Block PGS)** algorithm:

- **Phase A (Restitution Pre-Solve)**: Applies normal-only restitution impulses per contact, sequentially updating velocities. This isolates the `(1+e)` restitution factor from the friction cone constraint.
- **Phase B (Dissipative Block PGS)**: Solves the full 3x3 effective mass matrix per contact with Coulomb cone projection, using a purely dissipative RHS (`b = -Jv`, no restitution term). This phase can only remove kinetic energy, never inject it.

The motivation is physical correctness: the current decoupled approach computes normal impulses without accounting for friction-induced torques (the K_nt coupling terms), causing underestimated normal forces during sliding, tight friction bounds, and warm-start lag at contact onset. The Block PGS solve captures the K_nt coupling, while the two-phase structure prevents the energy injection mechanism identified in DD-0070-H2 (where coupling `(1+e)` restitution with the friction cone constraint caused spurious energy gain).

## Architecture Changes

### PlantUML Diagram

See: `./0075_unified_contact_constraint.puml`

---

### New Components

#### `BlockPGSSolver`

- **Purpose**: Two-phase contact solver. Phase A applies normal-only restitution impulses (isolating `(1+e)` from friction). Phase B runs purely dissipative per-contact 3x3 block PGS sweeps with Coulomb cone projection, coupling normal and friction through the full effective mass matrix without risk of energy injection.

- **Physical meaning of the 3x3 effective mass matrix**: Each contact point generates three constraint directions: one normal (n) and two tangent (t1, t2). The 3x3 matrix `K = J_block * M^{-1} * J_block^T` is the **effective mass** seen by the constraint in these three directions. Its entries have the following physical interpretation:

  | Entry | Physical Meaning |
  |-------|-----------------|
  | `K_nn` (diagonal) | How much a unit normal impulse changes the normal relative velocity. Depends on both bodies' inverse masses and how the lever arms project onto the normal direction. |
  | `K_t1t1`, `K_t2t2` (diagonal) | How much a unit tangent impulse changes the tangent relative velocity. Larger when lever arms are long (rotational coupling amplifies tangent response). |
  | `K_nt1`, `K_nt2` (off-diagonal) | **The coupling terms.** How much a unit friction impulse along t1 or t2 changes the *normal* relative velocity (and vice versa). These are non-zero when the lever arm `r × n` has components along the tangent directions — i.e., when friction torques tilt the body toward or away from the contact surface. This is the effect the decoupled solve ignores. |
  | `K_t1t2` (off-diagonal) | Cross-coupling between the two tangent directions. Non-zero when lever arms create asymmetric rotational response. |

  Concretely, for body A with inverse mass `w_A` and inverse inertia `I_A^{-1}`, and lever arm `r_A` from CoM to contact point:
  ```
  K = w_A * n⊗n + (r_A×n)^T * I_A^{-1} * (r_A×n)    [normal-normal block]
    + w_A * t1⊗t1 + (r_A×t1)^T * I_A^{-1} * (r_A×t1)  [tangent-tangent block]
    + cross terms from n⊗t1, n⊗t2, t1⊗t2               [off-diagonal coupling]
    + (same structure for body B)
  ```

  When K_nt is large relative to K_nn (e.g., long lever arms, tumbling contact), ignoring the coupling causes the normal solve to underestimate the impulse needed to prevent penetration during sliding.
- **Header location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/BlockPGSSolver.cpp`
- **Key interfaces**:

  > **Investigation: Template on contact count.** The current interface uses heap-allocated `std::vector` and `Eigen::VectorXd` for per-solve results. As an initial step toward eliminating heap allocations in the solver hot path, investigate making `BlockPGSSolver` a class template parameterized on the number of contacts (known at island-decomposition time). This would allow `SolveResult::lambdas` to use `Eigen::Vector<double, 3*N>` and `bodyForces` to use `std::array<BodyForces, M>`, keeping all solver workspace on the stack for small islands. This is out of scope for the initial implementation but should be designed with this evolution in mind (e.g., avoid baking `VectorXd` into public APIs that would be hard to templatize later).

  ```cpp
  class BlockPGSSolver
  {
  public:
    struct BodyForces
    {
      ForceVector linearForce;
      TorqueVector angularTorque;
    };

    struct SolveResult
    {
      std::vector<BodyForces> bodyForces;
      Eigen::VectorXd lambdas;   // 3 per contact: [lambda_n, lambda_t1, lambda_t2]
      bool converged{false};
      int iterations{0};
      double residual{std::numeric_limits<double>::quiet_NaN()};
    };

    // Rule of Zero: all members are value types or containers with correct
    // copy/move semantics. No custom special member functions needed.

    [[nodiscard]] SolveResult solve(
      const std::vector<Constraint*>& constraints,
      const std::vector<std::reference_wrapper<const InertialState>>& states,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias,
      size_t numBodies,
      double dt,
      const std::optional<Eigen::VectorXd>& initialLambda = std::nullopt);

    void setMaxSweeps(int n) { maxSweeps_ = n; }
    void setConvergenceTolerance(double tol) { convergenceTolerance_ = tol; }

  private:
    // Block 3x3 effective mass: K = J_block * M^{-1} * J_block^T
    // J_block is 3x12 per contact: [J_n; J_t1; J_t2]
    [[nodiscard]] Eigen::Matrix3d buildBlockK(
      const ContactConstraint& c,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias) const;

    // Project lambda_block = (lambda_n, lambda_t1, lambda_t2) onto Coulomb cone:
    //   if lambda_n < 0: return (0, 0, 0)
    //   if ||lambda_t|| > mu * lambda_n: scale tangent to cone surface
    [[nodiscard]] static Eigen::Vector3d projectCoulombCone(
      const Eigen::Vector3d& lambda_block, double mu);

    // --- Phase A: Restitution Pre-Solve ---
    // Apply normal-only restitution impulse for each bouncing contact.
    // Iterates contacts sequentially, updating vRes_ after each so that
    // subsequent contacts see post-bounce velocities.
    // Returns per-contact bounce lambda (for accumulation into final result).
    [[nodiscard]] std::vector<double> applyRestitutionPreSolve(
      const std::vector<ContactConstraint*>& contacts,
      const std::vector<Eigen::Matrix3d>& blockK,
      const std::vector<std::reference_wrapper<const InertialState>>& states,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias);

    // Update vRes_ for normal row only (Phase A)
    void updateVResNormalOnly(
      const ContactConstraint& c,
      double deltaLambdaNormal,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias);

    // --- Phase B: Dissipative Block PGS ---
    // Execute one complete sweep over all contacts, returning the maximum
    // impulse change (convergence metric). Each contact is processed as a
    // 3x3 block:
    //
    // For each contact c:
    //   1. Read current constraint-space velocity:
    //      v_err = J_block * (v_pre[bodyA,bodyB] + vRes_[bodyA,bodyB])
    //      This is a Vec3: (v_normal, v_tangent1, v_tangent2)
    //
    //   2. Compute unconstrained impulse correction:
    //      delta_lambda = K_inv * (-v_err)
    //      K_inv is the pre-inverted 3x3 effective mass for this contact.
    //      The target is to drive v_err to zero (purely dissipative).
    //
    //   3. Accumulate and project onto Coulomb cone:
    //      lambda_temp = lambda_acc[c] + delta_lambda
    //      lambda_proj = projectCoulombCone(lambda_temp, mu)
    //
    //   4. Apply actual change to velocity state:
    //      delta = lambda_proj - lambda_acc[c]
    //      vRes_ += M^{-1} * J_block^T * delta
    //      lambda_acc[c] = lambda_proj
    //
    // Returns max(||delta||) across all contacts for convergence check.
    double sweepOnce(
      const std::vector<ContactConstraint*>& contacts,
      const std::vector<Eigen::Matrix3d>& blockK,
      Eigen::VectorXd& lambda,
      const std::vector<std::reference_wrapper<const InertialState>>& states,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias);

    // Update velocity residual: vRes += M^{-1} * J_block^T * delta3
    void updateVRes3(
      const ContactConstraint& c,
      const Eigen::Vector3d& delta3,
      const std::vector<double>& inverseMasses,
      const std::vector<Eigen::Matrix3d>& inverseInertias);

    // Compute v_err = J_block * (v_pre + vRes_[bodyA, bodyB])
    [[nodiscard]] Eigen::Vector3d computeBlockVelocityError(
      const ContactConstraint& c,
      const std::vector<std::reference_wrapper<const InertialState>>& states) const;

    int maxSweeps_{50};
    double convergenceTolerance_{1e-6};
    static constexpr double kCFMEpsilon{1e-8};  // CFM regularization on K diagonal

    Eigen::VectorXd vRes_;                // 6*numBodies velocity residual workspace
    std::vector<double> bounceLambdas_;   // Phase A bounce impulse per contact
  };
  ```

- **Physical meaning of `vRes_` (velocity residual)**: `vRes_` is a 6N vector (3 linear + 3 angular per body) that accumulates the **change in velocity** caused by all constraint impulses applied so far. It represents `M^{-1} * J^T * lambda_accumulated` — the total velocity correction from constraints, expressed in world-frame linear and angular velocity for each body.

  At any point during the solve, the actual velocity of body `i` is `v_pre[i] + vRes_[6i : 6i+6]`, where `v_pre` is the unconstrained velocity (after gravity and external forces, before any contact impulse). The solver never modifies `v_pre` directly; instead it builds up `vRes_` incrementally:
  - Phase A adds restitution bounce contributions (normal direction only)
  - Phase B adds non-penetration and friction corrections (all 3 directions per contact)

  After convergence, the final body forces are extracted as `F_body = J^T * lambda_total / dt`, which is equivalent to `M * vRes_ / dt`.

- **Algorithm (Two-Phase)**:

  **Phase A — Restitution Pre-Solve** (sequential, once per solve):

  For each contact `c` (sequentially):
  1. Skip if `e == 0` (rest velocity threshold already zeroed restitution upstream)
  2. Compute normal constraint velocity: `Jv_n = J_n * (v_pre + vRes_[bodyA, bodyB])`
  3. Skip if `Jv_n >= 0` (already separating, no bounce needed)
  4. Compute bounce impulse: `lambda_bounce = max(0, -(1+e) * Jv_n / K_nn)` where `K_nn = K_block(0,0)`
  5. Update velocity residual (normal only): `vRes_ += M^{-1} * J_n^T * lambda_bounce`
  6. Store `bounceLambdas_[c] = lambda_bounce`

  Phase A is sequential so that each contact's bounce updates the velocity state seen by subsequent contacts. For resting contacts (e=0, the common case for persistent contact), this loop is a no-op.

  **Phase B — Dissipative Block PGS Sweeps** (iterative):

  For each sweep, for each contact:
  1. Compute velocity error: `v_err = J_block * (v_pre + vRes_[bodyA, bodyB])`
  2. Solve unconstrained 3x3 system: `delta_lambda = K_inv * (-v_err)` — **no restitution term, no bias vector**
  3. Accumulate: `lambda_temp = lambda_acc + delta_lambda`
  4. Project onto Coulomb cone (normal floor + tangent scale)
  5. Compute actual delta: `delta = lambda_projected - lambda_acc`
  6. Update velocity residual: `vRes_ += M^{-1} * J_block^T * delta`
  7. Update accumulator: `lambda_acc = lambda_projected`

  **Final**: Add `bounceLambdas_[c]` to `lambda[c](0)` for each contact to produce the total normal impulse in the output.

- **Energy safety proof**: Phase A injects controlled restitution energy, bounded by `e` and `v_pre` — identical to the current ASM normal-only solve proven energy-correct in DD-0070-H1. Phase B's RHS is `-v_err` for ALL rows (normal and tangent). The unconstrained optimum targets zero constraint velocity (maximum dissipation). The Coulomb cone projection can only reduce impulse magnitudes. Therefore Phase B is provably dissipative — it can never inject energy. Combined: total energy change equals restitution budget plus dissipation. The DD-0070-H2 energy injection mechanism is eliminated because `(1+e)` never enters the coupled 3x3 system or interacts with the cone projection.

- **CFM regularization**: `K_diag += kCFMEpsilon` prevents singularity at extreme mass ratios. The 3x3 system is solved via `Eigen::Matrix3d::ldlt()` (Cholesky for SPD matrices; falls back to LU if conditioning is poor).

- **Dependencies**:
  - `ContactConstraint` (unified, provides 3x12 Jacobian block)
  - `InertialState` (body kinematics)
  - `Eigen::Dense` (linear algebra)

- **Thread safety**: Not thread-safe (owns `vRes_` workspace)
- **Error handling**: Logs warning and returns zero-impulse result if K matrix is singular after regularization

#### `UnifiedContactConstraintRecord`

- **Purpose**: Single transfer record combining geometry from `ContactConstraintRecord` and friction fields from `FrictionConstraintRecord`. Supersedes both.
- **Header location**: `msd/msd-transfer/src/UnifiedContactConstraintRecord.hpp`
- **Key interface**:

  ```cpp
  struct UnifiedContactConstraintRecord : public cpp_sqlite::BaseTransferObject {
    // Contact geometry (from ContactConstraintRecord)
    uint32_t body_a_id{0};
    uint32_t body_b_id{0};
    Vector3DRecord normal;
    Vector3DRecord lever_arm_a;
    Vector3DRecord lever_arm_b;
    double penetration_depth{std::numeric_limits<double>::quiet_NaN()};
    double restitution{std::numeric_limits<double>::quiet_NaN()};
    double pre_impact_rel_vel_normal{std::numeric_limits<double>::quiet_NaN()};

    // Friction fields (from FrictionConstraintRecord)
    Vector3DRecord tangent1;                // Zero vector if frictionless
    Vector3DRecord tangent2;                // Zero vector if frictionless
    double friction_coefficient{std::numeric_limits<double>::quiet_NaN()};
    double normal_lambda{std::numeric_limits<double>::quiet_NaN()};
    double tangent1_lambda{std::numeric_limits<double>::quiet_NaN()};
    double tangent2_lambda{std::numeric_limits<double>::quiet_NaN()};

    cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
  };

  BOOST_DESCRIBE_STRUCT(UnifiedContactConstraintRecord,
                        (cpp_sqlite::BaseTransferObject),
                        (body_a_id, body_b_id, normal,
                         lever_arm_a, lever_arm_b,
                         penetration_depth, restitution, pre_impact_rel_vel_normal,
                         tangent1, tangent2, friction_coefficient,
                         normal_lambda, tangent1_lambda, tangent2_lambda,
                         frame));
  ```

---

### Modified Components

#### `ContactConstraint`

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactConstraint.hpp/.cpp`
- **Changes required**:
  1. Add tangent basis fields: `tangent1_`, `tangent2_` (populated from `TangentBasis` when `frictionCoefficient > 0`)
  2. Add `friction_coefficient_` field (0.0 for frictionless, validated `>= 0`)
  3. Add `accumulated_impulse_: Eigen::Vector3d` (warm-start cache, `{0,0,0}` initialized)
  4. Add `is_sliding_mode_: bool` and sliding mode fields from `FrictionConstraint`
  5. `dimension()` returns 1 when `mu == 0`, 3 when `mu > 0`
  6. `jacobian()` returns 1x12 (frictionless) or 3x12 (friction) matrix
  7. `lambdaBounds()` returns `unilateral()` for frictionless; for friction returns a sentinel indicating cone projection (the Block PGS solver handles cone projection directly, not via `lambdaBounds()`)
  8. `hasFriction()` returns `friction_coefficient_ > 0`
  9. Absorb `setSlidingMode(dir)`, `isSlidingMode()`, `setTangentLambdas(t1, t2)` from `FrictionConstraint`
  10. `recordState()` builds and dispatches `UnifiedContactConstraintRecord` to visitor
  11. Constructor extended with optional `frictionCoefficient` parameter (default: 0.0)

- **Constructor signature change**:

  ```cpp
  ContactConstraint(size_t bodyAIndex,
                    size_t bodyBIndex,
                    const Coordinate& normal,
                    const Coordinate& contactPointA,
                    const Coordinate& contactPointB,
                    double penetrationDepth,
                    const Coordinate& comA,
                    const Coordinate& comB,
                    double restitution,
                    double preImpactRelVelNormal,
                    double frictionCoefficient = 0.0);  // NEW
  ```

- **Backward compatibility**: The 10-argument constructor (no friction) remains valid via default argument. Existing callsites in `ContactConstraintFactory` that don't pass friction are unaffected. New callsite passes friction coefficient.

#### `ContactConstraintFactory`

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp/.cpp`
- **Changes required**:
  1. `createFromCollision()` gains a `frictionCoefficient` parameter
  2. Creates unified `ContactConstraint` with friction when `frictionCoefficient > 0`
  3. No longer needs to create separate `FrictionConstraint` instances
  4. Remove the separate `FrictionConstraint` creation path (was paired after each `ContactConstraint`)

- **Updated signature**:

  ```cpp
  std::vector<std::unique_ptr<ContactConstraint>> createFromCollision(
    size_t bodyAIndex,
    size_t bodyBIndex,
    const CollisionResult& result,
    const InertialState& stateA,
    const InertialState& stateB,
    const Coordinate& comA,
    const Coordinate& comB,
    double restitution,
    double frictionCoefficient = 0.0);  // NEW
  ```

#### `ConstraintSolver`

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp/.cpp`
- **Changes required**:
  1. Add `BlockPGSSolver blockPgsSolver_` member
  2. Dispatch to `blockPgsSolver_` when any constraint returns `hasFriction() == true`
  3. Detect friction by iterating constraints and checking `ContactConstraint::hasFriction()` (replaces `dynamic_cast<FrictionConstraint*>`)
  4. Remove or deprecate `flattenConstraints()`, `buildFrictionSpec()` helpers (no longer needed)
  5. Remove `NLoptFrictionSolver` dependency once Block PGS is validated (transition period: keep behind `#ifdef USE_NLOPT_SOLVER` flag)
  6. Remove `assembleBlock3RHS()` — no longer needed since Phase B uses `-v_err` directly (no pre-assembled bias vector)
  7. Update `kASMThreshold` comment: ASM used only for frictionless contacts (no change to threshold value)

- **Dispatch table**:

  | Condition | Solver |
  |-----------|--------|
  | No friction AND numRows <= 20 | Active Set Method (unchanged) |
  | No friction AND numRows > 20 | PGS scalar (unchanged) |
  | Has friction AND numRows <= 60 | Block PGS (3x per contact) |
  | Has friction AND numRows > 60 | Block PGS (scaled down threshold) |

- **Note**: The ASM path for frictionless contacts is completely unchanged. The Block PGS replaces NLopt only for the friction path.

#### `CollisionPipeline`

- **Current location**: `msd/msd-sim/src/Physics/Collision/CollisionPipeline.hpp/.cpp`
- **Changes required**:
  1. `createConstraints()` calls unified `ContactConstraintFactory::createFromCollision()` with `frictionCoefficient` — no separate FrictionConstraint creation loop
  2. `buildSolverView()` removes the `interleaved` parameter (no more `[CC, FC, CC, FC, ...]` — all constraints are `ContactConstraint`)
  3. `buildContactView()` removes `dynamic_cast<FrictionConstraint*>` — all items in `allConstraints_` are `ContactConstraint`
  4. Remove `propagateSolvedLambdas()` — the unified constraint owns its own `tangent_lambda_` fields, which are set by the solver after the block solve
  5. `#include "FrictionConstraint.hpp"` removed from header
  6. Remove interleaving stride comments throughout

- **Backward compatibility**: All removed methods are implementation-private; external interface unchanged.

#### `ContactCache` and `CachedContact`

- **Current location**: `msd/msd-sim/src/Physics/Constraints/ContactCache.hpp/.cpp`
- **Changes required**:
  1. `CachedContact::lambdas: vector<double>` replaced with `impulses_: vector<Eigen::Vector3d>` — one `Vec3` per contact point encoding `{lambda_n, lambda_t1, lambda_t2}`
  2. Add `getWarmStart3()`: returns `vector<Eigen::Vector3d>` (one per contact point, matched by proximity)
  3. Add `update3()`: stores `vector<Eigen::Vector3d>` after solve
  4. Old `getWarmStart()` / `update()` (vector<double>) removed
  5. `BlockPGSSolver::solve()` converts warm-start `vector<Vec3>` to flat `VectorXd` internally
  6. Sliding mode fields (`slidingDirection`, `slidingFrameCount`) unchanged

- **Key insight**: The cache stores the **total** 3-component impulse per contact point (Phase A bounce + Phase B correction combined), which is exactly what the Block PGS warm-start needs. Previously, the warm-start vector interleaved `[n_0, t1_0, t2_0, n_1, t1_1, t2_1, ...]` scalars; the new format stores them as structured `Vec3` values for clarity.

- **Two-phase interaction with warm-start**:
  1. Warm-start initializes `vRes_` from cached impulses: `vRes_ = M^{-1} * J^T * lambda_warm`
  2. Phase A computes restitution impulse from current `v_pre + vRes_` and adds to `vRes_`
  3. Phase B iterates with the warm-started + bounced velocity state
  4. After convergence, the total impulse (Phase A bounce + Phase B correction) is stored in cache
  5. Phase A bounce is **not** cached separately — it is re-derived each frame from the current velocity state
  6. For resting contacts (e=0, typical for persistent contacts), Phase A is a no-op and warm-start flows directly into Phase B

#### `ConstraintRecordVisitor`

- **Current location**: `msd/msd-transfer/src/ConstraintRecordVisitor.hpp`
- **Changes required**:
  1. Add `visit(const UnifiedContactConstraintRecord&)` overload
  2. Remove `visit(const ContactConstraintRecord&)` and `visit(const FrictionConstraintRecord&)` overloads
  3. Update `DataRecorderVisitor` implementation accordingly

#### `FrictionConstraint`

- **Current location**: `msd/msd-sim/src/Physics/Constraints/FrictionConstraint.hpp/.cpp`
- **Changes required**:
  1. **Delete entirely** — all logic merged into `ContactConstraint`
  2. The `typeName()` "FrictionConstraint" is replaced by `ContactConstraint::typeName()` returning "ContactConstraint" (unified)

- **Removal order**:
  1. Merge all fields and methods into `ContactConstraint`
  2. Update `ContactConstraintFactory` to stop creating `FrictionConstraint`
  3. Update `ConstraintSolver` to remove `FrictionConstraint` dispatch
  4. Update `ConstraintRecordVisitor`
  5. Delete header and source file
  6. Remove all includes of `FrictionConstraint.hpp`

---

### Integration Points

| New/Modified Component | Existing Component | Integration Type | Notes |
|------------------------|-------------------|------------------|-------|
| `ContactConstraint` (unified) | `TangentBasis` | Uses (construction) | Computes tangent basis when friction > 0 |
| `ContactConstraint` (unified) | `ConstraintRecordVisitor` | Visitor dispatch | Now dispatches `UnifiedContactConstraintRecord` |
| `BlockPGSSolver` | `ConstraintSolver` | Composition (owns) | Replaces NLopt friction path |
| `BlockPGSSolver` | `ContactConstraint` | Uses | Reads 3x12 Jacobian block; reads/writes `accumulatedImpulse_` |
| `ContactConstraintFactory` | `CollisionPipeline` | Called by | Factory now takes friction coefficient; no second factory call |
| `ContactCache` (updated) | `CollisionPipeline` | Composition (owns) | Stores `Vec3` impulses instead of scalar vector |
| `UnifiedContactConstraintRecord` | `DataRecorder` | DAO registration | New table; replaces two old tables |

---

## Sliding Mode Integration

The sliding mode logic from ticket 0069 (`FrictionConstraint::setSlidingMode()`) moves into the unified `ContactConstraint`:

- `setSlidingMode(slidingDirection)`: Overrides tangent basis to align `t1` with `-slidingDirection` (opposing motion), `t2` perpendicular to both `t1` and normal. Sets `is_sliding_mode_ = true`.
- `isSlidingMode()`: Returns `is_sliding_mode_`.

In Block PGS, sliding mode affects the RHS (the bias term for the tangent directions points along the sliding direction), but the cone projection naturally handles the bound. No separate `tangent1LowerBounds` heuristic is needed — the coupled solve inherently produces the correct tangent impulse direction.

`CollisionPipeline::createConstraints()` continues to call `ContactCache::getSlidingState()` and invokes `setSlidingMode()` on the unified constraint when sustained sliding is detected, as in the current implementation.

---

## Phase 3: Hold-and-Resolve (Optional, Post-Validation)

If prototype testing reveals residual penetration at the Coulomb cone surface during sliding, implement the refined hold-and-resolve projection described in the ticket:

1. After the standard cone projection produces a clamped tangent impulse
2. If `||lambda_t|| == mu * lambda_n` (cone is saturated)
3. Re-solve normal row: `lambda_n = (1/K_nn) * (-v_bias_n - K_nt1 * lambda_t1 - K_nt2 * lambda_t2)`

This is a secondary refinement and is **out of scope for Phase 1/2 implementation**. Design it as an optional `postProjectionCorrection()` method on `BlockPGSSolver`.

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|-----------------|
| `test/Physics/ContactConstraintTest.cpp` (if exists) | Any test constructing `FrictionConstraint` | Breaking | Update to use unified `ContactConstraint` with friction param |
| `test/Physics/ConstraintSolverTest.cpp` (if exists) | Tests checking for `FrictionConstraint*` in constraint list | Breaking | Update to check `ContactConstraint::hasFriction()` |
| `test/Physics/CollisionPipelineTest.cpp` | `buildSolverView(interleaved)` call | Breaking | Update to use non-interleaved `buildSolverView()` |
| `test/Physics/ContactCacheTest.cpp` (if exists) | Lambda vector format | Breaking | Update to use `Vec3` impulse format |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `ContactConstraint` | `UnifiedDimension_NoFriction` | `dimension()` == 1, `hasFriction()` == false when mu=0 |
| `ContactConstraint` | `UnifiedDimension_WithFriction` | `dimension()` == 3, `hasFriction()` == true when mu>0 |
| `ContactConstraint` | `Jacobian3x12_Shape` | Friction Jacobian is 3x12, rows match n/t1/t2 directions |
| `ContactConstraint` | `SlidingModeIntegrated` | `setSlidingMode()` updates tangent basis correctly |
| `BlockPGSSolver` | `RestitutionPreSolve_BounceCorrect` | Phase A: single bouncing contact produces correct post-bounce velocity matching `v_target = -e * v_pre` |
| `BlockPGSSolver` | `RestitutionPreSolve_RestingSkipped` | Phase A: contact with e=0 produces zero bounce impulse |
| `BlockPGSSolver` | `RestitutionPreSolve_SeparatingSkipped` | Phase A: already-separating contact (Jv_n >= 0) gets no impulse |
| `BlockPGSSolver` | `RestitutionPreSolve_MultiContact` | Phase A: sequential application handles cross-body coupling correctly for 4-point manifold |
| `BlockPGSSolver` | `PhaseBNoBounce` | Phase B RHS is always `-v_err` with no `(1+e)` factor for any row |
| `BlockPGSSolver` | `TwoPhaseEnergyBound` | Total energy after solve bounded by restitution budget: for e=1 dKE=0, for e<1 dKE<0 |
| `BlockPGSSolver` | `CoulombConeProjection_Separating` | lambda_n < 0 projects to (0,0,0) |
| `BlockPGSSolver` | `CoulombConeProjection_Static` | Static contact (||t|| < mu*n) untouched |
| `BlockPGSSolver` | `CoulombConeProjection_Sliding` | Sliding contact scales tangent to cone surface |
| `BlockPGSSolver` | `SingleContact_RampSlide` | Box on 30-degree ramp with mu=0.3: steady-state lambda_n = mg*cos(30)*dt ± 5% |
| `BlockPGSSolver` | `WarmStart_Convergence` | Warm-started solve converges in < 5 iterations for persistent contact |
| `BlockPGSSolver` | `EnergyConservation_PhaseB` | Phase B: no energy injected over 60 frames of resting contact (Phase A is no-op since e=0 for resting) |
| `BlockPGSSolver` | `ExtremeMassRatio` | 1000:1 mass ratio converges without NaN |
| `BlockPGSSolver` | `ObliqueImpact_HighKnt` | Phase A + B: high rotational inertia body hitting at shallow angle with high friction; rebound KE does not exceed pre-impact KE |
| `ContactCache` | `Vec3ImpulseRoundTrip` | Store and retrieve `Vec3` warm-start correctly |
| `ContactConstraintFactory` | `CreateUnifiedWithFriction` | Factory creates single `ContactConstraint` per contact (not CC+FC pair) |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `RampSlide_StableNormalImpulse` | `CollisionPipeline`, `BlockPGSSolver`, `ContactConstraint` | Normal impulse within 5% of `mg*cos(theta)*dt` during steady sliding |
| `FlightToSlide_SmoothTransition` | `CollisionPipeline`, `ContactCache`, `BlockPGSSolver` | No impulse spike when transitioning from ballistic to sliding contact |
| `TumblingSlide_EnergyConservation` | `WorldModel`, `EnergyTracker`, `BlockPGSSolver` | Zero energy injection during sliding contact episodes (F4 variant) |
| `StackCollapse4_NoRegression` | `CollisionPipeline`, `BlockPGSSolver` | 4-box stack stable at rest; lambda_n converges in < 5 iterations warm |
| `StackCollapse16_NoRegression` | `CollisionPipeline`, `BlockPGSSolver` | 16-box stack stable; Block PGS handles multi-island case |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| `BlockPGSSolver` | `ClusterDrop32_BlockPGS` | Solver throughput for 32 objects with friction | No regression vs NLopt (target: equal or faster) |
| `BlockPGSSolver` | `SingleContact_SweepCount` | Average sweeps per solve (warm-started) | < 5 sweeps for resting contact |
| `CollisionPipeline` | `ConstraintCreation_Unified` | Time to create constraints (no separate FC allocation) | Faster than paired CC+FC allocation |

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Transfer Record Migration Strategy**

   When `FrictionConstraintRecord` and `ContactConstraintRecord` are replaced by `UnifiedContactConstraintRecord`, existing simulation recordings will have the old schema. Should we:

   - Option A: Schema migration — Add migration script that combines existing tables into unified table on load. Pros: Historical recordings readable. Cons: Migration complexity.
   - Option B: Version flag — `DataRecorder` uses old schema when replaying old recordings; new schema for new recordings. Pros: Clean separation. Cons: Two code paths.
   - Option C: Break compatibility — New recordings use new schema; old recordings incompatible. Pros: Cleanest code. Cons: Historical replay broken.
   - **Recommendation**: Option C for development speed. Replay tooling is already tied to the code version.

2. **NLopt Removal Timing**

   Should `NLoptFrictionSolver` be removed in this ticket or kept as a validation fallback?

   - Option A: Keep NLopt behind `#ifdef USE_NLOPT_SOLVER` during development and prototype validation, remove in a follow-on ticket.
   - Option B: Remove immediately, trusting the Block PGS implementation and prototype validation.
   - **Recommendation**: Option A during implementation phase; delete in a follow-on cleanup ticket once integration tests pass.

3. **ASM Path for Frictionless Contacts**

   The current ASM (Active Set Method) path handles small frictionless islands exactly. Should frictionless contacts continue using ASM, or switch to Block PGS with 1x1 blocks?

   - Option A: Keep ASM for frictionless (current behavior, no regression risk). Block PGS adds overhead for contacts that already work well.
   - Option B: Unify everything under Block PGS with `dim=1` contacts treated as degenerate 3x3 blocks. Simpler solver dispatch.
   - **Recommendation**: Option A. The ASM path is well-validated and performant; mixing frictionless contacts into Block PGS would be unnecessary churn.

4. **Per-Contact `accumulatedImpulse_` Ownership**

   The ticket proposes storing `accumulatedImpulse_` in the constraint itself, but constraints are ephemeral (created and destroyed each frame). The warm-start is persisted in `ContactCache`.

   - Option A: `ContactConstraint` stores `Vec3 accumulatedImpulse_` as a staging area that `BlockPGSSolver` reads/writes during the solve. After solve, `CollisionPipeline` reads it and calls `ContactCache::update3()`. Constraint destroyed after `clearEphemeralState()`.
   - Option B: `BlockPGSSolver` maintains its own `vector<Vec3>` indexed to constraint order. After solve, `CollisionPipeline` extracts these for cache update.
   - **Recommendation**: Option B. Keeps solver state in the solver; constraints remain data-only. Avoids mutable non-const methods on the constraint.

### Prototype Required

1. **K_nt Coupling Magnitude** (Investigation I1): Before committing to full implementation, measure K_nt/K_nn ratios in existing sliding recordings to confirm the coupling terms are significant. Target: K_nt/K_nn > 5% for the observed instability cases.

2. **Two-Phase Block PGS Convergence vs NLopt**: Run the ramp-slide and F4 tumbling test cases through a standalone two-phase Block PGS prototype to confirm convergence in < 50 sweeps with warm-starting. Compare lambda_n to analytical expectation. Verify Phase A produces correct bounce velocity before Phase B begins.

3. **Energy Injection Test**: Prototype the two-phase solver and verify that: (a) Phase A energy injection matches restitution budget exactly, (b) Phase B accumulated kinetic energy does not increase over a 60-frame resting contact sequence, (c) combined solve on F4 tumbling case shows no net energy injection beyond restitution.

4. **K_block Condition Number Analysis**: Beyond K_nt/K_nn ratio, measure condition number of the full 3x3 K_block matrices in existing recordings. If condition number > 1e6, verify CFM regularization (1e-8) is sufficient.

### Requirements Clarification

1. **ASM Threshold for Block PGS**: The current `kASMThreshold = 20` refers to total Jacobian rows. For Block PGS, the equivalent is 20/3 ≈ 7 contacts with friction. Should the threshold be expressed in contacts (more intuitive) or rows (consistent with existing code)?

2. **Sliding Mode Handling in Block PGS**: **Resolution**: Keep `setSlidingMode()` as a safety net during prototype validation. Phase B's tangent bias is `-Jv_t` which naturally handles friction direction. However, `setSlidingMode()` aligns the tangent basis with the sliding direction, which may improve convergence for sustained sliding. Add telemetry to measure activation frequency. Remove only if Block PGS produces zero activations across all test scenarios.

3. **Phase A Iteration Count**: Should Phase A iterate multiple passes over contacts (improving multi-contact bounce accuracy) or is a single pass sufficient? **Recommendation**: Single pass. Multi-contact bounce accuracy is secondary to energy stability, and Phase B will correct any residual normal velocity anyway. Multiple Phase A passes risk over-estimating bounce for tightly coupled contact manifolds.

---

## Implementation Order

The following phased order minimizes risk and enables incremental validation:

### Phase 1: Data Structure Unification
1. Create `UnifiedContactConstraintRecord` in `msd-transfer`
2. Update `ConstraintRecordVisitor` to use new record type
3. Extend `ContactConstraint` with friction fields (constructor, dimension, jacobian, accessors)
4. Update `ContactConstraintFactory` to create unified constraints
5. Update `CollisionPipeline::createConstraints()` to use unified factory (remove FC creation)
6. Update `ContactCache` to use `Vec3` impulse storage
7. Update all includes and references

**Validation gate**: Build without errors. Existing tests pass (friction behavior may regress temporarily as NLopt no longer receives correct input — accept this during Phase 1).

### Phase 2: Block PGS Solver
1. Implement `BlockPGSSolver` with two-phase structure: `applyRestitutionPreSolve()` (Phase A) + `sweepOnce()` (Phase B)
2. Implement `projectCoulombCone()`, `buildBlockK()`, `updateVResNormalOnly()`, `computeBlockVelocityError()`
3. Wire `BlockPGSSolver` into `ConstraintSolver::solve()` for the friction dispatch path
4. Remove `FrictionConstraint` class and all references
5. Run investigation I1 (K_nt analysis) on existing recordings to validate coupling assumption
6. Run integration tests (ramp slide, energy conservation, Phase A isolation tests)
7. Profile against `ClusterDrop32` benchmark baseline

### Phase 3: Cleanup
1. Remove `NLoptFrictionSolver` once Block PGS passes all tests
2. Delete `FrictionConstraintRecord`, `ContactConstraintRecord` (superseded by `UnifiedContactConstraintRecord`)
3. Update `CLAUDE.md` and diagrams in `docs/msd/`

---

## Risk Mitigations

| Risk | Mitigation |
|------|------------|
| R1: Energy injection from Block PGS | **Eliminated by design**: Phase A injects controlled restitution energy bounded by `e` and `v_pre` (identical to current ASM normal solve, proven energy-correct in DD-0070-H1). Phase B with `b = -v_err` and cone projection is provably dissipative. The `(1+e)` factor never enters the coupled 3x3 system. DD-0070-H2 mechanism cannot occur. Validated by `TwoPhaseEnergyBound` and `EnergyConservation_PhaseB` tests. |
| R2: K matrix singularity at extreme mass ratios | CFM regularization (`kCFMEpsilon = 1e-8`) on K diagonal; fallback to lambda=0 if LDLT fails. Validate K_block condition numbers in prototype. |
| R3: Warm-start basis rotation kick | Project cached `Vec3` impulse onto new contact frame before use; invalidate if normal rotated > 15° (already handled by `ContactCache::kNormalThreshold`) |
| R4: Resting contact regression | Run `StackCollapse4` and `StackCollapse16` benchmarks; keep NLopt behind runtime toggle (not `#ifdef`) during validation for A/B comparison |
| R5: Investigation I1 shows K_nt is negligible | If K_nt/K_nn < 1% across all sliding cases, reconsider whether Block PGS complexity is justified for the observed instability |
| R6: Phase A ordering in multi-contact | Sequential Gauss-Seidel-style velocity update in Phase A handles cross-body coupling. Validated by `RestitutionPreSolve_MultiContact` test. For resting contacts (e=0, the common persistent contact case), Phase A is a no-op. |
| R7: Solver drift from fixed constraint ordering | Block PGS (Gauss-Seidel variant) is sensitive to constraint processing order. Mitigate by alternating sweep direction (forward/backward) on successive iterations. |
| R8: Tangent basis instability near zero sliding velocity | When relative tangential velocity is near zero (static friction), the tangent basis can spin between frames. Mitigate by locking tangent basis at timestep start and holding fixed through all sweeps. |

---

## References

- Erin Catto, "Sequential Impulses" (GDC 2006) — Block PGS formulation for coupled normal+friction
- Tonge (2012), "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation" — PGS convergence analysis
- Bullet Physics, `btSequentialImpulseConstraintSolver` — Reference implementation of coupled block solve
- Box2D, `b2ContactSolver` — 2-constraint block solve with manifold projection
- Jolt Physics, `ContactConstraintManager` — Modern coupled contact solver design
- Ticket 0070 (`nlopt_convergence_energy_injection`) — Documents why decoupled solve was adopted and its limitations
- Ticket 0069 (`friction_velocity_reversal`) — Sliding mode detection and direction alignment
- Ticket 0073 (`hybrid_pgs_large_islands`) — Existing PGS infrastructure being extended
