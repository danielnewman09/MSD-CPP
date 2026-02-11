# Ticket 0055b: Friction Direction Root Cause Investigation

## Status
- [x] Draft
- [x] Investigation In Progress
- [ ] Investigation Complete — Root Cause Identified
- [ ] Merged / Complete

**Current Phase**: Investigation In Progress
**Type**: Investigation
**Priority**: High
**Assignee**: workflow-orchestrator
**Created**: 2026-02-10
**Branch**: 0055b-friction-direction-root-cause
**GitHub Issue**: #39
**GitHub PR**: #40 (draft)
**Parent Ticket**: [0055_tilted_cube_friction_direction](0055_tilted_cube_friction_direction.md)
**Dependencies**: [0055a](0055a_tilted_cube_trajectory_test_suite.md)
**Generate Tutorial**: No

---

## Objective

Using the failing tests from 0055a as diagnostic evidence, systematically identify why the friction force direction is incorrect for certain tilt orientations. Produce a documented root cause analysis with specific code locations.

---

## Investigation Strategy

### Phase 1: Characterize the Failure Pattern

Run the 0055a test suite and categorize results:

| Question | How to Answer |
|----------|---------------|
| Which tilt orientations fail? | Run all T1-T8, record pass/fail |
| Is the failure in direction or magnitude? | Compare actual vs expected displacement vectors |
| Do symmetry tests fail? | If T1 passes but T2 fails, the sign handling is suspect |
| Does the failure depend on friction? | Re-run with friction = 0 — if trajectory is correct without friction, friction is the culprit |
| Does the failure depend on tilt magnitude? | Try θ = 0.01, 0.05, 0.1, 0.2, 0.5 |

### Phase 2: Isolate the Source

Based on Phase 1 results, investigate these subsystems in order:

#### 2a: EPA Contact Normal Accuracy

For each failing configuration, extract the EPA contact normal at the first frame of contact and verify:
- Is the normal approximately `(0, 0, 1)` (vertical, as expected for cube-on-flat-floor)?
- Does the normal have unexpected lateral components that would skew the tangent basis?
- Does the SAT fallback activate, and if so, is its normal correct?

**Diagnostic**: Add logging to `CollisionPipeline` or `CollisionHandler` to print contact normals for the tilted cube contact.

#### 2b: Tangent Basis Alignment

For the EPA normal from 2a, compute the Duff tangent basis and verify:
- Are `t1` and `t2` in the floor plane (perpendicular to `z`)?
- If the normal has a small lateral component, do the tangent vectors rotate significantly?
- Is there a discontinuity in the tangent basis for normals near `(0, 0, 1)` that depends on the sign of the lateral perturbation?

**Diagnostic**: Print tangent basis vectors for each contact at first frame.

#### 2c: Contact Point Location

Verify the EPA contact point is correct:
- Is it on the correct face/edge/corner of the tilted cube?
- Is the lever arm `r = contactPoint - CoM` consistent with the tilt direction?
- For multi-contact manifolds, are the contact points distributed correctly?

**Diagnostic**: Print contact points and lever arms at first frame.

#### 2d: Friction Jacobian and Impulse Direction

Verify the friction constraint produces the correct impulse:
- Compute the relative velocity at the contact point in the tangent basis frame
- Verify the friction impulse opposes the sliding direction
- Check if the impulse, when transformed back to world coordinates, pushes in the expected direction

**Diagnostic**: Print `J * v` (relative tangential velocity), `λ_t` (friction impulse), and the resulting world-space force.

#### 2e: Constraint Solver Flattening

Verify the constraint flattening in `ConstraintSolver::flattenConstraints()` preserves the correct association between normal and friction rows:
- Does the 3-row grouping `[n, t1, t2]` maintain the correct pairing?
- Is the friction cone constraint `||λ_t|| ≤ μ·λ_n` applied to the correct contact?

### Phase 3: Root Cause Documentation

Document the root cause with:
1. The specific code location(s) where the error occurs
2. Why it only manifests for certain tilt orientations
3. The mathematical explanation of the incorrect behavior
4. A proposed fix strategy

---

## Key Hypotheses (Ranked by Likelihood)

### H1: EPA Normal Lateral Perturbation Skews Tangent Basis

For a tilted cube on a flat floor, the true contact normal should be purely vertical `(0,0,1)`. But EPA extracts the normal from the Minkowski difference, and for a tilted cube the support mapping may produce a normal with small lateral components. These lateral components could rotate the tangent basis away from the floor plane, causing friction to act partially in the normal direction.

**Test**: Compare EPA normal to `(0,0,1)` for each tilt configuration.

### H2: Contact Point Asymmetry from EPA vs SAT

When SAT fallback activates (for near-zero penetration), the contact manifold construction may produce different contact points than EPA. If the contact points are on the wrong face of the tilted cube, the lever arm will produce incorrect torque.

**Test**: Check which code path (EPA vs SAT) produces the contact for each tilt configuration.

### H3: ReferenceFrame Overload Bug (Coordinate vs Vector3D)

The known overload bug where `globalToLocal(Coordinate)` and `globalToLocal(Vector3D)` produce different results could affect friction direction if a contact normal or tangent vector is inadvertently passed as a Coordinate instead of a Vector3D.

**Test**: Search friction and collision code for `globalToLocal`/`localToGlobal` calls and verify argument types.

### H4: Sign Convention Inconsistency in Friction Jacobian

The friction Jacobian rows are `[t^T, (r×t)^T, -t^T, -(r×t)^T]` where A is the inertial body and B is the environment. If the body ordering is inconsistent (sometimes A is environment), the sign flips and friction pushes the wrong way.

**Test**: Verify body ordering in `FrictionConstraint` construction for inertial-vs-environment pairs.

---

## Acceptance Criteria

1. Root cause identified with specific file and line number references
2. Failure pattern fully characterized (which orientations fail and why)
3. At least one hypothesis confirmed or ruled out with evidence
4. Proposed fix strategy documented
5. Investigation findings recorded in ticket for 0055c to implement

---

## Workflow Log

### Investigation In Progress Phase
- **Started**: 2026-02-10
- **Branch**: 0055b-friction-direction-root-cause
- **GitHub Issue**: #39
- **GitHub PR**: #40 (draft)
- **Artifacts**:
  - `docs/investigations/0055b_friction_direction_root_cause/investigation-log.md`
  - `docs/investigations/0055b_friction_direction_root_cause/diagnostic-normal-extraction.cpp` (reference implementation)
- **Progress**:
  - Phase 1 COMPLETE: Characterized failure pattern (16/19 tests pass, 3 fail with 21x energy injection)
  - Phase 2a IN PROGRESS: Investigating EPA contact normal accuracy
  - Created diagnostic framework for EPA normal extraction
  - Reviewed tangent basis construction (Duff et al. 2017 algorithm)
  - Identified H1 (EPA Normal Lateral Perturbation) as most likely root cause
- **Key Findings**:
  - Tiny 0.01 rad perturbation causes 20.7m spurious Y displacement (21x amplification)
  - Bug manifests only when velocity + asymmetric tilt break symmetry
  - Symmetry tests pass (deterministic bug, not numerical instability)
  - All tilt orientation tests (T1-T8) pass without NaN
- **Next Steps**:
  - Add logging to capture EPA normals at first contact
  - Compare EPA normal to expected (0, 0, 1) for floor contact
  - If lateral components confirmed, trace to EPA vs SAT fallback source
  - Document root cause with specific code locations
