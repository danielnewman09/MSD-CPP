# Investigation Log: 0055b Friction Direction Root Cause

**Created**: 2026-02-10
**Investigator**: workflow-orchestrator → diagnostic analysis
**Objective**: Systematically identify why friction force direction is incorrect for certain tilt orientations

---

## Phase 1: Characterize the Failure Pattern

### Test Suite Status (from 0055a)

Running the TiltedCubeTrajectoryTest suite to characterize failures:

**Command**:
```bash
cd /Users/danielnewman/Documents/GitHub/MSD-CPP
cmake --build --preset conan-debug --target msd_sim_test
./build/Debug/debug/msd_sim_test --gtest_filter="TiltedCubeTrajectory*"
```

### Phase 1 Questions

| Question | Status | Findings |
|----------|--------|----------|
| Which tilt orientations fail? | PENDING | Will run T1-T8 and record pass/fail |
| Is failure in direction or magnitude? | PENDING | Compare actual vs expected displacement vectors |
| Do symmetry tests fail? | PENDING | If T1 passes but T2 fails, sign handling is suspect |
| Does failure depend on friction? | PENDING | Re-run with friction=0 to isolate friction vs other forces |
| Does failure depend on tilt magnitude? | PENDING | Try θ = 0.01, 0.05, 0.1, 0.2, 0.5 |

### Test Execution Results

**Date**: 2026-02-10
**Command**: `./build/Debug/debug/msd_sim_test --gtest_filter="TiltedCubeTrajectory*"`
**Result**: 16 PASS, 3 FAIL

#### Passing Tests (16)
- T1-T8 (all tilt orientation tests) — **Interesting: no NaN, but failures are in magnitude/cross-axis tests**
- Symmetry tests (4) — All pass
- Several sliding/compound tests pass

#### Failing Tests (3)

1. **Compound_SuperpositionMagnitude**
   - Compound displacement at only **26.6%** of superposition prediction
   - Single-X mag = 0.0417m, Single-Y mag = 0.0417m
   - Superposition prediction = 0.0589m
   - **Actual compound = 0.0157m** (suppressed)

2. **Sliding_ThrownCube_SDLAppConfig** (CRITICAL)
   - Config: tilt = `(π/3, 0.01, 0)`, velocity = `(5, 0, 0)`, friction = 0.5
   - **Spurious Y displacement = 20.7m** (should be ~0)
   - Total X displacement = 57.4m
   - Y/X ratio = 36.1% (massive cross-axis motion)
   - Peak vy = 4.27 m/s

3. **Sliding_PurePitch_vs_CompoundTilt_SpuriousY**
   - Pure pitch (π/3, 0, 0): X = 2.72m, Y ≈ 0
   - Compound (π/3, 0.01, 0): X = 57.4m, Y = -20.7m
   - **Compound travels 21x farther than pure pitch**
   - Massive energy injection from tiny 0.01 rad perturbation

### Phase 1 Answers

| Question | Answer | Evidence |
|----------|--------|----------|
| Which tilt orientations fail? | **None fail NaN-wise**, but compound tilts cause **energy injection** | All T1-T8 pass, failures are in magnitude/cross-axis tests |
| Is failure in direction or magnitude? | **Both** — spurious cross-axis motion (direction) AND 21x energy injection (magnitude) | Sliding_ThrownCube: Y = 20.7m from 0.01 rad, 21x travel distance |
| Do symmetry tests fail? | **No** — symmetry is preserved | All 4 symmetry tests pass |
| Does failure depend on friction? | **Yes, suspected** — zero lateral displacement in dropped cubes suggests friction is involved | T1-T8 show minimal displacement, sliding tests show massive errors |
| Does failure depend on tilt magnitude? | **Yes** — 0.01 rad perturbation sufficient to trigger | Pure pitch (π/3, 0, 0) correct, compound (π/3, 0.01, 0) catastrophic |

### Key Findings

1. **The bug is NOT in basic tilt handling** — T1-T8 all pass without NaN
2. **The bug IS in cross-axis coupling when velocity + tilt break symmetry**
3. **Tiny perturbations (0.01 rad) cause massive energy injection** (21x amplification)
4. **Spurious Y displacement of 20.7m from 0.01 rad perturbation** — this is unreasonable
5. **Symmetry is preserved** — suggests the bug is deterministic, not a numerical instability

### Hypothesis Ranking Update

Based on Phase 1 results, re-rank hypotheses:

1. **H1: EPA Normal Lateral Perturbation** — MOST LIKELY
   - Small tilt → small lateral normal component → tangent basis rotation → friction pushes wrong direction
   - Explains why pure pitch works but compound fails

2. **H2: Contact Point Asymmetry** — POSSIBLE
   - EPA vs SAT fallback may produce different contact points for tilted cubes

3. **H4: Sign Convention Inconsistency** — LESS LIKELY
   - Symmetry tests pass, so signs are consistent across configurations

4. **H3: ReferenceFrame Overload Bug** — UNLIKELY FOR THIS CASE
   - Would affect all tests uniformly, but we see selective failures

---

## Phase 2: Isolate the Source

### Phase 2a: EPA Contact Normal Accuracy

**Hypothesis (H1)**: For a tilted cube on a flat floor, the true contact normal should be purely vertical `(0, 0, 1)`. But EPA extracts the normal from the Minkowski difference, and for a tilted cube the support mapping may produce a normal with small lateral components. These lateral components could rotate the tangent basis away from the floor plane, causing friction to act partially in the normal direction.

**Diagnostic Strategy**:
1. Add logging to `CollisionHandler::checkCollision()` to capture EPA normal for first contact frame
2. Run failing test (`Sliding_ThrownCube_SDLAppConfig`) with logging enabled
3. Compare EPA normal to expected `(0, 0, 1)`
4. If normal has lateral components, compute tangent basis and check if tangents are in floor plane

**Implementation Plan**:
1. Create diagnostic test variant with collision logging
2. Extract EPA normal at first contact
3. Compute tangent basis from normal (using `tangent_basis::computeTangentBasis`)
4. Verify tangent vectors are perpendicular to Z-axis (floor plane)

### Subsystems to Investigate (in order)

1. **EPA Contact Normal Accuracy** — IN PROGRESS
2. Tangent Basis Alignment
3. Contact Point Location
4. Friction Jacobian and Impulse Direction
5. Constraint Solver Flattening

---

## Phase 3: Root Cause Documentation

_Will be populated after isolation_

### Root Cause Summary

**Status**: Not yet identified

**Code Location(s)**: TBD

**Mathematical Explanation**: TBD

**Proposed Fix Strategy**: TBD

---

## Iteration Log

### Iteration 1: Phase 1 Execution and Initial Findings
**Date**: 2026-02-10
**Actions**:
- Created investigation artifacts directory and initialized log
- Ran full TiltedCubeTrajectory test suite (19 tests)
- Analyzed failure pattern
- Reviewed tangent basis construction algorithm (Duff et al. 2017)
- Reviewed EPA/SAT collision detection architecture
- Created diagnostic program skeleton for EPA normal extraction

**Results**:
- 16/19 tests PASS
- 3/19 tests FAIL with massive energy injection (21x amplification from 0.01 rad perturbation)
- Bug is NOT in basic tilt handling (all orientation tests pass)
- Bug IS in cross-axis coupling when velocity + asymmetric tilt break symmetry
- Symmetry is preserved (deterministic bug, not numerical instability)

**Key Finding**: Tiny 0.01 rad perturbation causes:
- 20.7m spurious Y displacement (should be ~0)
- 21x travel distance vs pure pitch baseline
- Peak Y velocity of 4.27 m/s from ~0 initial

**Hypothesis Ranking**: H1 (EPA Normal Lateral Perturbation) is MOST LIKELY

**Next Steps**:
- Add temporary logging to capture EPA normals during failing test
- Compare EPA normal to expected (0, 0, 1) for floor contact
- Verify if lateral normal components cause tangent basis misalignment
- If confirmed, determine if EPA or SAT fallback is the source
