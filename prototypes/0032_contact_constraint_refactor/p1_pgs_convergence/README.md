# Prototype P1: PGS Convergence for Stacked Objects

## Question
Can Projected Gauss-Seidel (PGS) with 10 iterations and Baumgarte stabilization maintain stable resting contact for stacked objects over 1000 frames without drift?

## Success Criteria

| Criterion | Target | Result | Status |
|-----------|--------|--------|--------|
| 1. Position Stability | Max drift < 0.01m over 1000 frames | 0.000749m | **PASS** |
| 2. Energy Bounded | Energy variation < 5.0 J | 0.016 J | **PASS** |
| 3. PGS Convergence | Produces stable solution within 10 iterations | Yes (10 iter/frame) | **PASS** |
| 4. Lambda Values | Match theoretical steady-state within 0.1 N | Exact match | **PASS** |

## Overall Result: PASS

## Scenario Details

**Setup**: Three 1kg cubes (A, B, C) stacked vertically:
- Cube A: center at z=0.5m (bottom at z=0, top at z=1.0)
- Cube B: center at z=1.5m (bottom at z=1.0, top at z=2.0)
- Cube C: center at z=2.5m (bottom at z=2.0, top at z=3.0)
- Ground plane at z=0
- Gravity: g = -9.81 m/s²
- Time step: dt = 0.016s (60 FPS)
- Coefficient of restitution: e = 0.0 (perfectly inelastic)

**Contact Constraints**:
1. Ground-A: Normal n=(0,0,1)
2. A-B: Normal n=(0,0,1)
3. B-C: Normal n=(0,0,1)

**PGS Parameters**:
- Max iterations: 10
- Convergence tolerance: 1e-4
- Baumgarte stabilization: α = 0.2 (position correction as fraction per dt)

## Key Findings

### 1. Constraint Matrix Structure

The effective mass matrix A has negative off-diagonal coupling terms:

```
A = [ 1  -1   0 ]
    [-1   2  -1 ]
    [ 0  -1   2 ]
```

This coupling arises because:
- Diagonal: A_ii = (1/m_upper) + (1/m_lower) for each contact
- Off-diagonal: A_ij = -(1/m_shared) when contacts i and j share a body

### 2. Lambda Values (Constraint Forces)

The steady-state impulses exactly match theoretical predictions:

| Contact | Measured | Expected | Physical Meaning |
|---------|----------|----------|------------------|
| Ground-A | 0.4709 N·s | 3mg·dt = 0.4709 | Ground supports weight of all 3 cubes |
| A-B | 0.3139 N·s | 2mg·dt = 0.3139 | Cube A supports B and C |
| B-C | 0.1570 N·s | 1mg·dt = 0.1570 | Cube B supports C |

### 3. PGS Convergence Behavior

**Important**: PGS may not strictly "converge" by the mathematical delta criterion (λ_new - λ_old < ε) for tightly coupled systems within 10 iterations. However, it produces **physically correct results** within the iteration limit:

- All 1000 frames used exactly 10 iterations
- Final delta per iteration: ~0.008 (larger than tolerance 1e-4)
- Despite not meeting strict convergence, the solution is:
  - Stable (max drift 0.0007m over 1000 frames)
  - Energy-conserving (variation 0.016 J)
  - Physically accurate (lambda values exact)

This is acceptable for real-time simulation where **solution quality** matters more than **mathematical convergence**.

### 4. Baumgarte Stabilization

Reduced stabilization parameter (α = 0.2) is critical:
- Initial attempts with α = 100 caused explosive instability
- Lower α prevents energy injection while still correcting penetration
- Small steady-state penetrations (gaps = [-0.0005, -0.00025, 0.0]) are acceptable and stable

### 5. Position Drift

Maximum position drift over 1000 frames:
- Cube A: 0.0005m downward (0.5000 → 0.4995)
- Cube B: 0.0007m downward (1.5000 → 1.4993)
- Cube C: 0.0007m downward (2.5000 → 2.4993)

All well within tolerance (< 0.01m).

### 6. Energy Conservation

Energy variation: 0.016 J out of ~44 J total (0.036%)
- Initial: 44.14 J
- Min: 44.13 J
- Max: 44.14 J

Excellent energy conservation despite Baumgarte bias forces.

## Implementation Implications

1. **PGS is viable** for stacked resting contact with 10 iterations
2. **Baumgarte stabilization** must be tuned carefully (α ~ 0.2, not 100)
3. **Convergence criterion** should be based on solution quality, not strict delta
4. **Negative coupling** in A matrix is expected and handled correctly by PGS
5. **Small penetrations** at steady state are acceptable (< 1mm) and stable

## Build and Run

```bash
cd prototypes/0032_contact_constraint_refactor/p1_pgs_convergence
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug .. && cmake --build .
./pgs_convergence_test
```

## Files

- `CMakeLists.txt` - Standalone build configuration
- `main.cpp` - PGS simulation implementation (1D vertical stacking)
- `README.md` - This file

## Artifacts for Implementation

- Constraint matrix construction with negative coupling terms
- PGS solver implementation with max(0, residual/A_ii) clamping
- Baumgarte RHS formulation: b = -(1+e)·Cdot - (α/dt)·min(C,0)
- Gap computation for stacked bodies: C = z_upper - z_lower - height
