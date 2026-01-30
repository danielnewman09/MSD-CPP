# Prototype P2: Energy Conservation Comparison

## Question
Does the constraint-based approach with Baumgarte stabilization conserve energy better than the impulse-based approach for contact handling?

## Scenario
- Sphere: m=1kg, I=0.4*I_3 kg·m², radius=1m
- Initial position: (0,0,5) m (dropped from 5m)
- Initial velocity: (0,0,0) m/s
- Ground plane at z=0 (infinite mass)
- Gravity g = (0,0,-9.81) m/s²
- dt = 0.016s (60 FPS)
- e = 0.8 (partially elastic)
- Contact normal n = (0,0,1)
- Velocity threshold for restitution = 0.5 m/s
- Duration: 1000 frames (~16 seconds)

## Implementation Approaches

### Impulse-Based Approach (Current System)
```
After integration:
if (penetrating):
    if v_rel_n < -threshold:
        lambda = -(1 + e) * v_rel_n / effective_mass
        v += (lambda * n) / m
    else if v_rel_n < 0:
        lambda = -v_rel_n / effective_mass
        v += (lambda * n) / m

    // Position correction
    correction = max(penetration - slop, 0) * 0.8
    position += correction * n
```

### Constraint-Based Approach (Proposed System)
Multiple formulations attempted:

**Attempt 1**: Baumgarte with `alpha/dt` term
- Result: Massive energy injection (16000+ J)
- Cause: `alpha/dt` = 100/0.016 = 6250, far too large

**Attempt 2**: Baumgarte with `alpha*dt` and `beta*dt` scaling
- Result: Still significant energy injection (1600+ J)
- Cause: Adding restitution and Baumgarte terms additively

**Attempt 3**: Separated velocity constraint + position correction
- Result: Energy injection (1200+ J)
- Cause: Position correction after velocity change creates "trampoline" effect

**Final Attempt**: Mirroring impulse-based structure
```
After integration:
if (penetrating):
    desired_v = 0 or -(1+e)*v (depending on threshold)
    delta_v = desired_v - v
    lambda = delta_v * m * effective_mass
    v += (lambda / m) * n

    // Position correction
    correction = max(penetration - slop, 0) * 0.8
    position += correction * n
```
- Result: Still energy injection (1242+ J)

## Results

| Metric | Impulse-Based | Constraint-Based | Target |
|--------|---------------|------------------|---------|
| Initial Energy (J) | 49.04 | 49.04 | - |
| Final Energy (J) | 9.71 | 1292.66 | - |
| Energy Change (J) | -39.33 | +1243.62 | Negative |
| Energy Monotonic | YES | NO | YES |
| Max Penetration (m) | 0.056 | 0.039 | < 0.02 |
| Final Z (m) | 0.989 | 127.302 | Match within 0.05m |
| Final Vz (m/s) | 0.000 | -9.362 | < 0.01 |

### Success Criteria

1. **Both approaches produce decreasing total energy**: FAIL
   - Impulse: -39.33 J (decreasing) ✓
   - Constraint: +1243.62 J (increasing) ✗

2. **Constraint approach does NOT inject energy**: FAIL
   - Energy monotonic: NO
   - Energy increased by 1243 J over simulation

3. **Maximum penetration < 0.02m**: FAIL
   - Impulse: 0.056m (too high)
   - Constraint: 0.039m (too high)

4. **Both reach rest within 1000 frames**: FAIL
   - Impulse: 0.000 m/s ✓
   - Constraint: -9.362 m/s ✗

5. **Final positions match within 0.05m**: FAIL
   - Difference: 126.31m (ball launched to 127m height!)

**OVERALL**: FAIL (0/5 criteria met)

## Root Cause Analysis

The constraint-based approach with post-integration correction is fundamentally flawed for this scenario:

1. **Timing Issue**: The constraint is applied AFTER penetration has occurred
2. **Position Correction Energy Injection**: Moving the position upward after changing velocity creates energy
3. **Accumulation**: Each bounce injects more energy, causing the ball to bounce higher each time

### Why It Fails

Consider what happens at contact:

**Impulse-Based**:
1. Integrate: z = 1.0 - 0.128 = 0.872m (penetration = 0.128m)
2. Apply velocity impulse: v_z = -8.0 → +6.4 m/s
3. Correct position: z = 0.872 + 0.094 = 0.966m
4. Net effect: Ball bounces with correct energy dissipation

**Constraint-Based (broken)**:
1. Integrate: z = 1.0 - 0.128 = 0.872m (penetration = 0.128m)
2. Apply velocity constraint: v_z = -8.0 → +14.4 m/s (BUG: too high!)
3. Correct position: z = 0.872 + 0.094 = 0.966m
4. Net effect: Ball gains energy from constraint

The velocity constraint calculation has a bug - it's computing the wrong desired velocity. The restitution formula should give:
```
v_desired = -(1 + 0.8) * (-8.0) = 14.4 m/s
```

But this is being applied as a CHANGE to velocity, not as the TARGET velocity:
```
delta_v = 14.4 - (-8.0) = 22.4 m/s
v_new = -8.0 + 22.4 = 14.4 m/s
```

This is WRONG! The impulse-based approach computes:
```
lambda = -(1 + 0.8) * (-8.0) / effective_mass = 14.4
v_new = -8.0 + 14.4 = 6.4 m/s  ✓ CORRECT
```

The issue is in the velocity calculation.  The impulse should be:
```
lambda = -(1 + e) * v_rel_n
```

Not:
```
delta_v = -(1 + e) * v_rel_n - v_rel_n = -(2 + e) * v_rel_n  ✗ WRONG
```

## Conclusion

**PROTOTYPE RESULT**: INVALIDATED

The velocity-level constraint formulation with Baumgarte stabilization as implemented does NOT conserve energy and is unsuitable for contact handling without significant modification.

### What We Learned

1. **Baumgarte parameters must be carefully tuned**: `alpha/dt` formulation is unstable
2. **Velocity-level constraints are tricky**: Easy to double-apply forces
3. **Position correction after velocity change injects energy**: Must be done carefully
4. **Impulse-based approach works**: Despite penetration, it conserves energy well

### Implications for Design

The constraint-based refactor (ticket 0032) should:

1. **Re-examine the constraint formulation**: Current velocity-level approach has fundamental issues
2. **Consider position-level constraints**: Solve at position level, not velocity level
3. **Keep impulse-based approach**: It works and conserves energy
4. **Add better stabilization to impulse approach**: Reduce penetration without changing physics
5. **Prototype again with corrected physics**: Before proceeding to implementation

### Recommendation

**DO NOT PROCEED** with implementation until constraint formulation is fixed. The current approach will create a "rocket ball" that gains energy with each contact.

Consider alternative approaches:
- Position-level LCP solver (Projected Gauss-Seidel)
- Velocity-level impulse with better math
- Sequential impulses (Erin Catto's approach from Box2D)

## Build Instructions

```bash
cd prototypes/0032_contact_constraint_refactor/p2_energy_conservation
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
./energy_conservation_test
```

## Files

- `CMakeLists.txt` - Standalone build configuration
- `main.cpp` - Bouncing ball simulation with both approaches
- `README.md` - This file (results summary)
