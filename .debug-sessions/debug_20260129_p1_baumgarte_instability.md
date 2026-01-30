# Debug Session: P1 Baumgarte Instability with alpha=100

## Problem Description

**Symptom**: Prototype P1 (PGS stacked objects) exhibits "explosive instability" when using the design-recommended Baumgarte stabilization parameter `alpha=100`. The prototype only works with `alpha=0.2`.

**Source**: Design document (`design.md`) recommends `alpha=100, beta=20`. Math formulation (`math-formulation.md`) presents `alpha` in units of [1/s²] and `beta` in units of [1/s].

**Suspect Files**:
- `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/main.cpp` (line 21)
- `docs/designs/0032_contact_constraint_refactor/design.md` (line 174)
- `docs/designs/0032_contact_constraint_refactor/math-formulation.md` (Section 8)

---

## Investigation Log

### Hypothesis H1: Unit Mismatch Between Design and Prototype

**Status**: ❌ Confirmed as root cause

**Evidence gathered**:

The math formulation defines Baumgarte at the **acceleration level** (Section 8.2):

$$
\ddot{C} + \beta \dot{C} + \alpha C = 0
$$

With α in [1/s²] and β in [1/s]. The modified RHS is (Section 8.3):

$$
A λ = b - α·C - β·Ċ
$$

However, the **prototype code** (line 178-182) uses a **velocity-level** formulation:

```cpp
b(i) = -(1.0 + effectiveE) * gapVel;
if (gap < 0.0) {
    b(i) -= (alpha / dt) * gap;
}
```

This is: `b = -(1+e)·Ċ - (α/dt)·C`

The prototype intentionally rescales alpha, interpreting it as a **dimensionless fraction per dt** (see comment on line 21: "fraction per dt"). The division by `dt` converts a fraction (0.2) into a velocity-level correction rate.

**The design document's alpha=100 has units [1/s²]**. But the prototype code uses **alpha as a dimensionless fraction**, then divides by dt.

Let's trace the math:
- Design: α = 100 [1/s²], applied as `-α·C` in acceleration-level equation
- Prototype: α = 0.2 [dimensionless], applied as `-(α/dt)·C` = `-(0.2/0.016)·C` = `-12.5·C` [1/s]

These are velocity-level bias terms. The question is: what happens if you plug α=100 into the prototype's formula?

`-(100/0.016)·C = -6250·C` — this is a correction gain of **6250 /s**, which is extremely stiff.

Compare to the math formulation's practical values (Section 8.4):
- α_stiff = 4/dt² = 15625 [1/s²]
- β_stiff = 4/dt = 250 [1/s]

The design's α=100 [1/s²] should appear in the acceleration-level equation as `-α·C`.

To convert to velocity level (impulse formulation), we multiply by dt:
- Velocity-level bias = α·C·dt = 100 × C × 0.016 = 1.6·C per timestep

But the prototype computes `(α/dt)·C = (100/0.016)·C = 6250·C` — **dividing instead of multiplying**.

**This is the dimensional mismatch.**

---

### Hypothesis H2: The Prototype's alpha/dt Formula Is Correct But Uses Wrong Unit for alpha

**Status**: ✅ Confirmed — this is the same root cause as H1, different angle

**Evidence**:

The prototype comment on line 178 says:
```cpp
// RHS: b = -(1+e) * Cdot - (alpha/dt) * min(C, 0)
// Note: alpha/dt converts position error to velocity correction
```

This formula makes sense if alpha is a **dimensionless position correction fraction** (0 to 1):
- alpha=0.2 means "correct 20% of penetration per timestep"
- `(alpha/dt)·C` = `(0.2/0.016)·C` = `12.5·C` [m/s per m of penetration]

The design's alpha=100 is in **[1/s²]** units. To convert from design alpha to prototype alpha:

From the acceleration-level equation: `-α·C` contributes to `Ċ` change as `-α·C·dt`
In the velocity-level impulse formulation: the bias term is `α·C·dt`
The prototype formula is `(α_proto/dt)·C`

Setting equal: `α_proto/dt = α_design·dt`
Therefore: `α_proto = α_design·dt² = 100 × 0.016² = 0.0256`

So the design's α=100 [1/s²] maps to a prototype dimensionless fraction of **0.0256**, which is close to the range where the prototype is stable (α_proto=0.2 is actually higher — but still orders of magnitude lower than plugging 100 directly).

If you naively plug α=100 into the prototype formula:
- `(100/0.016)·C = 6250·C` — this is equivalent to α_design = 100/dt² = 100/0.000256 = **390,625 [1/s²]**
- That's 25x stiffer than the "stiff" theoretical value of 15,625!

---

### Hypothesis H3: Is the Prototype's Velocity-Level Formulation Itself Correct?

**Status**: ✅ Confirmed — prototype uses standard ERP formulation

The prototype operates at the **velocity level** (impulse formulation), not the acceleration level. This is standard for PGS solvers.

In an impulse-based PGS solver, the system being solved is:

$$
A·λ = b
$$

Where λ are **impulses** (force × dt), and b is the **velocity-level bias**.

The standard velocity-level Baumgarte bias is:

$$
b_{Baumgarte} = \frac{\beta_{ERP}}{\Delta t} \cdot C
$$

Where β_ERP is the **Error Reduction Parameter** (dimensionless, 0 to 1), commonly used in ODE, Bullet, and Box2D.

This matches the prototype's formula: `(alpha/dt) * C` where alpha = β_ERP = 0.2.

So the prototype is using the **ERP formulation** (standard in physics engines), while the design document uses the **Baumgarte ODE formulation** (standard in academic literature).

**Conclusion**: The prototype formulation is correct for a velocity-level solver. The design document's α=100 [1/s²] is an acceleration-level parameter that cannot be directly plugged into a velocity-level solver.

---

### Hypothesis H4: Does alpha=100 Actually Cause "Explosive" Instability?

**Status**: ✅ Resolved — NOT explosive, but FAILS quality criteria

**Evidence from diagnostic tests**:

Test A (alpha=100 ERP, 1000 frames):
- **Stable**: YES (no NaN, no explosion)
- **Max drift**: 0.0414 m (FAILS P1 criterion of < 0.01m)
- **Max velocity**: 0.825 m/s (large oscillation)
- **Energy range**: 1.18 J (PASSES < 5.0J criterion)

Test B (alpha=100 ERP, 10000 frames):
- **Stable**: YES
- **Max drift grows**: 0.077 m (worse at longer time, growing slowly)
- **Max velocity**: 1.13 m/s

**Conclusion**: The P1 README's characterization of alpha=100 as "explosive instability" is an **overstatement**. The system is numerically stable — it does not diverge to infinity or produce NaN. However, with alpha=100 in the ERP slot, the excessive stiffness causes:
1. Large oscillations around the rest position (max velocity 0.83 m/s vs 0.009 m/s for alpha=0.2)
2. Significant drift (4.1 cm vs 0.075 cm) — **4× above** the P1 position tolerance
3. Growing cumulative error over longer simulations

This is **solution quality degradation** due to overshoot, not explosion.

---

## Root Cause Summary

**The behavior with alpha=100 is caused by a unit/formulation mismatch:**

| Aspect | Math Formulation (Section 8) | Design Document | Prototype Code |
|--------|------------------------------|-----------------|----------------|
| **Formulation** | Acceleration-level ODE | References accel-level | Velocity-level (ERP) |
| **Parameter name** | α [1/s²], β [1/s] | alpha=100, beta=20 | alpha (dimensionless ERP) |
| **Baumgarte term** | −α·C − β·Ċ | Same | −(ERP/dt)·C |
| **Conversion** | N/A | N/A | ERP = α_accel · dt² |

1. The **math formulation** (Section 8) defines Baumgarte at the acceleration level: `C̈ + β·Ċ + α·C = 0` with α in [1/s²], β in [1/s]

2. The **design document** recommends `alpha=100, beta=20` in these acceleration-level units

3. The **prototype** implements a velocity-level (impulse) PGS solver, where the standard Baumgarte correction is the ERP formulation: `b_bias = (ERP/dt)·C` with ERP as a dimensionless fraction (0 to 1)

4. Plugging α=100 into the prototype's `(α/dt)·C` formula yields an effective correction gain of 6250/s, equivalent to α_accel ≈ 390,625 [1/s²] — 25× stiffer than the theoretical maximum for dt=0.016s

**The mapping between formulations is:**
- `ERP = α_accel · dt²` (acceleration-level → velocity-level)
- `α_accel = ERP / dt²` (velocity-level → acceleration-level)

| Design Value | Converted ERP | Prototype Value | Converted α_accel |
|-------------|---------------|-----------------|-------------------|
| α=100 [1/s²] | 0.0256 | α=0.2 (ERP) | 781.3 [1/s²] |

**Classification**: Not a bug in the prototype code. Not a fundamental mathematical issue. It is a **parameter interpretation problem** — the design document's parameters are in acceleration-level units, while the prototype uses velocity-level (ERP) parameters, and the P1 agent used the design value directly without conversion.

**Severity**: The "explosive instability" reported by P1 is overstated. alpha=100 in the ERP slot causes oscillation and 4× drift violation, but the system remains bounded. It is a quality failure, not a stability failure.

---

## Diagnostic Test Results

### Test 1: alpha=100 ERP (design value, wrong units)
- Effective gain: 6250 /s
- Equivalent α_accel: 390,625 /s²
- **Stable: YES** (not explosive)
- Max drift: 0.041 m (FAILS < 0.01m criterion)
- Max velocity: 0.825 m/s (large oscillation)

### Test 2: ERP=0.0256 (correct conversion from α_accel=100)
- Effective gain: 1.6 /s
- Equivalent α_accel: 100.0 /s²
- **Stable: YES, PASSES** all P1 criteria
- Max drift: 0.0058 m (< 0.01m)
- Energy range: 0.149 J

### Test 3: Parameter Sweep (ERP values)

| ERP | Gain [1/s] | Equiv α [1/s²] | Stable | Max Drift [m] | Max Vel [m/s] | Energy Range [J] |
|-----|-----------|-----------------|--------|---------------|---------------|-------------------|
| 0.01 | 0.6 | 39.1 | YES | 0.013 | 0.009 | 0.336 |
| 0.0256 | 1.6 | 100.0 | YES | 0.0058 | 0.009 | 0.149 |
| 0.1 | 6.2 | 390.6 | YES | 0.0015 | 0.009 | 0.036 |
| **0.2** | **12.5** | **781.3** | **YES** | **0.00075** | **0.009** | **0.016** |
| 0.4 | 25.0 | 1562.5 | YES | 0.00038 | 0.009 | 0.006 |
| 1.0 | 62.5 | 3906.2 | YES | 0.00016 | 0.009 | 0.001 |
| 2.0 | 125.0 | 7812.5 | YES | 0.00014 | 0.009 | 0.003 |
| 5.0 | 312.5 | 19531.2 | YES | 0.00040 | 0.034 | 0.016 |
| 10.0 | 625.0 | 39062.5 | YES | 0.0011 | 0.077 | 0.039 |
| 50.0 | 3125.0 | 195312.5 | YES | 0.012 | 0.408 | 0.374 |
| 100.0 | 6250.0 | 390625.0 | YES | 0.041 | 0.825 | 1.177 |

**Observations**:
- The system is numerically stable across the entire range (0.01 to 100.0)
- **Optimal ERP range: 0.2–1.0** (minimum drift, minimum velocity oscillation)
- Below ERP=0.1: drift increases (too soft to correct penetration)
- Above ERP=2.0: oscillation begins, drift increases (overcorrection/overshoot)
- ERP=0.2 is a well-chosen default (excellent drift/energy balance)

### Test 4: Long-Run Stability (10,000 frames)

| ERP | 1000-frame drift | 10000-frame drift | Growing? |
|-----|------------------|--------------------|----------|
| 100.0 | 0.041 m | 0.077 m | Yes (slowly) |
| 0.2 | 0.00075 m | 0.00075 m | No (converged) |
| 0.0256 | 0.0058 m | 0.0059 m | No (converged) |
| 1.0 | 0.00016 m | 0.00016 m | No (converged) |
| 2.0 | 0.00014 m | 0.00014 m | No (converged) |

**Key finding**: ERP=100 shows slowly growing error over time, while all reasonable ERP values converge quickly.

---

## Recommendations

### For Design Document Updates

1. **Add velocity-level (ERP) parameterization** to the design document alongside the acceleration-level formulation. The implementation will use ERP, not α_accel.

2. **Conversion formulas** should be documented:
   ```
   ERP = α_accel · dt²    (acceleration-level → velocity-level)
   α_accel = ERP / dt²    (velocity-level → acceleration-level)
   ```

3. **Recommended default**: ERP = 0.2 (equivalent to α_accel ≈ 781 [1/s²] at 60 FPS)

4. **Remove or revise** the design document's `alpha=100, beta=20` recommendation. Replace with ERP=0.2 as the implementation parameter.

### For Math Formulation Updates

5. **Section 8** should include the velocity-level discretization explicitly, showing how the continuous Baumgarte ODE maps to the discrete impulse-level bias term used by PGS.

### For P1 README Correction

6. The P1 README should correct the characterization of alpha=100 as "explosive instability". It causes oscillation and drift violation, but the system remains bounded.

---

## Diagnostic Files

- `diagnostic_test.cpp` — Parameter sweep and conversion verification
- `diagnostic_longrun.cpp` — Long-run stability comparison (10,000 frames)
- Both in `prototypes/0032_contact_constraint_refactor/p1_pgs_convergence/`
