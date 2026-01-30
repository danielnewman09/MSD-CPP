# Debug Findings: P1 Baumgarte Parameter Instability

## Problem Statement

Prototype P1 (PGS stacked objects) reports "explosive instability" when using the design-recommended Baumgarte stabilization parameter `alpha=100`. The prototype only produces correct results with `alpha=0.2`.

- **Design document** (`design.md` line 174): recommends `alpha=100, beta=20`
- **Math formulation** (`math-formulation.md` Section 8): defines $\alpha$ in [1/s$^2$], $\beta$ in [1/s]
- **Prototype code** (`main.cpp` line 21): uses `alpha=0.2` labeled "fraction per dt"

---

## Root Cause: Unit/Formulation Mismatch

The design document and the prototype use **two different parameterizations** of Baumgarte stabilization. The design value cannot be plugged directly into the prototype formula.

### The Two Formulations

**Math formulation (acceleration-level)** — Section 8.2:

$$
\ddot{C} + \beta \dot{C} + \alpha C = 0
$$

Parameters: $\alpha$ [1/s$^2$], $\beta$ [1/s]. The stabilized RHS (Section 8.3):

$$
\mathbf{A} \boldsymbol{\lambda} = \mathbf{b} - \alpha C - \beta \dot{C}
$$

**Prototype code (velocity-level / ERP)** — `main.cpp` line 178:

```cpp
b(i) = -(1.0 + effectiveE) * gapVel;
if (gap < 0.0) {
    b(i) -= (alpha / dt) * gap;  // alpha is dimensionless ERP
}
```

This is the **Error Reduction Parameter (ERP)** formulation standard in physics engines (ODE, Bullet, Box2D). Here `alpha` is a dimensionless fraction (0 to 1) representing how much penetration to correct per timestep.

### The Conversion

To map between formulations:

$$
\text{ERP} = \alpha_{\text{accel}} \cdot \Delta t^2
$$

$$
\alpha_{\text{accel}} = \frac{\text{ERP}}{\Delta t^2}
$$

| Value | Source | Converted |
|-------|--------|-----------|
| $\alpha_{\text{accel}} = 100$ [1/s$^2$] | Design document | ERP $= 100 \times 0.016^2 = 0.0256$ |
| ERP $= 0.2$ | Prototype code | $\alpha_{\text{accel}} = 0.2 / 0.016^2 = 781.3$ [1/s$^2$] |

### What Happens When alpha=100 Is Used Directly

Plugging $\alpha = 100$ into the ERP formula:

$$
\frac{100}{0.016} \cdot C = 6250 \cdot C
$$

This is equivalent to $\alpha_{\text{accel}} = 100 / 0.016^2 = 390{,}625$ [1/s$^2$], which is **25$\times$ stiffer** than the theoretical maximum ($4/\Delta t^2 = 15{,}625$ [1/s$^2$]).

---

## Diagnostic Evidence

### Finding 1: alpha=100 Does Not Actually Cause Explosion

Contrary to the P1 README's claim of "explosive instability", alpha=100 in the ERP slot produces a **bounded but oscillatory** solution:

| Metric | alpha=100 (ERP) | alpha=0.2 (ERP) |
|--------|-----------------|-----------------|
| Numerically stable | Yes | Yes |
| Max drift (1000 frames) | 0.041 m | 0.00075 m |
| Max drift (10,000 frames) | 0.077 m | 0.00075 m |
| Max velocity | 0.825 m/s | 0.009 m/s |
| Energy range | 1.18 J | 0.016 J |
| P1 position criterion (<0.01m) | **FAIL** (4$\times$ over) | **PASS** |

The system remains bounded at all times — no NaN, no divergence. It is **solution quality degradation** from overcorrection overshoot, not instability.

### Finding 2: Correct Conversion Passes All Criteria

Converting the design's $\alpha_{\text{accel}} = 100$ to ERP = 0.0256 produces a stable, passing result:

- Max drift: 0.0058 m (passes < 0.01m)
- Energy range: 0.149 J (passes < 5.0J)
- Converges within 10,000 frames (no growing error)

### Finding 3: Parameter Sweep Identifies Optimal Range

18 ERP values tested across 200–10,000 frames:

| ERP | Gain [1/s] | Equiv $\alpha$ [1/s$^2$] | Max Drift [m] | Max Vel [m/s] | Energy Range [J] |
|-----|-----------|--------------------------|---------------|---------------|-------------------|
| 0.01 | 0.6 | 39 | 0.013 | 0.009 | 0.336 |
| 0.0256 | 1.6 | 100 | 0.0058 | 0.009 | 0.149 |
| 0.1 | 6.2 | 391 | 0.0015 | 0.009 | 0.036 |
| **0.2** | **12.5** | **781** | **0.00075** | **0.009** | **0.016** |
| 0.4 | 25.0 | 1,563 | 0.00038 | 0.009 | 0.006 |
| 1.0 | 62.5 | 3,906 | 0.00016 | 0.009 | 0.001 |
| 2.0 | 125.0 | 7,813 | 0.00014 | 0.009 | 0.003 |
| 5.0 | 312.5 | 19,531 | 0.00040 | 0.034 | 0.016 |
| 10.0 | 625.0 | 39,063 | 0.0011 | 0.077 | 0.039 |
| 50.0 | 3,125.0 | 195,313 | 0.012 | 0.408 | 0.374 |
| 100.0 | 6,250.0 | 390,625 | 0.041 | 0.825 | 1.177 |

**Optimal ERP range: 0.2–1.0**
- Below 0.1: too soft, penetration drift grows
- 0.2–1.0: minimum drift and velocity, excellent energy conservation
- Above 2.0: overcorrection oscillation begins
- Above 10.0: visible quality degradation

### Finding 4: Long-Run Convergence

| ERP | 1,000-frame drift | 10,000-frame drift | Converged? |
|-----|-------------------|--------------------|------------|
| 0.0256 | 0.0058 m | 0.0059 m | Yes |
| 0.2 | 0.00075 m | 0.00075 m | Yes |
| 1.0 | 0.00016 m | 0.00016 m | Yes |
| 2.0 | 0.00014 m | 0.00014 m | Yes |
| 100.0 | 0.041 m | 0.077 m | No (growing) |

All reasonable ERP values converge quickly. ERP=100 shows slowly growing cumulative error.

---

## Classification

| Aspect | Assessment |
|--------|------------|
| Bug in prototype code? | **No** — ERP formulation is correct and standard |
| Fundamental math issue? | **No** — both formulations are valid |
| Parameter scaling problem? | **Yes** — design uses accel-level units, prototype uses ERP |
| "Explosive instability"? | **Overstated** — bounded oscillation, not divergence |

---

## Recommendations

### Design Document Updates

1. **Add ERP parameterization** alongside the acceleration-level formulation. The implementation uses a velocity-level PGS solver, so ERP is the relevant parameter.

2. **Document conversion formulas**:
   ```
   ERP = α_accel · dt²    (acceleration-level → velocity-level)
   α_accel = ERP / dt²    (velocity-level → acceleration-level)
   ```

3. **Revise default recommendation** from `alpha=100, beta=20` (acceleration-level) to `ERP=0.2` (velocity-level). This is equivalent to $\alpha_{\text{accel}} \approx 781$ [1/s$^2$] at 60 FPS.

### Math Formulation Updates

4. **Section 8** should include the velocity-level discretization explicitly, showing how the continuous Baumgarte ODE $\ddot{C} + \beta\dot{C} + \alpha C = 0$ maps to the discrete impulse-level bias $b_{\text{Baumgarte}} = \frac{\text{ERP}}{\Delta t} \cdot C$ used by PGS solvers.

### P1 README Correction

5. Replace "explosive instability" with "oscillation and drift violation" — the system remains bounded.

---

## Diagnostic Files

| File | Purpose |
|------|---------|
| `diagnostic_test.cpp` | Parameter sweep (18 ERP values) and unit conversion verification |
| `diagnostic_longrun.cpp` | Long-run stability comparison (10,000 frames, 6 configurations) |
