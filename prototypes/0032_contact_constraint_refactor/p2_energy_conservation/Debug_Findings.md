# Debug Findings: P2 Energy Injection ("Rocket Ball")

## Problem Statement

Prototype P2 (energy conservation) injects +1243 J of energy into a bouncing ball instead of dissipating it. The ball launches to 127m height instead of settling to rest. All 5 success criteria FAIL. The P2 README concluded this was a "fundamental flaw" in the velocity-level constraint formulation and recommended not proceeding with implementation.

---

## Root Cause: Single-Line Restitution Formula Bug

The constraint-based code at `main.cpp` line 161 confuses the **constraint RHS** with the **target velocity**:

```cpp
// BUGGY (line 161):
desired_gap_dot = -(1.0 + restitution_) * gap_dot;

// CORRECT:
desired_gap_dot = -restitution_ * gap_dot;
```

### Why This Is Wrong

The math formulation (Section 6.2) gives the constraint RHS as:

$$
b = -(1 + e) \cdot \mathbf{J}\dot{\mathbf{q}}^-
$$

This is the right-hand side of $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$, where $\boldsymbol{\lambda}$ is the **impulse magnitude**. The prototype code misinterprets $b$ as the **desired post-collision velocity**, then subtracts the current velocity to get a delta. This double-counts the pre-velocity contribution.

### Algebraic Proof

For pre-collision velocity $v$ and restitution $e$:

**Impulse-based approach** (correct):
```
lambda = -(1+e)*v / (1/m) = -(1+e)*v*m
v_new  = v + lambda/m = v - (1+e)*v = -e*v  ✓
```

**Constraint-based approach** (buggy):
```
desired = -(1+e)*v          ← this is b, not v_target!
delta   = desired - v = -(2+e)*v
v_new   = v + delta = -(1+e)*v  ✗
```

**Fixed constraint approach**:
```
desired = -e*v              ← correct target velocity
delta   = -e*v - v = -(1+e)*v
v_new   = v + delta = -e*v  ✓
```

### Energy Impact

| Quantity | Correct ($v_{\text{new}} = -ev$) | Buggy ($v_{\text{new}} = -(1+e)v$) |
|----------|----------------------------------|-------------------------------------|
| Speed ratio | $e = 0.8$ | $1+e = 1.8$ |
| KE ratio | $e^2 = 0.64$ (dissipation) | $(1+e)^2 = 3.24$ (injection!) |
| Per-bounce energy | $\times 0.64$ (loses 36%) | $\times 3.24$ (gains 224%) |

Each bounce multiplies kinetic energy by $(1+e)^2 = 3.24$ instead of $e^2 = 0.64$, causing exponential energy growth — the "rocket ball" effect.

---

## Diagnostic Evidence

### Finding 1: First Bounce Trace (Frame 55)

At the first ground contact ($v = -8.79$ m/s):

| Step | Impulse-Based | Constraint-Based (Buggy) |
|------|---------------|--------------------------|
| $v_{\text{pre}}$ | $-8.790$ m/s | $-8.790$ m/s |
| $\lambda$ | $15.822$ | $24.611$ |
| $\Delta v$ | $+15.822$ m/s | $+24.611$ m/s |
| $v_{\text{post}}$ | $+7.032$ m/s | $+15.822$ m/s |
| KE change | $-13.9$ J (dissipation) | $+86.5$ J (injection) |
| Energy after bounce | $34.45$ J | $134.89$ J |

The constraint approach produces $v_{\text{post}} = +15.82$ m/s — faster than the pre-collision speed of $8.79$ m/s. This is non-physical; the ball bounces back harder than it hit.

The impulse approach correctly produces $v_{\text{post}} = +7.03$ m/s $\approx 0.8 \times 8.79$ m/s.

### Finding 2: Position Correction Is Not a Factor

The P2 README's "trampoline effect" theory is incorrect. At the first bounce:
- gap = $-0.008$ m, slop = $0.01$ m
- correction = $\max(0, 0.008 - 0.01) \times 0.8 = 0$

No position correction is applied. The energy injection is **entirely** from the velocity computation.

### Finding 3: All Four P2 Attempts Shared the Same Bug

The README documents 4 attempts with decreasing energy injection (16000 → 1600 → 1200 → 1243 J). All four used the same buggy restitution formula. The improvement across attempts came from removing Baumgarte bias terms, not from fixing the core bug.

### Finding 4: Fix Verification Passes All Criteria

Running the corrected formula (`desired = -e * gap_dot`):

| Metric | Buggy | Fixed | Impulse Reference |
|--------|-------|-------|-------------------|
| Energy change | +1243 J | **-39.3 J** | -39.3 J |
| Energy monotonic | NO | **YES** | YES |
| Final Z | 127.3 m | **0.989 m** | 0.989 m |
| Final Vz | -9.36 m/s | **0.0 m/s** | 0.0 m/s |
| Bounces | continuous | **10** | 10 |

The fixed constraint approach **exactly matches** the impulse-based approach.

Frame-by-frame output of the fixed simulation:
```
Frame    0: z=4.9975 vz=-0.1570 E=49.0377 J
Frame  100: z=3.4555 vz=-0.0314 E=33.8991 J
Frame  200: z=1.9462 vz=-3.2397 E=24.3399 J
Frame  300: z=1.4346 vz=-1.4393 E=15.1094 J
Frame  400: z=1.0014 vz=-0.8437 E=10.1792 J
Frame  500: z=0.9894 vz= 0.0000 E= 9.7057 J  (at rest)
```

---

## Classification

| Aspect | Assessment |
|--------|------------|
| Bug in prototype code? | **Yes** — single-line restitution formula error |
| Fundamental formulation issue? | **No** — the velocity-level approach works correctly |
| Baumgarte-related? | **No** — Baumgarte is not used in the final code |
| Position correction issue? | **No** — correction is zero at first bounce |
| P2 "DO NOT PROCEED" valid? | **No** — should be withdrawn after fix |

---

## Relationship to P1 Findings

The P1 debug (Baumgarte parameter mismatch) and P2 debug (restitution formula bug) are **independent issues**:

| | P1 Issue | P2 Issue |
|---|---|---|
| **Root cause** | Unit mismatch (accel-level vs ERP) | Restitution formula double-count |
| **Location** | Baumgarte bias term | Restitution velocity target |
| **Fix** | Use ERP=0.2 instead of α=100 | Use $-e \cdot v$ instead of $-(1+e) \cdot v$ |
| **Severity** | Quality degradation (oscillation) | Correctness failure (energy explosion) |

The P2 agent's earlier attempts with Baumgarte terms (attempts 1–3) compounded both issues. The final attempt (attempt 4) isolated the restitution bug by removing Baumgarte entirely.

---

## Recommendations

### Code Fix

Change `main.cpp` line 161 from:
```cpp
desired_gap_dot = -(1.0 + restitution_) * gap_dot;
```
to:
```cpp
desired_gap_dot = -restitution_ * gap_dot;
```

### Documentation Updates

1. **Math formulation Section 6.2**: Add explicit note distinguishing between:
   - Constraint RHS: $b = -(1+e) \cdot \mathbf{J}\dot{\mathbf{q}}^-$ (for $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$)
   - Target velocity: $v_{\text{target}} = -e \cdot v_{\text{pre}}$ (for velocity-target formulations)

2. **P2 README**: Replace "fundamental flaw" conclusion with "single-line bug in restitution formula"

3. **Withdraw "DO NOT PROCEED"**: The constraint approach works correctly with the fix

### Implications for Implementation

The constraint-based contact refactor (ticket 0032) can proceed. The velocity-level formulation is sound:
- For PGS solver (P1 scenario): use the constraint RHS `b = -(1+e)·Jq̇⁻` directly in $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$
- For direct velocity target: use `v_target = -e·v_pre` and compute $\Delta v = v_{\text{target}} - v_{\text{pre}}$

Both produce the same impulse $\lambda = (1+e) \cdot |v_{\text{rel},n}| / A_{ii}$, but through different code paths that must not be confused.

---

## Diagnostic Files

| File | Purpose |
|------|---------|
| `diagnostic_test.cpp` | Three-part analysis: first-bounce trace, algebraic proof, end-to-end fix verification |
