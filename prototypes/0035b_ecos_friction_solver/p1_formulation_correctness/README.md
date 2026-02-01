# Prototype P1: ECOS Formulation Correctness

## Question

Does the ECOS problem formulation correctly encode the friction LCP?

## Success Criterion

ECOS solution matches hand-computed result within 1e-6 for a known single-contact friction scenario.

## Approach

1. Create a minimal single-contact friction problem with known solution
2. Formulate as ECOS SOCP problem (sparse CSC matrices, cone constraints)
3. Call ECOS to solve
4. Validate:
   - Solution satisfies friction cone: $\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$
   - Solution satisfies LCP: $\mathbf{A}\boldsymbol{\lambda} = \mathbf{b}$ (or close due to numerical tolerance)
   - Solution matches expected behavior (stick vs slip based on applied forces)

## Test Scenario

**Single contact with friction** (from M8 numerical examples):
- Mass: $m = 1.0$ kg
- Friction coefficient: $\mu = 0.5$
- Applied tangential force: $F_t = 2.0$ N (less than $\mu \cdot m \cdot g = 4.9$ N)
- Expected: **Stick** (interior solution, $v_t = 0$, $\|\boldsymbol{\lambda}_t\| < \mu \lambda_n$)

**Build and Run**:
```bash
cd prototypes/0035b_ecos_friction_solver/p1_formulation_correctness
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/prototype
```

## Expected Output

```
=== ECOS Friction Formulation Prototype ===
Test: Single contact, stick regime
ECOS converged: Yes
Iterations: [5-15]
Lambda: [lambda_n, lambda_t1, lambda_t2]
Friction cone satisfied: Yes (||lambda_t|| <= mu * lambda_n)
LCP residual: < 1e-6
Result: PASS/FAIL
```
