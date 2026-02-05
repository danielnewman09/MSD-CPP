# ECOS Module Architecture

> Architectural context for ECOS (Embedded Conic Solver) integration utilities.
> For API details, use MCP: `find_class ECOSData`, `get_class_members ECOSSparseMatrix`, etc.

## Architecture Overview

```
ConstraintSolver (solveWithECOS for friction constraints)
    └── ECOSData (RAII wrapper)
        ├── ECOSSparseMatrix (Eigen → CSC format)
        ├── FrictionConeSpec (friction coefficients per contact)
        └── ECOSWorkspacePtr (unique_ptr with custom deleter)

ECOSProblemBuilder transforms contact LCP → ECOS SOCP form
```

---

## Design Decisions

### Why RAII for ECOS Workspace?

ECOS is a C library with manual memory management. `ECOSData` provides:
- Automatic `ECOS_cleanup()` on destruction
- Exception safety (cleanup even if exception thrown)
- Move-only semantics (no accidental workspace sharing)

### ECOS Equilibration Constraint (Critical)

ECOS v2.0.10 compiles with `EQUILIBRATE = 1`, meaning `ECOS_cleanup()` writes back to the caller's data arrays (G, h, c). This imposes:

1. **Member declaration order**: `workspace_` declared **last** (destroyed **first**)
2. **Custom move assignment**: Must call `cleanup()` before moving data arrays

**Why default move assignment is unsafe**: Default moves data arrays before destroying old workspace → use-after-free when cleanup writes to invalidated memory.

### Why Second-Order Cone Programming?

Friction constraints require `||λ_t|| ≤ μ·λ_n` (circular cone). Box friction approximation has ~29% error. ECOS solves SOCP exactly:

```
min c^T·x  s.t.  G·x + s = h,  s ∈ K
```

where K is a product of 3D second-order cones (one per contact).

---

## Mathematical Formulation

**Per contact i**: `||[λ_t1, λ_t2]|| ≤ μ_i·λ_n`

**G matrix** (block-diagonal, 3C × 3C):
- Row 3i: `-μ_i` at column 3i (normal scaling)
- Row 3i+1: `-1` at column 3i+1 (tangent 1)
- Row 3i+2: `-1` at column 3i+2 (tangent 2)

This gives cone constraint: `s_0 = μ·λ_n`, `s_1 = λ_t1`, `s_2 = λ_t2`

---

## ECOS Exit Codes

| Code | Name | Action |
|------|------|--------|
| 0 | `ECOS_OPTIMAL` | Success, extract solution |
| -1 | `ECOS_MAXIT` | Return `converged=false` |
| -2, -3 | Numerics | Log warning, `converged=false` |
| 1, 2 | Infeasible | Problem malformed, throw |

---

## Querying This Module

Use MCP tools for API details:
- `find_class ECOSData` — RAII wrapper lifecycle
- `find_class ECOSSparseMatrix` — CSC format conversion
- `find_class FrictionConeSpec` — Friction cone specification
- `find_class ECOSProblemBuilder` — LCP to SOCP conversion
- `search_documentation ECOS` — Find ECOS-related docs

---

## Related Documentation

- **Constraint solver**: [Constraints/CLAUDE.md](../CLAUDE.md)
- **Design documents**: `docs/designs/0035b*` (ECOS utilities, data wrapper, problem builder)

## References

- **ECOS GitHub**: https://github.com/embotech/ecos
- **SOCP**: Second-Order Cone Programming
