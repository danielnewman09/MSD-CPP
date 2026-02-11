# Ticket 0054: Collision Pipeline Scaling Investigation

## Status
- [ ] Draft
- [ ] Investigation Complete
- [ ] Design Complete
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Performance / Investigation
**Priority**: High
**Assignee**: N/A
**Created**: 2026-02-10
**Branch**: TBD
**Related Tickets**: [0053_collision_pipeline_performance](0053_collision_pipeline_performance.md), [0052_custom_friction_cone_solver](0052_custom_friction_cone_solver.md)

---

## Problem Statement

FPS drops become severe with 20-40 friction-enabled objects in real simulation workloads. Ticket 0053 achieved micro-level optimizations (-11% to -19% per benchmark), but the fundamental scaling characteristics of the collision pipeline have not been addressed.

Current benchmarks only test body-vs-floor scenarios with widely-spaced bodies (no body-body collisions). Real workloads involve clustered, interacting objects that create fundamentally different performance profiles.

### Why Current Benchmarks Underestimate Cost

| Factor | Benchmark (0053) | Real Workload |
|--------|------------------|---------------|
| Collision pairs | N (body-floor only) | O(N²) (all-pairs) |
| Contacts per pair | 1-4 (cube-on-plane) | Variable, multi-manifold |
| Dynamic bodies per pair | 1 (floor is static) | 2 (body-body) |
| Constraint system size | Small (isolated pairs) | Large (coupled via shared bodies) |
| Friction solver matrix | 3C×3C per pair | 3C×3C total (all contacts) |

### Scaling Concerns

- **Broad phase**: Currently O(N²) — every inertial checked against every other inertial + every environment
- **Friction solver**: Newton solver on 3C×3C system with Cholesky is O(C³) where C = total contacts across all pairs
- **GJK/EPA**: Called once per candidate pair — O(N²) invocations
- **Position corrector**: Solves constraint system per iteration — scales with total contacts

---

## Investigation Plan

### Phase 1: Realistic Benchmark Suite

Build benchmarks that match real workloads:

1. **Stacking scenario**: N cubes stacked vertically (worst case for coupled contacts)
2. **Pile scenario**: N cubes dropped into a bowl/floor (clustered body-body contacts)
3. **Spread scenario**: N cubes on floor, widely spaced (current benchmark, for comparison)
4. **Mixed scenario**: N/2 stacked + N/2 spread (mixed workload)

Parameterize at N = 5, 10, 20, 40, 80 to establish scaling curves.

### Phase 2: Full-Frame Profiling

Profile `WorldModel::update()` with the realistic benchmarks to identify:

1. **Time breakdown**: What percentage goes to broad phase, GJK/EPA, constraint assembly, friction solver, position correction, integration, rendering?
2. **Scaling bottleneck**: Which subsystem dominates as N grows?
3. **Contact count growth**: How does total contact count C scale with N for each scenario?
4. **Friction solver iterations**: How does Newton iteration count scale with C?

### Phase 3: Optimization Candidates

Based on profiling data, evaluate and prioritize:

| Optimization | Expected Impact | Complexity | Prerequisite |
|--------------|----------------|------------|--------------|
| **Spatial partitioning** (AABB grid/tree) | Reduce broad phase from O(N²) to O(N log N) | Medium | Phase 2 data |
| **Per-pair friction solving** | Reduce friction solver from O(C³) global to O(c³) per-pair | High | Architectural analysis |
| **Contact reduction** | Limit contacts per pair, reduce C | Low | Phase 2 contact data |
| **AABB early-out** | Skip GJK for non-overlapping AABBs | Low | Broad phase refactor |
| **Solver sparsity** | Exploit block-diagonal structure in A matrix | Medium | Phase 2 solver data |
| **Constraint island detection** | Solve independent groups separately | High | Graph analysis |
| **Parallel pair evaluation** | Thread GJK/EPA across pairs | Medium | Thread safety audit |

---

## Acceptance Criteria

- [ ] AC1: Realistic benchmark suite covering stacking, pile, and spread scenarios at N = 5-80
- [ ] AC2: Full-frame profiling data identifying the dominant bottleneck(s) as N scales
- [ ] AC3: Scaling curve characterization (empirical O(N^k) for each subsystem)
- [ ] AC4: Prioritized optimization plan with expected impact estimates grounded in profiling data
- [ ] AC5: At least one optimization implemented that meaningfully improves 40-body FPS

---

## Notes

- The friction solver (FrictionConeSolver) uses a global 3C×3C Newton solve. If C grows quadratically with N (body-body contacts), the solver alone becomes O(N⁶) — this is the most likely bottleneck.
- Constraint island detection would allow decomposing one large solve into many small independent solves — this is the approach used by Box2D, Bullet, and other mature physics engines.
- Spatial partitioning (broad phase) is standard practice but may not be the bottleneck if the solver dominates at 20-40 bodies.
