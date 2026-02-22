# Ticket 0077: Broad-Phase Collision Detection

## Status
- [x] Draft
- [ ] Investigation Complete
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Investigation / Performance
**Priority**: Medium
**Created**: 2026-02-21
**Generate Tutorial**: No
**Parent Ticket**: [0054_collision_scaling_investigation](0054_collision_scaling_investigation.md)
**Related Tickets**: [0071_collision_pipeline_profiling](0071_collision_pipeline_profiling.md), [0071a_constraint_solver_scalability](0071a_constraint_solver_scalability.md)

---

## Summary

`CollisionPipeline::detectCollisions()` uses O(n^2) pairwise iteration over all inertial bodies and all inertial-vs-environment pairs. Each pair invokes `CollisionHandler::checkCollision()`, which enters GJK (including a per-pair world-space AABB computation and overlap test). For scenes with many well-separated objects, most GJK calls are wasted on pairs that could be eliminated cheaply by a dedicated broad-phase pass.

This ticket investigates introducing a broad-phase culling step that eliminates non-overlapping pairs before entering the narrow-phase GJK/EPA pipeline.

---

## Problem

### Current Detection Flow

```
detectCollisions()
  for each pair (i, j):                          ← O(n²) pairs
    CollisionHandler::checkCollision(A, B)
      GJK::intersects()
        computeWorldAABB(A)                       ← recomputed per pair
        computeWorldAABB(B)                       ← recomputed per pair
        AABB overlap test                         ← early-out if no overlap
        GJK simplex iteration                     ← ~500-2000 FLOPs if reached
```

**Problems**:
1. World-space AABBs are recomputed redundantly — body A's AABB is recomputed for every pair involving A
2. The AABB test happens inside GJK, after function call overhead and setup
3. No spatial data structure eliminates distant pairs before iteration

### Per-Pair AABB Cost

The existing AABB computation in `GJK::intersects()` (GJK.cpp:42-75) transforms all 8 corners of each body's local bounding box to world space every time GJK is called. For n inertial bodies, body i's AABB is computed (n-1) times in the inertial-vs-inertial loop alone.

### Where This Matters

With n inertial bodies + m environment bodies, the current cost is:
- **Inertial-vs-inertial**: n(n-1)/2 GJK calls (each with 2 AABB computations)
- **Inertial-vs-environment**: n*m GJK calls (each with 2 AABB computations)
- **Total AABB computations**: O(n^2 + nm) — many redundant

A broad phase that precomputes AABBs once per body reduces AABB computations to O(n + m) and eliminates pairs that don't overlap before entering GJK.

---

## Investigation Questions

1. **Profiling data**: What fraction of `detectCollisions()` time is spent on pairs that fail the AABB test vs. pairs that proceed to GJK simplex iteration? (Use ClusterDrop/32 or similar multi-body benchmark from ticket 0071.)
2. **Algorithm selection**: For the expected body counts (10-100), which broad-phase structure provides the best cost/complexity trade-off?
   - **Sort-and-sweep (SAP)**: O(n log n) build, good for coherent motion
   - **Uniform grid**: O(n) build, good for similar-sized objects
   - **AABB tree / BVH**: O(n log n) build, general-purpose
   - **Simple AABB precompute + O(n^2) overlap**: O(n) precompute, still O(n^2) overlap but eliminates per-pair AABB recomputation
3. **Integration point**: Should broad phase live inside `CollisionPipeline::detectCollisions()` (replacing the current nested loop) or as a separate component owned by `CollisionPipeline`?
4. **Environment bodies**: Environment bodies are static — their AABBs never change. Can they be precomputed once at scene setup and reused across frames?
5. **Interaction with ContactCache**: `detectCollisions()` currently queries `ContactCache::hasEntry()` per pair to decide whether to skip SAT validation. A broad-phase culling pass must preserve this optimization for persistent contacts.

---

## Candidate Approaches

### Option A: AABB Precompute Only (Minimal Change)

Precompute world-space AABBs once per body at the start of `detectCollisions()`, pass them into `checkCollision()` to avoid redundant recomputation inside GJK. Keep the O(n^2) loop structure but eliminate the dominant per-pair cost.

- **Pro**: Minimal code change, no new data structure
- **Con**: Still O(n^2) pair iteration; only helps if AABB computation is a significant fraction of per-pair cost

### Option B: Sort-and-Sweep (SAP)

Sort bodies by one axis (e.g., X), sweep to find overlapping intervals, then test remaining axes. Well-suited for coherent motion where the sorted order changes little frame-to-frame.

- **Pro**: O(n log n) average, exploits temporal coherence
- **Con**: Worst case O(n^2) for pathological axis alignment

### Option C: Uniform Spatial Grid

Partition space into cells, assign each body to cells its AABB overlaps, test only pairs sharing a cell.

- **Pro**: O(n) expected for uniform distributions, simple implementation
- **Con**: Cell size tuning, poor for widely varying object sizes

### Option D: Dynamic AABB Tree (BVH)

Maintain a balanced tree of AABBs, query for overlapping pairs.

- **Pro**: O(n log n), handles varying sizes, well-studied
- **Con**: More complex implementation, tree maintenance cost

---

## Success Criteria

- Investigation identifies which approach (if any) is warranted for current body counts
- Profiling data quantifies the cost of the current O(n^2) detection relative to total frame time
- If an approach is selected, design document specifies integration with `CollisionPipeline` and `ContactCache`

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-21
- **Notes**:
  - Current O(n^2) pairwise detection noted as a limitation in Collision/CLAUDE.md and ticket 0054
  - GJK already has per-pair AABB early-out (GJK.cpp:35-85) but AABBs are recomputed redundantly per pair
  - Ticket 0071a addressed solver scalability via island decomposition; this ticket targets detection scalability
  - Environment body AABBs are static and could be cached across frames
