# Epic: Physics Module Refactor and Motion Integration

## Status
- [X] Epic (Parent Ticket)

## Metadata
- **Created**: 2026-01-10
- **Author**: Daniel M Newman
- **Priority**: High
- **Estimated Complexity**: Large (Epic)
- **Target Component(s)**: msd-sim, msd-gui

---

## Summary

This epic encompasses the complete refactoring and implementation of the physics system, from consolidating redundant state representations through to visible physics-driven motion in the GUI. The work is broken into four sequential phases, each with its own ticket.

## Phase Tickets

| Phase | Ticket | Title | Dependencies |
|-------|--------|-------|--------------|
| 1 | [0017](0017_physics_state_consolidation.md) | Physics State Consolidation | None |
| 2 | [0018](0018_force_application_mechanism.md) | Force Application Mechanism | 0017 |
| 3 | [0019](0019_physics_motion_integration.md) | Physics Motion Integration | 0017, 0018 |
| 4 | [0020](0020_physics_gui_propagation.md) | Physics GUI Propagation | 0019 |

## Dependency Graph

```
0017 Physics State Consolidation
  │
  ├──► 0018 Force Application Mechanism
  │      │
  │      ▼
  └────► 0019 Physics Motion Integration
           │
           ▼
         0020 Physics GUI Propagation
```

## Phase Summaries

### Phase 1: Physics State Consolidation (0017)
Remove architectural redundancy by consolidating `InertialState` and `DynamicState` into a single working implementation. Delete dead code (`AssetPhysical`, `AssetInertial`, `AssetEnvironment`). Implement all placeholder methods in `DynamicState`.

**Key Deliverables**:
- Functional `DynamicState` class with working setters/getters
- Removal of 6 redundant files
- Updated `Platform` class

### Phase 2: Force Application Mechanism (0018)
Design and implement force generators for gravity, collision response, and contact forces. Create a `ForceAccumulator` to collect and apply forces each timestep.

**Key Deliverables**:
- `ForceGenerator` abstract interface
- `GravityForce`, `CollisionResponse`, `ContactForce` implementations
- `ForceAccumulator` for force collection

### Phase 3: Physics Motion Integration (0019)
Implement the numerical integration step that converts forces into motion. Create `PhysicsWorld` orchestrator with fixed timestep support.

**Key Deliverables**:
- `Integrator` (Semi-implicit Euler or Velocity Verlet)
- `PhysicsWorld` simulation loop
- Objects that actually move under forces

### Phase 4: Physics GUI Propagation (0020)
Connect physics state updates to the rendering system. Implement frame interpolation for smooth motion and optional debug visualization.

**Key Deliverables**:
- Transform propagation to renderer
- Frame interpolation for smooth rendering
- Debug visualization (forces, velocities, contacts)

## Motivation

The current physics implementation has several critical gaps documented in the codebase analysis:

1. **Redundant State Representations**: `DynamicState` and `InertialState` serve identical purposes
2. **Placeholder Implementations**: `DynamicState` methods are all no-ops
3. **Dead Code**: Unused `Asset*` class hierarchy with syntax errors
4. **Missing Motion Integration**: Forces computed but never integrated into motion
5. **No Force Sources**: No gravity or collision response
6. **No GUI Propagation**: No pathway from physics state to visual rendering

## Current State Analysis

See detailed analysis in each phase ticket's appendix. Key findings:

### Redundancy: DynamicState vs InertialState

| Aspect | DynamicState | InertialState |
|--------|--------------|---------------|
| Location | `Physics/RigidBody/` | `Environment/` |
| Implementation | Placeholder (no-op) | Functional |
| Used By | PhysicsComponent | Platform |

### Dead Code

| Class | Status |
|-------|--------|
| `AssetPhysical` | Broken (syntax error) |
| `AssetInertial` | Unused |
| `AssetEnvironment` | Unused |

### Working Components (Preserve)

- `ConvexHull` — Collision geometry
- `GJK` — Collision detection
- `PhysicsComponent` — Force/torque calculation
- `Object` — Entity management
- `InertialCalculations` — Inertia tensor computation

---

## Human Feedback

{Add feedback here. This will be visible to all phase tickets.}
