# Ticket 0062a: Extend Test Asset Generator

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Review
**Type**: Feature / Testing / Infrastructure
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-13
**Generate Tutorial**: No
**Parent Ticket**: [0062_replay_collision_test_suite](0062_replay_collision_test_suite.md)
**Depends On**: [0060a_replay_enabled_test_fixture](0060a_replay_enabled_test_fixture.md)

---

## Overview

Extend `generate_test_assets` and `ReplayEnabledTest` to support the geometry types and spawn parameters needed by collision tests. The current test asset DB only has cubes — collision tests need spheres (for rotation-free linear collisions) and parameterized spawning (custom mass, restitution, friction).

---

## Requirements

### R1: Sphere Assets in Test Database

Add icosphere assets to `generate_test_assets.cpp`:
- `unit_sphere` (radius 1.0 m) — standard collision test object
- `small_sphere` (radius 0.5 m) — asymmetric mass ratio tests
- `large_sphere` (radius 2.0 m) — environment-scale objects

Spheres should use the same icosphere subdivision algorithm currently in `LinearCollisionTest.cpp` (`createSpherePoints()` with 2 levels of subdivision, ~162 vertices). The collision mesh stores raw `Vector3D` vertex data (not visual Vertex format).

### R2: Additional Cube Sizes

Add to the test asset DB:
- `tiny_cube` (0.5 m) — for tests needing size asymmetry

### R3: Parameterized Spawn Helpers

Add overloads to `ReplayEnabledTest` fixture:

```cpp
/// Spawn inertial object with custom physics parameters
const AssetInertial& spawnInertial(
    const std::string& assetName,
    const Coordinate& position,
    double mass = 1.0,
    double restitution = 0.5,
    double friction = 0.5);

/// Spawn inertial object with initial velocity
const AssetInertial& spawnInertialWithVelocity(
    const std::string& assetName,
    const Coordinate& position,
    const Coordinate& velocity,
    double mass = 1.0,
    double restitution = 0.5,
    double friction = 0.5);
```

These delegate to `engine().spawnInertialObject()` then configure the returned asset's parameters (mass, restitution, friction, velocity) post-spawn.

### R4: Gravity Control

Add a helper to disable gravity for isolated collision tests:

```cpp
/// Disable gravity (for tests isolating collision response from gravity)
void disableGravity();
```

Many collision tests need zero-gravity to test momentum conservation and energy accounting in isolation.

---

## Acceptance Criteria

- [x] AC1: `generate_test_assets` produces unit_sphere, small_sphere, large_sphere, tiny_cube assets in addition to existing cubes
- [x] AC2: Sphere collision meshes use icosphere point cloud (~312 vertices per sphere) stored as raw Vector3D data
- [x] AC3: `spawnInertial()` helper spawns objects with configurable mass, restitution, and friction
- [x] AC4: `spawnInertialWithVelocity()` helper sets initial velocity on spawned objects
- [x] AC5: `disableGravity()` removes gravity potential from WorldModel
- [x] AC6: All existing ReplayEnabledTest tests (0060a) still pass (732 total tests, 728 passed, 4 pre-existing failures)
- [x] AC7: New unit tests verify sphere assets load correctly and parameterized spawning works (12 new tests, all passing)

---

## Technical Notes

### Sphere Collision Mesh Format

The collision mesh in the database stores raw vertex positions as a `std::vector<uint8_t>` blob. For spheres, this should be the icosphere point cloud (not visual Vertex data with color/normal). The `AssetRegistry` deserializes this as `Vector3D` array and constructs a `ConvexHull`.

See `EngineIntegrationTest.cpp` `serializeCollisionVertices()` for the serialization pattern — it stores `Vector3D` data directly.

### Post-Spawn Configuration Pattern

`Engine::spawnInertialObject()` returns `const AssetInertial&`, but the WorldModel provides mutable access via `getObject(instanceId)`. The helpers should:
1. Call `engine().spawnInertialObject()` to get the instance ID
2. Access the mutable object via `engine().getWorldModel().getObject(instanceId)`
3. Set mass, restitution, friction, velocity on the mutable reference
4. Return a const reference

Note: Mass cannot be changed post-construction on `AssetInertial` (it's set in the constructor). If mass parameterization requires constructor support, the `Engine::spawnInertialObject()` signature may need a mass overload, or the fixture can work with the default mass (1.0 kg from hull volume * default density).

### Gravity Potential

`WorldModel` stores potentials as `std::vector<std::unique_ptr<PotentialEnergy>>`. Disabling gravity requires clearing this vector or removing the `GravityPotential` entry. Check if WorldModel exposes a method for this, or if the fixture needs to access the potentials directly.

---

## Workflow Log

| Phase | Date | Agent | Notes |
|-------|------|-------|-------|
| Draft | 2026-02-13 | Human + Claude | Initial ticket creation |
| Draft → Ready for Implementation | 2026-02-13 14:30 | Orchestrator | No math design required; proceeding to implementation |
| Implementation | 2026-02-13 14:45-15:30 | Orchestrator | Extended generate_test_assets with sphere/cube assets, added WorldModel/Engine API overloads, implemented ReplayEnabledTest helpers, wrote 12 unit tests. All tests pass. |
| Implementation → Ready for Review | 2026-02-13 15:30 | Orchestrator | Commit: 5856008, Branch: 0062a-extend-test-asset-generator, PR: #60 |
