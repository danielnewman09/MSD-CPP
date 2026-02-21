# Ticket 0072a: Engine pybind11 Bindings

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Merged / Complete

**Current Phase**: Merged / Complete
**Type**: Feature
**Priority**: High
**Assignee**: TBD
**Created**: 2026-02-17
**Generate Tutorial**: No
**Requires Math Design**: No
**Parent Ticket**: [0072_live_browser_simulation](0072_live_browser_simulation.md)
**Depends On**: None
**Blocks**: [0072b_websocket_simulation_endpoint](0072b_websocket_simulation_endpoint.md)

---

## Overview

Expose the C++ `msd_sim::Engine` class to Python via pybind11 so the FastAPI server can instantiate simulations, spawn objects, step the physics, and extract frame state — all from Python.

This follows the existing wrapper pattern (like `DatabaseWrapper` in `database_bindings.cpp`): an `EngineWrapper` class that converts C++ types to Python-native dicts/tuples internally.

---

## Requirements

### R1: EngineWrapper Class

Create `msd/msd-pybind/src/engine_bindings.cpp` with an `EngineWrapper` providing:

```python
# Constructor
engine = msd_reader.Engine("path/to/assets.db")

# Spawn objects
result = engine.spawn_inertial_object(
    "cube", 0.0, 0.0, 5.0, 0.0, 0.0, 0.0,
    mass=10.0, restitution=0.8, friction=0.5)
# Returns: {"instance_id": int, "asset_id": int}

result = engine.spawn_environment_object(
    "large_cube", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Returns: {"instance_id": int, "asset_id": int}

# Step simulation
engine.update(16)  # milliseconds

# Extract current state
frame = engine.get_frame_state()
# Returns: {
#   "simulation_time": float,
#   "states": [
#     {"body_id": int, "asset_id": int,
#      "position": {"x": float, "y": float, "z": float},
#      "velocity": {"x": float, "y": float, "z": float},
#      "orientation": {"w": float, "x": float, "y": float, "z": float},
#      "angular_velocity": {"x": float, "y": float, "z": float}}
#   ]
# }

# Query available assets
assets = engine.list_assets()
# Returns: [(asset_id, "asset_name"), ...]

# Get geometry for Three.js
vertices = engine.get_collision_vertices(asset_id)
# Returns: [(x, y, z), ...]
```

### R2: get_frame_state() Implementation

The `get_frame_state()` method must:
1. Call `worldModel_.getInertialAssets()` to iterate dynamic bodies
2. For each `AssetInertial`: extract position from `getReferenceFrame().getOrigin()`, orientation from `getReferenceFrame().getQuaternion()`, velocity from `getState().velocity`, angular velocity from `getState().angularVelocity`
3. Include environmental objects with zero velocity/angular_velocity
4. Include `asset_id` per body so the frontend can look up geometry
5. Return the simulation time from `worldModel_.getTime()`

### R3: Type Conversion in C++ Wrapper

All Eigen types (`Coordinate`, `AngularCoordinate`, `Eigen::Quaterniond`) are converted to Python dicts/tuples inside the C++ wrapper. No Eigen type casters are used. This keeps the Python API simple and avoids pybind11-Eigen compatibility issues.

### R4: CMake Integration

- Add `src/engine_bindings.cpp` to `pybind11_add_module()` sources in `msd/msd-pybind/CMakeLists.txt`
- Add `msd_sim` to `target_link_libraries` (currently only links `msd_assets`)

### R5: Module Registration

- Add `void bind_engine(py::module_& m);` forward declaration in `msd_bindings.cpp`
- Call `bind_engine(m);` in the `PYBIND11_MODULE` block

---

## Constraints
- `Engine` constructor requires a valid assets.db path (created by `generate_assets`)
- `Engine` is not thread-safe — only one thread should call methods at a time
- The `spawnInertialObject()` overload with defaults uses mass=10.0, restitution=0.5, friction=0.5 in C++; wrapper can expose different Python defaults
- `AngularCoordinate` constructor takes (pitch, roll, yaw) in radians

## Acceptance Criteria
- [ ] `import msd_reader; e = msd_reader.Engine("assets.db")` succeeds
- [ ] `e.list_assets()` returns list of (id, name) tuples
- [ ] `e.spawn_inertial_object("cube", 0, 0, 5, 0, 0, 0)` returns dict with `instance_id` and `asset_id`
- [ ] `e.spawn_environment_object("large_cube", 0, 0, 0, 0, 0, 0)` returns dict
- [ ] `e.update(16)` advances simulation time
- [ ] `e.get_frame_state()` returns dict with `simulation_time` and `states` array
- [ ] `e.get_frame_state()["states"]` contains entries with position, velocity, orientation, angular_velocity
- [ ] `e.get_collision_vertices(asset_id)` returns vertex tuples
- [ ] Python test suite passes: `test_engine_bindings.py`

---

## Design Decisions (Human Input)

### Preferred Approaches
- Follow `DatabaseWrapper` / `AssetRegistryWrapper` pattern exactly
- Use `py::arg()` with default values for mass, restitution, friction
- Return Python dicts (not custom classes) for maximum flexibility

### Things to Avoid
- Do not expose `Coordinate`, `AngularCoordinate`, `ReferenceFrame`, `InertialState` as Python classes
- Do not use pybind11 Eigen type casters
- Do not expose `WorldModel` or `DataRecorder` directly

---

## References

### Related Code
- `msd/msd-sim/src/Engine.hpp` — Engine public API (constructor, update, spawn*, getWorldModel)
- `msd/msd-pybind/src/database_bindings.cpp` — DatabaseWrapper pattern to follow
- `msd/msd-pybind/src/asset_registry_bindings.cpp` — AssetRegistryWrapper pattern to follow
- `msd/msd-pybind/src/msd_bindings.cpp` — Module entry point to modify
- `msd/msd-pybind/CMakeLists.txt` — Build config to modify
- `msd/msd-sim/src/Environment/WorldModel.hpp` — WorldModel::getInertialAssets(), getTime()
- `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp` — AssetInertial state access
- `msd/msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp` — AssetEnvironment state access

### Related Tickets
- [0056c_python_bindings](0056c_python_bindings.md) — Original pybind11 module implementation

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-17 00:00
- **Completed**: 2026-02-17 00:00
- **Branch**: 0072a-engine-pybind-bindings
- **PR**: N/A
- **Files Created**:
  - `msd/msd-pybind/src/engine_bindings.cpp` — EngineWrapper class + bind_engine() function
  - `msd/msd-pybind/test/test_engine_bindings.py` — 36 Python tests (all passing)
- **Files Modified**:
  - `msd/msd-pybind/src/msd_bindings.cpp` — Added bind_engine() forward declaration and call
  - `msd/msd-pybind/CMakeLists.txt` — Added engine_bindings.cpp source and msd_sim link dependency
- **Notes**:
  - All 36 acceptance criteria tests pass
  - EngineWrapper follows DatabaseWrapper / AssetRegistryWrapper pattern exactly
  - All Eigen types converted to Python dicts/tuples inside C++ wrapper (no Eigen type casters)
  - Discovered that update(simTime) takes absolute simulation time, not delta — documented in wrapper and tests
  - C++ spawnEnvironmentObject returns AssetEnvironment which requires getInstanceId/getAssetId from AssetPhysical base

### Implementation Review Phase
- **Started**: 2026-02-17 00:00
- **Completed**: 2026-02-17 00:00
- **Branch**: 0072a-engine-pybind-bindings
- **PR**: N/A
- **Status**: APPROVED
- **Reviewer Notes**: All 5 requirements and all 9 acceptance criteria satisfied. Pattern conformance excellent. 36 tests pass. Minor: two redundant includes (not actionable). Ready to merge.

### Documentation Phase
- **Started**: 2026-02-17
- **Completed**: 2026-02-17
- **Branch**: 0072a-engine-pybind-bindings
- **PR**: #75
- **Artifacts**:
  - `msd/msd-pybind/CLAUDE.md` — Added Engine Bindings section; updated Overview, File Structure, Testing, References
  - `msd/msd-sim/CLAUDE.md` — Updated Engine Component public interface; added Python binding callout
  - `msd/CLAUDE.md` — Updated msd-pybind row in Libraries Summary
  - `docs/designs/0072a_engine_pybind_bindings/doc-sync-summary.md` — Documentation sync summary
- **Notes**: No PlantUML diagram required (implementation-only ticket, followed existing wrapper pattern). Record layer sync not applicable (msd-transfer not touched). Tutorial skipped (Generate Tutorial: No).

---

## Human Feedback

### Retroactive Design Review — 2026-02-20

A retroactive design review was conducted to reconstruct what the Design, Integration Design, and Python Design phases should have produced. The full review document is at:

`docs/designs/0072a_engine_pybind_bindings/retroactive-review.md`

**Overall assessment**: Implementation is sound and follows project coding standards. Five follow-up items were identified.

#### Coding Standards — PASS

All project standards from CLAUDE.md are satisfied: brace initialization, Rule of Zero, no raw pointers, no `shared_ptr`, `const T&` for non-owning access, correct naming conventions, Doxygen comments, ticket reference.

Two redundant includes (`AssetRegistry.hpp` and `AngularVelocity.hpp`) were noted by the original implementation reviewer (m1, m2) and are acceptable as-is.

#### Follow-Up Items

**FU-002 (Medium) — Requirement R2 deviation: environment objects absent from `get_frame_state()`**

The ticket stated: "Include environmental objects with zero velocity/angular_velocity" in `get_frame_state()`. The implementation includes only inertial (dynamic) assets. Static environment objects are not in the returned `states` list.

Decision required from ticket author:
- Option A: Fix `EngineWrapper::getFrameState()` to also iterate `worldModel.getEnvironmentalObjects()` and include them with zeroed velocity/angular_velocity fields.
- Option B: Formally close R2 as "won't fix" with rationale: environment object positions are known at spawn time and do not change, so per-frame inclusion is redundant for the WebSocket use case.

This decision must be made before 0072b's design phase begins, as it affects the `get_frame_state()` schema.

**FU-003 (Medium) — No formal API contract for `get_frame_state()` schema**

The `get_frame_state()` return dict is the primary integration contract between 0072a and the 0072b WebSocket endpoint. It is documented only in CLAUDE.md prose and implicit in the tests. A JSON Schema or YAML contract should be created in `docs/api-contracts/` as part of 0072b's Integration Design phase.

**FU-001 (Low) — `update()` absolute-time semantics must be communicated to 0072b**

`engine.update(ms)` receives absolute simulation time, not a delta. The 0072b WebSocket loop must maintain a cumulative counter. This should be explicitly noted in the 0072b ticket's Design Decisions section before implementation.

**FU-004 (Low) — Keyword-arg usage for physics parameters (acknowledged, no action)**

`pitch`, `roll`, `yaw`, `mass`, `restitution`, `friction` in `spawn_inertial_object` can be passed positionally, which may cause confusion. pybind11 has no keyword-only separator equivalent. Callers should use keyword arguments. This is already the pattern in all tests. No code change required.

**FU-005 (Low) — Thread safety pattern for FastAPI must be addressed in 0072b**

`EngineWrapper` is not thread-safe. The 0072b Python Design phase should specify the concurrency pattern (asyncio lock, thread pool executor, or single background task).

#### Design Decisions Reconstructed

The retroactive review formally documented five design decisions (DD-0072a-001 through DD-0072a-005) that were made implicitly during implementation:

- DD-0072a-001: Wrapper owns Engine by value
- DD-0072a-002: No Eigen type casters — all conversion in C++ wrapper
- DD-0072a-003: `update()` receives absolute simulation time, not delta
- DD-0072a-004: `get_frame_state()` excludes environment objects (deviation from R2)
- DD-0072a-005: `getCollisionVertices` duplicated on EngineWrapper for self-contained API
