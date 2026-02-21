# Design: Live Simulation Cleanup (0072e)

**Ticket**: [0072e_live_simulation_cleanup](../../../tickets/0072e_live_simulation_cleanup.md)
**Date**: 2026-02-21
**Status**: Initial Design

---

## Summary

This ticket addresses six medium-priority gaps identified during the retroactive design reviews of
tickets 0072a, 0072b, and 0072c. The gaps arose because those tickets skipped the Design,
Integration Design, Python Design, and Frontend Design phases. The fixes tighten input validation,
add missing output data, remove dead code, and clean up payload hygiene across the three-layer
live simulation stack: C++ pybind11 binding, Python FastAPI server, and JavaScript frontend.

The changes are intentionally small and surgical. No new classes, no new endpoints, no wire
protocol changes that break existing clients. All changes are either additive (FR-1: new data in
frame state) or tightening (FR-3, FR-4: stricter validation) or removal of dead code (FR-5).

---

## Architecture Changes

### PlantUML Diagram

See: `./0072e_live_simulation_cleanup.puml`

### Modified Components

This ticket contains no new components — only targeted modifications to three existing files.

---

#### 1. `EngineWrapper` in `msd/msd-pybind/src/engine_bindings.cpp` — FR-1

**Current location**: `msd/msd-pybind/src/engine_bindings.cpp`

**Modification**: Extend `getFrameState()` to also iterate `worldModel.getEnvironmentalObjects()`
and append each environment object as a body state entry with:
- Zero velocity: `{"x": 0.0, "y": 0.0, "z": 0.0}`
- Zero angular velocity: `{"x": 0.0, "y": 0.0, "z": 0.0}`
- Position from `asset.getReferenceFrame().getOrigin()`
- Orientation from `asset.getInertialState().orientation` (quaternion identity, since environment
  objects are initialized with zero angular state)
- `is_environment: true` flag to distinguish static bodies from dynamic ones

**New `getFrameState()` output schema** (extended from current):

```
{
  "simulation_time": float,       // seconds
  "states": [
    // Dynamic (inertial) bodies — existing, unchanged:
    {
      "body_id":          int,
      "asset_id":         int,
      "position":         {"x": float, "y": float, "z": float},
      "velocity":         {"x": float, "y": float, "z": float},
      "orientation":      {"w": float, "x": float, "y": float, "z": float},
      "angular_velocity": {"x": float, "y": float, "z": float},
      "is_environment":   false
    },
    // Static (environment) bodies — NEW:
    {
      "body_id":          int,
      "asset_id":         int,
      "position":         {"x": float, "y": float, "z": float},
      "velocity":         {"x": 0.0, "y": 0.0, "z": 0.0},   // always zero
      "orientation":      {"w": float, "x": float, "y": float, "z": float},
      "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},   // always zero
      "is_environment":   true
    }
  ]
}
```

**Design decisions**:

- `is_environment` is added to **both** inertial and environment entries so that downstream
  consumers (the frontend's SceneManager) can distinguish body types without needing separate
  metadata. This is consistent with `LiveBodyMetadata.is_environment` that already exists in the
  Python layer.

- The `body_id` for environment objects uses `asset.getInstanceId()`, identical to inertial
  bodies. The WebSocket server assigns sequential body IDs starting at 1 (in the configure
  phase), and these IDs are already stored in `LiveBodyMetadata`. The C++ frame state uses the
  same `getInstanceId()` value, so the IDs will match across the metadata and frame messages.

- Environment objects are **appended after** all inertial body states, so the existing portion
  of the states array is unchanged. Existing tests that only check inertial body states will
  continue to pass.

- Orientation comes from `AssetEnvironment::getInertialState().orientation` — this is a
  quaternion identity (w=1, x=y=z=0) for objects spawned with default orientation, or the
  quaternion corresponding to the spawn-time Euler angles for objects spawned with non-zero
  orientation. The `InertialState` for environment objects is set once at spawn time and never
  mutated.

**Backward compatibility**: The change is additive. Existing consumers that process `states`
entries will encounter the new `is_environment` field (previously absent on inertial entries).
The field is `false` for existing inertial entries, matching the `LiveBodyMetadata.is_environment`
convention already in use.

---

#### 2. `SpawnObjectConfig` in `replay/replay/models.py` — FR-3, FR-4

**Current location**: `replay/replay/models.py`, class `SpawnObjectConfig`

**FR-3 modification**: Change `object_type: str` to `object_type: Literal["inertial", "environment"]`.

```python
# Before:
object_type: str           # "inertial" or "environment"

# After (FR-3):
from typing import Literal
object_type: Literal["inertial", "environment"]
```

Pydantic v2 validates `Literal` fields against the allowed set at parse time. A client sending
`"object_type": "kinematic"` will receive a `422 Unprocessable Entity` response (or a structured
error JSON on the WebSocket path) before any Engine call is made.

**FR-4 modification**: Add length constraints to `position` and `orientation`.

```python
# Before:
position: list[float]      # [x, y, z] in metres
orientation: list[float]   # [pitch, roll, yaw] in radians

# After (FR-4):
from typing import Annotated
from pydantic import Field

position: Annotated[list[float], Field(min_length=3, max_length=3)]
orientation: Annotated[list[float], Field(min_length=3, max_length=3)]
```

Using `Annotated` with `Field` is the Pydantic v2 idiomatic pattern for list-length constraints.
This rejects `[0, 1]` (too short) and `[0, 1, 2, 3]` (too long) before reaching C++.

**Import changes required**: Add `Annotated` and `Literal` from `typing`, and `Field` from
`pydantic`. All are already available in the project's Python environment.

**Backward compatibility**: These are intentionally breaking changes at the validation layer —
they reject previously-accepted malformed inputs. Existing well-formed clients (position with
3 elements, valid object_type) are unaffected.

---

#### 3. `_run_simulation` in `replay/replay/routes/live.py` — FR-5

**Current location**: `replay/replay/routes/live.py`, lines 88–134

**Modification**: Delete the dead `_run_simulation` async function entirely. This function:
- Is defined but never called (the simulation loop is inlined in `live_simulation`)
- Does not support stop-signal (the inlined version does via `asyncio.Event`)
- Would mislead future developers into thinking it is the canonical simulation loop

No callers exist outside this file. The inline loop at lines 293–321 is the real implementation.

**Backward compatibility**: No runtime impact. Removing dead code cannot break callers.

---

#### 4. `onStartSimulation` in `replay/static/js/live-app.js` — FR-6

**Current location**: `replay/static/js/live-app.js`, `onStartSimulation()` function, lines 253–262

**Modification**: When building the `objects` payload for the `configure` message, omit `mass`,
`restitution`, and `friction` for environment objects.

```javascript
// Before (sends mass/restitution/friction for all objects):
const objects = spawnList.map(entry => ({
    asset_name:  entry.asset_name,
    position:    entry.position,
    orientation: entry.orientation,
    object_type: entry.object_type,
    mass:        entry.mass,
    restitution: entry.restitution,
    friction:    entry.friction,
}));

// After (FR-6 — omit physics fields for environment objects):
const objects = spawnList.map(entry => {
    const obj = {
        asset_name:  entry.asset_name,
        position:    entry.position,
        orientation: entry.orientation,
        object_type: entry.object_type,
    };
    if (entry.object_type === 'inertial') {
        obj.mass        = entry.mass;
        obj.restitution = entry.restitution;
        obj.friction    = entry.friction;
    }
    return obj;
});
```

**Rationale**: Environment objects have no dynamics; `mass` is meaningless for them. The backend
`SpawnObjectConfig` has `mass`, `restitution`, and `friction` with defaults, so omitting these
fields is safe — the server fills defaults when fields are absent. The change eliminates
misleading data from the wire protocol and aligns the payload with the UI behavior (the mass
input is already hidden for environment objects).

**Note on restitution/friction for environment objects**: These are kept in `SpawnObjectConfig`
as optional fields with defaults because they are used by the collision solver (environment
objects participate in collision response from the inertial body's perspective). The frontend
does not currently expose restitution/friction controls for environment objects, so omitting
them from the payload lets the backend apply the defaults.

**Backward compatibility**: The backend accepts the omitted fields via Pydantic defaults. The
`configure` message schema in `docs/api-contracts/contracts.yaml` already marks `mass`,
`restitution`, and `friction` as optional (they have `default` values in the schema). No
breaking change.

---

### FR-2: API Contract Documentation for `get_frame_state()`

**Current location**: `docs/api-contracts/contracts.yaml` and inline CLAUDE.md docs

**Modification**: The `EngineFrameState` schema (the dict returned by
`engine.get_frame_state()`) is currently defined only implicitly in the C++ wrapper and in
CLAUDE.md prose. Since FR-1 extends this schema, this is the right moment to formalize it in
the contracts file.

The `EngineFrameState` schema will be added to `docs/api-contracts/contracts.yaml` as a new
`x-pybind11-schemas` section — a non-standard extension documenting the Python-level data
shapes produced by C++ pybind11 bindings (as opposed to the REST/WebSocket API shapes in
`components/schemas`).

This is informational documentation — the contract captures what the C++ code produces and what
the Python/JS downstream consumes, making the implicit contract explicit for future maintainers.

---

## Integration Points

| Modified Component | Downstream Consumer | Integration Type | Change Impact |
|--------------------|--------------------|--------------------|---------------|
| `EngineWrapper::getFrameState()` | `live.py` simulation loop | Python dict | Additive: new `is_environment` field and environment body entries in `states` |
| `SpawnObjectConfig` | `live.py::live_simulation` | Pydantic model | Tightening: invalid inputs now rejected at parse time |
| `_run_simulation` | (none — dead code) | None | Removal: no callers |
| `onStartSimulation` configure payload | `SpawnObjectConfig` validation | JSON over WebSocket | Payload cleanup: fewer fields sent for environment objects |
| `contracts.yaml` | Developers and downstream tickets | Documentation | Additive: new schema section for pybind11 return types |

---

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|-----------------|
| `msd/msd-pybind/test/test_engine_bindings.py` | `test_get_frame_state_*` | May fail if tests assert exact states list length or assert no `is_environment` key | Update to handle new `is_environment` field on inertial entries; update state count if environment objects are spawned in test |
| `replay/tests/test_live_api.py` | Tests sending `object_type` other than `"inertial"/"environment"` | Will now correctly fail with 422 / ValidationError | No action needed — this is the intended new behavior |
| `replay/tests/test_live_api.py` | Tests sending 2-element position or orientation | Will now correctly fail at Pydantic validation layer | No action needed — this is the intended new behavior |
| `replay/tests/test_live_api.py` | Any test that calls `_run_simulation` directly | Will fail if such a test exists | Remove or refactor (dead function removal) |

### New Tests Required

#### C++ Pybind11 Tests (`test_engine_bindings.py`)

| Test Case | What It Validates |
|-----------|-------------------|
| `test_get_frame_state_includes_environment_objects` | When an environment object is spawned, `get_frame_state()["states"]` includes an entry with `is_environment: True`, zeroed velocity, and zeroed angular_velocity |
| `test_get_frame_state_environment_velocity_is_zero` | Environment body entry has `velocity == {"x": 0, "y": 0, "z": 0}` at every step |
| `test_get_frame_state_inertial_has_is_environment_false` | Inertial body entries have `is_environment: False` |
| `test_get_frame_state_environment_position_matches_spawn` | Environment body position matches spawn coordinates |

#### Python API Tests (`test_live_api.py`)

| Test Case | What It Validates |
|-----------|-------------------|
| `test_configure_invalid_object_type_rejected` | Sending `object_type: "kinematic"` results in a ValidationError / error message |
| `test_configure_short_position_rejected` | Sending `position: [0, 1]` results in a ValidationError / error message |
| `test_configure_long_position_rejected` | Sending `position: [0, 1, 2, 3]` results in a ValidationError / error message |
| `test_configure_short_orientation_rejected` | Sending `orientation: [0, 1]` results in ValidationError |
| `test_configure_long_orientation_rejected` | Sending `orientation: [0, 1, 2, 3]` results in ValidationError |

#### Frontend Tests

No automated frontend tests exist or are introduced. The FR-6 change is verified by code inspection (the payload builder conditional is straightforward).

---

## Open Questions

### Design Decisions (Human Input Needed)

**None blocking** — All design decisions are resolved per the ticket's "Preferred Approaches" section and the retroactive review recommendations.

---

### Requirements Clarification

1. **FR-1: `is_environment` field on inertial body entries**

   The ticket requires environment objects to appear in `get_frame_state()`. This design also adds
   `is_environment: false` to **inertial body entries** so that consumers have a uniform shape for
   all entries in `states`. The ticket does not explicitly require this for inertial entries, but it
   is the correct design choice for a discriminated union.

   **Decision in ticket**: "FR-1: Iterate `worldModel.getEnvironmentalObjects()` in `getFrameState()`,
   include with `is_environment: true` flag" — the flag language implies it is present on
   environment entries specifically. This design adds it to both to maintain a uniform schema.
   If the reviewer prefers it on environment entries only, this can be adjusted without other
   design changes.

2. **FR-2: API contract doc format**

   The ticket asks for "a simple YAML or markdown schema" for `docs/api-contracts/`. The open
   question in the ticket is whether it lives in `docs/api-contracts/` as a new directory or
   inline in the 0072a design doc.

   **Decision in this design**: Add to `contracts.yaml` under a new `x-pybind11-schemas` key.
   This keeps all API contracts in a single file, consistent with the existing structure.
   The pybind11 schemas section is informational (not REST/WebSocket), clearly marked with
   an `x-` prefix per OpenAPI extension conventions.

---

## Notes

### `spawn_inertial_object` Signature in `live.py` — Not Part of This Ticket

The retroactive review (0072b, GAP-001 through GAP-003) noted that `spawn_inertial_object` is
called in `live.py` with positional list arguments:

```python
engine.spawn_inertial_object(
    cfg.asset_name,
    cfg.position,    # list[float]
    cfg.orientation, # list[float]
    cfg.mass,
    ...
)
```

However, the C++ binding signature is:

```python
spawn_inertial_object(asset_name, x, y, z, pitch=0.0, roll=0.0, yaw=0.0, ...)
```

This means `cfg.position` (a list) is being passed as the `x` argument, which would pass the
list object into the C++ wrapper. Either:
- The tests use mocks that accept arbitrary arguments (and this is never tested with a real engine), or
- The actual call signature in `live.py` unpacks the list (`*cfg.position`)

This is out of scope for this ticket but should be verified during implementation. If the real
call is `engine.spawn_inertial_object(cfg.asset_name, *cfg.position, *cfg.orientation, ...)`,
then FR-4 (adding length constraints) is especially important — a wrong-length list would cause
a C++ argument count error.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-21
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `getFrameState()` / `spawnInertialObject()` follow existing camelCase method naming; `is_environment` follows snake_case field naming consistent with the rest of the frame state dict |
| Namespace organization | ✓ | No new namespaces introduced; all changes are confined to existing files and packages |
| File/folder structure | ✓ | `engine_bindings.cpp` stays in `msd/msd-pybind/src/`; `models.py` and `live.py` stay in `replay/replay/`; contract doc stays in `docs/api-contracts/contracts.yaml`. No new directories introduced |
| Dependency direction | ✓ | No new dependencies introduced. `engine_bindings.cpp` already uses `WorldModel::getEnvironmentalObjects()` (via `Engine::getWorldModel()`); that method exists and returns `const std::vector<AssetEnvironment>&`. Dependency direction is unchanged |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII | ✓ | No resources introduced. The `py::list` / `py::dict` objects in `getFrameState()` are pybind11-managed RAII types — no manual cleanup needed |
| Smart pointer appropriateness | ✓ | No smart pointers introduced. `AssetEnvironment` is accessed via const reference from `WorldModel::getEnvironmentalObjects()` which returns `const std::vector<AssetEnvironment>&` — correct non-owning access |
| Value/reference semantics | ✓ | Loop body uses `const auto& asset` range-for — correct, avoids unnecessary copies of AssetEnvironment. The `getReferenceFrame()` and `getInertialState()` calls return const references, consistent with the existing inertial body loop |
| Rule of 0/3/5 | ✓ | No new C++ classes introduced. `AssetEnvironment` follows Rule of Five already (custom move, deleted copy) — no impact from this change |
| Const correctness | ✓ | `getFrameState()` is non-const (consistent with existing signature); all asset accessors called on const references. `static_state_` inside `AssetEnvironment` is returned as `const InertialState&` — correct |
| Exception safety | ✓ | No exception paths added. pybind11 py::dict/py::list constructors and `append()` may throw `py::error_already_set` on memory exhaustion, but this is equivalent to the existing inertial body loop behavior |
| Initialization | ✓ | Zero velocity/angular_velocity dicts will be initialized with literal `0.0` values — no NaN concern since these are wire protocol values, not C++ member variables. Brace initialization is not applicable to py::dict construction (uses `[]` operator, pybind11 idiom) |
| Return values | ✓ | Existing pattern of building and returning `py::dict` is preserved. No output parameters introduced |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | `WorldModel::getEnvironmentalObjects()` is already available in the scope of `EngineWrapper::getFrameState()` via `engine_.getWorldModel()`. `AssetEnvironment.hpp` is included transitively. No new headers needed |
| Template complexity | ✓ | No templates introduced |
| Memory strategy | ✓ | Environment objects are appended to the existing `py::list states` — no additional allocation strategy needed |
| Thread safety | ✓ | `getFrameState()` is called from `asyncio.to_thread` in `live.py` (one call at a time), same as before. The underlying simulation is single-threaded. No thread safety regression |
| Build integration | ✓ | No new source files, no CMakeLists.txt changes required. All changes are confined to `.cpp`, `.py`, `.js`, and `.yaml` files already in the build |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | The new loop in `getFrameState()` is exercised by spawning an environment object and calling `get_frame_state()` — same pattern as existing inertial body tests in `test_engine_bindings.py` |
| Mockable dependencies | ✓ | Python-layer changes (FR-3, FR-4) use Pydantic validation which is directly testable by constructing `SpawnObjectConfig` with invalid inputs — no mocking required. FR-5 is a deletion — trivially testable by confirming the symbol no longer exists |
| Observable state | ✓ | `get_frame_state()["states"]` is a plain Python list of dicts — all fields directly inspectable in tests without any helper infrastructure |

---

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | `AssetEnvironment::getInertialState()` returns `static_state_` which is initialized in the constructor from the spawn-time frame but does NOT store orientation as a quaternion in `InertialState::orientation` in the expected WXYZ format — the constructor path from `spawnEnvironmentObject` uses `AngularCoordinate` (pitch/roll/yaw), and the conversion to quaternion happens inside `AssetEnvironment`'s constructor | Technical | Low | Low | Verify in implementation that `static_state_.orientation` is a valid quaternion (not a zero or NaN quaternion). If `AssetEnvironment` is constructed with a zero angular coordinate, Eigen's `AngleAxisd` path should produce the identity quaternion w=1,x=y=z=0. This should be confirmed by the test `test_get_frame_state_environment_position_matches_spawn` | No |
| R2 | `spawn_inertial_object` list-unpacking concern noted in design Notes section: if `live.py` passes `cfg.position` as a list object (not `*cfg.position`), FR-4 length constraints are the only safety net preventing a C++ type mismatch | Integration | Low | Medium | FR-4 is precisely the right mitigation. The implementation must verify the actual call site in `live.py` and confirm list unpacking (`*cfg.position`) is used, or the binding call will fail at the Python/C++ boundary regardless of length validation | No |
| R3 | The `BodyState` schema in `contracts.yaml` (line 478) does not include `is_environment` or `asset_id` fields, but the pybind11 `EngineFrameState` schema (in `x-pybind11-schemas`) does. The live WebSocket `frame` message sends `EngineFrameState` data, not `BodyState` — so the two schemas describe different shapes at different layers. This inconsistency already existed before this ticket but becomes more pronounced with FR-1 adding `is_environment` | Maintenance | Low | Low | Acceptable for now — the design correctly places the new schema under `x-pybind11-schemas` as a separate layer. Future tickets that formalize the live frame message in the REST/WebSocket `components/schemas` section can reconcile the two schemas. No action needed in this ticket | No |

---

### Notes

#### FR-1: `is_environment` on Inertial Entries — Design Choice Endorsed

The design adds `is_environment: false` to inertial body entries as well as `is_environment: true` to environment entries. This is the correct choice for a discriminated union — a uniform schema where all entries have the same fields is significantly easier for downstream consumers (Python and JavaScript) to process without special-casing the absence of a field.

The ticket's preferred approach says "include with `is_environment: true` flag" — this refers to environment objects specifically, but does not preclude adding the flag to inertial entries. The design's interpretation (uniform schema) is endorsed.

The `x-pybind11-schemas/EngineBodyState` section in `contracts.yaml` already correctly documents `is_environment` as required on both body types.

#### FR-2: `x-pybind11-schemas` in `contracts.yaml` — Already Committed

The review notes that the `x-pybind11-schemas` section is already present in the current `docs/api-contracts/contracts.yaml` on this branch (containing both `EngineBodyState` and `EngineFrameState` schemas). This artifact is complete and correctly structured. No further action is required for FR-2 in the design phase.

#### FR-5: Dead Code Removal — Verified Safe

The `_run_simulation` function at lines 88–134 of `live.py` is confirmed dead code by inspection: `grep "def "` over the file shows only `_build_asset_geometries`, `_run_simulation`, `live_simulation`, `_listen_for_stop`, and `list_live_assets` as top-level definitions. No call site for `_run_simulation` exists in `live.py` or any other file. The inline simulation loop in `live_simulation` (lines 295–320) is the real implementation with full stop-signal support via `asyncio.Event`. Deletion is safe.

#### FR-6: Frontend Payload Hygiene — JavaScript Pattern is Clean

The proposed conditional payload builder in `onStartSimulation()` follows idiomatic JavaScript (property assignment after object creation). The existing UI already hides the mass input group for environment objects (`elMassGroup.classList.add('hidden')`), so the payload omission is consistent with the UI behavior. The backend Pydantic defaults (`mass=10.0`, `restitution=0.8`, `friction=0.5`) will apply when these fields are absent, which is the correct behavior.

#### `spawn_inertial_object` Call Site — Must Be Verified

The design correctly flags in its Notes section that `spawn_inertial_object` may be called with `cfg.position` as a list object rather than unpacked as `*cfg.position`. This must be confirmed during implementation. If the call site uses list unpacking, FR-4's length constraint is the last line of defense against a wrong-length list causing a pybind11 argument count error. If the call site does NOT unpack, this is a separate bug (outside this ticket's scope) that FR-4 does not fix but at least makes detectable earlier via a Pydantic 422 rather than a cryptic C++ error.

---

### Summary

The design is clean, minimal, and well-scoped. All six functional requirements are addressed with surgical changes to three existing files plus one documentation artifact. No new classes, endpoints, or architectural layers are introduced. The C++ change (FR-1) correctly uses the existing `AssetEnvironment` API (`getEnvironmentalObjects()`, `getInstanceId()`, `getAssetId()`, `getReferenceFrame()`, `getInertialState()`) in a pattern directly parallel to the existing inertial body loop. The Python changes (FR-3, FR-4) follow Pydantic v2 idioms correctly. The JavaScript change (FR-6) is consistent with the existing UI state management.

The design is approved for progression to Integration Design. The risks identified are low-likelihood or already mitigated by the design itself.
