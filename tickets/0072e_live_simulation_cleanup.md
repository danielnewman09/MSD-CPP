# Feature Ticket: Live Simulation Cleanup

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Ready for Integration Design
- [x] Integration Design Complete — Awaiting Review
- [x] Integration Design Approved
- [x] Ready for Python Design
- [x] Python Design Complete — Awaiting Review
- [x] Ready for Frontend Design
- [x] Frontend Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review (skipped — surgical changes, no algorithmic uncertainty)
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Test Writing
- [x] Test Writing Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-20
- **Author**: Daniel Newman
- **Priority**: Medium
- **Estimated Complexity**: Small
- **Target Component(s)**: msd-pybind, replay
- **Languages**: C++, Python, Frontend
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**: #84

---

## Summary

Address medium-priority issues identified during the retroactive design reviews of tickets 0072a, 0072b, and 0072c. These gaps resulted from the original tickets skipping formal Design, Integration Design, Python Design, and Frontend Design phases. The fixes tighten input validation, add missing output data, remove dead code, and clean up payload hygiene.

## Motivation

The retroactive reviews revealed that missing design phases led to:
1. No formal contract between the C++ pybind11 layer and the Python WebSocket layer — causing missing output fields and unvalidated inputs
2. Input validation gaps that allow malformed data to reach C++ code
3. Dead code and unnecessary payload fields that add confusion

These are not production-breaking bugs, but they represent contract violations and defensive programming gaps that will compound as the live simulation feature evolves.

## Requirements

### Functional Requirements

#### From 0072a retroactive review

1. **FR-1**: `EngineWrapper::getFrameState()` shall include environment objects in the returned `states` array with position, orientation, and zeroed velocity/angular_velocity fields (addresses 0072a-FU-002, requirement R2)
2. **FR-2**: The `get_frame_state()` return schema shall be formally documented as an API contract in `docs/api-contracts/` so downstream consumers (0072b, 0072c) have a single source of truth (addresses 0072a-FU-003)

#### From 0072b retroactive review

3. **FR-3**: `SpawnObjectConfig.object_type` shall use `Literal["inertial", "environment"]` instead of `str` to reject invalid values at the Pydantic validation layer (addresses 0072b-FU-002)
4. **FR-4**: `SpawnObjectConfig.position` and `SpawnObjectConfig.orientation` shall enforce exactly 3 elements using `Field(min_length=3, max_length=3)` (addresses 0072b-FU-003)
5. **FR-5**: Remove the dead `_run_simulation` function from `replay/replay/routes/live.py` (addresses 0072b-FU-001)

#### From 0072c retroactive review

6. **FR-6**: The `configure` message payload in `live-app.js` shall omit `mass`, `restitution`, and `friction` fields for environment objects (addresses 0072c-FU-01)

#### From Integration Design Review

7. **FR-7**: `live.py` configure phase shall capture the `instance_id` returned by `engine.spawn_inertial_object()` and `engine.spawn_environment_object()` and use it as the `body_id` in `LiveBodyMetadata`, replacing the current `enumerate`-based assignment. This guarantees that `body_id` values in the `metadata` message match `body_id` values in `get_frame_state()` frame entries. This fix simultaneously resolves the N2 list-unpacking issue (`*cfg.position`, `*cfg.orientation`) identified in the original design review. (addresses Integration Design Review I1 and I2)

### Non-Functional Requirements
- **Performance**: No impact — validation changes are negligible
- **Memory**: No change
- **Thread Safety**: No change
- **Backward Compatibility**: The wire protocol changes (FR-1, FR-6) are additive or field-removal on optional fields — no breaking changes for existing clients

## Constraints
- FR-1 requires modifying `engine_bindings.cpp` — must not break existing `test_engine_bindings.py` tests
- FR-3 and FR-4 may cause existing tests that send invalid data to fail — update tests accordingly
- FR-6 modifies frontend JavaScript — must not affect the existing replay viewer

## Acceptance Criteria
- [ ] `engine.get_frame_state()["states"]` includes entries for environment objects with `is_environment: true` and zeroed velocity fields
- [ ] API contract document exists at `docs/api-contracts/get_frame_state.yaml` (or equivalent)
- [ ] `SpawnObjectConfig(object_type="typo", ...)` raises a Pydantic `ValidationError`
- [ ] `SpawnObjectConfig(position=[1.0, 2.0], ...)` raises a Pydantic `ValidationError`
- [ ] `_run_simulation` function no longer exists in `live.py`
- [ ] Environment objects in the `configure` message do not include `mass`, `restitution`, or `friction`
- [ ] All existing tests pass (`test_engine_bindings.py`, `test_live_api.py`)
- [ ] New tests cover the added validation (invalid object_type, wrong-length position/orientation)
- [ ] `LiveBodyMetadata.body_id` is sourced from the `instance_id` returned by `engine.spawn_inertial_object()` / `engine.spawn_environment_object()`, not from `enumerate`
- [ ] `engine.spawn_inertial_object()` and `engine.spawn_environment_object()` are called with unpacked position/orientation arguments (`*cfg.position`, `*cfg.orientation`)

---

## Design Decisions (Human Input)

### Preferred Approaches
- FR-1: Iterate `worldModel.getEnvironmentalObjects()` in `getFrameState()`, include with `is_environment: true` flag
- FR-3/FR-4: Use Pydantic v2 `Literal` and `Field` constraints — no custom validators needed
- FR-6: Conditionally build the object config dict in `live-app.js` based on object type

### Things to Avoid
- Do not change the `SceneManager` in `scene.js` — it already handles environment objects correctly
- Do not add a separate endpoint for environment object state — include inline in frame data
- Do not over-engineer the API contract doc — a simple YAML or markdown schema is sufficient

### Open Questions
- Should the API contract for `get_frame_state()` live in `docs/api-contracts/` as a new directory, or inline in the 0072a design doc?

---

## References

### Related Code
- `msd/msd-pybind/src/engine_bindings.cpp` — FR-1: add environment objects to getFrameState()
- `replay/replay/models.py` — FR-3, FR-4: tighten SpawnObjectConfig validation
- `replay/replay/routes/live.py` — FR-5: remove dead _run_simulation function
- `replay/static/js/live-app.js` — FR-6: omit physics fields for environment objects
- `msd/msd-pybind/test/test_engine_bindings.py` — Update tests for FR-1
- `replay/tests/test_live_api.py` — Update tests for FR-3, FR-4

### Related Documentation
- `docs/designs/0072a_engine_pybind_bindings/retroactive-review.md` — Source of FR-1, FR-2
- `docs/designs/0072b-websocket-simulation-endpoint/retroactive-review.md` — Source of FR-3, FR-4, FR-5
- `docs/designs/0072c_live_simulation_frontend/retroactive-review.md` — Source of FR-6

### Related Tickets
- [0072](0072_live_browser_simulation.md) — Parent: Live Browser Simulation
- [0072a](0072a_engine_pybind_bindings.md) — Engine pybind bindings (FR-1, FR-2 source)
- [0072b](0072b_websocket_simulation_endpoint.md) — WebSocket endpoint (FR-3, FR-4, FR-5 source)
- [0072c](0072c_live_simulation_frontend.md) — Live simulation frontend (FR-6 source)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: TBD (draft PR pending push)
- **Artifacts**:
  - `docs/designs/0072e_live_simulation_cleanup/design.md`
  - `docs/designs/0072e_live_simulation_cleanup/0072e_live_simulation_cleanup.puml`
- **Design Decisions**:
  - FR-1: `is_environment` flag added to both inertial and environment entries in `states` for uniform schema
  - FR-2: `EngineFrameState` schema added to `contracts.yaml` under `x-pybind11-schemas` extension key
  - FR-3: `Literal["inertial", "environment"]` annotation on `SpawnObjectConfig.object_type`
  - FR-4: `Annotated[list[float], Field(min_length=3, max_length=3)]` on `position` and `orientation`
  - FR-5: `_run_simulation` function deleted (dead code — never called)
  - FR-6: `mass`/`restitution`/`friction` omitted from configure payload for environment objects
- **Notes**:
  - All six FRs are addressed with surgical changes to three files
  - No new classes or endpoints introduced
  - All changes are either additive (FR-1, FR-2), tightening (FR-3, FR-4), or removal of dead code (FR-5)
  - Open question noted: verify that `live.py` correctly unpacks `cfg.position` as `*cfg.position` when calling the C++ binding

### Design Review Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Status**: APPROVED WITH NOTES
- **Artifacts**:
  - `docs/designs/0072e_live_simulation_cleanup/design.md` (review appended)
- **Reviewer Notes**:
  - All six FRs pass architectural fit, C++ quality, feasibility, and testability criteria
  - `is_environment: false` on inertial entries (uniform discriminated union schema) endorsed
  - `x-pybind11-schemas` approach in contracts.yaml (FR-2) endorsed and already committed
  - FR-2 artifact (`x-pybind11-schemas` in contracts.yaml) confirmed already complete on branch
  - N1: Verify `AssetEnvironment::static_state_.orientation` is a valid unit quaternion in tests
  - N2: Verify list-unpacking (`*cfg.position`) at the `spawn_inertial_object` call site in `live.py` during implementation
  - N3: `BodyState` vs `EngineBodyState` schema discrepancy in contracts.yaml is acceptable for this ticket; reconcile in a future ticket

### Integration Design Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Artifacts**:
  - `docs/designs/0072e_live_simulation_cleanup/integration-design.md`
  - `docs/designs/0072e_live_simulation_cleanup/0072e_live_simulation_cleanup-sequence.puml`
  - `docs/api-contracts/contracts.yaml` — no additional changes needed; Design phase already committed the `x-pybind11-schemas` and constrained `SpawnObjectConfig` schema
- **Notes**:
  - Four formal contracts documented: pybind11 boundary (FR-1), configure validation (FR-3/FR-4/FR-6), frame wire format (FR-1), dead code removal (FR-5)
  - `body_id` consistency guarantee documented: `getInstanceId()` in C++ matches sequential IDs assigned in `live.py` configure phase; clients can join frame state to metadata by `body_id` without additional mapping
  - Pydantic `ValidationError` for FR-3/FR-4 violations propagates through the generic `except Exception` handler in `live.py` — produces an `error` WebSocket message then closes; acceptable for now
  - FR-6 omission of `mass`/`restitution`/`friction` for environment objects fills Pydantic defaults; `metadata` message carries those defaults — frontend ignores them for environment objects
  - Confirmed: `contracts.yaml` already complete (Design phase); no changes needed for FR-2 in this phase
  - spawn_inertial_object call site open question (N2 from design review) retained as an implementation-time verification item

### Integration Design Review Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Status**: APPROVED WITH MANDATORY ADDITION
- **Artifacts**:
  - `docs/designs/0072e_live_simulation_cleanup/integration-design.md` (review appended)
- **Reviewer Notes**:
  - I1 (Critical): `body_id` consistency guarantee in Contract 1 is unsupported by current `live.py` code. Python uses `enumerate(spawn_configs, start=1)` for `body_id` but never captures return values from spawn calls. C++ `getFrameState()` uses `asset.getInstanceId()`. IDs may coincidentally match today but the guarantee is implementation-dependent and unverified.
  - I2 (High): N2 bug confirmed — `live.py` passes `cfg.position` as a `list` object, not `*cfg.position`, to `spawn_inertial_object`. This is a runtime `TypeError` against a real (non-mocked) Engine.
  - Fix: Capture `result = engine.spawn_inertial_object(...)`, use `result["instance_id"]` as `body_id`, unpack `*cfg.position` and `*cfg.orientation` at all spawn call sites.
  - This fix is scoped as FR-7, mandatory addition to Python Design phase.
  - All other six FR contracts are logically correct and complete.
  - Integration design approved to proceed to Python Design with FR-7 added to scope.

### Python Design Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Artifacts**:
  - `docs/designs/0072e_live_simulation_cleanup/python/design.md`
- **Notes**:
  - FR-3: `SpawnObjectConfig.object_type` changed to `Literal["inertial", "environment"]`
  - FR-4: `position` and `orientation` changed to `Annotated[list[float], Field(min_length=3, max_length=3)]`
  - FR-5: `_run_simulation` function deleted (confirmed dead code, no callers)
  - FR-7 (mandatory from integration review): Spawn loop rewritten to capture `result["instance_id"]` and use as `body_id`; `*cfg.position` / `*cfg.orientation` unpacking fixes N2 bug simultaneously
  - Two existing tests (`test_inertial_object_spawned_with_physics_params`, `test_environment_object_spawned_without_physics_params`) must be updated to assert unpacked scalar args
  - `_make_mock_engine` helper must be updated to configure spawn return values
  - New test classes: `TestSpawnObjectConfigValidation`, `TestWebSocketValidationErrors`, `TestDeadCodeRemoval`, `TestBodyIdConsistency`
  - FR-6 has no Python implementation change — Pydantic defaults already handle absent fields

### Python Design Review Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Status**: APPROVED
- **Reviewer Notes**:
  - All FR-3, FR-4, FR-5, FR-7 + N2 fix correctly specified
  - Pydantic v2 `Literal` and `Annotated[list[float], Field(min_length=3, max_length=3)]` idioms are correct
  - FR-7 test uses adversarial non-sequential mock IDs (42, 99) to definitively detect enumerate-based bug
  - `_make_mock_engine` update with spawn return values is the right single place to fix mock behavior
  - Two existing tests updated to assert unpacked scalar args — test suite will correctly validate N2 fix
  - No risks requiring prototype

### Frontend Design Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Artifacts**:
  - `docs/designs/0072e_live_simulation_cleanup/frontend/design.md`
- **Notes**:
  - FR-6: `onStartSimulation()` in `live-app.js` — replace unconditional `objects` map with conditional builder that omits `mass`/`restitution`/`friction` for environment objects
  - No changes to `live.html`, `live.css`, or `scene.js`
  - `handleFrame()` confirmed no-op for FR-1 extended frame data (SceneManager dispatches on body_id only)
  - Change is 4 lines in one function; no new modules, no state changes

### Frontend Design Review Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Status**: APPROVED
- **Reviewer Notes**:
  - Minimal and correct — conditional builder pattern is idiomatic JS
  - Scope correctly bounded: `scene.js` unchanged per ticket constraint
  - FR-1 frame handling confirmed no-op — `SceneManager.updateFrame()` dispatches on body_id; `is_environment` field ignored
  - No risks identified; no prototype needed

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
  - P1: {name} — {result}
- **Artifacts**:
  - `docs/designs/0072e_live_simulation_cleanup/prototype-results.md`
- **Notes**:

### Implementation Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Files Created**: None
- **Files Modified**:
  - `msd/msd-pybind/src/engine_bindings.cpp` — FR-1: environment objects in getFrameState()
  - `replay/replay/models.py` — FR-3: Literal object_type, FR-4: Field length constraints
  - `replay/replay/routes/live.py` — FR-5: removed _run_simulation, FR-7: capture instance_id + *cfg.position unpacking
  - `replay/static/js/live-app.js` — FR-6: conditional physics fields for environment objects
- **Notes**:
  - FR-1: Added second loop over `worldModel.getEnvironmentalObjects()` in `getFrameState()`. Environment entries get `is_environment: true` and zeroed velocity/angular_velocity. Inertial entries get `is_environment: false`. Orientation comes from `asset.getInertialState().orientation` (quaternion set at spawn time).
  - FR-2: Already complete from Design phase (contracts.yaml x-pybind11-schemas section).
  - FR-3: `object_type: str` → `object_type: Literal["inertial", "environment"]`. Added `Literal` to `typing` imports.
  - FR-4: `position/orientation: list[float]` → `Annotated[list[float], Field(min_length=3, max_length=3)]`. Added `Annotated` to `typing` imports and `Field` to `pydantic` imports.
  - FR-5: Deleted `_run_simulation` async function (47 lines). Confirmed no callers existed.
  - FR-6: Replaced unconditional `objects` map in `onStartSimulation()` with conditional builder. Inertial objects still get mass/restitution/friction; environment objects do not.
  - FR-7 + N2 fix: Rewrote spawn loop — removed `enumerate()`, captures `result = engine.spawn_inertial_object(...)`, uses `result["instance_id"]` as `body_id`. All spawn calls now use `*cfg.position` and `*cfg.orientation` to unpack lists into scalars. Also removed the now-unused `name_to_id` dict from the spawn loop (it was only needed when `asset_id` came from `list_assets()`; now it comes from `result["asset_id"]`).
  - Design Review N1: Orientation validity is tested in `test_engine_bindings.py` via `test_environment_body_orientation_is_valid_quaternion`.

### Test Writing Phase
- **Started**: 2026-02-21 00:00
- **Completed**: 2026-02-21 00:00
- **Branch**: 0072e-live-simulation-cleanup
- **PR**: #88
- **Test Files Modified**:
  - `msd/msd-pybind/test/test_engine_bindings.py` — Updated existing test + 9 new tests in `TestGetFrameStateEnvironmentObjects`
  - `replay/tests/test_live_api.py` — Updated 2 existing tests + 4 new test classes (37 total tests, up from 16)
- **Test Coverage Summary**:
  - FR-1: `TestGetFrameStateEnvironmentObjects` (9 tests) — covers environment objects in states, is_environment flag, zero velocity/angular_velocity, position at spawn, orientation is valid quaternion, mixed inertial+environment, velocity stays zero after update
  - FR-3: `TestSpawnObjectConfigValidation` — invalid type raises ValidationError; `TestWebSocketValidationErrors` — propagation to WebSocket error message
  - FR-4: `TestSpawnObjectConfigValidation` — parametrized over short/long/empty position and orientation; `TestWebSocketValidationErrors` — position/orientation length errors over WebSocket
  - FR-5: `TestDeadCodeRemoval` — `test_run_simulation_no_longer_exists` uses `hasattr` check
  - FR-7: `TestBodyIdConsistency` — adversarial non-sequential instance_id (42, 99) test definitively detects enumerate bug; two scalar-argument call-site tests
  - N2: Updated `test_inertial_object_spawned_with_physics_params` and `test_environment_object_spawned_without_physics_params` to assert unpacked scalar args
- **Notes**:
  - All test classes in `test_live_api.py` are skipped when `msd_reader` is not available (standard project convention). Tests run correctly in the build directory where `msd_reader.cpython-*.so` is present.
  - Pydantic model validation tests (FR-3, FR-4) verified directly via Python without msd_reader — all pass.
  - FR-5 dead code removal verified via AST parsing — `_run_simulation` not present in live.py.

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**:
- **Completed**:
- **CLAUDE.md Updates**:
- **Diagrams Indexed**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Integration Design
{Your comments on the integration design, API contracts, wire protocols}

### Feedback on Python Design
{Your comments on the Python architecture}

### Feedback on Frontend Design
{Your comments on the frontend architecture}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

### Feedback on Tests
{Your comments on test coverage, test quality, or missing test scenarios}
