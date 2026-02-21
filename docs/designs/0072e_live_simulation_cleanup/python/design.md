# Python Design: Live Simulation Cleanup (0072e)

**Ticket**: [0072e_live_simulation_cleanup](../../../../tickets/0072e_live_simulation_cleanup.md)
**Date**: 2026-02-21
**Status**: Initial Python Design
**Depends On**:
- [design.md](../design.md) (APPROVED WITH NOTES)
- [integration-design.md](../integration-design.md) (APPROVED WITH MANDATORY ADDITION)

---

## Summary

This document specifies the internal Python architecture for the 0072e cleanup changes. The changes
are confined to two existing files (`replay/replay/models.py` and `replay/replay/routes/live.py`)
and their associated test file (`replay/tests/test_live_api.py`). No new routes, services, or
Pydantic model classes are introduced — all changes are modifications to existing code.

The scope covers six original functional requirements (FR-1 through FR-6) plus one mandatory
addition identified during the integration design review:

- **FR-3**: `SpawnObjectConfig.object_type` constrained to `Literal["inertial", "environment"]`
- **FR-4**: `SpawnObjectConfig.position` and `orientation` constrained to exactly 3 elements
- **FR-5**: Remove dead `_run_simulation` function from `live.py`
- **FR-6**: `configure` message payload omits `mass`/`restitution`/`friction` for environment
  objects (frontend-side; Python side receives and validates via Pydantic defaults — no Python
  code change needed beyond what FR-3/FR-4 already provide)
- **FR-7** (new): Capture `instance_id` from spawn return values; use as `body_id` in
  `LiveBodyMetadata`; unpack position/orientation with `*cfg.position` / `*cfg.orientation`

FR-1 (extend `getFrameState()`) is implemented in C++ and covered by the C++ design phase.
FR-2 (API contract doc) is already complete on the branch.

---

## Integration Contract Reference

- Integration design: `docs/designs/0072e_live_simulation_cleanup/integration-design.md`
- Pydantic model changes: `SpawnObjectConfig` in `replay/replay/models.py`
- Route changes: configure phase in `replay/replay/routes/live.py`
- Contracts to fulfill:
  - Contract 2: WebSocket configure validation (FR-3, FR-4, FR-6)
  - Contract 4: Dead code removal (FR-5)
  - FR-7: `body_id` sourced from C++ `instance_id` (new, mandatory)

---

## Route Architecture

No new routes. One existing route function is modified.

### Modified: `live_simulation` in `replay/replay/routes/live.py`

#### Configure Phase Changes (FR-5, FR-7, N2)

The configure phase loop (lines 202–230 in current `live.py`) is rewritten as follows:

```python
# BEFORE (current code — three problems):
for body_id, cfg in enumerate(spawn_configs, start=1):   # Problem 1: enumerate body_id
    if cfg.object_type == "inertial":
        engine.spawn_inertial_object(
            cfg.asset_name,
            cfg.position,    # Problem 2: list passed as x (N2 bug)
            cfg.orientation, # Problem 2: list passed as pitch
            cfg.mass,
            cfg.restitution,
            cfg.friction,
        )
    else:
        engine.spawn_environment_object(
            cfg.asset_name,
            cfg.position,    # Problem 2: same bug
            cfg.orientation,
        )

    body_metadata.append(
        LiveBodyMetadata(
            body_id=body_id,        # Problem 1: enumerate ID, not C++ instance_id
            asset_id=asset_id,      # Problem 3: from list_assets(), not spawn return
            ...
        )
    )

# AFTER (FR-7 + N2 fix):
for cfg in spawn_configs:
    if cfg.object_type == "inertial":
        result = engine.spawn_inertial_object(
            cfg.asset_name,
            *cfg.position,      # Unpack [x, y, z] into x, y, z scalars (N2 fix)
            *cfg.orientation,   # Unpack [pitch, roll, yaw] into scalars (N2 fix)
            cfg.mass,
            cfg.restitution,
            cfg.friction,
        )
    else:
        result = engine.spawn_environment_object(
            cfg.asset_name,
            *cfg.position,      # Unpack [x, y, z] (N2 fix)
            *cfg.orientation,   # Unpack [pitch, roll, yaw] (N2 fix)
        )

    body_metadata.append(
        LiveBodyMetadata(
            body_id=result["instance_id"],  # C++-assigned ID (FR-7 fix)
            asset_id=result["asset_id"],    # C++-returned asset_id (FR-7 secondary fix)
            asset_name=cfg.asset_name,
            mass=cfg.mass,
            restitution=cfg.restitution,
            friction=cfg.friction,
            is_environment=(cfg.object_type == "environment"),
        )
    )
```

The `name_to_id` dict (built from `engine.list_assets()`) is no longer needed for the spawn loop
body metadata construction. It is still needed for the `_build_asset_geometries` helper call
(which uses `asset_id` from `list_assets()` to look up geometry). The `name_to_id` dict is
retained for that purpose only.

#### FR-5: Dead Code Removal

Delete `_run_simulation` (lines 88–134 in current `live.py`) entirely. The function signature,
docstring, and body are all removed. No replacement is needed — the inline loop in
`live_simulation` is the only implementation.

After removal, the top-level definitions in `live.py` are:

| Symbol | Type | Role |
|--------|------|------|
| `_build_asset_geometries` | function | Helper: resolves asset names to geometry |
| `live_simulation` | async function | WebSocket endpoint |
| `_listen_for_stop` | nested function | Background stop-signal listener (inside `live_simulation`) |
| `list_live_assets` | async function | REST GET /live/assets |

---

## Pydantic Model Changes

### Modified: `SpawnObjectConfig` in `replay/replay/models.py`

#### FR-3: `object_type` constrained to `Literal["inertial", "environment"]`

```python
# BEFORE:
from pydantic import BaseModel

class SpawnObjectConfig(BaseModel):
    object_type: str           # "inertial" or "environment"

# AFTER:
from typing import Literal
from pydantic import BaseModel

class SpawnObjectConfig(BaseModel):
    object_type: Literal["inertial", "environment"]
```

Pydantic v2 validates `Literal` fields at parse time. Passing `"kinematic"` or any other string
will raise `ValidationError` before any C++ call is made.

#### FR-4: `position` and `orientation` constrained to exactly 3 elements

```python
# BEFORE:
class SpawnObjectConfig(BaseModel):
    position: list[float]      # [x, y, z] in metres
    orientation: list[float]   # [pitch, roll, yaw] in radians

# AFTER:
from typing import Annotated
from pydantic import BaseModel, Field

class SpawnObjectConfig(BaseModel):
    position: Annotated[list[float], Field(min_length=3, max_length=3)]
    orientation: Annotated[list[float], Field(min_length=3, max_length=3)]
```

`Annotated[list[float], Field(min_length=3, max_length=3)]` is the Pydantic v2 idiomatic pattern
for list-length constraints. The constraint is enforced at parse time:
- `[0, 1]` → `ValidationError` (too short)
- `[0, 1, 2, 3]` → `ValidationError` (too long)
- `[0.0, 1.0, 2.0]` → passes

#### Required Import Changes

```python
# Add to top of models.py:
from typing import Annotated, Literal
from pydantic import BaseModel, Field  # Field is new
```

`Annotated` and `Literal` come from the standard library `typing` module (Python 3.8+, already
in use). `Field` from `pydantic` is not currently imported in `models.py` — it must be added.

---

## Error Handling

No changes to error handling in this phase. Existing behavior:

| Error Source | Behavior | Notes |
|-------------|----------|-------|
| `SpawnObjectConfig(**obj)` — `ValidationError` (FR-3/FR-4) | Propagates to outer `except Exception as exc` handler; sends `{"type": "error", "message": str(exc)}`; closes connection | Already correct — no explicit `ValidationError` catch needed |
| `engine.spawn_inertial_object(...)` — `RuntimeError` from C++ | Propagates to outer `except Exception as exc` handler; same error response pattern | No change |
| `KeyError` on `result["instance_id"]` | Propagates to outer `except Exception as exc` handler | Would indicate a pybind11 API contract violation — acceptable to surface as generic error |

The outer `except Exception as exc` handler in `live_simulation` (lines 335–344 in current
`live.py`) is adequate for all error cases in this ticket.

---

## Async Patterns

No changes to async patterns. The configure phase spawn calls are synchronous (fast, no I/O), so
they remain direct calls (not `asyncio.to_thread`). The `_build_asset_geometries` call remains
wrapped in `asyncio.to_thread` as before.

---

## Test Strategy

### Test Files to Modify

| Test File | Tests Impacted | Action |
|-----------|----------------|--------|
| `replay/tests/test_live_api.py` | `test_inertial_object_spawned_with_physics_params` | Update mock assertion to use unpacked arguments |
| `replay/tests/test_live_api.py` | `test_environment_object_spawned_without_physics_params` | Update mock assertion to use unpacked arguments |
| `replay/tests/test_live_api.py` | All | Add mock `spawn_inertial_object.return_value` and `spawn_environment_object.return_value` to return `{"instance_id": N, "asset_id": M}` |

### Existing Test Updates Required

#### Update `_make_mock_engine` Fixture

The `_make_mock_engine` helper must be extended to configure spawn return values:

```python
def _make_mock_engine(asset_list: list[tuple[int, str]] | None = None) -> MagicMock:
    ...
    # NEW: spawn methods return instance_id and asset_id dicts
    engine.spawn_inertial_object.return_value = {"instance_id": 1, "asset_id": 1}
    engine.spawn_environment_object.return_value = {"instance_id": 2, "asset_id": 2}
    ...
```

#### Update `test_inertial_object_spawned_with_physics_params`

```python
# BEFORE (current test — asserts old broken calling convention):
mock_engine.spawn_inertial_object.assert_called_once_with(
    "cube", [0.0, 0.0, 5.0], [0.0, 0.0, 0.0], 10.0, 0.8, 0.5
)

# AFTER (correct unpacked calling convention):
mock_engine.spawn_inertial_object.assert_called_once_with(
    "cube", 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 10.0, 0.8, 0.5
)
```

#### Update `test_environment_object_spawned_without_physics_params`

```python
# BEFORE:
mock_engine.spawn_environment_object.assert_called_once_with(
    "large_cube", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
)

# AFTER:
mock_engine.spawn_environment_object.assert_called_once_with(
    "large_cube", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
)
```

### New Tests Required

#### FR-3: Invalid `object_type` Validation

```python
class TestSpawnObjectConfigValidation:
    """Tests for SpawnObjectConfig Pydantic validation (FR-3, FR-4)."""

    def test_invalid_object_type_raises_validation_error(self) -> None:
        """object_type not in {inertial, environment} raises ValidationError."""
        from pydantic import ValidationError
        from replay.models import SpawnObjectConfig

        with pytest.raises(ValidationError):
            SpawnObjectConfig(
                asset_name="cube",
                position=[0.0, 0.0, 5.0],
                orientation=[0.0, 0.0, 0.0],
                object_type="kinematic",
            )

    def test_valid_object_types_accepted(self) -> None:
        """Both 'inertial' and 'environment' are accepted."""
        from replay.models import SpawnObjectConfig

        cfg_inertial = SpawnObjectConfig(
            asset_name="cube",
            position=[0.0, 0.0, 5.0],
            orientation=[0.0, 0.0, 0.0],
            object_type="inertial",
        )
        assert cfg_inertial.object_type == "inertial"

        cfg_env = SpawnObjectConfig(
            asset_name="plane",
            position=[0.0, 0.0, 0.0],
            orientation=[0.0, 0.0, 0.0],
            object_type="environment",
        )
        assert cfg_env.object_type == "environment"
```

#### FR-4: Position and Orientation Length Validation

```python
    @pytest.mark.parametrize("position", [
        [0.0, 1.0],          # too short
        [0.0, 1.0, 2.0, 3.0],  # too long
        [],                   # empty
    ])
    def test_invalid_position_length_raises_validation_error(
        self, position: list[float]
    ) -> None:
        """position with wrong element count raises ValidationError."""
        from pydantic import ValidationError
        from replay.models import SpawnObjectConfig

        with pytest.raises(ValidationError):
            SpawnObjectConfig(
                asset_name="cube",
                position=position,
                orientation=[0.0, 0.0, 0.0],
                object_type="inertial",
            )

    @pytest.mark.parametrize("orientation", [
        [0.0, 1.0],          # too short
        [0.0, 1.0, 2.0, 3.0],  # too long
        [],                   # empty
    ])
    def test_invalid_orientation_length_raises_validation_error(
        self, orientation: list[float]
    ) -> None:
        """orientation with wrong element count raises ValidationError."""
        from pydantic import ValidationError
        from replay.models import SpawnObjectConfig

        with pytest.raises(ValidationError):
            SpawnObjectConfig(
                asset_name="cube",
                position=[0.0, 0.0, 5.0],
                orientation=orientation,
                object_type="inertial",
            )
```

#### FR-3 and FR-4: WebSocket-Level Validation Error Propagation

```python
class TestWebSocketValidationErrors:
    """Tests that Pydantic ValidationError propagates to 'error' WebSocket message."""

    def test_invalid_object_type_triggers_error_message(
        self, client: TestClient
    ) -> None:
        """configure with invalid object_type results in error WebSocket message."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json({
                    "type": "configure",
                    "objects": [{
                        "asset_name": "cube",
                        "position": [0.0, 0.0, 5.0],
                        "orientation": [0.0, 0.0, 0.0],
                        "object_type": "kinematic",  # invalid
                    }]
                })
                msg = ws.receive_json()

        assert msg["type"] == "error"
        assert "object_type" in msg["message"].lower() or "kinematic" in msg["message"].lower()

    @pytest.mark.parametrize("bad_position", [
        [0.0, 1.0],       # too short
        [0.0, 1.0, 2.0, 3.0],  # too long
    ])
    def test_invalid_position_length_triggers_error_message(
        self, client: TestClient, bad_position: list[float]
    ) -> None:
        """configure with wrong-length position results in error WebSocket message."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json({
                    "type": "configure",
                    "objects": [{
                        "asset_name": "cube",
                        "position": bad_position,
                        "orientation": [0.0, 0.0, 0.0],
                        "object_type": "inertial",
                    }]
                })
                msg = ws.receive_json()

        assert msg["type"] == "error"

    @pytest.mark.parametrize("bad_orientation", [
        [0.0, 1.0],
        [0.0, 1.0, 2.0, 3.0],
    ])
    def test_invalid_orientation_length_triggers_error_message(
        self, client: TestClient, bad_orientation: list[float]
    ) -> None:
        """configure with wrong-length orientation results in error WebSocket message."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json({
                    "type": "configure",
                    "objects": [{
                        "asset_name": "cube",
                        "position": [0.0, 0.0, 5.0],
                        "orientation": bad_orientation,
                        "object_type": "inertial",
                    }]
                })
                msg = ws.receive_json()

        assert msg["type"] == "error"
```

#### FR-5: Dead Code Removal Verification

```python
class TestDeadCodeRemoval:
    """Tests verifying dead code was removed (FR-5)."""

    def test_run_simulation_no_longer_exists(self) -> None:
        """_run_simulation function is no longer defined in live module."""
        import replay.routes.live as live_module

        assert not hasattr(live_module, "_run_simulation"), (
            "_run_simulation should have been removed (dead code, FR-5)"
        )
```

#### FR-7: `body_id` Sourced from C++ `instance_id`

```python
class TestBodyIdConsistency:
    """Tests verifying body_id in metadata comes from C++ instance_id (FR-7)."""

    def test_metadata_body_id_uses_instance_id_from_spawn(
        self, client: TestClient
    ) -> None:
        """body_id in metadata message matches instance_id returned by spawn calls."""
        mock_engine = _make_mock_engine()
        # Configure spawn to return non-sequential instance_ids to detect enumerate bug
        mock_engine.spawn_inertial_object.return_value = {"instance_id": 42, "asset_id": 1}
        mock_engine.spawn_environment_object.return_value = {"instance_id": 99, "asset_id": 2}

        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                metadata = ws.receive_json()

        assert metadata["type"] == "metadata"
        body_ids = {b["body_id"] for b in metadata["bodies"]}
        assert body_ids == {42, 99}, (
            f"Expected body_ids {{42, 99}} from C++ instance_id, got {body_ids}. "
            "Likely still using enumerate-based body_id assignment."
        )

    def test_spawn_inertial_called_with_unpacked_position(
        self, client: TestClient
    ) -> None:
        """spawn_inertial_object is called with unpacked scalar position args (N2 fix)."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

        # Assert unpacked scalar args: (name, x, y, z, pitch, roll, yaw, mass, rest, fric)
        mock_engine.spawn_inertial_object.assert_called_once_with(
            "cube", 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 10.0, 0.8, 0.5
        )

    def test_spawn_environment_called_with_unpacked_position(
        self, client: TestClient
    ) -> None:
        """spawn_environment_object is called with unpacked scalar position args (N2 fix)."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

        # Assert unpacked scalar args: (name, x, y, z, pitch, roll, yaw)
        mock_engine.spawn_environment_object.assert_called_once_with(
            "large_cube", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )
```

---

## Mock Strategy

| Dependency | Current Mock | Updated Mock (this ticket) |
|-----------|--------------|---------------------------|
| `msd_reader.Engine` | `MagicMock()` with `list_assets`, `get_collision_vertices`, `get_frame_state` configured | Add `spawn_inertial_object.return_value = {"instance_id": 1, "asset_id": 1}` and `spawn_environment_object.return_value = {"instance_id": 2, "asset_id": 2}` to `_make_mock_engine` |
| Pydantic models | Not mocked — real `SpawnObjectConfig` validates | No change — real Pydantic validation is the point of FR-3/FR-4 tests |

The `_make_mock_engine` helper is the single place to add spawn return values. All existing
tests that call `_make_mock_engine` will automatically gain the new return values, and their
existing assertions about metadata structure will continue to work (body_id=1, body_id=2 from
the default mock return values).

The critical FR-7 test (`test_metadata_body_id_uses_instance_id_from_spawn`) overrides the
spawn return values with non-sequential IDs (42, 99) to prove the fix is in place.

---

## Files to Create

None. This phase creates no new files.

---

## Files to Modify

| File | Changes | FRs |
|------|---------|-----|
| `replay/replay/models.py` | Add `Annotated`, `Literal` imports; add `Field` from pydantic; change `object_type: str` to `object_type: Literal["inertial", "environment"]`; change `position` and `orientation` to `Annotated[list[float], Field(min_length=3, max_length=3)]` | FR-3, FR-4 |
| `replay/replay/routes/live.py` | Delete `_run_simulation` function; rewrite configure spawn loop to capture return values, use `*cfg.position` / `*cfg.orientation` unpacking, use `result["instance_id"]` as `body_id` | FR-5, FR-7, N2 |
| `replay/tests/test_live_api.py` | Update `_make_mock_engine` spawn return values; update two existing spawn-call assertion tests; add new test classes for FR-3, FR-4, FR-5, FR-7 | FR-3, FR-4, FR-5, FR-7 |

---

## Dependencies

- Existing: `pydantic` (already in `requirements.txt`), `typing` (stdlib)
- New: None

The `Field` import from `pydantic` is the only new import. `pydantic` is already a project
dependency. `Annotated` and `Literal` are from the standard library `typing` module.

---

## Integration Contract Coverage

| Contract | Requirement | Coverage |
|----------|-------------|----------|
| Contract 2 — FR-3 | `SpawnObjectConfig.object_type: Literal["inertial", "environment"]` | `SpawnObjectConfig` model change + unit tests + WebSocket propagation test |
| Contract 2 — FR-4 | `SpawnObjectConfig.position` and `orientation` exactly 3 elements | `SpawnObjectConfig` model change + unit tests + WebSocket propagation tests |
| Contract 2 — FR-6 | Environment objects omit `mass`/`restitution`/`friction` | No Python code change needed; Pydantic defaults already fill absent fields. Frontend change is in frontend design phase |
| Contract 4 — FR-5 | `_run_simulation` removed | Delete function + dead code removal verification test |
| FR-7 (Integration Review I1) | `body_id` from `instance_id`; `*cfg.position` unpacking | Spawn loop rewrite + body_id consistency test + spawn call-site tests |

---

## Open Questions

None. All design decisions are resolved.

### Resolved

1. **`name_to_id` dict after FR-7** — The `name_to_id` dict built from `engine.list_assets()` is
   still needed for `_build_asset_geometries` (which requires `asset_id` to look up geometry). It
   is no longer needed for the spawn loop body metadata construction. The dict is retained for
   `_build_asset_geometries` only. The `asset_id` in `LiveBodyMetadata` is now sourced from
   `result["asset_id"]` (C++ return value), which may differ from `name_to_id[cfg.asset_name]`
   if the asset registry uses a different ID for the same asset name. The C++ value is canonical.

2. **ValidationError message content in WebSocket error test** — The test
   `test_invalid_object_type_triggers_error_message` checks that the error message contains
   "object_type" or "kinematic". Pydantic v2 ValidationError messages are structured but contain
   the field name and invalid value. The `str(exc)` passed to the WebSocket error message will
   contain enough context to satisfy this check. The test is lenient enough to not break on
   minor Pydantic version changes.

3. **FR-6 Python-side impact** — FR-6 (omitting `mass`/`restitution`/`friction` for environment
   objects in the frontend `configure` message) has no Python implementation change. The Pydantic
   model already has `mass`, `restitution`, and `friction` as optional fields with defaults.
   When the frontend omits them, the defaults apply. No new Python code is needed.

---

## Python Design Review

**Reviewer**: Python Design Reviewer
**Date**: 2026-02-21
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Design Conformance

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| All integration contract requirements addressed | ✓ | FR-3, FR-4, FR-5, FR-6 (no-op), FR-7 all explicitly covered |
| No new files outside scope | ✓ | All changes confined to `models.py`, `live.py`, `test_live_api.py` |
| No changes to C++ or frontend | ✓ | Scope is correctly bounded |
| Existing patterns followed | ✓ | `Annotated[list[float], Field(...)]` is Pydantic v2 idiom used consistently; `Literal` from `typing` follows project Python style |

#### Pydantic Model Correctness

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| FR-3 `Literal` type is correct | ✓ | `Literal["inertial", "environment"]` raises `ValidationError` on any other value at parse time; this is Pydantic v2 correct |
| FR-4 `Annotated[list[float], Field(min_length=3, max_length=3)]` is correct | ✓ | This is the Pydantic v2 idiomatic pattern; `min_length`/`max_length` constraints on list fields are enforced at parse time |
| Import additions are complete | ✓ | `Annotated`, `Literal` from `typing`; `Field` from `pydantic` — all three additions specified. `Field` is the only non-stdlib new import |
| Existing model defaults preserved | ✓ | `mass=10.0`, `restitution=0.8`, `friction=0.5` defaults on `SpawnObjectConfig` are unchanged |

#### Route Change Correctness

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| FR-7 spawn loop captures return values | ✓ | `result = engine.spawn_inertial_object(...)` / `result = engine.spawn_environment_object(...)` and `body_id=result["instance_id"]` is correct |
| N2 fix `*cfg.position` unpacking is correct | ✓ | `*cfg.position` with `Annotated[list[float], Field(min_length=3, max_length=3)]` guarantees exactly 3 scalars reach the C++ pybind11 `x, y, z` parameters |
| `asset_id` source updated consistently | ✓ | `asset_id=result["asset_id"]` from spawn return value replaces `name_to_id[cfg.asset_name]` — C++ value is canonical |
| `name_to_id` dict retained for geometry | ✓ | Design correctly notes `name_to_id` is still needed for `_build_asset_geometries` and should not be removed |
| FR-5 dead code removal is safe | ✓ | `_run_simulation` is confirmed dead — no callers in the file or test suite |

#### Test Strategy

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| FR-3 unit test (direct Pydantic) | ✓ | `TestSpawnObjectConfigValidation.test_invalid_object_type_raises_validation_error` imports and constructs `SpawnObjectConfig` directly |
| FR-3 integration test (WebSocket) | ✓ | `TestWebSocketValidationErrors.test_invalid_object_type_triggers_error_message` tests error propagation to wire |
| FR-4 parametrized position/orientation tests | ✓ | `@pytest.mark.parametrize` on `[0.0, 1.0]`, `[0.0, 1.0, 2.0, 3.0]`, `[]` covers under, over, and empty cases |
| FR-4 integration test (WebSocket) | ✓ | `@pytest.mark.parametrize` on wrong-length position and orientation covers both directions |
| FR-5 dead code test | ✓ | `TestDeadCodeRemoval.test_run_simulation_no_longer_exists` uses `hasattr` — simple and reliable |
| FR-7 body_id consistency test | ✓ | Uses non-sequential mock IDs (42, 99) — will definitively detect if `enumerate` is still used instead of `result["instance_id"]` |
| FR-7 call-site unpacking tests | ✓ | `assert_called_once_with("cube", 0.0, 0.0, 5.0, ...)` correctly asserts scalars, not list |
| Existing test updates specified | ✓ | Both `test_inertial_object_spawned_with_physics_params` and `test_environment_object_spawned_without_physics_params` updated to assert unpacked scalars |
| `_make_mock_engine` update specified | ✓ | Default spawn return values (`{"instance_id": 1, "asset_id": 1}`, `{"instance_id": 2, "asset_id": 2}`) will make all existing tests continue to pass |

#### Async Correctness

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| No async changes introduced | ✓ | Configure phase spawn calls are synchronous (in-memory, fast) and correctly remain direct calls. `asyncio.to_thread` is not needed and not added |

### Risks Identified

| ID | Risk | Category | Likelihood | Impact | Mitigation |
|----|------|----------|------------|--------|------------|
| R1 | If any existing test passes `position` as a 2-element list (testing other behavior), it will now raise `ValidationError` at parse time instead of propagating to the spawn call — test semantics change | Test | Low | Low | All existing tests in `test_live_api.py` use `[0.0, 0.0, 5.0]` (3 elements) for position — existing tests pass the constraint |
| R2 | `_make_mock_engine` spawn return value defaults (`instance_id=1` for inertial, `instance_id=2` for environment) assume a single object of each type per test session. Tests with multiple inertial objects would need different return value sequences | Test | Low | Low | All existing tests in `_CONFIGURE_MSG` spawn exactly one inertial and one environment object. Acceptable for current scope |

### Summary

The Python design is minimal, correct, and complete. All five in-scope FRs (3, 4, 5, 7 + N2 fix) are addressed with targeted modifications to two production files and one test file. The design correctly identifies that FR-6 requires no Python code change (Pydantic defaults handle the case). The test strategy is thorough: unit tests validate Pydantic directly; WebSocket integration tests validate error propagation; the FR-7 body_id consistency test uses adversarial mock IDs (42, 99) to definitively detect the enumerate-based bug. The design is approved for progression to Frontend Design.
