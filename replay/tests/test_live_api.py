"""
Integration tests for the live simulation WebSocket endpoint and asset REST endpoint.

Ticket: 0072b_websocket_simulation_endpoint

These tests require msd_reader to be built and available (via PYTHONPATH).
They are skipped automatically when the module is absent.

Test strategy
-------------
- Use FastAPI's TestClient (httpx) for REST tests.
- Use starlette.testclient.TestClient.websocket_connect() for WebSocket tests.
- Mock msd_reader.Engine where needed so tests run without a real assets.db.
"""

from __future__ import annotations

import json
from typing import Generator
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Availability guard
# ---------------------------------------------------------------------------

try:
    import msd_reader  # noqa: F401

    MSD_READER_AVAILABLE = True
except ImportError:
    MSD_READER_AVAILABLE = False

if MSD_READER_AVAILABLE:
    from fastapi.testclient import TestClient
    from replay.app import app

pytestmark = pytest.mark.skipif(
    not MSD_READER_AVAILABLE, reason="msd_reader module not available"
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def client() -> Generator[TestClient, None, None]:
    """FastAPI test client."""
    with TestClient(app) as c:
        yield c


def _make_mock_engine(asset_list: list[tuple[int, str]] | None = None) -> MagicMock:
    """Return a minimal mock of msd_reader.Engine.

    Args:
        asset_list: List of (asset_id, name) pairs to return from list_assets().
                    Defaults to a small representative set.
    """
    if asset_list is None:
        asset_list = [(1, "cube"), (2, "large_cube")]

    engine = MagicMock()
    engine.list_assets.return_value = asset_list

    # get_collision_vertices returns a list of (x, y, z) tuples
    engine.get_collision_vertices.return_value = [
        (0.5, 0.5, 0.5),
        (-0.5, 0.5, 0.5),
        (0.5, -0.5, 0.5),
    ]

    # get_frame_state returns a FrameData-compatible dict
    engine.get_frame_state.return_value = {
        "simulation_time": 0.016,
        "states": [
            {
                "body_id": 1,
                "position": {"x": 0.0, "y": 0.0, "z": 4.99},
                "velocity": {"x": 0.0, "y": 0.0, "z": -0.16},
                "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        ],
        "collisions": [],
        "friction_constraints": [],
        "solver": None,
    }

    # FR-7 (0072e): spawn methods return C++-assigned instance_id and asset_id.
    # Default values match the default asset_list indices so existing tests
    # continue to pass with body_id=1 (inertial) and body_id=2 (environment).
    engine.spawn_inertial_object.return_value = {"instance_id": 1, "asset_id": 1}
    engine.spawn_environment_object.return_value = {"instance_id": 2, "asset_id": 2}

    return engine


# ---------------------------------------------------------------------------
# REST: GET /api/v1/live/assets
# ---------------------------------------------------------------------------


class TestListLiveAssets:
    """Tests for GET /api/v1/live/assets."""

    def test_returns_200_with_asset_list(self, client: TestClient) -> None:
        """Endpoint returns 200 and a non-empty list when msd_reader available."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            response = client.get("/api/v1/live/assets")

        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)
        assert len(data) == 2

    def test_asset_schema(self, client: TestClient) -> None:
        """Each item has asset_id (int) and name (str)."""
        mock_engine = _make_mock_engine([(3, "sphere"), (7, "capsule")])
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            response = client.get("/api/v1/live/assets")

        assert response.status_code == 200
        data = response.json()
        names = {item["name"] for item in data}
        ids = {item["asset_id"] for item in data}
        assert names == {"sphere", "capsule"}
        assert ids == {3, 7}

    def test_returns_503_when_msd_reader_missing(self, client: TestClient) -> None:
        """Returns 503 if msd_reader is unavailable (simulated by patching to None)."""
        with patch("replay.routes.live.msd_reader", None):
            response = client.get("/api/v1/live/assets")
        assert response.status_code == 503


# ---------------------------------------------------------------------------
# WebSocket: /api/v1/live
# ---------------------------------------------------------------------------

_CONFIGURE_MSG = {
    "type": "configure",
    "objects": [
        {
            "asset_name": "cube",
            "position": [0.0, 0.0, 5.0],
            "orientation": [0.0, 0.0, 0.0],
            "object_type": "inertial",
            "mass": 10.0,
            "restitution": 0.8,
            "friction": 0.5,
        },
        {
            "asset_name": "large_cube",
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0],
            "object_type": "environment",
        },
    ],
}

_START_MSG_SHORT = {"type": "start", "timestep_ms": 16, "duration_s": 0.016}


class TestWebSocketLifecycle:
    """End-to-end lifecycle tests for ws://.../api/v1/live."""

    def test_configure_returns_metadata(self, client: TestClient) -> None:
        """After configure message, server sends metadata with bodies and assets."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                metadata = ws.receive_json()

        assert metadata["type"] == "metadata"
        assert "bodies" in metadata
        assert "assets" in metadata
        assert len(metadata["bodies"]) == 2

    def test_metadata_body_schema(self, client: TestClient) -> None:
        """Each body in metadata has required fields."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                metadata = ws.receive_json()

        required_fields = {
            "body_id", "asset_id", "asset_name", "mass",
            "restitution", "friction", "is_environment",
        }
        for body in metadata["bodies"]:
            assert required_fields.issubset(body.keys()), (
                f"Missing fields in body: {required_fields - body.keys()}"
            )

    def test_metadata_assets_contains_geometry(self, client: TestClient) -> None:
        """Assets in metadata include positions list."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                metadata = ws.receive_json()

        # Both "cube" and "large_cube" should appear
        asset_names = {a["name"] for a in metadata["assets"]}
        assert "cube" in asset_names
        assert "large_cube" in asset_names
        for asset in metadata["assets"]:
            assert "positions" in asset
            assert isinstance(asset["positions"], list)

    def test_start_streams_frames(self, client: TestClient) -> None:
        """After start message, server streams at least one frame message."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

                ws.send_json(_START_MSG_SHORT)

                # Collect until complete
                messages = []
                while True:
                    msg = ws.receive_json()
                    messages.append(msg)
                    if msg["type"] in ("complete", "error"):
                        break

        frame_msgs = [m for m in messages if m["type"] == "frame"]
        assert len(frame_msgs) >= 1, "Expected at least one frame message"

    def test_frame_schema(self, client: TestClient) -> None:
        """Frame messages include frame_id and data dict with required keys."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

                ws.send_json(_START_MSG_SHORT)

                frame_msg = None
                while True:
                    msg = ws.receive_json()
                    if msg["type"] == "frame" and frame_msg is None:
                        frame_msg = msg
                    if msg["type"] in ("complete", "error"):
                        break

        assert frame_msg is not None
        data = frame_msg["data"]
        assert "frame_id" in data
        assert "states" in data
        assert "collisions" in data
        assert data["frame_id"] == 0  # first frame

    def test_complete_message_sent_after_duration(self, client: TestClient) -> None:
        """Server sends 'complete' with total_frames and elapsed_s after full run."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

                ws.send_json(_START_MSG_SHORT)

                complete_msg = None
                while True:
                    msg = ws.receive_json()
                    if msg["type"] == "complete":
                        complete_msg = msg
                        break
                    if msg["type"] == "error":
                        pytest.fail(f"Got error message: {msg}")

        assert complete_msg is not None
        assert "total_frames" in complete_msg
        assert "elapsed_s" in complete_msg
        assert complete_msg["total_frames"] >= 1

    def test_stop_message_terminates_simulation(self, client: TestClient) -> None:
        """Sending stop before start returns immediate complete with zero frames."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

                # Send stop instead of start
                ws.send_json({"type": "stop"})
                msg = ws.receive_json()

        assert msg["type"] == "complete"
        assert msg["total_frames"] == 0

    def test_invalid_asset_name_returns_error(self, client: TestClient) -> None:
        """configure with unknown asset_name triggers error message."""
        mock_engine = _make_mock_engine()  # only has "cube" and "large_cube"
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                bad_configure = {
                    "type": "configure",
                    "objects": [
                        {
                            "asset_name": "nonexistent_asset",
                            "position": [0.0, 0.0, 5.0],
                            "orientation": [0.0, 0.0, 0.0],
                            "object_type": "inertial",
                        }
                    ],
                }
                ws.send_json(bad_configure)
                msg = ws.receive_json()

        assert msg["type"] == "error"
        assert "nonexistent_asset" in msg["message"]

    def test_missing_msd_reader_closes_connection(self, client: TestClient) -> None:
        """If msd_reader is None the WebSocket closes with an error code."""
        with patch("replay.routes.live.msd_reader", None):
            with pytest.raises(Exception):
                # TestClient raises when server closes with code != 1000
                with client.websocket_connect("/api/v1/live") as ws:
                    ws.receive_text()  # triggers the close

    def test_wrong_first_message_type_returns_error(self, client: TestClient) -> None:
        """Sending 'start' before 'configure' triggers an error message."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json({"type": "start", "timestep_ms": 16, "duration_s": 1.0})
                msg = ws.receive_json()

        assert msg["type"] == "error"

    def test_frame_ids_are_sequential(self, client: TestClient) -> None:
        """Frame IDs start at 0 and increment by 1 each frame."""
        mock_engine = _make_mock_engine()
        # Run two full timesteps: duration = 2 * timestep = 32 ms
        start_msg = {"type": "start", "timestep_ms": 16, "duration_s": 0.032}

        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata
                ws.send_json(start_msg)

                frames = []
                while True:
                    msg = ws.receive_json()
                    if msg["type"] == "frame":
                        frames.append(msg["data"]["frame_id"])
                    if msg["type"] in ("complete", "error"):
                        break

        assert frames == list(range(len(frames))), (
            f"Frame IDs not sequential: {frames}"
        )

    def test_inertial_object_spawned_with_physics_params(
        self, client: TestClient
    ) -> None:
        """spawn_inertial_object called with unpacked scalar args and physics params.

        N2 fix (0072e): position/orientation are unpacked into individual scalars
        rather than passed as list objects.
        """
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata
                # Don't start — just check spawn was called correctly

        # First object is inertial "cube" — position and orientation are unpacked
        # into scalars: (name, x, y, z, pitch, roll, yaw, mass, restitution, friction)
        mock_engine.spawn_inertial_object.assert_called_once_with(
            "cube", 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 10.0, 0.8, 0.5
        )

    def test_environment_object_spawned_without_physics_params(
        self, client: TestClient
    ) -> None:
        """spawn_environment_object called with unpacked scalar args only.

        N2 fix (0072e): position/orientation are unpacked into individual scalars.
        No mass/restitution/friction arguments for environment objects.
        """
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

        # Second object is environment "large_cube" — position and orientation
        # are unpacked into scalars: (name, x, y, z, pitch, roll, yaw)
        mock_engine.spawn_environment_object.assert_called_once_with(
            "large_cube", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )


# ---------------------------------------------------------------------------
# Pydantic model validation (FR-3, FR-4 — Ticket 0072e)
# ---------------------------------------------------------------------------


class TestSpawnObjectConfigValidation:
    """Tests for SpawnObjectConfig Pydantic validation (FR-3, FR-4).

    These tests construct SpawnObjectConfig directly (no WebSocket needed)
    to verify that invalid inputs are rejected at the model layer.
    """

    def test_invalid_object_type_raises_validation_error(self) -> None:
        """FR-3: object_type not in {'inertial', 'environment'} raises ValidationError."""
        from pydantic import ValidationError

        from replay.models import SpawnObjectConfig

        with pytest.raises(ValidationError):
            SpawnObjectConfig(
                asset_name="cube",
                position=[0.0, 0.0, 5.0],
                orientation=[0.0, 0.0, 0.0],
                object_type="kinematic",
            )

    def test_valid_inertial_object_type_accepted(self) -> None:
        """FR-3: object_type='inertial' is accepted."""
        from replay.models import SpawnObjectConfig

        cfg = SpawnObjectConfig(
            asset_name="cube",
            position=[0.0, 0.0, 5.0],
            orientation=[0.0, 0.0, 0.0],
            object_type="inertial",
        )
        assert cfg.object_type == "inertial"

    def test_valid_environment_object_type_accepted(self) -> None:
        """FR-3: object_type='environment' is accepted."""
        from replay.models import SpawnObjectConfig

        cfg = SpawnObjectConfig(
            asset_name="plane",
            position=[0.0, 0.0, 0.0],
            orientation=[0.0, 0.0, 0.0],
            object_type="environment",
        )
        assert cfg.object_type == "environment"

    @pytest.mark.parametrize("position", [
        [0.0, 1.0],              # too short (2 elements)
        [0.0, 1.0, 2.0, 3.0],   # too long (4 elements)
        [],                      # empty (0 elements)
    ])
    def test_invalid_position_length_raises_validation_error(
        self, position: list[float]
    ) -> None:
        """FR-4: position with wrong element count raises ValidationError."""
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
        [0.0, 1.0],              # too short (2 elements)
        [0.0, 1.0, 2.0, 3.0],   # too long (4 elements)
        [],                      # empty (0 elements)
    ])
    def test_invalid_orientation_length_raises_validation_error(
        self, orientation: list[float]
    ) -> None:
        """FR-4: orientation with wrong element count raises ValidationError."""
        from pydantic import ValidationError

        from replay.models import SpawnObjectConfig

        with pytest.raises(ValidationError):
            SpawnObjectConfig(
                asset_name="cube",
                position=[0.0, 0.0, 5.0],
                orientation=orientation,
                object_type="inertial",
            )

    def test_valid_three_element_position_accepted(self) -> None:
        """FR-4: position with exactly 3 elements is accepted."""
        from replay.models import SpawnObjectConfig

        cfg = SpawnObjectConfig(
            asset_name="cube",
            position=[1.0, 2.0, 3.0],
            orientation=[0.0, 0.0, 0.0],
            object_type="inertial",
        )
        assert cfg.position == [1.0, 2.0, 3.0]

    def test_valid_three_element_orientation_accepted(self) -> None:
        """FR-4: orientation with exactly 3 elements is accepted."""
        from replay.models import SpawnObjectConfig

        cfg = SpawnObjectConfig(
            asset_name="cube",
            position=[0.0, 0.0, 5.0],
            orientation=[0.1, 0.2, 0.3],
            object_type="inertial",
        )
        assert cfg.orientation == [0.1, 0.2, 0.3]


# ---------------------------------------------------------------------------
# WebSocket-level validation error propagation (FR-3, FR-4 — Ticket 0072e)
# ---------------------------------------------------------------------------


class TestWebSocketValidationErrors:
    """Tests that Pydantic ValidationError propagates to 'error' WebSocket message.

    When invalid configure data is sent over the WebSocket, the server should
    respond with an error message rather than crashing or spawning incorrectly.
    """

    def test_invalid_object_type_triggers_error_message(
        self, client: TestClient
    ) -> None:
        """FR-3: configure with invalid object_type results in error WebSocket message."""
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
        # Pydantic v2 ValidationError string includes the field name or invalid value
        assert "object_type" in msg["message"].lower() or "kinematic" in msg["message"].lower()

    @pytest.mark.parametrize("bad_position", [
        [0.0, 1.0],              # too short
        [0.0, 1.0, 2.0, 3.0],   # too long
    ])
    def test_invalid_position_length_triggers_error_message(
        self, client: TestClient, bad_position: list[float]
    ) -> None:
        """FR-4: configure with wrong-length position results in error WebSocket message."""
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
        """FR-4: configure with wrong-length orientation results in error WebSocket message."""
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


# ---------------------------------------------------------------------------
# Dead code removal verification (FR-5 — Ticket 0072e)
# ---------------------------------------------------------------------------


class TestDeadCodeRemoval:
    """Tests verifying that dead code was removed (FR-5)."""

    def test_run_simulation_no_longer_exists(self) -> None:
        """FR-5: _run_simulation function is no longer defined in the live module."""
        import replay.routes.live as live_module

        assert not hasattr(live_module, "_run_simulation"), (
            "_run_simulation should have been removed as dead code (FR-5, 0072e). "
            "The inline simulation loop in live_simulation() is the only implementation."
        )


# ---------------------------------------------------------------------------
# body_id consistency: sourced from C++ instance_id (FR-7 — Ticket 0072e)
# ---------------------------------------------------------------------------


class TestBodyIdConsistency:
    """Tests verifying body_id in metadata comes from C++ instance_id (FR-7).

    The critical test uses non-sequential mock IDs (42, 99) to definitively
    detect whether the enumerate-based assignment bug has been fixed.
    If enumerate() is still used, body_ids would be {1, 2} instead of {42, 99}.
    """

    def test_metadata_body_id_uses_instance_id_from_spawn(
        self, client: TestClient
    ) -> None:
        """FR-7: body_id in metadata message matches instance_id returned by spawn calls.

        Uses adversarial non-sequential mock IDs (42 and 99) to detect the
        enumerate-based body_id assignment bug. With the fix, body_ids must be
        {42, 99}. With the bug, they would be {1, 2}.
        """
        mock_engine = _make_mock_engine()
        # Override with non-sequential IDs to expose the enumerate bug
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
            "Likely still using enumerate-based body_id assignment (FR-7 not fixed)."
        )

    def test_spawn_inertial_called_with_unpacked_position(
        self, client: TestClient
    ) -> None:
        """FR-7 / N2: spawn_inertial_object is called with unpacked scalar position args."""
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
        """FR-7 / N2: spawn_environment_object is called with unpacked scalar position args."""
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

    def test_metadata_body_ids_match_sequential_instance_ids(
        self, client: TestClient
    ) -> None:
        """FR-7: body_ids in metadata match the default sequential instance_ids from mock."""
        mock_engine = _make_mock_engine()
        # Default mock returns instance_id=1 for inertial, instance_id=2 for environment
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                metadata = ws.receive_json()

        assert metadata["type"] == "metadata"
        body_ids = {b["body_id"] for b in metadata["bodies"]}
        assert body_ids == {1, 2}
