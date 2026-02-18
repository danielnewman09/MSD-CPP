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
        """spawn_inertial_object called with mass/restitution/friction from config."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata
                # Don't start — just check spawn was called correctly

        # First object is inertial "cube"
        mock_engine.spawn_inertial_object.assert_called_once_with(
            "cube", [0.0, 0.0, 5.0], [0.0, 0.0, 0.0], 10.0, 0.8, 0.5
        )

    def test_environment_object_spawned_without_physics_params(
        self, client: TestClient
    ) -> None:
        """spawn_environment_object called without mass/restitution/friction."""
        mock_engine = _make_mock_engine()
        with patch("replay.routes.live.msd_reader") as mock_mod:
            mock_mod.Engine.return_value = mock_engine
            with client.websocket_connect("/api/v1/live") as ws:
                ws.send_json(_CONFIGURE_MSG)
                ws.receive_json()  # metadata

        # Second object is environment "large_cube"
        mock_engine.spawn_environment_object.assert_called_once_with(
            "large_cube", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        )
