"""
Live simulation WebSocket endpoint and asset-list REST endpoint.

Ticket: 0072b_websocket_simulation_endpoint

WebSocket: ws://<host>/api/v1/live
REST:       GET /api/v1/live/assets

Wire protocol (client -> server)
---------------------------------
{"type": "configure", "objects": [<SpawnObjectConfig>, ...]}
{"type": "start", "timestep_ms": 16, "duration_s": 30.0}
{"type": "stop"}

Wire protocol (server -> client)
---------------------------------
{"type": "metadata", "bodies": [...], "assets": [...]}
{"type": "frame",    "data": {<FrameData-compatible dict>}}
{"type": "complete", "total_frames": N, "elapsed_s": T}
{"type": "error",    "message": "..."}
"""

import asyncio
import time

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect

from ..config import config
from ..models import AssetGeometry, AssetInfo, LiveBodyMetadata, SpawnObjectConfig

import msd_reader

router = APIRouter(prefix="/live", tags=["live"])


# ---------------------------------------------------------------------------
# Helper: build geometry list for spawned assets
# ---------------------------------------------------------------------------

def _build_asset_geometries(engine, asset_names: list[str]) -> list[dict]:
    """Return flat geometry dicts for each unique asset name.

    Uses Engine.list_assets() to resolve name -> asset_id, then
    Engine.get_collision_vertices(asset_id) for geometry.

    Args:
        engine: msd_reader.Engine instance (already constructed).
        asset_names: Ordered list of unique asset names to resolve.

    Returns:
        List of dicts compatible with AssetGeometry schema.
    """
    # Build name -> asset_id map from engine
    all_assets: list[tuple[int, str]] = engine.list_assets()
    name_to_id: dict[str, int] = {name: aid for aid, name in all_assets}

    seen: set[str] = set()
    geometries: list[dict] = []
    for name in asset_names:
        if name in seen:
            continue
        seen.add(name)
        asset_id = name_to_id.get(name)
        if asset_id is None:
            continue  # already validated; skip silently
        vertices = engine.get_collision_vertices(asset_id)
        positions: list[float] = []
        for v in vertices:
            positions.extend([v[0], v[1], v[2]])
        geometries.append(
            AssetGeometry(
                asset_id=asset_id,
                name=name,
                positions=positions,
                vertex_count=len(vertices),
            ).model_dump()
        )
    return geometries


# ---------------------------------------------------------------------------
# LiveSimulationSession: manages a single WebSocket simulation lifecycle
# ---------------------------------------------------------------------------

class LiveSimulationSession:
    """Manages a single WebSocket live simulation session.

    Each session creates an isolated Engine instance so that physics state
    is fully independent across concurrent connections.

    Lifecycle:
        1. Client sends ``configure`` — server constructs Engine, spawns objects,
           sends ``metadata``.
        2. Client sends ``start`` — server streams ``frame`` messages.
        3. Client may send ``stop`` at any time; server sends ``complete``.
    """

    def __init__(self, websocket: WebSocket, db_path: str):
        self.websocket = websocket
        self.db_path = db_path
        self.engine = None

    async def run(self) -> None:
        """Main entry point — orchestrates the full session lifecycle."""
        await self.websocket.accept()
        try:
            await self._configure()
            timestep_ms, duration_s = await self._wait_for_start()
            if timestep_ms > 0:
                await self._run_simulation(timestep_ms, duration_s)
        except WebSocketDisconnect:
            # Client disconnected mid-session — clean up silently
            pass
        except Exception as exc:
            # Best-effort error reporting before closing
            try:
                await self.websocket.send_json({"type": "error", "message": str(exc)})
            except Exception:
                pass
            try:
                await self.websocket.close()
            except Exception:
                pass
        finally:
            # Engine holds C++ resources; release deterministically
            self.engine = None

    async def _configure(self) -> list[LiveBodyMetadata]:
        """Phase 1: receive configure msg, construct Engine, spawn objects, send metadata."""
        configure_msg: dict = await self.websocket.receive_json()
        if configure_msg.get("type") != "configure":
            await self.websocket.send_json(
                {"type": "error", "message": "Expected 'configure' message first"}
            )
            await self.websocket.close()
            return []

        raw_objects: list[dict] = configure_msg.get("objects", [])
        if not raw_objects:
            await self.websocket.send_json(
                {"type": "error", "message": "'configure' must include at least one object"}
            )
            await self.websocket.close()
            return []

        spawn_configs = [SpawnObjectConfig(**obj) for obj in raw_objects]

        # Construct Engine (one per connection for memory isolation)
        self.engine = msd_reader.Engine(self.db_path)

        # Validate asset names before spawning
        all_assets: list[tuple[int, str]] = self.engine.list_assets()
        valid_names: set[str] = {name for _, name in all_assets}
        for cfg in spawn_configs:
            if cfg.asset_name not in valid_names:
                await self.websocket.send_json(
                    {"type": "error", "message": f"Asset '{cfg.asset_name}' not found"}
                )
                await self.websocket.close()
                return []

        # Spawn objects and capture C++-assigned instance_id for each body.
        # FR-7 (0072e): Use result["instance_id"] as body_id rather than
        # enumerate(), so metadata body_ids are guaranteed to match the
        # body_id values returned by get_frame_state().
        # N2 fix: Unpack position/orientation lists into scalar arguments
        # with *cfg.position / *cfg.orientation so the C++ binding receives
        # individual x, y, z scalars rather than a list object.
        body_metadata: list[LiveBodyMetadata] = []

        for cfg in spawn_configs:
            if cfg.object_type == "inertial":
                result = self.engine.spawn_inertial_object(
                    cfg.asset_name,
                    *cfg.position,      # unpack [x, y, z] → x, y, z scalars
                    *cfg.orientation,   # unpack [pitch, roll, yaw] → scalars
                    cfg.mass,
                    cfg.restitution,
                    cfg.friction,
                )
            else:
                result = self.engine.spawn_environment_object(
                    cfg.asset_name,
                    *cfg.position,      # unpack [x, y, z] → x, y, z scalars
                    *cfg.orientation,   # unpack [pitch, roll, yaw] → scalars
                )

            body_metadata.append(
                LiveBodyMetadata(
                    body_id=result["instance_id"],  # C++-assigned ID (FR-7)
                    asset_id=result["asset_id"],
                    asset_name=cfg.asset_name,
                    mass=cfg.mass,
                    restitution=cfg.restitution,
                    friction=cfg.friction,
                    is_environment=(cfg.object_type == "environment"),
                )
            )

        # Build geometry for unique assets referenced in this session
        asset_names_ordered = [cfg.asset_name for cfg in spawn_configs]
        asset_geometries = await asyncio.to_thread(
            _build_asset_geometries, self.engine, asset_names_ordered
        )

        await self.websocket.send_json(
            {
                "type": "metadata",
                "bodies": [bm.model_dump() for bm in body_metadata],
                "assets": asset_geometries,
            }
        )

        return body_metadata

    async def _wait_for_start(self) -> tuple[int, float]:
        """Phase 2: await start/stop message, return (timestep_ms, duration_s).

        Returns (0, 0.0) if the client sends ``stop`` before ``start``.
        """
        start_msg: dict = await self.websocket.receive_json()
        msg_type = start_msg.get("type")
        if msg_type == "stop":
            await self.websocket.send_json(
                {"type": "complete", "total_frames": 0, "elapsed_s": 0.0}
            )
            return (0, 0.0)
        if msg_type != "start":
            await self.websocket.send_json(
                {"type": "error", "message": f"Expected 'start' or 'stop', got '{msg_type}'"}
            )
            await self.websocket.close()
            return (0, 0.0)

        timestep_ms: int = int(start_msg.get("timestep_ms", 16))
        duration_s: float = float(start_msg.get("duration_s", 30.0))
        if timestep_ms <= 0:
            await self.websocket.send_json(
                {"type": "error", "message": "'timestep_ms' must be positive"}
            )
            await self.websocket.close()
            return (0, 0.0)

        return (timestep_ms, duration_s)

    async def _run_simulation(self, timestep_ms: int, duration_s: float) -> None:
        """Phase 3: simulation loop with stop-signal support."""
        sim_start = time.monotonic()

        # Run simulation loop; client can send "stop" concurrently
        stop_requested = asyncio.Event()

        async def _listen_for_stop() -> None:
            """Background task: watch for a stop message from the client."""
            try:
                while True:
                    msg = await self.websocket.receive_json()
                    if msg.get("type") == "stop":
                        stop_requested.set()
                        return
            except (WebSocketDisconnect, Exception):
                stop_requested.set()

        listener = asyncio.create_task(_listen_for_stop())

        total_frames = 0
        sim_time_ms: int = 0
        max_time_ms: int = int(duration_s * 1000)
        target_interval: float = timestep_ms / 1000.0

        try:
            while sim_time_ms <= max_time_ms and not stop_requested.is_set():
                t0 = time.monotonic()

                await asyncio.to_thread(self.engine.update, sim_time_ms)
                frame_dict: dict = await asyncio.to_thread(self.engine.get_frame_state)
                frame_dict["frame_id"] = total_frames

                await self.websocket.send_json({"type": "frame", "data": frame_dict})

                sim_time_ms += timestep_ms
                total_frames += 1

                elapsed = time.monotonic() - t0
                sleep_for = target_interval - elapsed
                if sleep_for > 0 and not stop_requested.is_set():
                    await asyncio.sleep(sleep_for)

        finally:
            listener.cancel()
            try:
                await listener
            except (asyncio.CancelledError, Exception):
                pass

        elapsed_s = time.monotonic() - sim_start
        await self.websocket.send_json(
            {
                "type": "complete",
                "total_frames": total_frames,
                "elapsed_s": round(elapsed_s, 3),
            }
        )


# ---------------------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------------------

@router.websocket("")
async def live_simulation(websocket: WebSocket) -> None:
    """WebSocket endpoint for live physics simulation.

    Each connection creates an isolated Engine instance.
    """
    session = LiveSimulationSession(websocket, str(config.assets_db_path))
    await session.run()


# ---------------------------------------------------------------------------
# REST: list available assets
# ---------------------------------------------------------------------------

@router.get("/assets", response_model=list[AssetInfo])
async def list_live_assets(request: Request) -> list[AssetInfo]:
    """Return all available asset types from the assets database.

    Uses the persistent Engine instance from app.state (created at startup).

    Returns:
        List of AssetInfo with asset_id and name.
    """
    engine = request.app.state.engine
    assets: list[tuple[int, str]] = engine.list_assets()
    return [AssetInfo(asset_id=aid, name=name) for aid, name in assets]
