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

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..config import config
from ..models import AssetGeometry, AssetInfo, LiveBodyMetadata, SpawnObjectConfig

try:
    import msd_reader
except ImportError:
    msd_reader = None

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
# Simulation loop
# ---------------------------------------------------------------------------

async def _run_simulation(
    engine,
    websocket: WebSocket,
    timestep_ms: int,
    duration_s: float,
    frame_count_start: int = 0,
) -> int:
    """Stream frame data over *websocket* until duration or disconnect.

    CPU-bound Engine calls are offloaded to a thread-pool via
    ``asyncio.to_thread`` so the event loop stays responsive.

    Args:
        engine: Constructed msd_reader.Engine (objects already spawned).
        websocket: Active FastAPI WebSocket connection.
        timestep_ms: Simulation step size in milliseconds.
        duration_s: Total simulation time in seconds.
        frame_count_start: Frame counter offset (default 0).

    Returns:
        Total number of frames sent.
    """
    sim_time_ms: int = 0
    max_time_ms: int = int(duration_s * 1000)
    frame_count: int = frame_count_start
    target_interval: float = timestep_ms / 1000.0

    while sim_time_ms <= max_time_ms:
        t0 = time.monotonic()

        # Run physics step in thread pool (Engine is not coroutine-safe)
        await asyncio.to_thread(engine.update, sim_time_ms)
        frame_dict: dict = await asyncio.to_thread(engine.get_frame_state)
        frame_dict["frame_id"] = frame_count

        await websocket.send_json({"type": "frame", "data": frame_dict})

        sim_time_ms += timestep_ms
        frame_count += 1

        # Real-time pacing: yield remaining slice to event loop
        elapsed = time.monotonic() - t0
        sleep_for = target_interval - elapsed
        if sleep_for > 0:
            await asyncio.sleep(sleep_for)

    return frame_count


# ---------------------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------------------

@router.websocket("")
async def live_simulation(websocket: WebSocket) -> None:
    """WebSocket endpoint for live physics simulation.

    Lifecycle
    ---------
    1. Client sends ``configure`` message with spawn configs.
    2. Server constructs Engine, spawns objects, sends ``metadata``.
    3. Client sends ``start`` message with timestep and duration.
    4. Server streams ``frame`` messages; client may send ``stop``.
    5. Server sends ``complete`` when duration is reached or ``stop`` received.

    Each connection creates an isolated Engine instance.
    """
    if msd_reader is None:
        await websocket.close(code=1011, reason="msd_reader module not available")
        return

    await websocket.accept()
    engine = None

    try:
        # ------------------------------------------------------------------
        # Phase 1: configure
        # ------------------------------------------------------------------
        configure_msg: dict = await websocket.receive_json()
        if configure_msg.get("type") != "configure":
            await websocket.send_json(
                {"type": "error", "message": "Expected 'configure' message first"}
            )
            await websocket.close()
            return

        raw_objects: list[dict] = configure_msg.get("objects", [])
        if not raw_objects:
            await websocket.send_json(
                {"type": "error", "message": "'configure' must include at least one object"}
            )
            await websocket.close()
            return

        spawn_configs = [SpawnObjectConfig(**obj) for obj in raw_objects]

        # Construct Engine (one per connection for memory isolation)
        engine = msd_reader.Engine(str(config.assets_db_path))

        # Validate asset names before spawning
        all_assets: list[tuple[int, str]] = engine.list_assets()
        valid_names: set[str] = {name for _, name in all_assets}
        for cfg in spawn_configs:
            if cfg.asset_name not in valid_names:
                await websocket.send_json(
                    {"type": "error", "message": f"Asset '{cfg.asset_name}' not found"}
                )
                await websocket.close()
                return

        # Spawn objects; Engine assigns sequential body_ids starting at 1
        body_metadata: list[LiveBodyMetadata] = []
        name_to_id: dict[str, int] = {name: aid for aid, name in all_assets}

        for body_id, cfg in enumerate(spawn_configs, start=1):
            asset_id = name_to_id[cfg.asset_name]
            if cfg.object_type == "inertial":
                engine.spawn_inertial_object(
                    cfg.asset_name,
                    cfg.position,
                    cfg.orientation,
                    cfg.mass,
                    cfg.restitution,
                    cfg.friction,
                )
            else:
                engine.spawn_environment_object(
                    cfg.asset_name,
                    cfg.position,
                    cfg.orientation,
                )

            body_metadata.append(
                LiveBodyMetadata(
                    body_id=body_id,
                    asset_id=asset_id,
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
            _build_asset_geometries, engine, asset_names_ordered
        )

        await websocket.send_json(
            {
                "type": "metadata",
                "bodies": [bm.model_dump() for bm in body_metadata],
                "assets": asset_geometries,
            }
        )

        # ------------------------------------------------------------------
        # Phase 2: wait for start (or stop)
        # ------------------------------------------------------------------
        start_msg: dict = await websocket.receive_json()
        msg_type = start_msg.get("type")
        if msg_type == "stop":
            await websocket.send_json(
                {"type": "complete", "total_frames": 0, "elapsed_s": 0.0}
            )
            return
        if msg_type != "start":
            await websocket.send_json(
                {"type": "error", "message": f"Expected 'start' or 'stop', got '{msg_type}'"}
            )
            await websocket.close()
            return

        timestep_ms: int = int(start_msg.get("timestep_ms", 16))
        duration_s: float = float(start_msg.get("duration_s", 30.0))
        if timestep_ms <= 0:
            await websocket.send_json(
                {"type": "error", "message": "'timestep_ms' must be positive"}
            )
            await websocket.close()
            return

        # ------------------------------------------------------------------
        # Phase 3: simulation loop with stop-signal support
        # ------------------------------------------------------------------
        sim_start = time.monotonic()

        # Run simulation loop; client can send "stop" concurrently
        stop_requested = asyncio.Event()

        async def _listen_for_stop() -> None:
            """Background task: watch for a stop message from the client."""
            try:
                while True:
                    msg = await websocket.receive_json()
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

                await asyncio.to_thread(engine.update, sim_time_ms)
                frame_dict: dict = await asyncio.to_thread(engine.get_frame_state)
                frame_dict["frame_id"] = total_frames

                await websocket.send_json({"type": "frame", "data": frame_dict})

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
        await websocket.send_json(
            {
                "type": "complete",
                "total_frames": total_frames,
                "elapsed_s": round(elapsed_s, 3),
            }
        )

    except WebSocketDisconnect:
        # Client disconnected mid-session — clean up silently
        pass
    except Exception as exc:
        # Best-effort error reporting before closing
        try:
            await websocket.send_json({"type": "error", "message": str(exc)})
        except Exception:
            pass
        try:
            await websocket.close()
        except Exception:
            pass
    finally:
        # Engine holds C++ resources; release deterministically
        engine = None


# ---------------------------------------------------------------------------
# REST: list available assets
# ---------------------------------------------------------------------------

@router.get("/assets", response_model=list[AssetInfo])
async def list_live_assets() -> list[AssetInfo]:
    """Return all available asset types from the assets database.

    Uses a temporary Engine instance to query available assets.

    Returns:
        List of AssetInfo with asset_id and name.
    """
    if msd_reader is None:
        from fastapi import HTTPException
        raise HTTPException(status_code=503, detail="msd_reader module not available")

    engine = msd_reader.Engine(str(config.assets_db_path))
    assets: list[tuple[int, str]] = engine.list_assets()
    return [AssetInfo(asset_id=aid, name=name) for aid, name in assets]
