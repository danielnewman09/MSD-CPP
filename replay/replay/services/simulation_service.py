"""
Simulation Service — Database query wrapper

Ticket: 0056d_fastapi_backend
"""

import sqlite3
from pathlib import Path
from typing import Optional

try:
    import msd_reader
except ImportError:
    msd_reader = None

from ..models import (
    BodyMetadata,
    BodyState,
    CollisionInfo,
    ContactPoint,
    EnergyPoint,
    FrameData,
    FrameInfo,
    Quaternion,
    SimulationMetadata,
    SolverDiagnostics,
    SystemEnergyPoint,
    Vec3,
)


class SimulationService:
    """Service layer for simulation database queries using msd_reader."""

    def __init__(self, db_path: Path):
        """Initialize service with database connection."""
        if msd_reader is None:
            raise RuntimeError(
                "msd_reader module not available. "
            )

        self.db_path = db_path
        self.db = msd_reader.Database(str(db_path))

    def get_frames(self) -> list[FrameInfo]:
        """Get list of all frames with timestamps."""
        frame_records = self.db.select_all_frames()
        return [
            FrameInfo(
                frame_id=frame.id, simulation_time=frame.simulation_time
            )
            for frame in frame_records
        ]

    def get_metadata(self) -> SimulationMetadata:
        """Get simulation metadata (body properties and frame count)."""
        # Body metadata from AssetInertialStaticRecord (via msd_reader)
        static_assets = self.db.select_all_static_assets()

        # Read AssetPhysicalStaticRecord via sqlite3 (not yet in msd_reader)
        physical_map: dict[int, tuple[int, bool]] = {}  # body_id -> (asset_id, is_env)
        conn = sqlite3.connect(str(self.db_path))
        try:
            cursor = conn.execute(
                "SELECT body_id, asset_id, is_environment FROM AssetPhysicalStaticRecord"
            )
            for row in cursor:
                physical_map[row[0]] = (row[1], bool(row[2]))
        except sqlite3.OperationalError:
            pass  # Table may not exist in older recordings
        finally:
            conn.close()

        bodies = []
        for asset in static_assets:
            asset_id, is_env = physical_map.get(asset.body_id, (None, False))
            bodies.append(
                BodyMetadata(
                    body_id=asset.body_id,
                    mass=asset.mass,
                    restitution=asset.restitution,
                    friction=asset.friction,
                    asset_id=asset_id,
                    is_environment=is_env,
                )
            )

        # Total frame count
        frames = self.db.select_all_frames()
        total_frames = len(frames)

        return SimulationMetadata(bodies=bodies, total_frames=total_frames)

    def _get_states_by_frame(self, frame_id: int) -> list[BodyState]:
        """Get body states for a frame via SQL join through AssetDynamicStateRecord.

        InertialStateRecord doesn't have frame_id directly — it's nested inside
        AssetDynamicStateRecord which holds the frame_id FK.
        """
        conn = sqlite3.connect(str(self.db_path))
        try:
            rows = conn.execute("""
                SELECT
                    adr.body_id,
                    pos.x, pos.y, pos.z,
                    vel.x, vel.y, vel.z,
                    quat.w, quat.x, quat.y, quat.z
                FROM AssetDynamicStateRecord adr
                JOIN InertialStateRecord isr ON adr.kinematicState_id = isr.id
                JOIN CoordinateRecord pos ON isr.position_id = pos.id
                JOIN VelocityRecord vel ON isr.velocity_id = vel.id
                JOIN QuaternionDRecord quat ON isr.orientation_id = quat.id
                WHERE adr.frame_id = ?
            """, (frame_id,)).fetchall()
        finally:
            conn.close()

        return [
            BodyState(
                body_id=row[0],
                position=Vec3(x=row[1], y=row[2], z=row[3]),
                velocity=Vec3(x=row[4], y=row[5], z=row[6]),
                orientation=Quaternion(w=row[7], x=row[8], y=row[9], z=row[10]),
            )
            for row in rows
        ]

    def get_frame_data(self, frame_id: int) -> FrameData:
        """Get complete frame data (states, collisions, solver)."""
        # Frame metadata
        frames = self.db.select_all_frames()
        frame = next((f for f in frames if f.id == frame_id), None)
        if frame is None:
            raise ValueError(f"Frame {frame_id} not found")

        # Body states via SQL join
        states = self._get_states_by_frame(frame_id)

        # Collisions
        collision_records = self.db.select_collisions_by_frame(frame_id)
        collisions = [
            CollisionInfo(
                body_a_id=collision.body_a_id,
                body_b_id=collision.body_b_id,
                normal=Vec3(
                    x=collision.normal.x,
                    y=collision.normal.y,
                    z=collision.normal.z,
                ),
                penetration_depth=collision.penetrationDepth,
                contacts=[
                    ContactPoint(
                        point_a=Vec3(
                            x=contact.pointA.x,
                            y=contact.pointA.y,
                            z=contact.pointA.z,
                        ),
                        point_b=Vec3(
                            x=contact.pointB.x,
                            y=contact.pointB.y,
                            z=contact.pointB.z,
                        ),
                        depth=contact.depth,
                    )
                    for contact in collision.contacts
                ],
            )
            for collision in collision_records
        ]

        # Solver diagnostics (optional, may not exist for all frames)
        solver_records = self.db.select_solver_diagnostic_by_frame(frame_id)
        solver = None
        if solver_records:
            solver_rec = solver_records[0]  # Typically 0 or 1 per frame
            solver = SolverDiagnostics(
                iterations=solver_rec.iterations,
                residual=solver_rec.residual,
                converged=solver_rec.converged,
                num_constraints=solver_rec.num_constraints,
                num_contacts=solver_rec.num_contacts,
            )

        return FrameData(
            frame_id=frame_id,
            simulation_time=frame.simulation_time,
            states=states,
            collisions=collisions,
            solver=solver,
        )

    def get_frame_range(self, start: int, count: int) -> list[FrameData]:
        """Get bulk frame data for playback buffering."""
        frames = self.db.select_all_frames()
        frame_ids = [f.id for f in frames if f.id >= start][:count]
        return [self.get_frame_data(fid) for fid in frame_ids]

    def get_energy_by_body(self, body_id: int) -> list[EnergyPoint]:
        """Get energy timeseries for a specific body."""
        energy_records = self.db.select_energy_by_body(body_id)
        frames = self.db.select_all_frames()
        frame_time_map = {f.id: f.simulation_time for f in frames}

        return [
            EnergyPoint(
                body_id=body_id,
                frame_id=energy.frame_id,
                simulation_time=frame_time_map.get(energy.frame_id, 0.0),
                linear_ke=energy.linear_ke,
                rotational_ke=energy.rotational_ke,
                potential_e=energy.potential_e,
                total_e=energy.total_e,
            )
            for energy in energy_records
        ]

    def get_system_energy(self) -> list[SystemEnergyPoint]:
        """Get system-level energy timeseries."""
        system_energy_records = self.db.select_all_system_energy()
        frames = self.db.select_all_frames()
        frame_time_map = {f.id: f.simulation_time for f in frames}

        return [
            SystemEnergyPoint(
                frame_id=sys_energy.frame_id,
                simulation_time=frame_time_map.get(sys_energy.frame_id, 0.0),
                total_system_e=sys_energy.total_system_e,
                delta_e=sys_energy.delta_e,
            )
            for sys_energy in system_energy_records
        ]
