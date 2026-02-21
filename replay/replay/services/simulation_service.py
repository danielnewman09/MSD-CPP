"""
Simulation Service — Database query wrapper

Ticket: 0056d_fastapi_backend
"""

import math
import sqlite3
from pathlib import Path
from typing import Optional

import msd_reader

from ..models import (
    BodyMetadata,
    BodyState,
    CollisionInfo,
    ContactPoint,
    EnergyPoint,
    FrameData,
    FrameInfo,
    FrictionConstraintInfo,
    Quaternion,
    SimulationMetadata,
    SolverDiagnostics,
    SystemEnergyPoint,
    Vec3,
    VelocityPoint,
)


class SimulationService:
    """Service layer for simulation database queries using msd_reader."""

    def __init__(self, db_path: Path):
        """Initialize service with database connection."""
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

    @staticmethod
    def _qdot_to_omega(
        qw: float, qx: float, qy: float, qz: float,
        qdx: float, qdy: float, qdz: float, qdw: float,
    ) -> Vec3:
        """Convert quaternion + quaternion rate to angular velocity.

        Formula: omega = 2 * conj(Q) * Qdot (vector part)
        where conj(Q) = (w, -x, -y, -z).
        """
        # conj(Q) components
        cw, cx, cy, cz = qw, -qx, -qy, -qz
        # Quaternion product conj(Q) * Qdot — extract vector part and scale by 2
        ox = 2.0 * (cw * qdx + cx * qdw + cy * qdz - cz * qdy)
        oy = 2.0 * (cw * qdy - cx * qdz + cy * qdw + cz * qdx)
        oz = 2.0 * (cw * qdz + cx * qdy - cy * qdx + cz * qdw)
        return Vec3(x=ox, y=oy, z=oz)

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
                    quat.w, quat.x, quat.y, quat.z,
                    qdot.x, qdot.y, qdot.z, qdot.w
                FROM AssetDynamicStateRecord adr
                JOIN InertialStateRecord isr ON adr.kinematicState_id = isr.id
                JOIN CoordinateRecord pos ON isr.position_id = pos.id
                JOIN VelocityRecord vel ON isr.velocity_id = vel.id
                JOIN QuaternionDRecord quat ON isr.orientation_id = quat.id
                JOIN Vector4DRecord qdot ON isr.quaternionRate_id = qdot.id
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
                angular_velocity=self._qdot_to_omega(
                    row[7], row[8], row[9], row[10],  # q: w, x, y, z
                    row[11], row[12], row[13], row[14],  # qdot: x, y, z, w
                ),
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

        # Friction constraints (optional, older DBs may lack the table)
        friction_constraints: list[FrictionConstraintInfo] = []
        try:
            friction_records = self.db.select_friction_constraints_by_frame(frame_id)
            friction_constraints = [
                FrictionConstraintInfo(
                    body_a_id=fc.body_a_id,
                    body_b_id=fc.body_b_id,
                    normal=Vec3(x=fc.normal.x, y=fc.normal.y, z=fc.normal.z),
                    tangent1=Vec3(x=fc.tangent1.x, y=fc.tangent1.y, z=fc.tangent1.z),
                    tangent2=Vec3(x=fc.tangent2.x, y=fc.tangent2.y, z=fc.tangent2.z),
                    lever_arm_a=Vec3(
                        x=fc.lever_arm_a.x, y=fc.lever_arm_a.y, z=fc.lever_arm_a.z
                    ),
                    lever_arm_b=Vec3(
                        x=fc.lever_arm_b.x, y=fc.lever_arm_b.y, z=fc.lever_arm_b.z
                    ),
                    friction_coefficient=fc.friction_coefficient,
                    normal_lambda=fc.normal_lambda,
                    tangent1_lambda=fc.tangent1_lambda,
                    tangent2_lambda=fc.tangent2_lambda,
                )
                for fc in friction_records
            ]
        except Exception:
            pass  # Table may not exist in older recordings

        return FrameData(
            frame_id=frame_id,
            simulation_time=frame.simulation_time,
            states=states,
            collisions=collisions,
            friction_constraints=friction_constraints,
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

    def get_velocity_by_body(self, body_id: int) -> list[VelocityPoint]:
        """Get velocity timeseries for a specific body."""
        frames = self.db.select_all_frames()
        frame_time_map = {f.id: f.simulation_time for f in frames}

        conn = sqlite3.connect(str(self.db_path))
        try:
            rows = conn.execute("""
                SELECT
                    adr.frame_id,
                    vel.x, vel.y, vel.z,
                    quat.w, quat.x, quat.y, quat.z,
                    qdot.x, qdot.y, qdot.z, qdot.w
                FROM AssetDynamicStateRecord adr
                JOIN InertialStateRecord isr ON adr.kinematicState_id = isr.id
                JOIN VelocityRecord vel ON isr.velocity_id = vel.id
                JOIN QuaternionDRecord quat ON isr.orientation_id = quat.id
                JOIN Vector4DRecord qdot ON isr.quaternionRate_id = qdot.id
                WHERE adr.body_id = ?
                ORDER BY adr.frame_id
            """, (body_id,)).fetchall()
        finally:
            conn.close()

        results = []
        for row in rows:
            frame_id = row[0]
            vx, vy, vz = row[1], row[2], row[3]
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)

            omega = self._qdot_to_omega(
                row[4], row[5], row[6], row[7],   # q: w, x, y, z
                row[8], row[9], row[10], row[11],  # qdot: x, y, z, w
            )
            omega_mag = math.sqrt(omega.x**2 + omega.y**2 + omega.z**2)

            results.append(VelocityPoint(
                body_id=body_id,
                frame_id=frame_id,
                simulation_time=frame_time_map.get(frame_id, 0.0),
                vx=vx, vy=vy, vz=vz,
                speed=speed,
                omega_x=omega.x, omega_y=omega.y, omega_z=omega.z,
                omega_magnitude=omega_mag,
            ))

        return results

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
