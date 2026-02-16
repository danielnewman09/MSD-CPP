"""
Pydantic response models for MSD Replay API

Ticket: 0056d_fastapi_backend
"""

from pydantic import BaseModel


class Vec3(BaseModel):
    """3D vector representation."""

    x: float
    y: float
    z: float


class Quaternion(BaseModel):
    """Quaternion representation for orientation."""

    w: float
    x: float
    y: float
    z: float


class BodyState(BaseModel):
    """Per-body kinematic state at a frame."""

    body_id: int
    position: Vec3
    velocity: Vec3
    orientation: Quaternion
    angular_velocity: Vec3 | None = None


class ContactPoint(BaseModel):
    """Per-contact-point geometry from ContactPointRecord."""

    point_a: Vec3
    point_b: Vec3
    depth: float


class CollisionInfo(BaseModel):
    """
    Flattened collision result from CollisionResultRecord + ContactPointRecord.

    CollisionResultRecord contains body_a_id, body_b_id, normal (Vector3DRecord),
    and penetrationDepth at the pair level. ContactPointRecords are nested via
    RepeatedField and contain per-contact pointA/pointB (CoordinateRecord) and depth.

    This model flattens the structure: one CollisionInfo per CollisionResultRecord,
    with contacts expanded into a list.
    """

    body_a_id: int
    body_b_id: int
    normal: Vec3
    penetration_depth: float
    contacts: list[ContactPoint]


class SolverDiagnostics(BaseModel):
    """From SolverDiagnosticRecord — per-frame solver stats."""

    iterations: int
    residual: float
    converged: bool
    num_constraints: int
    num_contacts: int


class FrictionConstraintInfo(BaseModel):
    """From FrictionConstraintRecord — per-contact friction constraint data."""

    body_a_id: int
    body_b_id: int
    normal: Vec3
    tangent1: Vec3
    tangent2: Vec3
    lever_arm_a: Vec3
    lever_arm_b: Vec3
    friction_coefficient: float
    normal_lambda: float  # Actual normal force magnitude from solver (Newtons)
    tangent1_lambda: float = 0.0  # Solved tangent1 force (Newtons)
    tangent2_lambda: float = 0.0  # Solved tangent2 force (Newtons)


class FrameData(BaseModel):
    """Complete frame data including states, collisions, and solver diagnostics."""

    frame_id: int
    simulation_time: float
    states: list[BodyState]
    collisions: list[CollisionInfo]
    friction_constraints: list[FrictionConstraintInfo] = []
    solver: SolverDiagnostics | None  # None if no collisions this frame


class EnergyPoint(BaseModel):
    """Per-body per-frame energy from EnergyRecord."""

    body_id: int
    frame_id: int
    simulation_time: float
    linear_ke: float
    rotational_ke: float
    potential_e: float
    total_e: float


class SystemEnergyPoint(BaseModel):
    """System-level per-frame energy from SystemEnergyRecord."""

    frame_id: int
    simulation_time: float
    total_system_e: float
    delta_e: float


class BodyMetadata(BaseModel):
    """From AssetInertialStaticRecord + AssetPhysicalStaticRecord — static body properties."""

    body_id: int
    mass: float
    restitution: float
    friction: float
    asset_id: int | None = None
    is_environment: bool = False


class SimulationMetadata(BaseModel):
    """Metadata about the simulation."""

    bodies: list[BodyMetadata]
    total_frames: int


class FrameInfo(BaseModel):
    """Frame list entry with timestamp."""

    frame_id: int
    simulation_time: float


class SimulationInfo(BaseModel):
    """Information about an available simulation database."""

    id: str
    name: str
    path: str


class AssetGeometry(BaseModel):
    """Three.js-compatible geometry data."""

    asset_id: int
    name: str
    positions: list[float]  # Flat [x,y,z,x,y,z,...] for BufferGeometry
    vertex_count: int
