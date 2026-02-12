"""
Geometry Service — Three.js geometry conversion

Ticket: 0056d_fastapi_backend
"""

from pathlib import Path

try:
    import msd_reader
except ImportError:
    msd_reader = None

from ..models import AssetGeometry


class GeometryService:
    """Service for converting MeshRecord BLOBs to Three.js format."""

    def __init__(self, db_path: Path):
        """Initialize service with database connection."""
        if msd_reader is None:
            raise RuntimeError(
                "msd_reader module not available. "
                "Build with -o '&:enable_pybind=True' to enable Python bindings."
            )

        self.db_path = db_path
        self.db = msd_reader.RecordingDatabase(str(db_path))

    def get_all_geometries(self) -> list[AssetGeometry]:
        """Get all asset geometries in Three.js BufferGeometry format."""
        mesh_records = self.db.select_all_meshes()

        geometries = []
        for mesh in mesh_records:
            # Deserialize BLOB to flat float array
            positions = msd_reader.deserialize_visual_vertices(
                mesh.vertex_data
            )

            geometries.append(
                AssetGeometry(
                    asset_id=mesh.id,
                    name=f"mesh_{mesh.id}",
                    positions=positions,
                    vertex_count=mesh.vertex_count,
                )
            )

        return geometries
