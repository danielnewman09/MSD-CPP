"""
Geometry Service — Three.js geometry conversion

Ticket: 0056d_fastapi_backend
Ticket: 0056e_threejs_core_visualization (R0d)
"""

from pathlib import Path

try:
    import msd_reader
except ImportError:
    msd_reader = None

from ..models import AssetGeometry


class GeometryService:
    """Service for converting MeshRecord BLOBs to Three.js format.

    Reads from asset database (not recording database).
    Two-database architecture (Ticket: 0056e):
    - Asset DB (input): MeshRecord, ObjectRecord (geometry definitions)
    - Recording DB (output): Frames, states, AssetPhysicalStaticRecord (links to asset_id)
    """

    def __init__(self, assets_db_path: Path):
        """Initialize service with asset database connection.

        Args:
            assets_db_path: Path to asset database (not recording database)

        Raises:
            RuntimeError: If msd_reader module not available
        """
        if msd_reader is None:
            raise RuntimeError(
                "msd_reader module not available. "
                "Build with -o '&:enable_pybind=True' to enable Python bindings."
            )

        self.db_path = assets_db_path
        self.db = msd_reader.Database(str(assets_db_path))

    def get_geometries(self, asset_ids: list[int] | None = None) -> list[AssetGeometry]:
        """Get asset geometries in Three.js BufferGeometry format.

        Args:
            asset_ids: Optional list of asset IDs to filter. If None, returns all geometries.

        Returns:
            List of AssetGeometry with positions for Three.js BufferGeometry
        """
        mesh_records = self.db.select_all_meshes()

        # Filter by asset_ids if provided
        if asset_ids is not None:
            asset_id_set = set(asset_ids)
            mesh_records = [m for m in mesh_records if m.id in asset_id_set]

        geometries = []
        for mesh in mesh_records:
            # Deserialize BLOB to 9-tuples (px,py,pz, r,g,b, nx,ny,nz)
            visual_vertices = msd_reader.deserialize_visual_vertices(
                mesh.vertex_data
            )

            # Extract positions (first 3 floats) and flatten
            positions = []
            for vertex in visual_vertices:
                positions.extend([vertex[0], vertex[1], vertex[2]])

            geometries.append(
                AssetGeometry(
                    asset_id=mesh.id,
                    name=f"mesh_{mesh.id}",
                    positions=positions,
                    vertex_count=mesh.vertex_count,
                )
            )

        return geometries
