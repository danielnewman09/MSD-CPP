"""
Geometry Service — Three.js geometry conversion via AssetRegistry

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
    """Service for loading asset geometry via AssetRegistry.

    Uses msd_reader.AssetRegistry to resolve ObjectRecord.id → geometry,
    matching the C++ AssetRegistry pipeline.

    Two-database architecture (Ticket: 0056e):
    - Asset DB (input): ObjectRecord → MeshRecord (geometry definitions)
    - Recording DB (output): AssetPhysicalStaticRecord.asset_id → ObjectRecord.id
    """

    def __init__(self, assets_db_path: Path):
        if msd_reader is None:
            raise RuntimeError(
                "msd_reader module not available. "
            )

        self.registry = msd_reader.AssetRegistry(str(assets_db_path))

    def get_geometries(self, asset_ids: list[int] | None = None) -> list[AssetGeometry]:
        """Get asset geometries in Three.js BufferGeometry format.

        Args:
            asset_ids: Optional list of ObjectRecord IDs to filter.
                       If None, returns geometry for all assets.

        Returns:
            List of AssetGeometry with positions for Three.js BufferGeometry
        """
        # Determine which assets to load
        if asset_ids is not None:
            assets_to_load = [(aid, self.registry.get_asset_name(aid)) for aid in asset_ids]
        else:
            assets_to_load = self.registry.list_assets()

        geometries = []
        for asset_id, name in assets_to_load:
            vertices = self.registry.get_collision_vertices(asset_id)
            if not vertices:
                continue

            # Flatten to [x,y,z,x,y,z,...] for Three.js BufferGeometry
            positions = []
            for v in vertices:
                positions.extend([v[0], v[1], v[2]])

            geometries.append(
                AssetGeometry(
                    asset_id=asset_id,
                    name=name,
                    positions=positions,
                    vertex_count=len(vertices),
                )
            )

        return geometries
