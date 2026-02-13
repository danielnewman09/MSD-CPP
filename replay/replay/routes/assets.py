"""
Asset geometry endpoints

Ticket: 0056d_fastapi_backend
Ticket: 0056e_threejs_core_visualization (R0d)
"""

from fastapi import APIRouter, HTTPException, Query

from ..config import config
from ..models import AssetGeometry
from ..services import GeometryService

router = APIRouter(prefix="/simulations/{sim_id}", tags=["assets"])


@router.get("/assets", response_model=list[AssetGeometry])
async def get_assets(
    sim_id: str,
    asset_ids: list[int] | None = Query(None, description="Filter by asset IDs"),
):
    """Get geometry data for bodies in Three.js BufferGeometry format.

    Two-database architecture (Ticket: 0056e):
    - Reads geometry from asset database (input)
    - Frontend gets asset_ids from AssetPhysicalStaticRecord in recording database (output)

    Args:
        sim_id: Simulation ID (used for validation only - geometry comes from asset DB)
        asset_ids: Optional list of asset IDs to filter

    Returns:
        List of AssetGeometry with positions for Three.js
    """
    try:
        # Validate sim_id exists (but geometry comes from asset DB)
        config.get_database_path(sim_id)

        # Read from asset database (not recording database)
        service = GeometryService(config.assets_db_path)
        return service.get_geometries(asset_ids)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading assets: {e}")
