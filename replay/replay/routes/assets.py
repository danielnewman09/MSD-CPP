"""
Asset geometry endpoints

Ticket: 0056d_fastapi_backend
"""

from fastapi import APIRouter, HTTPException

from ..config import config
from ..models import AssetGeometry
from ..services import GeometryService

router = APIRouter(prefix="/simulations/{sim_id}", tags=["assets"])


@router.get("/assets", response_model=list[AssetGeometry])
async def get_assets(sim_id: str):
    """Get geometry data for all bodies in Three.js BufferGeometry format."""
    try:
        db_path = config.get_database_path(sim_id)
        service = GeometryService(db_path)
        return service.get_all_geometries()
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading assets: {e}")
