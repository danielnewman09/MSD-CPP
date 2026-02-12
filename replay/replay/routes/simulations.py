"""
Simulation endpoints

Ticket: 0056d_fastapi_backend
"""

from fastapi import APIRouter, HTTPException

from ..config import config
from ..models import SimulationInfo, SimulationMetadata
from ..services import SimulationService

router = APIRouter(prefix="/simulations", tags=["simulations"])


@router.get("", response_model=list[SimulationInfo])
async def list_simulations():
    """List available recording databases."""
    databases = config.list_databases()
    return [
        SimulationInfo(
            id=db.stem,  # Filename without extension
            name=db.stem,
            path=str(db),
        )
        for db in databases
    ]


@router.get("/{sim_id}/metadata", response_model=SimulationMetadata)
async def get_metadata(sim_id: str):
    """Get simulation metadata (body properties and frame count)."""
    try:
        db_path = config.get_database_path(sim_id)
        service = SimulationService(db_path)
        return service.get_metadata()
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading metadata: {e}")
