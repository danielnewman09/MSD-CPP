"""
Frame data endpoints

Ticket: 0056d_fastapi_backend
"""

from fastapi import APIRouter, HTTPException, Query

from ..config import config
from ..models import FrameData, FrameInfo, SystemEnergyPoint, EnergyPoint, VelocityPoint
from ..services import SimulationService

router = APIRouter(prefix="/simulations/{sim_id}", tags=["frames"])


@router.get("/frames", response_model=list[FrameInfo])
async def list_frames(sim_id: str):
    """Get list of all frames with timestamps."""
    try:
        db_path = config.get_database_path(sim_id)
        service = SimulationService(db_path)
        return service.get_frames()
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading frames: {e}")


@router.get("/frames/{frame_id}/state", response_model=FrameData)
async def get_frame_state(sim_id: str, frame_id: int):
    """Get complete frame data (states, collisions, solver)."""
    try:
        db_path = config.get_database_path(sim_id)
        service = SimulationService(db_path)
        return service.get_frame_data(frame_id)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading frame: {e}")


@router.get("/frames/range", response_model=list[FrameData])
async def get_frame_range(
    sim_id: str,
    start: int = Query(..., description="Starting frame ID"),
    count: int = Query(..., description="Number of frames to return"),
):
    """Get bulk frame data for playback buffering."""
    try:
        db_path = config.get_database_path(sim_id)
        service = SimulationService(db_path)
        return service.get_frame_range(start, count)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading frame range: {e}")


@router.get("/energy", response_model=list[SystemEnergyPoint])
async def get_system_energy(sim_id: str):
    """Get system-level energy timeseries."""
    try:
        db_path = config.get_database_path(sim_id)
        service = SimulationService(db_path)
        return service.get_system_energy()
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading energy: {e}")


@router.get("/energy/{body_id}", response_model=list[EnergyPoint])
async def get_body_energy(sim_id: str, body_id: int):
    """Get per-body energy timeseries."""
    try:
        db_path = config.get_database_path(sim_id)
        service = SimulationService(db_path)
        return service.get_energy_by_body(body_id)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading body energy: {e}")


@router.get("/velocity/{body_id}", response_model=list[VelocityPoint])
async def get_body_velocity(sim_id: str, body_id: int):
    """Get per-body velocity timeseries (linear + angular)."""
    try:
        db_path = config.get_database_path(sim_id)
        service = SimulationService(db_path)
        return service.get_velocity_by_body(body_id)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error reading body velocity: {e}")
