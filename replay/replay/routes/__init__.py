"""
API routes for MSD Replay Backend

Ticket: 0056d_fastapi_backend
"""

from .simulations import router as simulations_router
from .frames import router as frames_router
from .assets import router as assets_router

__all__ = ["simulations_router", "frames_router", "assets_router"]
