"""
API routes for MSD Replay Backend

Ticket: 0056d_fastapi_backend
Ticket: 0072b_websocket_simulation_endpoint
"""

from .simulations import router as simulations_router
from .frames import router as frames_router
from .assets import router as assets_router
from .live import router as live_router

__all__ = ["simulations_router", "frames_router", "assets_router", "live_router"]
