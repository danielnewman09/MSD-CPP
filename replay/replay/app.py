"""
FastAPI application for MSD simulation replay

Ticket: 0056d_fastapi_backend
"""

from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .routes import assets_router, frames_router, simulations_router

# Create FastAPI app
app = FastAPI(
    title="MSD Replay API",
    description="REST API for MSD simulation replay visualization",
    version="0.1.0",
)

# Add CORS middleware for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register API routes
app.include_router(simulations_router, prefix="/api/v1")
app.include_router(frames_router, prefix="/api/v1")
app.include_router(assets_router, prefix="/api/v1")


# Mount static files (Three.js frontend)
static_dir = Path(__file__).parent.parent / "static"
if static_dir.exists():
    app.mount("/", StaticFiles(directory=str(static_dir), html=True), name="static")


@app.get("/api/v1/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "ok"}
