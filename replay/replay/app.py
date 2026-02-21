"""
FastAPI application for MSD simulation replay

Ticket: 0056d_fastapi_backend
Ticket: 0072b_websocket_simulation_endpoint
"""

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from .routes import assets_router, frames_router, live_router, simulations_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Create a persistent msd_reader.Engine for the REST asset-list endpoint."""
    import msd_reader
    from .config import config

    app.state.engine = msd_reader.Engine(str(config.assets_db_path))
    yield


# Create FastAPI app
app = FastAPI(
    title="MSD Replay API",
    description="REST API for MSD simulation replay visualization",
    version="0.1.0",
    lifespan=lifespan,
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
# Live simulation WebSocket + asset-list endpoints (Ticket: 0072b)
app.include_router(live_router, prefix="/api/v1")


# Mount static files (Three.js frontend)
static_dir = Path(__file__).parent.parent / "static"
if static_dir.exists():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.get("/", response_class=HTMLResponse)
    async def serve_index():
        """Serve the main HTML page."""
        index_path = static_dir / "index.html"
        return index_path.read_text()

    @app.get("/live", response_class=HTMLResponse)
    async def serve_live():
        """Serve the live simulation HTML page (Ticket: 0072c)."""
        live_path = static_dir / "live.html"
        if live_path.exists():
            return live_path.read_text()
        # Placeholder until 0072c delivers live.html
        return "<html><body><p>Live simulation UI coming in ticket 0072c.</p></body></html>"


@app.get("/api/v1/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "ok"}
