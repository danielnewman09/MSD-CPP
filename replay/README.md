# MSD Replay Backend

FastAPI backend for serving MSD simulation data to the Three.js frontend.

**Ticket**: [0056d_fastapi_backend](../tickets/0056d_fastapi_backend.md)

## Quick Start

```bash
# 1. Build C++ simulation + recording generator
cd /path/to/MSD-CPP
conan install . --build=missing -s build_type=Debug
cmake --preset conan-debug
cmake --build --preset conan-debug --target generate_test_recording

# 2. Generate test recording
./build/Debug/debug/generate_test_recording replay/recordings/test_cube_drop.db

# 3. Install Python dependencies
cd replay
pip install -e .

# 4. Run Python example (validates pybind bindings)
export PYTHONPATH=../build/Debug/debug:$PYTHONPATH
python examples/query_recording.py

# 5. Start FastAPI server
uvicorn replay.app:app --reload
# Server runs at http://localhost:8000
# API docs at http://localhost:8000/docs
```

## Prerequisites

- Python 3.10+
- MSD-CPP project built with Python bindings enabled:
  ```bash
  conan install . --build=missing -s build_type=Debug
  cmake --preset conan-debug
  cmake --build --preset conan-debug
  ```

## Installation

```bash
cd replay
pip install -e .

# For development with tests
pip install -e ".[dev]"
```

## Configuration

The backend needs to know where recording databases are stored:

```bash
# Set recordings directory (defaults to ./recordings/)
export MSD_RECORDINGS_DIR=/path/to/recordings
```

## Running the Server

```bash
# Make sure msd_reader is in PYTHONPATH
export PYTHONPATH=/path/to/MSD-CPP/build/Debug/debug:$PYTHONPATH

# Start the server
uvicorn replay.app:app --reload

# Server runs at http://localhost:8000
```

## API Documentation

Interactive API documentation available at:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## API Endpoints

**Base URL**: `http://localhost:8000/api/v1`

### Simulations

- `GET /simulations` - List available recording databases
- `GET /simulations/{id}/metadata` - Body metadata for simulation

### Frames

- `GET /simulations/{id}/frames` - Frame list with timestamps
- `GET /simulations/{id}/frames/{fid}/state` - Full frame data
- `GET /simulations/{id}/frames/range?start={n}&count={n}` - Bulk frame data

### Assets

- `GET /simulations/{id}/assets` - Geometry data for all bodies

### Energy

- `GET /simulations/{id}/energy` - System-level energy timeseries
- `GET /simulations/{id}/energy/{body_id}` - Per-body energy timeseries

## Development

Run tests:
```bash
pytest
```

## Architecture

```
replay/
  replay/
    app.py                    # FastAPI application
    config.py                 # Configuration
    models.py                 # Pydantic response models
    routes/
      simulations.py          # /simulations endpoints
      frames.py               # /frames endpoints
      assets.py               # /assets endpoints
    services/
      simulation_service.py   # Database query wrapper
      geometry_service.py     # Geometry conversion
  static/
    index.html                # Placeholder (Three.js frontend in 0056e)
```

## MCP Server for AI-Assisted Physics Debugging

**Ticket**: [0063_replay_mcp_server](../tickets/0063_replay_mcp_server.md)

The MCP (Model Context Protocol) server exposes the simulation recording database to Claude Code for AI-assisted physics debugging. When you observe physically incorrect behavior in the replay viewer (e.g., "at frame 45 the friction contact seems wrong"), Claude can query the MCP server to retrieve frame data, constraint forces, energy timeseries, and solver diagnostics to diagnose the root cause.

### MCP Server Configuration

Add the following to `.mcp.json` in the MSD-CPP repository root:

```json
{
  "mcpServers": {
    "replay": {
      "command": "/path/to/MSD-CPP/scripts/.venv/bin/python3",
      "args": ["-m", "replay.mcp_server"],
      "cwd": "/path/to/MSD-CPP/replay"
    }
  }
}
```

### MCP Tools Available

**Session Management**:
- `list_recordings` - List available .db recording files
- `load_recording` - Open a recording database (sets active session)

**Core Query**:
- `get_metadata` - Body properties and frame count
- `get_frame` - Complete frame data (states, collisions, friction, solver)
- `get_body_state` - Single body's kinematic state at a frame
- `get_contacts_for_body` - All friction constraints involving a body

**Energy Analysis**:
- `get_body_energy` - Energy timeseries for one body
- `get_system_energy` - System-level energy timeseries
- `find_energy_anomalies` - Frames where |delta_e| exceeds threshold

**Diagnostic Comparison**:
- `compare_body_across_frames` - State delta between two frames
- `get_contact_history` - Track contact pairs across frame range
- `get_solver_diagnostics` - Solver convergence info for frame range

**Physics Validation**:
- `check_friction_cone` - Verify friction forces satisfy Coulomb cone
- `check_penetration` - Find contacts with excessive penetration depth
- `check_resting_contact` - Verify bodies at rest have near-zero velocity

### Example Interaction

```
User: "At frame 45, body 1 seems to be sliding on the floor even though
       it should be at rest. The friction coefficient is 0.5."

Claude: [calls get_frame tool with frame_id=45]
        [calls get_contacts_for_body with body_id=1, frame_id=45]
        [calls check_friction_cone with frame_id=45, body_id=1]

Claude: "Looking at frame 45, body 1 has a friction constraint with body 2
         (the floor). The normal force is 98.1 N (correct for a 10 kg body
         under gravity). The friction limit is mu * Fn = 0.5 * 98.1 = 49.05 N.
         However, the tangent forces are t1=49.04 N, t2=0.12 N — the friction
         is saturated at the Coulomb limit. The residual velocity is
         (0.003, 0.0, 0.0) m/s along tangent1..."
```

### Installation for MCP Server

The MCP server requires `mcp` and `numpy` dependencies, which are automatically installed with `pip install -e .`:

```bash
cd replay
pip install -e .
```

## Dependencies

The backend depends on:
- **0056c**: `msd_reader` Python module (pybind11 bindings)
- **FastAPI**: REST API framework
- **Uvicorn**: ASGI server
- **Pydantic**: Data validation and serialization
- **MCP SDK**: Model Context Protocol for AI debugging (0063)
- **NumPy**: Vector math for validation tools (0063)
