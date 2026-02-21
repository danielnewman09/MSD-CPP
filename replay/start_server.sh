#!/bin/bash
# Start the MSD Replay FastAPI server
# Ticket: 0056k_example_workflow_test_recording
# Ticket: 0056e_threejs_core_visualization (R0e - Two-database architecture)
# Ticket: 0065_python_environment_streamlining (Unified Python environment)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build/Debug/debug"

# Activate the project venv
source "$PROJECT_ROOT/python/.venv/bin/activate"
ASSETS_DB="$SCRIPT_DIR/recordings/assets.db"
RECORDING_DB="$SCRIPT_DIR/recordings/test_cube_drop.db"

# --- 1. Verify replay package is installed ---
if ! python3 -c "import replay" 2>/dev/null; then
    echo "ERROR: replay package not found"
    echo "Run the setup script to install: python/setup.sh"
    exit 1
fi

# --- 2. Check msd_reader is available ---
export PYTHONPATH="$BUILD_DIR:$PYTHONPATH"
if ! python3 -c "import msd_reader" 2>/dev/null; then
    echo "ERROR: msd_reader not found in $BUILD_DIR"
    echo "Build with pybind enabled first:"
    echo "  conan install . --build=missing -s build_type=Debug"
    echo "  cmake --preset conan-debug"
    echo "  cmake --build --preset conan-debug"
    exit 1
fi

# --- 3. Generate asset database if missing ---
mkdir -p "$SCRIPT_DIR/recordings"
if [ ! -f "$ASSETS_DB" ]; then
    ASSET_GEN="$BUILD_DIR/generate_assets"
    if [ ! -f "$ASSET_GEN" ]; then
        echo "ERROR: generate_assets not found at $ASSET_GEN"
        echo "Build it first: cmake --build --preset conan-debug --target generate_assets"
        exit 1
    fi
    echo "Generating asset database..."
    "$ASSET_GEN" "$ASSETS_DB"
fi

# --- 4. Generate test recording if missing ---
if [ ! -f "$RECORDING_DB" ]; then
    GENERATOR="$BUILD_DIR/generate_test_recording"
    if [ ! -f "$GENERATOR" ]; then
        echo "ERROR: generate_test_recording not found at $ASSET_GEN"
        echo "Build it first: cmake --build --preset conan-debug --target generate_test_recording"
        exit 1
    fi
    echo "Generating test recording..."
    "$GENERATOR" "$ASSETS_DB" "$RECORDING_DB"
fi

# --- 5. Export environment variables ---
export MSD_RECORDINGS_DIR="$SCRIPT_DIR/recordings"
export MSD_ASSETS_DB="$ASSETS_DB"

# --- 6. Start FastAPI server ---
echo ""
echo "Starting replay server..."
echo "  Asset DB:  $ASSETS_DB"
echo "  Recording: $RECORDING_DB"
echo "  API:  http://localhost:8000/api/v1"
echo "  Docs: http://localhost:8000/docs"
echo ""
exec python3 -m uvicorn replay.app:app --reload --app-dir "$SCRIPT_DIR"
