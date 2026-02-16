#!/bin/bash
# Start the MSD Replay FastAPI server
# Ticket: 0056k_example_workflow_test_recording
# Ticket: 0056e_threejs_core_visualization (R0e - Two-database architecture)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"
VENV_PY="$VENV_DIR/bin/python3"
BUILD_DIR="$PROJECT_ROOT/build/Debug/debug"
ASSETS_DB="$SCRIPT_DIR/recordings/assets.db"
RECORDING_DB="$SCRIPT_DIR/recordings/test_cube_drop.db"

# --- 1. Ensure venv exists ---
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating Python venv at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi

# --- 2. Install replay package ---
echo "Installing replay package..."
"$VENV_PY" -m pip install -q --upgrade pip
"$VENV_PY" -m pip install -q -e "$SCRIPT_DIR"

# --- 3. Check msd_reader is available ---
export PYTHONPATH="$BUILD_DIR:$PYTHONPATH"
if ! "$VENV_PY" -c "import msd_reader" 2>/dev/null; then
    echo "ERROR: msd_reader not found in $BUILD_DIR"
    echo "Build with pybind enabled first:"
    echo "  conan install . --build=missing -s build_type=Debug"
    echo "  cmake --preset conan-debug"
    echo "  cmake --build --preset conan-debug"
    exit 1
fi

# --- 4. Generate asset database if missing ---
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

# --- 5. Generate test recording if missing ---
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

# --- 6. Export environment variables ---
export MSD_RECORDINGS_DIR="$SCRIPT_DIR/recordings"
export MSD_ASSETS_DB="$ASSETS_DB"

# --- 7. Start FastAPI server ---
echo ""
echo "Starting replay server..."
echo "  Asset DB:  $ASSETS_DB"
echo "  Recording: $RECORDING_DB"
echo "  API:  http://localhost:8000/api/v1"
echo "  Docs: http://localhost:8000/docs"
echo ""
exec "$VENV_PY" -m uvicorn replay.app:app --reload --app-dir "$SCRIPT_DIR"
