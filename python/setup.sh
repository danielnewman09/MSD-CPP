#!/bin/bash
# Python Environment Setup for MSD-CPP Project
# Ticket: 0065_python_environment_streamlining
#
# Installs all Python dependencies for:
#   - Traceability indexing (tree-sitter)
#   - MCP servers (fastmcp)
#   - Replay server (fastapi, uvicorn, pydantic)
#   - Testing (pytest, httpx)
#   - Code generation (stdlib only)
#   - Documentation indexing (stdlib only)
#
# Usage:
#   ./python/setup.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REQUIREMENTS="$SCRIPT_DIR/requirements.txt"
REPLAY_DIR="$PROJECT_ROOT/replay"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo "========================================="
echo "MSD-CPP Python Environment Setup"
echo "========================================="
echo ""

VENV_DIR="$SCRIPT_DIR/.venv"
REQUIRED_PYTHON="3.12"
PYTHON_PKG_URL="https://www.python.org/ftp/python/3.12.9/python-3.12.9-macos11.pkg"

# --- 1. Ensure Python >= 3.10, install from python.org if needed ---
PYTHON_BIN="python${REQUIRED_PYTHON}"
if ! command -v "$PYTHON_BIN" &>/dev/null; then
  # Fall back to python3 if it's new enough
  if python3 -c "import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)" 2>/dev/null; then
    PYTHON_BIN="python3"
  else
    echo "Python >= 3.10 not found. Installing Python ${REQUIRED_PYTHON} from python.org..."
    PKG_PATH="/tmp/python-${REQUIRED_PYTHON}.pkg"
    curl -fSL -o "$PKG_PATH" "$PYTHON_PKG_URL"
    sudo installer -pkg "$PKG_PATH" -target /
    rm -f "$PKG_PATH"
    # python.org installs to /Library/Frameworks/Python.framework/Versions/X.Y/bin
    PYTHON_BIN="/Library/Frameworks/Python.framework/Versions/${REQUIRED_PYTHON}/bin/python${REQUIRED_PYTHON}"
    if [ ! -x "$PYTHON_BIN" ]; then
      echo -e "${RED}ERROR: Installation failed — $PYTHON_BIN not found.${NC}"
      exit 1
    fi
    echo "Installed $($PYTHON_BIN --version)"
  fi
fi
echo "Using $($PYTHON_BIN --version) ($PYTHON_BIN)"

# --- 2. Create venv (rebuild if Python version changed) ---
if [ -d "$VENV_DIR" ]; then
  VENV_VER=$("$VENV_DIR/bin/python3" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null || echo "unknown")
  WANT_VER=$("$PYTHON_BIN" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
  if [ "$VENV_VER" != "$WANT_VER" ]; then
    echo "Existing venv uses Python $VENV_VER, rebuilding with $WANT_VER..."
    rm -rf "$VENV_DIR"
  fi
fi
if [ ! -d "$VENV_DIR" ]; then
  echo "Creating virtual environment at $VENV_DIR..."
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

# Activate the venv for this script
source "$VENV_DIR/bin/activate"

# --- 2. Upgrade pip ---
echo "Upgrading pip..."
pip install --quiet --upgrade pip

# --- 3. Install requirements ---
echo ""
echo "Installing dependencies from $REQUIREMENTS..."
pip install --quiet -r "$REQUIREMENTS"

# --- 4. Install replay package in editable mode ---
echo ""
echo "Installing replay package in editable mode..."
if [ ! -d "$REPLAY_DIR" ]; then
    echo -e "${RED}ERROR: replay/ directory not found at $REPLAY_DIR${NC}"
    exit 1
fi
pip install --quiet -e "$REPLAY_DIR"

# --- 5. Display summary ---
echo ""
echo "========================================="
echo -e "${GREEN}Setup Complete!${NC}"
echo "========================================="
echo ""
echo "Python interpreter: $(which python3)"
echo "Installed packages:"
pip list | grep -E "(tree-sitter|fastmcp|fastapi|uvicorn|pydantic|pytest|httpx|msd-replay)"
echo ""
echo "Note: msd_reader (C++ pybind11 module) is added via PYTHONPATH from the build directory."
echo "      Build with: cmake --build --preset conan-debug"
echo ""
echo "Next steps:"
echo "  - Build C++ project: cmake --preset conan-debug && cmake --build --preset conan-debug"
echo "  - Run traceability: cmake --build --preset debug-traceability"
echo "  - Start replay server: replay/start_server.sh"
echo "  - See python/README.md for full documentation"
echo ""
