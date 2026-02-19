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

# --- 1. Create venv if it doesn't exist ---
if [ ! -d "$VENV_DIR" ]; then
  echo "Creating virtual environment at $VENV_DIR..."
  python3 -m venv "$VENV_DIR"
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
