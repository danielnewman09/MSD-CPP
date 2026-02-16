#!/bin/bash
# Python Environment Setup for MSD-CPP Project
# Ticket: 0065_python_environment_streamlining
#
# This script creates the unified Python virtual environment at python/.venv
# and installs all dependencies for:
#   - Traceability indexing (tree-sitter)
#   - MCP servers (fastmcp)
#   - Replay server (fastapi, uvicorn, pydantic)
#   - Testing (pytest, httpx)
#   - Code generation (stdlib only)
#   - Documentation indexing (stdlib only)
#
# Usage:
#   ./python/setup.sh          # Create/update venv
#   ./python/setup.sh --clean  # Remove and recreate venv from scratch

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"
REQUIREMENTS="$SCRIPT_DIR/requirements.txt"
REPLAY_DIR="$PROJECT_ROOT/replay"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================="
echo "MSD-CPP Python Environment Setup"
echo "========================================="
echo ""

# --- 1. Handle --clean flag ---
if [ "$1" == "--clean" ]; then
    echo -e "${YELLOW}Removing existing venv at $VENV_DIR${NC}"
    rm -rf "$VENV_DIR"
fi

# --- 2. Create venv if it doesn't exist ---
if [ ! -d "$VENV_DIR" ]; then
    echo -e "${GREEN}Creating Python venv at $VENV_DIR${NC}"
    python3 -m venv "$VENV_DIR"
else
    echo -e "${GREEN}Using existing venv at $VENV_DIR${NC}"
fi

# --- 3. Activate venv ---
source "$VENV_DIR/bin/activate"

# --- 4. Upgrade pip ---
echo ""
echo "Upgrading pip..."
pip install --quiet --upgrade pip

# --- 5. Install requirements ---
echo ""
echo "Installing dependencies from $REQUIREMENTS..."
pip install --quiet -r "$REQUIREMENTS"

# --- 6. Install replay package in editable mode ---
echo ""
echo "Installing replay package in editable mode..."
if [ ! -d "$REPLAY_DIR" ]; then
    echo -e "${RED}ERROR: replay/ directory not found at $REPLAY_DIR${NC}"
    exit 1
fi
pip install --quiet -e "$REPLAY_DIR"

# --- 7. Validate environment ---
echo ""
echo "Validating environment..."

# Check traceability dependencies
if ! python3 -c "import tree_sitter" 2>/dev/null; then
    echo -e "${RED}ERROR: tree_sitter import failed${NC}"
    exit 1
fi

if ! python3 -c "import tree_sitter_cpp" 2>/dev/null; then
    echo -e "${RED}ERROR: tree_sitter_cpp import failed${NC}"
    exit 1
fi

# Check MCP dependencies
if ! python3 -c "import fastmcp" 2>/dev/null; then
    echo -e "${RED}ERROR: fastmcp import failed${NC}"
    exit 1
fi

# Check replay server dependencies
if ! python3 -c "import fastapi" 2>/dev/null; then
    echo -e "${RED}ERROR: fastapi import failed${NC}"
    exit 1
fi

if ! python3 -c "import uvicorn" 2>/dev/null; then
    echo -e "${RED}ERROR: uvicorn import failed${NC}"
    exit 1
fi

if ! python3 -c "import pydantic" 2>/dev/null; then
    echo -e "${RED}ERROR: pydantic import failed${NC}"
    exit 1
fi

# Check replay package
if ! python3 -c "import replay" 2>/dev/null; then
    echo -e "${RED}ERROR: replay package import failed${NC}"
    exit 1
fi

# Check testing dependencies
if ! python3 -c "import pytest" 2>/dev/null; then
    echo -e "${RED}ERROR: pytest import failed${NC}"
    exit 1
fi

if ! python3 -c "import httpx" 2>/dev/null; then
    echo -e "${RED}ERROR: httpx import failed${NC}"
    exit 1
fi

echo -e "${GREEN}All imports successful!${NC}"

# --- 8. Display summary ---
echo ""
echo "========================================="
echo -e "${GREEN}Setup Complete!${NC}"
echo "========================================="
echo ""
echo "Python interpreter: $VENV_DIR/bin/python3"
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
