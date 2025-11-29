#!/bin/bash
set -e

echo "=== Installing SDL_shadercross ==="

# Configuration
INSTALL_DIR="/usr/local"
TEMP_DIR="/tmp/sdl_shadercross_build"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}This will install shadercross to: ${INSTALL_DIR}/bin${NC}"
echo -e "${BLUE}You may be prompted for your password (sudo required)${NC}"
echo "Press Enter to continue or Ctrl+C to cancel..."
read

# Clean up any previous build
rm -rf "${TEMP_DIR}"
mkdir -p "${TEMP_DIR}"

cd "${TEMP_DIR}"

echo -e "${BLUE}Cloning SDL_shadercross...${NC}"
git clone https://github.com/libsdl-org/SDL_shadercross.git
cd SDL_shadercross

echo -e "${BLUE}Fetching submodules...${NC}"
git submodule update --init --recursive

# Find SDL3 from Conan cache
echo -e "${BLUE}Looking for SDL3 in Conan cache...${NC}"
SDL3_PATH=$(conan cache path sdl/3.3.3 2>/dev/null || echo "")
if [ -z "$SDL3_PATH" ]; then
    echo -e "${BLUE}SDL3 not found in Conan cache. Installing SDL first...${NC}"
    echo "Please run 'conan create conan/sdl --build=missing' first, then re-run this script."
    exit 1
fi

echo -e "${BLUE}Found SDL3 at: ${SDL3_PATH}${NC}"

echo -e "${BLUE}Configuring with CMake...${NC}"
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DCMAKE_PREFIX_PATH="${SDL3_PATH}"

echo -e "${BLUE}Building...${NC}"
cmake --build build -j$(sysctl -n hw.ncpu)

echo -e "${BLUE}Installing to ${INSTALL_DIR}/bin (requires sudo)...${NC}"
sudo cmake --install build

# Add to PATH hint
echo ""
echo -e "${GREEN}=== Installation Complete! ===${NC}"
echo ""
echo "shadercross has been installed to: ${INSTALL_DIR}/bin/shadercross"
echo ""

# Test if it's in PATH
if command -v shadercross &> /dev/null; then
    echo -e "${GREEN}shadercross is now available in your PATH!${NC}"
    echo "Testing installation..."
    shadercross --version 2>&1 || echo "(Version info not available, but binary is installed)"
else
    echo -e "${BLUE}Note: You may need to restart your terminal for PATH changes to take effect${NC}"
fi

# Clean up
echo ""
echo -e "${BLUE}Cleaning up temporary files...${NC}"
rm -rf "${TEMP_DIR}"

echo -e "${GREEN}Done!${NC}"
