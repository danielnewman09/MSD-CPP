# msd-gui Library

## Overview
The `msd-gui` library provides a modern C++20 graphics user interface layer for the MSD (Multi-Spacecraft Dynamics) project. It wraps SDL3 (Simple DirectMedia Layer 3) with GPU-accelerated rendering capabilities using SDL's new GPU API.

## Architecture

### Core Components

#### SDLApplication (Singleton)
- **Purpose**: Main application lifecycle manager
- **Pattern**: Singleton pattern for single application instance
- **Location**: `src/SDLApp.hpp`, `src/SDLApp.cpp`
- **Responsibilities**:
  - Window creation and management
  - Event handling (user input, window events)
  - Main render loop coordination
  - Application state management (Starting, Running, Paused, Error, Exiting)
  - Base path management for resource loading

#### GPUManager
- **Purpose**: GPU device and rendering pipeline management
- **Location**: `src/SDLGPUManager.hpp`, `src/SDLGPUManager.cpp`
- **Responsibilities**:
  - GPU device initialization and lifecycle
  - Shader loading (SPIRV, MSL, DXIL formats)
  - Render pass execution
  - Command buffer management
  - Resource cleanup via RAII

### Design Separation
- **SDLApplication**: Handles high-level application logic, windowing, and event processing
- **GPUManager**: Encapsulates all GPU-specific operations and rendering
- **Relationship**: SDLApplication owns a GPUManager instance and delegates rendering operations to it

## Dependencies

### External Libraries
- **SDL3**: Core windowing and event system
- **SDL3_image**: Image loading support
- **SDL3_mixer**: Audio mixing capabilities
- **SDL3_ttf**: TrueType font rendering
- **spdlog**: Logging framework

### Internal Dependencies
- **msd_sim**: Simulation core library

## Resource Management

### Content Directory Structure
```
msd/msd-gui/src/Content/
├── Images/          # Image assets
└── Shaders/
    ├── Source/      # Shader source files (.vert, .frag)
    └── Compiled/    # Compiled shaders for different backends
        ├── SPIRV/   # Vulkan/OpenGL shaders
        ├── MSL/     # Metal shaders (macOS/iOS)
        └── DXIL/    # DirectX shaders (Windows)
```

### Build System Integration
- CMake automatically copies the `Content/` folder to the binary output directory during build
- Content is also installed to the installation directory for distribution
- Shaders are loaded at runtime based on the detected GPU backend

## Key Features

### Multi-Backend Shader Support
The GPUManager automatically detects and loads the appropriate shader format:
- **SPIRV**: Vulkan and OpenGL backends
- **MSL**: Metal backend (macOS/iOS)
- **DXIL**: DirectX 12 backend (Windows)

### RAII-Based Resource Management
- Custom deleters for SDL resources (SDLWindowDeleter, SDLDeviceDeleter, ShaderDeleter)
- Smart pointers (`std::unique_ptr`) ensure proper cleanup
- Exception-safe initialization

### Exception Handling
- `SDLException`: Custom exception class that combines error messages with SDL error details
- Used throughout for clear error reporting

## Usage Pattern

```cpp
// Get the singleton application instance
auto& app = msd_gui::SDLApplication::getInstance();

// Run the application (blocks until exit)
app.runApp();

// Application handles:
// - Window creation
// - GPU initialization
// - Event processing
// - Rendering loop
// - Resource cleanup
```

## Implementation Notes

### Shader Loading
- Shaders are referenced by base filename (e.g., "RawTriangle.vert")
- The system automatically:
  - Detects shader stage from extension (.vert or .frag)
  - Selects appropriate backend format based on GPU capabilities
  - Constructs full path with correct backend directory
  - Loads and compiles the shader

### Render Loop
1. Handle SDL events (user input, window events)
2. Acquire GPU command buffer
3. Acquire swapchain texture
4. Begin render pass with clear color
5. Execute rendering commands
6. End render pass
7. Submit command buffer

### Thread Safety
- SDLApplication is a singleton with deleted copy/move constructors
- Not designed for multi-threaded access to the main rendering context
- Event handling occurs on the main thread

## Future Considerations

### Potential Extensions
- Multiple window support
- Render target abstraction
- Material system
- Scene graph integration
- UI widget system
- Input abstraction layer
- Audio integration with SDL3_mixer
- Font rendering with SDL3_ttf

### Integration with msd_sim
- The GUI layer is designed to visualize simulation data from msd_sim
- Future work will connect simulation state to rendering pipeline
- Consider observer pattern for simulation updates

## Building

The library is built as part of the main MSD CMake project:
```bash
cmake --build build
```

Headers are installed to support external usage:
```cmake
target_link_libraries(your_target PRIVATE msd_gui)
```

## Testing

Currently focused on integration testing through the main executable.
Future work should include:
- Unit tests for resource management
- Mock SDL for headless testing
- Shader compilation verification
- Performance benchmarks
