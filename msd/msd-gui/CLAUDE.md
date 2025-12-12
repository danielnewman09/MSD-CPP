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

## Coding Standards

### Initialization and Construction

#### Uninitialized Member Variables
- **Always** use `std::numeric_limits<T>::quiet_NaN()` for default/uninitialized floating-point values
- **Never** use magic numbers like `-1.0f` or `0.0f` to represent uninitialized state
- **Rationale**: NaN propagates through calculations and makes uninitialized access immediately obvious

```cpp
// GOOD
class GPUBuffer {
private:
  float scale_{std::numeric_limits<float>::quiet_NaN()};
  float offset_{std::numeric_limits<float>::quiet_NaN()};
};

// BAD
class GPUBuffer {
private:
  float scale_{-1.0f};   // Magic number - unclear if -1 is valid or uninitialized
  float offset_{0.0f};   // Could be confused with actual zero value
};
```

#### Brace Initialization
- **Always** use brace initialization `{}` for constructing objects
- **Never** use parentheses `()` for initialization
- **Rationale**: Avoids the Most Vexing Parse problem and provides consistent syntax

```cpp
// GOOD
Vertex vertex{position, 1.0f, 0.0f, 0.0f, normal};
std::vector<Vertex> vertices{vertex1, vertex2, vertex3};
auto manager = GPUManager{window, basePath};

// BAD
Vertex vertex(position, 1.0f, 0.0f, 0.0f, normal);  // Can be confused with function
std::vector<Vertex> vertices(3);                     // Ambiguous syntax
auto manager = GPUManager(window, basePath);        // Most Vexing Parse risk
```

### Rule of Zero/Five
- **Prefer** the Rule of Zero: use compiler-generated special member functions when possible
- **Only** implement copy/move constructors/assignment if you need custom behavior
- **Use** `= default` explicitly to document that you're using the compiler's implementation
- **Delete** copy/move operations when they don't make sense (e.g., singletons, GPU resources)

```cpp
// GOOD - Rule of Zero with explicit default
struct Vertex {
  msd_sim::Coordinate position;
  float r, g, b;
  msd_sim::Coordinate normal;
  // Compiler-generated copy/move is perfect
};

// GOOD - Deleted for resource management
class GPUManager {
public:
  GPUManager(const GPUManager&) = delete;
  GPUManager& operator=(const GPUManager&) = delete;
  GPUManager(GPUManager&&) = delete;
  GPUManager& operator=(GPUManager&&) = delete;
};
```

### Naming Conventions
- **Don't** use `cached` prefix for member variables unless the value is truly cached (lazily computed)
- **Use** descriptive names that indicate the value's purpose
- **Distinguish** between computed-once values and lazily-cached values

```cpp
// GOOD
class RenderState {
private:
  SDL_GPUTexture* depthTexture_;        // Active depth texture
  uint32_t width_;                      // Current viewport width
  mutable Matrix4x4 viewMatrix_;        // Lazily computed view matrix
  mutable bool viewMatrixValid_;        // Cache validity flag
};

// BAD
class RenderState {
private:
  SDL_GPUTexture* cachedDepthTexture_;  // Misleading - not cached
  uint32_t cachedWidth_;                // Misleading - not cached
};
```

### Function Return Values
- **Prefer** returning values over modifying parameters passed by reference
- **Use** return values or return structs instead of output parameters
- **Rationale**: Makes code more functional, easier to reason about, and prevents accidental modifications

```cpp
// GOOD - Return a struct
struct Viewport {
  uint32_t width;
  uint32_t height;
};

Viewport getViewport() const {
  return Viewport{width_, height_};
}

auto viewport = manager.getViewport();
float aspectRatio = static_cast<float>(viewport.width) / viewport.height;

// BAD - Modify parameters by reference
void getViewport(uint32_t& width, uint32_t& height) const {
  width = width_;
  height = height_;
}

uint32_t width, height;
manager.getViewport(width, height);  // Harder to understand data flow
float aspectRatio = static_cast<float>(width) / height;
```

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
