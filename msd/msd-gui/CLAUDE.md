# msd-gui Library Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-gui/` for detailed component relationships.

## Project Overview

The msd-gui library provides a modern C++20 graphics user interface layer for the MSD (Multi-Spacecraft Dynamics) project. It wraps SDL3 (Simple DirectMedia Layer 3) with GPU-accelerated rendering using SDL's GPU API, supporting instanced 3D rendering with perspective projection. The library handles window management, keyboard input, camera control, and GPU pipeline management.

## Architecture Overview

### High-Level Architecture

See: [`docs/msd/msd-gui/msd-gui-core.puml`](../../docs/msd/msd-gui/msd-gui-core.puml)

The library consists of four main subsystems:
- **Application Layer** — Window management, event handling, main loop coordination
- **GPU Management** — Device initialization, shader loading, pipeline creation, rendering
- **Camera System** — 3D perspective camera with MVP matrix computation
- **Utilities** — Exception handling, shader loading, custom deleters

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| SDLApplication | `src/` | Application lifecycle, window management, event handling | [`sdl-application.puml`](../../docs/msd/msd-gui/sdl-application.puml) |
| GPUManager | `src/` | GPU device, pipeline, instanced rendering | [`gpu-manager.puml`](../../docs/msd/msd-gui/gpu-manager.puml) |
| Camera3D | `src/` | 3D camera with MVP matrix computation | [`camera3d.puml`](../../docs/msd/msd-gui/camera3d.puml) |
| SDLUtils | `src/` | Exception class and shader loading utilities | — |

---

## Component Details

### SDLApplication

**Location**: `src/SDLApp.hpp`, `src/SDLApp.cpp`
**Diagram**: [`docs/msd/msd-gui/sdl-application.puml`](../../docs/msd/msd-gui/sdl-application.puml)

#### Purpose
Manages the application lifecycle including window creation, event handling, and render loop coordination. Owns the GPUManager for rendering and msd_sim::Engine for simulation.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `SDLApplication` | `SDLApp.hpp` | Application lifecycle, window management, event handling |

#### Key Interfaces
```cpp
class SDLApplication {
public:
    enum class Status : uint8_t {
        Starting, Running, Paused, Error, Exiting
    };

    SDLApplication(const std::string& dbPath);
    int runApp();
    Status getStatus() const;
};
```

#### Usage Example
```cpp
msd_gui::SDLApplication app{"assets.db"};
int result = app.runApp();  // Blocks until exit
```

#### Thread Safety
- Not designed for multi-threaded access
- Event handling and rendering occur on the main thread
- Copy/move operations deleted

#### Error Handling
- Throws `SDLException` on window creation failure
- Returns `EXIT_SUCCESS` on normal exit

#### Dependencies
- `GPUManager` — GPU rendering (owned via `std::unique_ptr`)
- `msd_sim::Engine` — Simulation engine (owned via value semantics)
- SDL3 — Window and event management

---

### GPUManager

**Location**: `src/SDLGPUManager.hpp`, `src/SDLGPUManager.cpp`
**Diagram**: [`docs/msd/msd-gui/gpu-manager.puml`](../../docs/msd/msd-gui/gpu-manager.puml)

#### Purpose
Handles all GPU-related operations including device initialization, shader loading, pipeline creation, and instanced rendering with depth buffering.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `GPUManager` | `SDLGPUManager.hpp` | GPU device, pipeline, instanced rendering |
| `InstanceData` | `SDLGPUManager.hpp` | Per-instance position and color data |

#### Key Interfaces
```cpp
struct InstanceData {
    float position[3];  // World position offset
    float color[3];     // Instance color
};

class GPUManager {
public:
    explicit GPUManager(SDL_Window& window, const std::string& basePath);

    void render();
    Camera3D& getCamera();

    // Instance management
    void addInstance(float posX, float posY, float posZ,
                     float r, float g, float b);
    void removeInstance(size_t index);
    void updateInstance(size_t index, float posX, float posY, float posZ,
                        float r, float g, float b);
    void clearInstances();
    size_t getInstanceCount() const;
};
```

#### Usage Example
```cpp
GPUManager gpu{window, basePath};

// Add pyramid instances
gpu.addInstance(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);  // Red at origin
gpu.addInstance(2.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);  // Green at x=2

// Render frame
gpu.render();
```

#### Thread Safety
- Not designed for multi-threaded access
- All GPU operations on main thread
- Copy/move operations deleted

#### Error Handling
- Throws `SDLException` on device/pipeline creation failure
- Logs errors for invalid instance operations

#### Dependencies
- SDL3 GPU API — Device, pipeline, buffer management
- `Camera3D` — View/projection matrices (owned via value semantics)
- `msd_assets::GeometryFactory` — Primitive geometry generation

---

### Camera3D

**Location**: `src/Camera3D.hpp`, `src/Camera3D.cpp`
**Diagram**: [`docs/msd/msd-gui/camera3d.puml`](../../docs/msd/msd-gui/camera3d.puml)

#### Purpose
Provides a 3D camera with perspective projection, wrapping `msd_sim::ReferenceFrame` for position and orientation. Generates Model-View-Projection (MVP) matrices for shaders.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `Camera3D` | `Camera3D.hpp` | 3D perspective camera with MVP matrix computation |

#### Key Interfaces
```cpp
class Camera3D {
public:
    explicit Camera3D(const msd_sim::Coordinate& position,
                      float fovDegrees = 60.0f,
                      float aspectRatio = 16.0f / 9.0f,
                      float nearPlane = 0.1f,
                      float farPlane = 100.0f);

    msd_sim::ReferenceFrame& getReferenceFrame();
    const msd_sim::ReferenceFrame& getReferenceFrame() const;

    void setAspectRatio(float aspectRatio);
    void setFieldOfView(float fovDegrees);
    void setClippingPlanes(float nearPlane, float farPlane);

    Eigen::Matrix4f getViewMatrix() const;
    Eigen::Matrix4f getProjectionMatrix() const;
    Eigen::Matrix4f getMVPMatrix(
        const Eigen::Matrix4f& modelMatrix = Eigen::Matrix4f::Identity()) const;
};
```

#### Usage Example
```cpp
Camera3D camera{msd_sim::Coordinate{0., 0., 5.}};

// Move camera via reference frame
auto& frame = camera.getReferenceFrame();
frame.setOrigin(newPosition);
frame.getEulerAngles().yaw += rotationAmount;

// Get MVP for rendering
Eigen::Matrix4f mvp = camera.getMVPMatrix();
```

#### Coordinate System
Right-handed coordinate system:
- X: right
- Y: up
- Z: forward (out of screen, opposite viewing direction)

#### Thread Safety
- Not thread-safe (mutable reference frame)
- Designed for single-threaded use on main render thread

#### Error Handling
- No exceptions; caller responsible for valid parameters

#### Dependencies
- `msd_sim::ReferenceFrame` — Position and orientation (owned via value semantics)
- `Eigen::Matrix4f` — Matrix math

---

### SDLUtils

**Location**: `src/SDLUtils.hpp`, `src/SDLUtils.cpp`

#### Purpose
Utility classes and functions for SDL integration, including exception handling and shader loading.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `SDLException` | `SDLUtils.hpp` | Exception combining message with SDL error |
| `ShaderDeleter` | `SDLUtils.hpp` | Custom deleter for GPU shaders |

#### Key Interfaces
```cpp
class SDLException final : public std::runtime_error {
public:
    explicit SDLException(const std::string& message);
    // Message format: "{message}: {SDL_GetError()}"
};

using UniqueShader = std::unique_ptr<SDL_GPUShader, ShaderDeleter>;

UniqueShader loadShader(const std::string& shaderFilename,
                        SDL_GPUDevice& device,
                        const std::string& basePath,
                        uint32_t samplerCount,
                        uint32_t uniformBufferCount,
                        uint32_t storageBufferCount,
                        uint32_t storageTextureCount);
```

#### Usage Example
```cpp
auto vertexShader = loadShader(
    "Position3DColorTransform.vert", device, basePath, 0, 1, 0, 0);
if (!vertexShader) {
    throw SDLException("Failed to load vertex shader");
}
```

#### Thread Safety
- `loadShader` is stateless and thread-safe (but file I/O not synchronized)

#### Error Handling
- `SDLException` includes SDL error via `SDL_GetError()`
- `loadShader` returns nullptr on failure (no exceptions)

---

## Design Patterns in Use

### RAII Resource Management
**Used in**: Throughout library
**Purpose**: Custom deleters ensure proper cleanup of SDL resources

Custom deleters:
- `SDLWindowDeleter` — Window cleanup
- `SDLDeviceDeleter` — GPU device cleanup
- `PipelineDeleter` — Graphics pipeline cleanup
- `BufferDeleter` — GPU buffer cleanup
- `ShaderDeleter` — Shader cleanup

### Composition
**Used in**: `SDLApplication`, `GPUManager`
**Purpose**: Clear ownership hierarchy with value semantics and unique_ptr

Ownership chain:
- `SDLApplication` owns `GPUManager` (unique_ptr) and `Engine` (value)
- `GPUManager` owns `Camera3D` (value) and GPU resources (unique_ptr)
- `Camera3D` owns `ReferenceFrame` (value)

---

## Cross-Cutting Concerns

### Error Handling Strategy
- **Construction**: Throw `SDLException` for initialization failures
- **Runtime**: Log errors via `SDL_Log`, return error indicators
- **Resources**: RAII ensures cleanup on exceptions

### Memory Management
- `std::unique_ptr` with custom deleters for SDL/GPU resources
- Value semantics for simulation types (`Engine`, `ReferenceFrame`, `Camera3D`)
- No `std::shared_ptr` — clear ownership hierarchy
- Non-owning references for window access in `GPUManager`

### Thread Safety Conventions
- Single-threaded design for rendering context
- All GPU operations on main thread
- Copy/move deleted on resource-owning classes
- Event handling on main thread

### Coding Standards

This library follows the project-wide coding standards defined in the [root CLAUDE.md](../../CLAUDE.md#coding-standards).

Key standards applied in this library:
- **Initialization**: Brace initialization `{}` used throughout
- **Naming**: `snake_case_` for members, `PascalCase` for classes, `camelCase` for functions
- **Return Values**: Return values preferred over output parameters
- **Memory**: RAII via `std::unique_ptr` with custom deleters; no `std::shared_ptr`

See the [root CLAUDE.md](../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| W/S | Move camera forward/backward |
| A/D | Move camera left/right |
| Q/E | Move camera up/down |
| ↑/↓ | Pitch camera up/down |
| ←/→ | Yaw camera left/right |
| Z | Add random pyramid instance |
| X | Remove last instance |
| C | Clear all instances |

---

## Build & Configuration

### Build Requirements
- C++ Standard: C++20
- Compiler: GCC 11+, Clang 14+, or MSVC 2019+
- Build System: CMake 3.15+ with Conan 2.x package manager
- Dependencies (managed via Conan):
  - SDL3, SDL3_image, SDL3_mixer, SDL3_ttf — Graphics and media
  - spdlog — Logging
  - Eigen3 — Matrix math (via msd_sim)

### Building this Library

This library is built as part of the MSD-CPP project using Conan and CMake presets. See the [root CLAUDE.md](../../CLAUDE.md#build--configuration) for complete build instructions.

**Quick start:**
```bash
# Install dependencies with Conan
conan install . --build=missing -s build_type=Debug

# Build just this library (Debug)
cmake --preset conan-debug
cmake --build --preset debug-gui-only

# Or for Release
conan install . --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build --preset release-gui-only
```

**Component-specific presets:**
- Debug: `debug-gui-only` — Builds `msd-gui` library
- Release: `release-gui-only` — Builds `msd-gui` library

### Resource Directory

CMake automatically copies the `Content/` folder to the binary output directory:
```
src/Content/
├── Images/          # Image assets
└── Shaders/
    ├── Source/      # Shader source files (.vert, .frag)
    └── Compiled/    # Compiled shaders for different backends
        ├── SPIRV/   # Vulkan/OpenGL shaders (.spv)
        ├── MSL/     # Metal shaders (.msl)
        └── DXIL/    # DirectX shaders (.dxil)
```

Current shaders: `Position3DColorTransform.vert`, `SolidColor.frag`

---

## Testing

### Test Organization

Currently focused on integration testing through the main executable (`msd_exe`).

### Running the Application
```bash
# After building
./build/Debug/msd/msd-exe/msd_exe
```

---

## Conventions

### Naming Conventions
- Classes: `PascalCase`
- Functions/Methods: `camelCase`
- Member variables: `snake_case_` (trailing underscore)
- Constants: `kPascalCase`
- Namespaces: `snake_case`

### Code Organization
- Headers in `src/` (no separate include directory for this library)
- Implementation in `.cpp` files
- One class per header file
- Custom deleters as nested structs

### Documentation
- Public APIs: Doxygen-style comments with `@brief`, `@param`, `@return`
- Brief class-level documentation explaining purpose
- Implementation comments where non-obvious

---

## Diagrams Index

| Diagram | Description | Last Updated |
|---------|-------------|--------------|
| [`msd-gui-core.puml`](../../docs/msd/msd-gui/msd-gui-core.puml) | High-level architecture overview | 2026-01-01 |
| [`sdl-application.puml`](../../docs/msd/msd-gui/sdl-application.puml) | SDLApplication lifecycle and event handling | 2026-01-01 |
| [`gpu-manager.puml`](../../docs/msd/msd-gui/gpu-manager.puml) | GPUManager rendering pipeline | 2026-01-01 |
| [`camera3d.puml`](../../docs/msd/msd-gui/camera3d.puml) | Camera3D MVP matrix computation | 2026-01-01 |

---

## Getting Help

### For AI Assistants
1. Start with this document for architectural context
2. Reference the PlantUML diagrams in `docs/msd/msd-gui/` for component relationships:
   - [`msd-gui-core.puml`](../../docs/msd/msd-gui/msd-gui-core.puml) for high-level overview
   - Component-specific diagrams for detailed implementation
3. Check header files for detailed interface documentation
4. Refer to the main project [CLAUDE.md](../../CLAUDE.md) for overall coding standards
5. Review `SDLApp.cpp` for event handling and main loop logic
6. Review `SDLGPUManager.cpp` for GPU pipeline setup and rendering

### For Developers
- API documentation: See header file comments
- Example usage: See `msd-exe` for integration example
- Shader compilation: See `.vscode/tasks.json` for shader compilation task
- PlantUML diagrams: `docs/msd/msd-gui/`
