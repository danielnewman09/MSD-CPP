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
| Input Management | `src/` | Keyboard state tracking and input binding system | [`input-state-management.puml`](../../docs/designs/input-state-management/input-state-management.puml) |
| ShaderPolicy | `src/` | Template-based shader policy system for GPU pipeline configuration | [`modularize-gpu-shader-system.puml`](../../docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml) |
| GPUManager | `src/` | GPU device, pipeline, geometry registration, rendering orchestration | [`gpu-manager.puml`](../../docs/msd/msd-gui/gpu-manager.puml) |
| InstanceManager | `src/` | Per-instance data management, simulation-to-GPU synchronization | — |
| Camera3D | `src/` | 3D camera with MVP matrix computation | [`camera3d.puml`](../../docs/msd/msd-gui/camera3d.puml) |
| CameraController | `src/` | Camera movement control based on input state | [`input-state-management.puml`](../../docs/designs/input-state-management/input-state-management.puml) |
| SDLUtils | `src/` | Exception class and shader loading utilities | — |

---

## Component Details

### SDLApplication

**Location**: `src/SDLApp.hpp`, `src/SDLApp.cpp`
**Diagram**: [`docs/msd/msd-gui/sdl-application.puml`](../../docs/msd/msd-gui/sdl-application.puml)

#### Purpose
Manages the application lifecycle including window creation, event handling, and render loop coordination. Owns the GPUManager for rendering and msd_sim::Engine for simulation. Coordinates asset registration and simulation-to-rendering synchronization.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `SDLApplication` | `SDLApp.hpp` | Application lifecycle, window management, event handling, asset registration |

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

private:
    void registerAssets();        // Register visual geometries with GPUManager
    void handleEvents();          // SDL event processing
    void setupInputBindings();    // Configure keyboard bindings
    void spawnRandomObject(const std::string& geometryType);
};
```

#### Usage Example
```cpp
msd_gui::SDLApplication app{"assets.db"};
int result = app.runApp();  // Blocks until exit
```

#### Initialization Flow
1. Initialize SDL and create window
2. Create `msd_sim::Engine` with database path
3. Spawn player platform and obtain camera reference frame
4. Create `GPUManager` with camera frame reference
5. Setup input bindings
6. On `runApp()`: Register all visual assets with GPUManager

#### Main Loop
```cpp
while (status_ == Status::Running) {
    handleEvents();           // Process SDL events, update input state
    gpuManager_->update(engine_);  // Sync simulation → GPU and render
    engine_.update(currentTime);   // Step simulation forward
}
```

#### Thread Safety
- Not designed for multi-threaded access
- Event handling and rendering occur on the main thread
- Copy/move operations deleted

#### Error Handling
- Throws `SDLException` on window creation failure
- Throws if player platform's reference frame not found
- Returns `EXIT_SUCCESS` on normal exit

#### Dependencies
- `GPUManager<FullTransformShaderPolicy>` — GPU rendering (owned via `std::unique_ptr`)
- `msd_sim::Engine` — Simulation engine (owned via value semantics)
- `InputHandler` — Input binding management (owned via `std::unique_ptr`)
- SDL3 — Window and event management

---

### GPUManager

**Location**: `src/SDLGPUManager.hpp`, `src/SDLGPUManager.cpp`
**Diagram**: [`docs/msd/msd-gui/gpu-manager.puml`](../../docs/msd/msd-gui/gpu-manager.puml)

#### Purpose
Handles GPU-related operations including device initialization, shader loading, pipeline creation, geometry registration, and rendering orchestration. Delegates instance data management to `InstanceManager`.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `GPUManager<ShaderPolicy>` | `SDLGPUManager.hpp` | GPU device, pipeline, geometry registration, rendering |
| `GeometryInfo` | `GPUInstanceManager.hpp` | Tracks geometry location within unified vertex buffer |

#### Key Interfaces
```cpp
struct GeometryInfo {
    uint32_t baseVertex{0};   // Starting vertex index in unified buffer
    uint32_t vertexCount{0};  // Number of vertices for this geometry
};

template <typename ShaderPolicy>
class GPUManager {
public:
    explicit GPUManager(SDL_Window& window,
                        msd_sim::ReferenceFrame& cameraFrame,
                        const std::string& basePath);

    // Geometry registration (called during asset loading)
    uint32_t registerGeometry(uint32_t assetId,
                              const std::vector<msd_assets::Vertex>& vertices);

    // Object management (delegates to InstanceManager)
    void addObject(const msd_sim::AssetInertial& object, float r, float g, float b);

    // Frame update and rendering
    void update(const msd_sim::Engine& engine);
    void render();

    // Accessors
    Camera3D& getCamera();
    InstanceManager<ShaderPolicy>& getInstanceManager();
    SDL_GPUDevice& getDevice();
    SDL_GPUBuffer& getInstanceBuffer();
};
```

#### Usage Example
```cpp
// Create GPU manager with camera reference frame
GPUManager<FullTransformShaderPolicy> gpu{window, cameraFrame, basePath};

// Register geometry assets (called once during initialization)
gpu.registerGeometry(pyramidAssetId, pyramidVertices);
gpu.registerGeometry(cubeAssetId, cubeVertices);

// Add objects from simulation
gpu.addObject(inertialObject, 1.0f, 0.0f, 0.0f);  // Red object

// Game loop: update instances from simulation and render
gpu.update(engine);  // Syncs transforms and renders
```

#### Thread Safety
- Not designed for multi-threaded access
- All GPU operations on main thread
- Copy/move operations deleted

#### Error Handling
- Throws `SDLException` on device/pipeline creation failure
- Throws `SDLException` if object's asset ID not found in geometry registry
- Logs errors for invalid operations

#### Dependencies
- SDL3 GPU API — Device, pipeline, buffer management
- `Camera3D` — View/projection matrices (owned via value semantics)
- `InstanceManager<ShaderPolicy>` — Instance data management (owned via value semantics)
- `ShaderPolicy` — Compile-time shader configuration (template parameter)
- `msd_sim::Engine` — Source of simulation state for frame updates

---

### InstanceManager

**Location**: `src/GPUInstanceManager.hpp`
**Introduced**: Refactor objects integration (2026-01-18)

#### Purpose
Manages per-instance GPU data for rendering simulation objects. Handles the mapping between simulation object IDs and GPU instance indices, builds shader-specific instance data, and uploads instance buffers to the GPU.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `InstanceManager<ShaderPolicy>` | `GPUInstanceManager.hpp` | Instance data building, mapping, GPU upload |

#### Key Interfaces
```cpp
template <typename ShaderPolicy>
class InstanceManager {
public:
    using InstanceDataType = typename ShaderPolicy::InstanceDataType;

    // Object lifecycle
    void clearObjects();
    void removeObject(uint32_t instanceId);

    // Add object and upload to GPU
    size_t addObject(SDL_GPUDevice& device,
                     SDL_GPUBuffer& instanceBuffer,
                     const msd_sim::AssetInertial& object,
                     uint32_t geometryId,
                     float r, float g, float b);

    // Update all instances from simulation state
    void update(const msd_sim::Engine& engine);

    // GPU buffer management
    void uploadInstanceBuffer(SDL_GPUDevice& device,
                              SDL_GPUBuffer& instanceBuffer);

    // Accessors
    const std::vector<InstanceDataType>& getInstances() const;
};
```

#### Usage Example
```cpp
InstanceManager<FullTransformShaderPolicy> instanceManager;

// Add object with GPU upload
instanceManager.addObject(device, instanceBuffer, object, geometryId, 1.0f, 0.0f, 0.0f);

// Update all instances from simulation
instanceManager.update(engine);
instanceManager.uploadInstanceBuffer(device, instanceBuffer);

// Access instances for rendering
const auto& instances = instanceManager.getInstances();
```

#### Instance ID Mapping
The `InstanceManager` maintains a bidirectional mapping:
- **Simulation ID → GPU Index**: Maps `AssetInertial::getInstanceId()` to vector index
- Used for efficient lookup during updates and removals

#### Thread Safety
- Not thread-safe (single-threaded GUI operation assumed)
- All instance operations on main thread

#### Error Handling
- Logs errors for invalid instance removals (ID not found)
- Throws `SDLException` on GPU transfer buffer creation failure

#### Memory Management
- **Ownership**: Owned by value in `GPUManager`
- **Instance data**: `std::vector<InstanceDataType>` with value semantics
- **ID mapping**: `std::unordered_map<uint32_t, size_t>` for O(1) lookup

#### Dependencies
- `ShaderPolicy` — Determines instance data type and layout
- `msd_sim::Engine` — Source of simulation state for updates
- `msd_sim::AssetInertial` — Simulation object with transform data
- SDL3 GPU API — Transfer buffer operations

---

### ShaderPolicy

**Location**: `src/ShaderPolicy.hpp`, `src/ShaderPolicy.cpp`
**Diagram**: [`docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml`](../../docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml)
**Introduced**: [Ticket: 0002_remove_rotation_from_gpu](../../tickets/0002_remove_rotation_from_gpu.md)

#### Purpose
Provides a compile-time template-based policy system for configuring GPU pipeline shader types and instance data layouts. Enables GPUManager to support multiple shader configurations (position-only vs full transform) without runtime overhead, using C++20 policy-based design for zero-cost abstraction.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `PositionOnlyShaderPolicy` | `ShaderPolicy.hpp` | Shader policy for simple position-offset rendering (32-byte instance data) |
| `FullTransformShaderPolicy` | `ShaderPolicy.hpp` | Shader policy for full 4x4 model matrix rendering (96-byte instance data) |
| `PositionOnlyInstanceData` | `ShaderPolicy.hpp` | Instance data structure for position-only rendering |
| `FullTransformInstanceData` | `ShaderPolicy.hpp` | Instance data structure for full transform rendering |

#### Key Interfaces
```cpp
// Instance data for Position3DColorTransform shader
struct PositionOnlyInstanceData {
    float position[3];       // World position offset (12 bytes)
    float color[3];          // RGB color (12 bytes)
    uint32_t padding[2]{0};  // 16-byte alignment (8 bytes)
    // Total: 32 bytes
};

// Instance data for PositionRotation3DColorTransform shader
struct FullTransformInstanceData {
    float modelMatrix[16];      // 4x4 transform matrix (64 bytes)
    float color[3];             // RGB color (12 bytes)
    uint32_t geometryIndex{0};  // Geometry registry index (4 bytes)
    uint32_t padding[4]{0};     // 16-byte alignment (16 bytes)
    // Total: 96 bytes
};

// Shader policy interface (PositionOnlyShaderPolicy shown)
class PositionOnlyShaderPolicy {
public:
    using InstanceDataType = PositionOnlyInstanceData;

    static constexpr const char* kShaderName = "PositionOnly";
    static constexpr const char* kVertexShaderFile = "Position3DColorTransform.vert";
    static constexpr const char* kFragmentShaderFile = "SolidColor.frag";

    std::vector<SDL_GPUVertexAttribute> getVertexAttributes() const;
    std::vector<SDL_GPUVertexBufferDescription> getVertexBufferDescriptions() const;
    SDL_GPUVertexInputState getVertexInputState() const;

    // Build instance data from AssetInertial (color passed separately)
    std::vector<uint8_t> buildInstanceData(const msd_sim::AssetInertial& object,
                                           float r, float g, float b) const;

    std::string getVertexShaderFile() const;
    std::string getFragmentShaderFile() const;
    size_t getInstanceDataSize() const;
};

// FullTransformShaderPolicy adds geometryIndex parameter
class FullTransformShaderPolicy {
    // ... same interface plus:
    std::vector<uint8_t> buildInstanceData(const msd_sim::AssetInertial& object,
                                           float r, float g, float b,
                                           uint32_t geometryIndex) const;
};
```

#### Usage Example
```cpp
// Choose shader policy at compile time via template parameter
using AppGPUManager = GPUManager<PositionOnlyShaderPolicy>;

// Or use FullTransformShaderPolicy for rotation support
using AppGPUManager = GPUManager<FullTransformShaderPolicy>;

// GPUManager automatically uses the policy's configuration
AppGPUManager gpuManager{window, basePath};
```

#### Policy Selection
**Current default**: `PositionOnlyShaderPolicy` (set in `SDLApp.hpp`)

To switch shader policies, modify the type alias in `SDLApp.hpp`:
```cpp
// In SDLApp.hpp
using AppGPUManager = GPUManager<FullTransformShaderPolicy>;  // Enable rotation
```

#### Thread Safety
- All policy methods are const and stateless (thread-safe)
- Instance data structures are plain data (no synchronization needed)
- Policies are typically instantiated as members of GPUManager (single-threaded)

#### Error Handling
- Returns valid configurations for all methods (no exceptions)
- Instance data building uses brace initialization (guaranteed valid layout)
- Foreign key resolution returns `std::optional` for geometry lookups

#### Memory Management
- **Ownership**: Policies owned by value in GPUManager template instantiation
- **Instance data**: Plain structs with value semantics, managed in `std::vector<InstanceDataType>`
- **GPU alignment**: Both instance data types padded to 16-byte boundaries for GPU efficiency

#### Performance Characteristics
**PositionOnlyShaderPolicy**:
- Instance data: 32 bytes (62% reduction vs FullTransform)
- Rendering: Single draw call for all instances
- GPU bandwidth: 32 KB per frame for 1000 instances

**FullTransformShaderPolicy**:
- Instance data: 96 bytes (16-byte aligned 4x4 matrix)
- Rendering: Grouped by geometry index, multiple draw calls
- GPU bandwidth: 96 KB per frame for 1000 instances

#### Dependencies
- SDL3 GPU — Vertex attribute and buffer description types
- `msd_sim::Object` — Source data for instance data building
- `msd_sim::ReferenceFrame` — Transform data (FullTransformShaderPolicy only)
- `Eigen::Matrix4f` — 4x4 matrix math (FullTransformShaderPolicy only)

---

### Input Management

**Location**: `src/InputState.hpp`, `src/InputState.cpp`, `src/InputHandler.hpp`, `src/InputHandler.cpp`
**Diagram**: [`docs/designs/input-state-management/input-state-management.puml`](../../docs/designs/input-state-management/input-state-management.puml)
**Introduced**: [Ticket: 0004_gui_framerate](../../tickets/0004_gui_framerate.md)

#### Purpose
Provides a comprehensive input management system that separates input state tracking from input handling logic. Enables flexible control of both camera and simulation objects via keyboard input with support for multiple input modes (Continuous, TriggerOnce, Interval, PressAndHold).

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `InputState` | `InputState.hpp` | Tracks current state of all keyboard inputs with timestamp information |
| `InputHandler` | `InputHandler.hpp` | Manages collection of input bindings and processes them based on InputState |
| `InputBinding` | `InputHandler.hpp` | Binds a key to an action with specified input mode |
| `InputMode` | `InputHandler.hpp` | Enum defining how bindings should trigger (Continuous, TriggerOnce, Interval, PressAndHold) |

#### Key Interfaces
```cpp
// Input state tracking
struct KeyState {
  bool pressed{false};
  bool justPressed{false};
  std::chrono::milliseconds pressTime{0};
  std::chrono::milliseconds lastTriggerTime{0};
};

class InputState {
public:
  void updateKey(SDL_Keycode key, bool pressed);
  bool isKeyPressed(SDL_Keycode key) const;
  bool isKeyJustPressed(SDL_Keycode key) const;
  std::chrono::milliseconds getKeyHoldDuration(SDL_Keycode key) const;
  void update(std::chrono::milliseconds deltaTime);
};

// Input mode configuration
enum class InputMode : uint8_t {
  Continuous,   // Trigger every frame while key is held
  TriggerOnce,  // Trigger only on initial press
  Interval,     // Trigger at fixed intervals while held
  PressAndHold  // Trigger on release with hold duration
};

// Input binding system
class InputBinding {
public:
  InputBinding(SDL_Keycode key, InputMode mode,
               std::function<void()> action,
               std::chrono::milliseconds interval = std::chrono::milliseconds{0});

  bool shouldTrigger(const InputState& state);
  void execute();
};

class InputHandler {
public:
  void addBinding(InputBinding binding);
  void handleSDLEvent(const SDL_Event& event);
  void update(std::chrono::milliseconds deltaTime);
  void processInput();
  const InputState& getInputState() const;
};
```

#### Usage Example
```cpp
InputHandler inputHandler;

// Add continuous camera movement (W key)
inputHandler.addBinding(InputBinding{
  SDLK_W,
  InputMode::Continuous,
  [this]() { camera.moveForward(); }
});

// Add single-trigger object spawning (Z key)
inputHandler.addBinding(InputBinding{
  SDLK_Z,
  InputMode::TriggerOnce,
  [this]() { spawnObject("pyramid"); }
});

// In event loop
while (SDL_PollEvent(&event)) {
  inputHandler.handleSDLEvent(event);
}

// After event processing
inputHandler.update(deltaTime);
inputHandler.processInput();
```

#### Input Modes
- **Continuous**: Executes action every frame while key is held (e.g., WASD movement)
- **TriggerOnce**: Executes action only on initial keypress, ignoring hold (e.g., spawn object)
- **Interval**: Executes action at fixed intervals while key is held (e.g., auto-fire)
- **PressAndHold**: Executes action on key release with hold duration (e.g., charge throw)

#### Thread Safety
- Not thread-safe (single-threaded GUI operation assumed)
- All input processing on main thread
- InputState is mutable; InputHandler owns single source of truth

#### Error Handling
- InputState returns default values for unknown keys (no exceptions)
- InputBinding action execution failures propagate to caller
- InputHandler provides no exceptions from public interface

#### Memory Management
- **InputHandler**: Delete copy, allow move (owns unique InputState)
- **InputBinding**: Default copyable (std::function is copyable)
- **InputState**: Default copyable (value semantics for key state map)

#### Performance Characteristics
- InputState lookup: O(1) via std::unordered_map
- Binding evaluation: O(n) where n = number of bindings (typically <20)
- Memory overhead: ~100 bytes per key state, ~50 bytes per binding
- Frame rate impact: Negligible (<1% CPU for typical usage)

#### Dependencies
- SDL3 — Event handling and keycode definitions
- chrono — Timestamp tracking for duration-based queries

---

### CameraController

**Location**: `src/CameraController.hpp`, `src/CameraController.cpp`
**Diagram**: [`docs/designs/input-state-management/input-state-management.puml`](../../docs/designs/input-state-management/input-state-management.puml)
**Introduced**: [Ticket: 0004_gui_framerate](../../tickets/0004_gui_framerate.md)

#### Purpose
Encapsulates camera movement logic based on InputState. Separates camera control logic from SDLApplication (single responsibility principle). Provides frame-rate independent camera movement through delta-time scaling.

#### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `CameraController` | `CameraController.hpp` | Camera movement control based on input state |

#### Key Interfaces
```cpp
class CameraController {
public:
  explicit CameraController(Camera3D& camera,
                            float moveSpeed = 0.1f,
                            msd_sim::Angle rotSpeed = msd_sim::Angle::fromRadians(0.05));

  void updateFromInput(const InputState& inputState,
                       std::chrono::milliseconds deltaTime);

  void setMoveSpeed(float speed);
  void setRotationSpeed(msd_sim::Angle speed);
  void setSensitivity(float sensitivity);

  float getMoveSpeed() const;
  msd_sim::Angle getRotationSpeed() const;
};
```

#### Usage Example
```cpp
Camera3D camera{msd_sim::Coordinate{0., 0., 5.}};
CameraController controller{camera, 0.1f, msd_sim::Angle::fromRadians(0.05)};

// In render loop
controller.updateFromInput(inputHandler.getInputState(), deltaTime);
```

#### Camera Movement Mapping
- **W/S**: Move forward/backward in camera's local Z direction
- **A/D**: Move left/right in camera's local X direction
- **Q/E**: Move up/down in camera's local Y direction
- **Arrow Up/Down**: Pitch camera
- **Arrow Left/Right**: Yaw camera

#### Thread Safety
- Not thread-safe (mutable camera reference)
- Designed for single-threaded use on main render thread

#### Error Handling
- No exceptions; caller responsible for valid parameters

#### Memory Management
- **Ownership**: Non-owning reference to Camera3D (camera owned by GPUManager)
- **Value semantics**: Movement/rotation speeds stored by value
- **Delete copy**: Non-copyable (camera reference not rebindable)
- **Allow move**: Movable for flexibility

#### Dependencies
- `Camera3D` — Camera to control (non-owning reference)
- `InputState` — Input state querying
- `msd_sim::Angle` — Type-safe rotation speed

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

### Policy-Based Design (Template)
**Used in**: `GPUManager<ShaderPolicy>`
**Purpose**: Compile-time shader configuration for zero-cost abstraction

See implementation: [`docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml`](../../docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml)

The shader policy system enables:
- Zero runtime overhead (no virtual function calls)
- Type-safe instance data layouts via `ShaderPolicy::InstanceDataType`
- Compile-time shader selection (PositionOnly vs FullTransform)
- Future extensibility for additional shader types

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
| Z | Spawn random pyramid at random position |
| V | Spawn random cube at random position |
| C | Clear all simulation objects |

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

## Recent Architectural Changes

### Simulation-GPU Integration Refactor — 2026-01-18

Refactored the GUI layer to integrate directly with `msd_sim::Engine` and `msd_sim::AssetInertial` objects instead of using mock object vectors. Introduced `InstanceManager` to separate instance data management from GPU resource management.

**Key changes**:
- `msd/msd-gui/src/GPUInstanceManager.hpp` — New `InstanceManager<ShaderPolicy>` template class for instance data management
- `msd/msd-gui/src/SDLGPUManager.hpp` — Refactored to own `InstanceManager`, dynamic geometry registration via asset IDs
- `msd/msd-gui/src/SDLApp.hpp/.cpp` — Removed mock object storage, added `registerAssets()` for asset-to-geometry mapping
- `msd/msd-gui/src/ShaderPolicy.hpp/.cpp` — `buildInstanceData()` now takes `AssetInertial` + explicit color instead of `Object`

**Design decisions**:
- **InstanceManager separation**: Decouples instance data lifecycle (add/remove/update) from GPU pipeline management
- **Asset ID mapping**: `GPUManager` maintains `assetIdToGeometryIndex_` map for O(1) geometry lookup from simulation objects
- **Direct simulation integration**: `GPUManager::update(engine)` pulls transforms directly from `msd_sim::Engine::WorldModel`
- **Emscripten removal**: Removed browser/WebAssembly support to simplify codebase (native-only)
- **Color externalization**: Object color now passed as parameters rather than stored on simulation objects

**API changes**:
- `GPUManager::addObject()` now takes `AssetInertial&` instead of `Object`
- `GPUManager::updateObjects()` replaced with `GPUManager::update(engine)`
- `GPUManager::registerGeometry()` now uses `uint32_t assetId` instead of `string name`
- Removed `GPUManager::removeObject()` and `GPUManager::clearObjects()` (use `InstanceManager` directly)

---

### Input State Management System — 2026-01-05
**Ticket**: [0004_gui_framerate](../../tickets/0004_gui_framerate.md)
**Diagram**: [`docs/designs/input-state-management/input-state-management.puml`](../../docs/designs/input-state-management/input-state-management.puml)

Introduced comprehensive input management system that separates input state tracking from input handling logic. This enables flexible control of both camera and simulation objects via keyboard input with support for multiple input modes (Continuous, TriggerOnce, Interval, PressAndHold).

**Key changes**:
- `msd/msd-gui/src/InputState.hpp/.cpp` — Keyboard state tracking with timestamp information
- `msd/msd-gui/src/InputHandler.hpp/.cpp` — Binding management and processing with InputMode support
- `msd/msd-gui/src/CameraController.hpp/.cpp` — Camera movement encapsulation with delta-time scaling
- `msd/msd-gui/src/SDLApp.hpp/.cpp` — Integrated new input system, added frame timing

**Design decisions**:
- Separation of concerns: InputState (tracking) vs InputHandler (processing) vs CameraController (application)
- TriggerOnce bindings execute immediately in handleSDLEvent (not deferred to processInput)
- Frame-rate independence through delta time tracking
- Non-owning camera reference in CameraController (owned by GPUManager)
- Extensible InputMode enum for future input behaviors

### Shader Policy System — 2026-01-03
**Ticket**: [0002_remove_rotation_from_gpu](../../tickets/0002_remove_rotation_from_gpu.md)
**Diagram**: [`docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml`](../../docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml)

Refactored GPUManager to support multiple shader types through a compile-time template-based policy system. This enables switching between position-only rendering (32-byte instance data) and full transform rendering (96-byte instance data with 4x4 matrices) without runtime overhead.

**Key changes**:
- `msd/msd-gui/src/ShaderPolicy.hpp` — Shader policy interface and implementations
- `msd/msd-gui/src/ShaderPolicy.cpp` — Policy method implementations
- `msd/msd-gui/src/SDLGPUManager.hpp` — Template-ized with ShaderPolicy parameter
- `msd/msd-gui/src/SDLApp.hpp` — Type alias for AppGPUManager with default policy
- `msd/msd-gui/test/unit/shader_policy_test.cpp` — Unit tests for shader policies

**Design decisions**:
- Compile-time template approach for zero-cost abstraction
- 16-byte aligned instance data for GPU efficiency
- PositionOnlyShaderPolicy as default for simplified debugging
- Future extensibility for additional shader types (textured, normal-mapped, etc.)

---

## Diagrams Index

| Diagram | Description | Last Updated |
|---------|-------------|--------------|
| [`msd-gui-core.puml`](../../docs/msd/msd-gui/msd-gui-core.puml) | High-level architecture overview | 2026-01-01 |
| [`sdl-application.puml`](../../docs/msd/msd-gui/sdl-application.puml) | SDLApplication lifecycle and event handling | 2026-01-01 |
| [`input-state-management.puml`](../../docs/designs/input-state-management/input-state-management.puml) | Input management system architecture | 2026-01-05 |
| [`modularize-gpu-shader-system.puml`](../../docs/designs/modularize-gpu-shader-system/modularize-gpu-shader-system.puml) | Shader policy system architecture | 2026-01-03 |
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
