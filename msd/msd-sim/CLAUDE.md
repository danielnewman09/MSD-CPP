# msd-sim Library

## Overview
The `msd-sim` library is the core simulation engine for the MSD (Multi-Spacecraft Dynamics) project. It provides a modern C++20 framework for simulating dynamic systems with emphasis on aerospace applications, featuring robust mathematical primitives, coordinate transformations, 3D geometry handling, and agent-based simulation capabilities.

## Architecture

### Core Components

#### Engine
- **Purpose**: Main simulation orchestrator
- **Location**: `src/Engine.hpp`, `src/Engine.cpp`
- **Responsibilities**:
  - Time-stepped simulation updates
  - Integration with WorldModel
  - Top-level simulation lifecycle management

#### WorldModel
- **Purpose**: Container and manager for all simulated entities
- **Location**: `src/Environment/WorldModel.hpp`, `src/Environment/WorldModel.cpp`
- **Responsibilities**:
  - Platform collection management
  - Simulation time tracking
  - Coordinated updates across all platforms

#### Platform
- **Purpose**: Represents a single simulated entity (spacecraft, vehicle, etc.)
- **Location**: `src/Environment/Platform.hpp`, `src/Environment/Platform.cpp`
- **Components**:
  - Unique identifier (uint32_t)
  - InertialState (position, velocity, acceleration)
  - BaseAgent controller
  - Update tracking (last update time)
- **Responsibilities**:
  - State propagation
  - Agent control integration
  - Individual platform updates

### Mathematical Primitives

#### Coordinate System
- **Type**: Right-handed aerospace convention
- **Axes**: X-forward, Y-right, Z-up
- **Implementation**: Wraps Eigen::Vector3d for high-performance linear algebra
- **Location**: `src/Environment/Coordinate.hpp`

#### Angle
- **Purpose**: Type-safe angle representation with automatic normalization
- **Location**: `src/Environment/Angle.hpp`
- **Features**:
  - Stores values in radians internally
  - Two normalization modes:
    - `PI`: Normalizes to (-π, π]
    - `TWO_PI`: Normalizes to [0, 2π)
  - Factory methods: `fromRadians()`, `fromDegrees()`
  - Full arithmetic operator overloading
  - Conversion methods: `getRad()`, `toDeg()`
  - Normalization is applied lazily on access

#### EulerAngles
- **Purpose**: Represents 3D orientation using Euler angles
- **Convention**: ZYX intrinsic rotations (yaw-pitch-roll)
- **Location**: `src/Environment/EulerAngles.hpp`
- **Components**:
  - Roll: rotation around X-axis
  - Pitch: rotation around Y-axis
  - Yaw: rotation around Z-axis

#### InertialState
- **Purpose**: Complete kinematic state representation
- **Location**: `src/Environment/InertialState.hpp`
- **Components**:
  - Linear: position, velocity, acceleration (Coordinate)
  - Angular: angularPosition, angularVelocity, angularAcceleration (EulerAngles)

### Coordinate Transformation System

#### ReferenceFrame
- **Purpose**: Manages coordinate transformations between reference frames
- **Location**: `src/Environment/ReferenceFrame.hpp`, `src/Environment/ReferenceFrame.cpp`
- **Key Features**:
  - Translation: origin position in global coordinates
  - Rotation: 3x3 rotation matrix from Euler angles
  - Efficient batch transformations using Eigen matrix operations
  - Bidirectional transformations:
    - `globalToLocal()` / `globalToLocalInPlace()`
    - `localToGlobal()` / `localToGlobalInPlace()`
    - `globalToLocalBatch()` / `localToGlobalBatch()` (for Matrix3Xd)

#### Frame Hierarchy
```
Global Frame (inertial reference)
    ├── Platform Local Frame
    │   └── Sensor Frame (future)
    └── Viewer/Camera Frame
        └── Screen/Projection Space
```

### 3D Geometry System

#### Polyhedron
- **Purpose**: Represents 3D objects with triangulated geometry
- **Location**: `src/Environment/Polyhedron.hpp`, `src/Environment/Polyhedron.cpp`
- **Architecture**:
  - **Local vertices**: Constant geometry in object's local frame (Eigen::Matrix3Xd)
  - **ReferenceFrame**: Positions/orients object in global space
  - **Viewer frame vertices**: Cached transformed vertices for rendering
  - **SDL vertices**: 2D projected vertices with color for rendering
- **Features**:
  - Efficient batch transformations using Eigen
  - Perspective projection to 2D screen space
  - Wireframe and filled rendering modes
  - Configurable projection parameters (focal length, viewport)
  - Color management per object
  - Zero-copy access to SDL vertex buffers

#### STLLoader
- **Purpose**: Load 3D models from STL (STereoLithography) files
- **Location**: `src/Geometry/STLLoader.hpp`, `src/Geometry/STLLoader.cpp`
- **Supported Formats**:
  - **Binary STL**: Compact, fast, recommended (auto-detected)
  - **ASCII STL**: Human-readable, larger files
- **Features**:
  - Automatic format detection
  - Direct conversion to Polyhedron
  - Triangle data extractionP
  - File validation
- **Usage**: `STLLoader::loadSTL(filename, referenceFrame)`

### Agent System

#### BaseAgent (Abstract)
- **Purpose**: Interface for autonomous control logic
- **Location**: `src/Agent/BaseAgent.hpp`
- **Pattern**: Strategy pattern for control algorithms
- **Interface**: `virtual InertialState updateState(const InertialState& currentState) = 0;`
- **Design**: Pure virtual base class enables polymorphic agent behavior

### Utilities

#### GeometryFactory
- **Purpose**: Factory for creating common geometric primitives
- **Location**: `src/Utils/GeometryFactory.hpp`, `src/Utils/GeometryFactory.cpp`
- **Possible Functions**: Creating cubes, spheres, cylinders, etc.

#### Utils
- **Location**: `src/Utils/utils.hpp`, `src/Utils/utils.cpp`
- **Purpose**: Common utility functions

## Dependencies

### External Libraries
- **Eigen3**: Linear algebra (matrices, vectors, transformations)
- **spdlog**: Logging framework
- **SDL3**: Graphics rendering for Polyhedron visualization

### Internal Dependencies
- None (msd-sim is the base library)

## Module Structure

```
msd-sim/src/
├── Engine.{hpp,cpp}              # Main simulation engine
├── Agent/
│   └── BaseAgent.hpp             # Agent interface
├── Algorithms/                    # Future: path planning, control algorithms
│   ├── src/
│   └── test/
├── Environment/
│   ├── Angle.hpp                 # Angle with normalization
│   ├── Coordinate.hpp            # 3D coordinate (Eigen wrapper)
│   ├── EulerAngles.hpp           # Euler angle triplet
│   ├── InertialState.{hpp,cpp}   # Complete kinematic state
│   ├── Platform.{hpp,cpp}        # Simulated entity
│   ├── Polyhedron.{hpp,cpp}      # 3D geometric object
│   ├── ReferenceFrame.{hpp,cpp}  # Coordinate transformations
│   └── WorldModel.{hpp,cpp}      # Simulation world container
├── Geometry/
│   └── STLLoader.{hpp,cpp}       # STL file loading
└── Utils/
    ├── GeometryFactory.{hpp,cpp} # Geometric primitive factory
    └── utils.{hpp,cpp}           # General utilities
```

## Design Patterns

### RAII (Resource Acquisition Is Initialization)
- Smart pointers for agent ownership in Platform
- Automatic cleanup of dynamically allocated resources

### Strategy Pattern
- BaseAgent defines interface for control strategies
- Platforms use agents polymorphically
- Enables runtime swapping of control algorithms

### Factory Pattern
- GeometryFactory for creating geometric primitives
- STLLoader as factory for Polyhedron from files

### Value Semantics
- Coordinate, Angle, EulerAngles, InertialState are value types
- Copy-efficient due to small size and Eigen optimizations
- Clear ownership semantics

## Key Design Decisions

### Eigen Integration
- **Rationale**: High-performance matrix operations essential for transformations
- **Pattern**: Inherit from Eigen types (Coordinate extends Vector3d)
- **Benefit**: Direct access to Eigen's optimized linear algebra
- **Batch Operations**: Use Matrix3Xd for efficient multi-coordinate transforms

### Lazy Normalization
- **Angle normalization**: Applied on access (`getRad()`), not on every operation
- **Performance**: Reduces redundant normalization in arithmetic chains
- **Trade-off**: Internal value may be denormalized

### In-Place vs. Return Transformations
- **In-place** (`globalToLocalInPlace`): Modifies argument, no allocation
- **Return** (`globalToLocal`): Functional style, creates new object
- **Batch** operations: Always in-place for performance (Matrix3Xd)

### Frame Hierarchy
- **Three-level system**: Local → Global → Viewer
- **Separation**: Object geometry (local) separate from world position (global)
- **Rendering**: Viewer frame enables camera-relative rendering

## Performance Considerations

### Batch Transformations
- Use `Eigen::Matrix3Xd` for multiple coordinates
- Single matrix multiplication instead of N transformations
- Significant speedup for large polyhedra (hundreds/thousands of vertices)

### Memory Layout
- Contiguous storage in Eigen matrices
- Cache-friendly access patterns
- SIMD-friendly operations (Eigen auto-vectorization)

### SDL Integration
- Zero-copy vertex buffer access
- Reuse of transformation results
- Cached wireframe line points

## Usage Patterns

### Creating a Platform with State
```cpp
// Create initial state
InertialState initialState;
initialState.position = Coordinate(100.0, 200.0, 300.0);
initialState.velocity = Coordinate(1.0, 0.0, 0.0);
initialState.angularPosition.yaw = Angle::fromDegrees(45.0);

// Create platform
Platform platform(0);  // ID = 0
```

### Loading and Rendering 3D Models
```cpp
// Load STL file
auto spacecraft = STLLoader::loadSTL("spacecraft.stl");

// Position in world
spacecraft->getReferenceFrame().setOrigin(Coordinate(0, 0, 100));
spacecraft->getReferenceFrame().setRotation(EulerAngles{...});

// Setup camera/viewer
ReferenceFrame camera;
camera.setOrigin(Coordinate(0, 0, 1000));

// Transform to viewer frame and render
spacecraft->transformToViewerFrame(camera);
spacecraft->render(renderer);  // or renderWireframe()
```

### Coordinate Transformations
```cpp
// Create reference frame
ReferenceFrame localFrame;
localFrame.setOrigin(Coordinate(10, 20, 30));
localFrame.setRotation(EulerAngles{Angle::fromDegrees(0),
                                   Angle::fromDegrees(0),
                                   Angle::fromDegrees(90)});

// Transform single coordinate
Coordinate globalPoint(100, 200, 300);
Coordinate localPoint = localFrame.globalToLocal(globalPoint);

// Batch transform
Eigen::Matrix3Xd points(3, 1000);  // 1000 points
// ... populate points ...
localFrame.globalToLocalBatch(points);  // Efficient batch operation
```

### Angle Operations
```cpp
// Create angles
Angle heading = Angle::fromDegrees(350);
Angle turn = Angle::fromDegrees(30);

// Arithmetic (automatically maintains normalization)
Angle newHeading = heading + turn;  // 380° → normalized to 20°

// Access normalized value
double radians = newHeading.getRad();  // ≈ 0.349 rad
double degrees = newHeading.toDeg();   // 20°
```

## Testing

Test files located in:
- `msd-sim/test/`: Unit tests for core functionality
- `msd-sim/src/Algorithms/test/`: Algorithm-specific tests
- `msd-sim/examples/`: Example usage and integration tests

## Future Development

### Planned Features
- **Physics Integration**: Gravity models, orbital mechanics
- **Collision Detection**: For proximity operations
- **Sensor Models**: Camera, LIDAR, radar simulations
- **Communication**: Inter-platform messaging
- **Algorithms Module**: Path planning, guidance, navigation, control
- **Quaternion Support**: Alternative to Euler angles (gimbal lock avoidance)
- **Dynamics Models**: Force/torque-based motion
- **Multi-threading**: Parallel platform updates

### Integration Points
- **msd-gui**: Visualization layer consumes simulation state
- **Database**: State persistence and mission replay
- **External Tools**: Integration with flight dynamics software

## Mathematical Conventions

### Coordinate System
- **Type**: Right-handed Cartesian
- **Aerospace Convention**:
  - X-axis: Forward (velocity direction)
  - Y-axis: Right (starboard)
  - Z-axis: Up (zenith)

### Rotation Convention
- **Method**: Euler angles (ZYX intrinsic)
- **Order**: Yaw → Pitch → Roll
- **Ranges**:
  - Roll: (-π, π]
  - Pitch: (-π/2, π/2]
  - Yaw: (-π, π] or [0, 2π) depending on normalization

### Transformation Pipeline
```
Object vertices (local frame)
    ↓ localToGlobal (object's ReferenceFrame)
World coordinates (global frame)
    ↓ globalToViewer (camera's ReferenceFrame)
Viewer coordinates (camera frame)
    ↓ projectTo2D (perspective projection)
Screen coordinates (2D pixel space)
```

## Building

The library is built as part of the main MSD CMake project:
```bash
cmake -B build
cmake --build build
```

Link against msd-sim:
```cmake
target_link_libraries(your_target PRIVATE msd_sim)
```

## Coding Standards

### Initialization and Construction

#### Uninitialized Member Variables
- **Always** use `std::numeric_limits<T>::quiet_NaN()` for default/uninitialized floating-point values
- **Never** use magic numbers like `-1.0f` or `0.0f` to represent uninitialized state
- **Rationale**: NaN propagates through calculations and makes uninitialized access immediately obvious

```cpp
// GOOD
class Example {
private:
  float volume_{std::numeric_limits<float>::quiet_NaN()};
  float area_{std::numeric_limits<float>::quiet_NaN()};
};

// BAD
class Example {
private:
  float volume_{-1.0f};  // Magic number - unclear if -1 is valid or uninitialized
  float area_{0.0f};      // Could be confused with actual zero value
};
```

#### Brace Initialization
- **Always** use brace initialization `{}` for constructing objects
- **Never** use parentheses `()` for initialization
- **Rationale**: Avoids the Most Vexing Parse problem and provides consistent syntax

```cpp
// GOOD
Coordinate point{1.0f, 2.0f, 3.0f};
std::vector<int> values{1, 2, 3};
auto hull = ConvexHull{points};

// BAD
Coordinate point(1.0f, 2.0f, 3.0f);  // Can be confused with function declaration
std::vector<int> values(10, 0);      // Ambiguous syntax
auto hull = ConvexHull(points);      // Most Vexing Parse risk
```

### Rule of Zero/Five
- **Prefer** the Rule of Zero: use compiler-generated special member functions when possible
- **Only** implement copy/move constructors/assignment if you need custom behavior
- **Use** `= default` explicitly to document that you're using the compiler's implementation
- **Remove** manual implementations that just copy all members (let the compiler do it)

```cpp
// GOOD - Rule of Zero with explicit default
class ConvexHull {
public:
  ConvexHull(const ConvexHull&) = default;
  ConvexHull(ConvexHull&&) noexcept = default;
  ConvexHull& operator=(const ConvexHull&) = default;
  ConvexHull& operator=(ConvexHull&&) noexcept = default;
  ~ConvexHull() = default;

private:
  std::vector<Coordinate> vertices_;
  float volume_{std::numeric_limits<float>::quiet_NaN()};
};

// BAD - Unnecessary manual implementation
class ConvexHull {
public:
  ConvexHull(const ConvexHull& other)
    : vertices_(other.vertices_), volume_(other.volume_) {}
  // ... manual implementations for all special members
};
```

### Naming Conventions
- **Don't** use `cached` prefix for member variables unless the value is truly cached (lazily computed)
- **Use** descriptive names that indicate the value's purpose
- **Distinguish** between computed-once values and lazily-cached values

```cpp
// GOOD
class ConvexHull {
private:
  float volume_;                    // Computed once by Qhull
  float surfaceArea_;               // Computed once by Qhull
  mutable Coordinate centroid_;     // Lazily computed
  mutable bool centroidValid_;      // Cache validity flag
};

// BAD
class ConvexHull {
private:
  float cachedVolume_;              // Misleading - not cached, computed once
  float cachedSurfaceArea_;         // Misleading - not cached, computed once
  Coordinate cachedCentroid_;       // Correct - this IS cached
};
```

### Function Return Values
- **Prefer** returning values over modifying parameters passed by reference
- **Use** return values or return structs instead of output parameters
- **Rationale**: Makes code more functional, easier to reason about, and prevents accidental modifications

```cpp
// GOOD - Return a struct
struct BoundingBox {
  Coordinate min;
  Coordinate max;
};

BoundingBox getBoundingBox() const {
  return BoundingBox{boundingBoxMin_, boundingBoxMax_};
}

auto bbox = hull.getBoundingBox();
float width = bbox.max.x() - bbox.min.x();

// BAD - Modify parameters by reference
void getBoundingBox(Coordinate& min, Coordinate& max) const {
  min = boundingBoxMin_;
  max = boundingBoxMax_;
}

Coordinate min, max;
hull.getBoundingBox(min, max);  // Harder to understand data flow
float width = max.x() - min.x();
```

## Best Practices

### State Management
- Use InertialState for complete kinematic representation
- Update states through agents for controlled propagation
- Maintain clear ownership of state (platforms own their state)

### Transformations
- Use batch operations for multiple coordinates (10x+ speedup)
- Cache ReferenceFrame objects when possible (rotation matrix computation is expensive)
- Prefer in-place operations for large datasets

### Geometry
- Load STL models once, instantiate multiple times with different ReferenceFrames
- Reuse Polyhedron objects across frames (only transform, don't recreate)
- Set projection parameters once per render pass, not per object

### Angles
- Use Angle class instead of raw doubles for clarity and safety
- Choose appropriate normalization for your use case
- Leverage operator overloading for readable angle arithmetic

## Examples

See `msd-sim/examples/` for:
- `stl_loader_example.cpp`: Loading and displaying 3D models
- Additional examples coming soon

## Performance Benchmarks

_(To be added as performance testing framework is developed)_

### Target Performance
- Platform update: < 1μs per platform per timestep
- Coordinate transformation: < 10ns per coordinate (single)
- Batch transformation: < 1ms for 10,000 coordinates
- STL loading: < 100ms for typical spacecraft models (< 100k triangles)
