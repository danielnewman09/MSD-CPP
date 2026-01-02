# msd-exe Library Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/` for detailed component relationships.

## Project Overview

The `msd-exe` library is the main executable for the MSD (Multi-Spacecraft Dynamics) project. It serves as the entry point that integrates the simulation engine (msd-sim) with the graphical user interface (msd-gui) to create a complete simulation visualization application.

## Architecture Overview

### High-Level Architecture

See: [`docs/msd/msd-exe/msd-exe.puml`](../../docs/msd/msd-exe/msd-exe.puml)

The executable acts as a thin integration layer that:
- Resolves the asset database path
- Initializes the SDL application with the database connection
- Delegates all runtime behavior to msd-gui's SDLApplication

### System Integration

```
msd-exe (executable entry point)
    └── SDLApplication (from msd-gui)
        ├── GPUManager (rendering pipeline)
        └── Engine (from msd-sim)
            └── WorldModel (simulation state)
                └── Platform[] (simulated entities)
```

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| main | `src/main.cpp` | Application entry point and initialization | [`msd-exe.puml`](../../docs/msd/msd-exe/msd-exe.puml) |

---

## Component Details

### Main Application Entry Point

**Location**: `src/main.cpp`

#### Purpose
Provides the `main()` function that bootstraps the entire MSD application by:
1. Resolving the asset database path to an absolute path
2. Creating and initializing the SDLApplication instance
3. Running the main application loop
4. Returning the exit code

#### Implementation
```cpp
int main()
{
  std::string dbPath{"example_assets.db"};
  auto absolutePath = msd_utils::absolutePath(dbPath);

  msd_gui::SDLApplication application{absolutePath.string()};

  application.runApp();
  return 0;
}
```

#### Key Responsibilities
- **Path Resolution**: Converts relative database path to absolute path using `msd_utils::absolutePath()`
- **Application Initialization**: Constructs SDLApplication with database path
- **Lifecycle Management**: Delegates to `application.runApp()` for the main loop
- **Exit Handling**: Returns 0 on successful completion

#### Error Handling
- Path resolution errors propagate from `msd_utils::absolutePath()`
- SDLApplication constructor handles initialization errors
- Runtime exceptions handled within `application.runApp()`
- Process exits with 0 on success, non-zero on uncaught exception

#### Dependencies
- `msd_gui::SDLApplication` — Main application class
- `msd_utils::absolutePath()` — Path resolution utility
- Standard C++ (`std::string`)

---

## Design Rationale

### Why a Separate Executable Library?

**Modularity**: Keeps msd-sim and msd-gui as reusable libraries that can be integrated into other applications.

**Testing**: Allows independent testing of library components without requiring the full executable.

**Flexibility**: Future applications (headless simulation, different GUI, batch processing) can link against the libraries without using this main().

**Build Organization**: Clean separation between executable code and library code following CMake best practices.

### Single Responsibility

The executable has exactly one job: integrate the libraries and start the application. All actual functionality lives in:
- **msd-sim**: Simulation logic, physics, entities
- **msd-gui**: Rendering, windowing, event handling
- **msd-utils**: Utility functions like path resolution

This makes the executable trivial to understand and maintain.

### Database Path Management

**Current Implementation**: Hardcoded filename `"example_assets.db"` relative to working directory

**Path Resolution**: Converted to absolute path before passing to SDLApplication for reliability

**Rationale**: SDLApplication needs absolute path to reliably access database regardless of working directory changes

---

## Dependencies

### Internal Libraries

| Library | Purpose | Headers Used |
|---------|---------|--------------|
| msd_gui | SDL application and rendering | `msd-gui/src/SDLApp.hpp` |
| msd_sim | Simulation engine (transitive) | `msd-sim/src/Engine.hpp`, `msd-sim/src/Environment/Platform.hpp`, `msd-sim/src/Environment/WorldModel.hpp` |
| msd_utils | Utility functions | `msd-utils/src/PathUtils.hpp` |

### External Libraries

| Library | Purpose | Usage |
|---------|---------|-------|
| spdlog | Logging | Transitive dependency through msd_gui and msd_sim |

### Full Dependency Graph

```
msd_exe
  ├── msd_gui
  │   ├── SDL3 (windowing, events)
  │   ├── SDL3_image (image loading)
  │   ├── SDL3_mixer (audio)
  │   ├── SDL3_ttf (fonts)
  │   ├── spdlog (logging)
  │   └── msd_sim
  │       ├── Eigen3 (linear algebra)
  │       ├── SDL3 (rendering support)
  │       └── spdlog
  ├── msd_sim (see above)
  └── msd_utils
      └── C++17 filesystem
```

---

## Build & Configuration

### Build Requirements
- C++ Standard: C++20
- Compiler: GCC 11+, Clang 14+, or MSVC 2019+
- Build System: CMake 3.15+ with Conan 2.x package manager
- Dependencies (managed via Conan):
  - SDL3, SDL3_image, SDL3_mixer, SDL3_ttf (via msd-gui)
  - Eigen3 (via msd-sim)
  - spdlog (via msd-gui and msd-sim)
  - msd-sim, msd-gui, msd-utils (internal libraries)

### Building this Executable

This executable is built as part of the MSD-CPP project using Conan and CMake presets. See the [root CLAUDE.md](../../CLAUDE.md#build--configuration) for complete build instructions.

**Quick start:**
```bash
# Install dependencies with Conan
conan install . --build=missing -s build_type=Debug

# Build just this executable (Debug)
cmake --preset conan-debug
cmake --build --preset debug-exe-only

# Or for Release
conan install . --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build --preset release-exe-only
```

**Component-specific presets:**
- Debug: `debug-exe-only` — Builds `msd_exe` executable
- Release: `release-exe-only` — Builds `msd_exe` executable

**Build entire project:**
```bash
# After conan install
cmake --preset conan-debug
cmake --build --preset conan-debug    # Builds all components including msd_exe
```

### CMake Configuration

The executable is configured in [`msd/msd-exe/CMakeLists.txt`](CMakeLists.txt):

```cmake
set(MSD_EXE_NAME msd_exe)

add_executable(${MSD_EXE_NAME})
set_target_properties(${MSD_EXE_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${MSD_EXE_NAME}
    PRIVATE
      msd_sim
      msd_gui
      msd_utils
      spdlog::spdlog)

target_include_directories(${MSD_EXE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>)

target_sources(${MSD_EXE_NAME} PRIVATE src/main.cpp)
```

### Configuration Options
| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_TESTING` | `ON` | Build the testing tree (not applicable to exe) |
| `ENABLE_COVERAGE` | `OFF` | Enable code coverage (GCC/Clang only) |

---

## Runtime Requirements

### Required Files

**Asset Database**:
- `example_assets.db` — SQLite database with 3D geometry assets
- Must be in working directory or provide absolute path

**Shader Files** (from msd-gui):
- `Content/Shaders/Compiled/{BACKEND}/` — Compiled shaders for detected GPU backend
  - `SPIRV/` — Vulkan/OpenGL (Linux, some Windows)
  - `MSL/` — Metal (macOS)
  - `DXIL/` — DirectX 12 (Windows)

### Platform Support

| Platform | GPU Backend | Status |
|----------|-------------|--------|
| Linux | Vulkan, OpenGL | Supported |
| macOS | Metal | Supported |
| Windows | DirectX 12 | Supported |

### Working Directory

The executable expects to be run from a directory where:
1. `example_assets.db` is accessible (relative path)
2. `Content/` folder is adjacent to the executable (copied by CMake build)

---

## Usage

### Basic Usage

```bash
# Run from project root (recommended)
./build/Debug/debug/msd_exe

# Or for Release build
./build/Release/release/msd_exe

# From build directory
cd build/Debug/debug
./msd_exe
```

### Expected Behavior

1. **Startup**: Window opens with initialized GPU context
2. **Database Connection**: Loads asset database for 3D models
3. **Simulation Loop**: Runs simulation and renders visualization
4. **User Interaction**: Handles keyboard/mouse input events
5. **Shutdown**: Clean exit on window close or quit event

---

## Code Structure

```
msd/msd-exe/
├── CMakeLists.txt          # Build configuration
├── CLAUDE.md               # This documentation
└── src/
    └── main.cpp            # Application entry point (17 lines)
```

### File Inventory

| File | Lines | Purpose |
|------|-------|---------|
| `main.cpp` | ~17 | Entry point, path resolution, app initialization |
| `CMakeLists.txt` | ~28 | Build configuration |
| `CLAUDE.md` | This file | Architecture documentation |

---

## Coding Standards

This library follows the project-wide coding standards defined in the [root CLAUDE.md](../../CLAUDE.md#coding-standards).

### Applied Standards

**Initialization**:
- Brace initialization: `msd_gui::SDLApplication application{absolutePath.string()}`
- Value semantics for `std::string` and `std::filesystem::path`

**Naming**:
- Variables: `camelCase` (dbPath, absolutePath, application)
- Namespace qualified types: `msd_gui::SDLApplication`, `msd_utils::absolutePath`

**Function Return Values**:
- `main()` returns `int` exit code (0 on success)
- Delegates to library functions that follow return value best practices

See the [root CLAUDE.md](../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Future Enhancements

### Near-Term Extensions

**Command-Line Argument Parsing**:
```cpp
int main(int argc, char* argv[]) {
  std::string dbPath = parseArgs(argc, argv, "example_assets.db");
  // ...
}
```

**Configuration File Support**:
```cpp
auto config = loadConfig("msd_config.json");
msd_gui::SDLApplication application{config};
```

**Environment Variables**:
```cpp
const char* dbEnv = std::getenv("MSD_ASSET_DB");
std::string dbPath = dbEnv ? dbEnv : "example_assets.db";
```

### Long-Term Extensions

**Headless Mode**:
- Run simulation without GUI for batch processing
- Command-line flag: `--headless`
- Output results to file or database

**Scenario Loading**:
- Load specific simulation scenarios from database
- Command-line: `--scenario=orbital_rendezvous`

**Replay Mode**:
- Playback recorded simulation data
- Command-line: `--replay=mission_123.sim`

**Network Support**:
- Remote simulation control via network API
- Multi-instance coordination for distributed simulation

**Enhanced Error Handling**:
```cpp
try {
  auto absolutePath = msd_utils::absolutePath(dbPath);
  msd_gui::SDLApplication application{absolutePath.string()};
  application.runApp();
  return 0;
} catch (const std::exception& e) {
  spdlog::error("Fatal error: {}", e.what());
  return 1;
}
```

---

## Troubleshooting

### Common Issues

**Issue**: `Cannot find example_assets.db`
```
Error: Database file not found
Solution: Ensure database exists in working directory or provide absolute path
```

**Issue**: Shader loading failure
```
Error: Failed to load shaders for backend
Solution: Verify Content/Shaders/Compiled/{BACKEND}/ exists with compiled shaders
CMake should copy Content/ folder during build
```

**Issue**: Window creation failure
```
Error: SDL initialization failed
Solution:
  - Check graphics drivers are up to date
  - Verify SDL3 is properly installed
  - Check system supports required GPU backend (Vulkan/Metal/DirectX)
```

**Issue**: Black screen / no rendering
```
Possible causes:
  - GPU backend mismatch
  - Missing shader files
  - Database geometry loading failed
Check logs for detailed error messages
```

### Debug Tips

**Enable Verbose Logging**:
- msd-gui and msd-sim use spdlog for logging
- Check console output for initialization messages
- Look for ERROR and WARN level messages

**Verify Database**:
```bash
sqlite3 example_assets.db "SELECT name FROM objects;"
# Should list available 3D models
```

**Check Content Directory**:
```bash
ls -R Content/
# Should show Shaders/Compiled/{BACKEND}/ directories
```

---

## Performance Considerations

The executable itself has negligible performance impact:
- **Startup overhead**: < 1ms (path resolution, object construction)
- **Runtime overhead**: 0 (all execution in library code)
- **Memory overhead**: Minimal (single SDLApplication object)

All performance-critical code resides in:
- **msd-sim**: Physics simulation, coordinate transformations
- **msd-gui**: GPU rendering, event processing

---

## Testing

### Manual Testing Checklist

- [ ] Executable launches without errors
- [ ] Window opens with correct dimensions
- [ ] GPU backend initializes (check logs)
- [ ] Database connects successfully
- [ ] Shaders load for current platform
- [ ] Simulation runs at target framerate
- [ ] User input is responsive
- [ ] Clean shutdown on window close

### Integration Testing

Currently manual testing through execution. Future work:

**Automated Integration Tests**:
- Headless execution test (exit code verification)
- Database connection test
- Command-line argument parsing tests

**CI/CD Integration**:
- Build verification
- Runtime smoke tests on multiple platforms

---

## Best Practices

### When Modifying main.cpp

**Keep It Simple**: The executable should remain a thin integration layer. Don't add business logic here.

**Use Libraries**: If you need new functionality, add it to the appropriate library (msd-sim, msd-gui, msd-utils).

**Error Handling**: Consider adding try-catch for production deployments, but keep error handling minimal.

**Configuration**: For configuration options, extend SDLApplication rather than parsing arguments in main().

### Adding Features

**New Command-Line Args** → Add to new `AppConfig` class in msd-utils
**New Rendering** → Add to msd-gui
**New Physics** → Add to msd-sim
**New Utilities** → Add to msd-utils

Keep msd-exe minimal.

---

## Version History

### Current Version (2025-01-01)

**Implementation**:
- Basic executable integrating msd-sim and msd-gui
- Hardcoded database path: `example_assets.db`
- Path resolution via msd_utils
- Delegates to SDLApplication for all runtime behavior

**Status**: Initial working version, feature-complete for basic integration

---

## Diagrams Index

| Diagram | Description | Last Updated |
|---------|-------------|--------------|
| [`msd-exe.puml`](../../docs/msd/msd-exe/msd-exe.puml) | Executable integration and entry point | 2025-01-01 |

---

## Getting Help

### For AI Assistants
1. This executable is intentionally minimal — most functionality lives in linked libraries
2. For simulation questions, see [msd-sim/CLAUDE.md](../msd-sim/CLAUDE.md)
3. For GUI/rendering questions, see [msd-gui/CLAUDE.md](../msd-gui/CLAUDE.md)
4. For utilities, see [msd-utils/CLAUDE.md](../msd-utils/CLAUDE.md)
5. Refer to main project [CLAUDE.md](../../CLAUDE.md) for overall architecture

### For Developers
- **Running the app**: `./build/msd/msd-exe/msd_exe`
- **Modifying behavior**: Edit library code, not main.cpp
- **Adding features**: Extend libraries, not the executable
- **Debugging**: Enable spdlog output, check console for errors
