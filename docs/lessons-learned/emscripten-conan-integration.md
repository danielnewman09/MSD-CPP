# Emscripten + Conan 2.x Integration: Lessons Learned

This document captures key findings from integrating Emscripten/WebAssembly builds with Conan 2.x package manager.

## Overview

Building a C++ project for WebAssembly with WebGPU support using Conan 2.x required solving several non-obvious issues around cross-compilation, dependency management, and shared memory requirements.

## Key Findings

### 1. Cross-Compilation Profile Setup

When cross-compiling for Emscripten, use dual profiles:
- `--profile:host` for the target platform (Emscripten)
- `--profile:build` for the build machine (typically `default`)

```bash
# Correct pattern for cross-compilation
conan create conan/mypackage --profile:host profiles/emscripten.profile --profile:build default

# For installing project dependencies
conan install . --profile:host profiles/emscripten.profile --profile:build default -of build/Emscripten
```

**Why:** Conan needs to know both the target platform settings (Emscripten, wasm) and the build machine settings (for tool requirements like cmake).

### 2. CMake find_package() with Conan-generated Configs

**Problem:** CMake's built-in `FindXXX.cmake` modules may conflict with Conan-generated config files.

**Solution:** Use `CONFIG` mode to force CMake to use Conan-generated configs:

```cmake
# WRONG - may use CMake's built-in FindSQLite3.cmake
find_package(SQLite3 REQUIRED)

# CORRECT - uses Conan-generated SQLite3Config.cmake
find_package(SQLite3 REQUIRED CONFIG)
```

**Why:** When Emscripten's CMake toolchain is active, it can interfere with module search paths. The `CONFIG` mode explicitly requests config-file-based discovery.

### 3. Transitive Dependency Header Propagation

**Problem:** Headers from transitive dependencies may not be available when using CMakeDeps.

**Symptom:** Build error like `fatal error: 'sqlite3.h' file not found` even though the dependency is correctly linked.

**Diagnosis:** Check the `conan graph info` output for `headers: False` on transitive dependencies.

**Solution:** Add the transitive dependency as a direct requirement:

```python
def requirements(self):
    self.requires("cpp_sqlite/0.1.0")
    # Explicitly require sqlite3 to ensure headers are available
    # (cpp_sqlite depends on it, but transitive dependency doesn't propagate headers)
    self.requires("sqlite3/3.47.0-local")
```

**Why:** CMakeDeps optimizes header propagation and may not include headers for pure transitive dependencies.

### 4. pthread/Atomics for Shared Memory (WebGPU Requirement)

**Problem:** WebGPU via emdawnwebgpu requires shared memory, which requires all object files to be compiled with `-pthread`.

**Symptom:**
```
wasm-ld: error: --shared-memory is disallowed by libqhull_r.c.o because it was
not compiled with 'atomics' or 'bulk-memory' features.
```

**Solution:** There are multiple approaches, each needed for different contexts:

#### For local Conan recipes (qhull, sqlite3, etc.):
```python
def generate(self):
    tc = CMakeToolchain(self)
    if self.settings.os == "Emscripten":
        # Inject flags via CMAKE_C_FLAGS_INIT (processed before CMakeLists.txt)
        pthread_flags = "-pthread -matomics -mbulk-memory"
        tc.cache_variables["CMAKE_C_FLAGS_INIT"] = pthread_flags
        tc.cache_variables["CMAKE_CXX_FLAGS_INIT"] = pthread_flags
    tc.generate()
```

#### For Conan Center packages (via profile):
```ini
[buildenv]
# Set environment variables picked up by emcc/em++
CFLAGS=-pthread
CXXFLAGS=-pthread
```

**Why:** Different approaches are needed because:
- `tc.extra_cflags`/`tools.build:cflags` don't work reliably with Emscripten's CMake toolchain
- `CMAKE_C_FLAGS_INIT` is processed early, before the toolchain file modifies flags
- `[buildenv]` sets environment variables that Emscripten's compiler wrapper reads directly

### 5. fmt/spdlog consteval Compatibility

**Problem:** Emscripten 3.1.68+ has issues with `consteval` used in fmt/spdlog.

**Symptom:** Compilation errors in format string validation at compile time.

**Solution:** Disable consteval format checking:
```python
tc.cache_variables["CMAKE_CXX_FLAGS"] = "-DFMT_USE_CONSTEVAL=0 -DFMT_CONSTEVAL="
```

**Reference:** https://github.com/emscripten-core/emscripten/issues/22795

### 6. CMake Preset Naming for Cross-Compilation

**Problem:** Default Conan preset names conflict between native and Emscripten builds.

**Solution:** Use a custom prefix for Emscripten presets:
```python
if self.settings.os == "Emscripten":
    tc.presets_prefix = "conan-emscripten"
```

This generates `conan-emscripten-release` instead of `conan-release`, avoiding conflicts with native build presets.

### 7. CMAKE_PREFIX_PATH with External Toolchains

**Problem:** When using Emscripten's CMake toolchain file, Conan's generated `CMAKE_PREFIX_PATH` may not be set correctly.

**Solution:** Explicitly set `CMAKE_PREFIX_PATH` and individual `*_DIR` variables:
```python
if self.settings.os == "Emscripten":
    generators_folder = os.path.join(
        self.recipe_folder, "build", "Emscripten", "build",
        str(self.settings.build_type), "generators"
    )
    tc.cache_variables["CMAKE_PREFIX_PATH"] = generators_folder
    tc.cache_variables["cpp_sqlite_DIR"] = generators_folder
    tc.cache_variables["spdlog_DIR"] = generators_folder
    # ... etc
```

### 8. Boost Configuration for Emscripten

**Problem:** Some Boost components don't compile or work with Emscripten.

**Solution:** Disable problematic components:
```python
if self.settings.os == "Emscripten":
    self.options["boost/*"].without_stacktrace = True
    self.options["boost/*"].without_context = True
    self.options["boost/*"].without_coroutine = True
    self.options["boost/*"].without_iostreams = True
```

## Debugging Tips

1. **Check package info:** Use `conan graph info . --profile:host profiles/emscripten.profile` to see how dependencies are resolved.

2. **Inspect generated files:** Look at `build/*/generators/` for:
   - `conan_toolchain.cmake` - Check what flags are actually set
   - `*-config.cmake` - Check target definitions and include paths

3. **Verify package contents:** After `conan create`, check the package folder structure to ensure headers and libraries are in expected locations.

4. **Force rebuild:** When modifying recipes, ensure packages are rebuilt:
   ```bash
   conan remove "packagename/*" -c
   conan create conan/packagename --profile:host profiles/emscripten.profile --profile:build default
   ```

## Complete Emscripten Profile Example

```ini
# profiles/emscripten.profile
[settings]
os=Emscripten
arch=wasm
compiler=emcc
compiler.version=4.0.22
compiler.libcxx=libc++
compiler.cppstd=20
build_type=Release

[tool_requires]
emsdk/4.0.22

[conf]
tools.cmake.cmaketoolchain:generator=Unix Makefiles
tools.build:exelinkflags=["-sALLOW_MEMORY_GROWTH=1", "-sMAXIMUM_MEMORY=536870912", "-sINITIAL_MEMORY=134217728", "-pthread"]
tools.build:sharedlinkflags=["-sALLOW_MEMORY_GROWTH=1", "-sMAXIMUM_MEMORY=536870912", "-sINITIAL_MEMORY=134217728", "-pthread"]
tools.build:cflags=["-pthread"]
tools.build:cxxflags=["-pthread"]

[buildenv]
CFLAGS=-pthread
CXXFLAGS=-pthread
```

## Build Commands Summary

```bash
# 1. Create local dependencies (one-time or after recipe changes)
conan create conan/emsdk
conan create conan/sqlite3 --profile:host profiles/emscripten.profile --profile:build default
conan create conan/cpp_sqlite --profile:host profiles/emscripten.profile --profile:build default
conan create conan/qhull --profile:host profiles/emscripten.profile --profile:build default

# 2. Install project dependencies
conan install . --profile:host profiles/emscripten.profile --profile:build default -of build/Emscripten --build=missing

# 3. Configure and build
cmake --preset emscripten-release
cmake --build --preset emscripten-release

# 4. Serve locally for testing
cd build/Emscripten/build/Release/msd/msd-exe
python3 -m http.server 8080
# Open http://localhost:8080/msd_exe.html in a WebGPU-capable browser
```
