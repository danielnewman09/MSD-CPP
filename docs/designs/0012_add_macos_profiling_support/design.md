# Design: Add macOS Profiling Support

## Summary

This design adds profiling infrastructure for macOS using Xcode Instruments. The implementation provides build configuration for profiling-optimized binaries (debug symbols with Release optimizations) and a helper script for streamlined profiling workflows via the `xctrace` command-line tool. This enables developers to perform deep CPU profiling, memory analysis, and call graph visualization on Apple Silicon and Intel Macs without requiring GUI interaction for every profiling session.

## Architecture Changes

### PlantUML Diagram
See: [`./0012_add_macos_profiling_support.puml`](./0012_add_macos_profiling_support.puml)

### New Components

#### profile-instruments.sh Helper Script

- **Purpose**: Automate Xcode Instruments Time Profiler invocation via `xctrace` CLI
- **Location**: `scripts/profile-instruments.sh`
- **Key interfaces**:
  ```bash
  #!/bin/bash
  # Usage: ./profile-instruments.sh <executable> [template]
  #
  # Arguments:
  #   executable  - Path to executable to profile (required)
  #   template    - Instruments template (default: "Time Profiler")
  #                 Options: "Time Profiler", "Allocations"
  #
  # Example:
  #   ./profile-instruments.sh ./build/Release/msd_sim_bench
  #   ./profile-instruments.sh ./build/Release/msd_sim_bench "Allocations"

  main() {
      validate_xcode_tools
      validate_macos_version
      validate_executable "$1"

      local template="${2:-Time Profiler}"
      local trace_file="profile_$(date +%Y%m%d_%H%M%S).trace"

      echo "Profiling: $1"
      echo "Template: $template"
      echo "Output: $trace_file"

      xctrace record --template "$template" --output "$trace_file" --launch -- "$1"

      echo "Profiling complete. Open with: open $trace_file"
  }
  ```
- **Dependencies**:
  - `xctrace` command (part of Xcode Command Line Tools)
  - macOS 12.0+ (for full `xctrace` functionality)
- **Error handling**:
  - Validates Xcode Command Line Tools installed
  - Checks macOS version compatibility
  - Verifies executable exists and is executable
  - Exits with non-zero code on validation failure
- **Output**:
  - `.trace` files in current directory with timestamp naming
  - Can be opened with `open <file>.trace` or Instruments.app GUI

#### profiling-release CMake Preset

- **Purpose**: CMake configure preset for profiling-optimized builds
- **Location**: `CMakeUserPresets.json` (added to `configurePresets` array)
- **Key configuration**:
  ```json
  {
      "name": "profiling-release",
      "inherits": "conan-release",
      "displayName": "Release with Profiling",
      "description": "Release build with debug symbols for profiling (use: conan install . -o \"&:enable_profiling=True\")",
      "cacheVariables": {
          "ENABLE_PROFILING": "ON"
      }
  }
  ```
- **Dependencies**: Inherits from `conan-release` preset
- **Rationale**:
  - Builds on top of Release configuration
  - Adds debug symbols without sacrificing optimization
  - Follows existing pattern for specialized build configurations (e.g., `coverage-debug`)

### Modified Components

#### conanfile.py

- **Current location**: `conanfile.py` (project root)
- **Changes required**:
  1. Add `enable_profiling` option to `options` dictionary:
     ```python
     options = {
         "enable_coverage": [True, False],
         "warnings_as_errors": [True, False],
         "enable_clang_tidy": [True, False],
         "enable_benchmarks": [True, False],
         "enable_profiling": [True, False]  # NEW
     }
     default_options = {
         "enable_coverage": False,
         "warnings_as_errors": False,
         "enable_clang_tidy": False,
         "enable_benchmarks": False,
         "enable_profiling": False  # NEW - disabled by default
     }
     ```
  2. Pass `ENABLE_PROFILING` to CMake in `generate()` method:
     ```python
     def generate(self):
         deps = CMakeDeps(self)
         deps.generate()
         tc = CMakeToolchain(self)

         # Pass options to CMake
         tc.variables["ENABLE_COVERAGE"] = self.options.enable_coverage
         tc.variables["ENABLE_CLANG_TIDY"] = self.options.enable_clang_tidy
         tc.variables["ENABLE_BENCHMARKS"] = self.options.enable_benchmarks
         tc.variables["ENABLE_PROFILING"] = self.options.enable_profiling  # NEW
     ```
- **Backward compatibility**:
  - No impact when disabled (default)
  - No new dependencies required
  - No change to build output when disabled
- **Rationale**:
  - Follows existing pattern for optional build features
  - No additional Conan packages needed (uses compiler flags only)
  - Consistent with coverage, clang-tidy, benchmarks options

#### CMakeLists.txt (Root)

- **Current location**: `CMakeLists.txt` (project root)
- **Changes required**:
  1. Add `ENABLE_PROFILING` option after existing build options:
     ```cmake
     # Build options
     option(BUILD_TESTING "Build the testing tree" ON)
     option(ENABLE_COVERAGE "Enable code coverage" OFF)
     option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
     option(ENABLE_BENCHMARKS "Build performance benchmarks" OFF)
     option(ENABLE_PROFILING "Enable profiling support (macOS only)" OFF)  # NEW
     ```
  2. Add profiling configuration block after coverage configuration:
     ```cmake
     # Profiling configuration (macOS only)
     if(ENABLE_PROFILING)
       if(APPLE)
         # Add debug symbols with Release optimizations
         # -g: Generate debug symbols for Instruments
         # -O2: Release-level optimizations for realistic performance
         # Keep assertions enabled (no -DNDEBUG)
         add_compile_options(-g -O2)
         message(STATUS "Profiling support enabled (debug symbols + optimizations)")
       else()
         message(WARNING "Profiling support is only available on macOS (Xcode Instruments)")
       endif()
     endif()
     ```
- **Backward compatibility**: Default OFF preserves existing behavior
- **Platform handling**: Applies flags only on macOS (APPLE check)
- **Rationale**:
  - `-g -O2` provides realistic performance with symbol information
  - No `-DNDEBUG` keeps assertions enabled (useful for catching issues during profiling)
  - Warning on non-macOS platforms informs developers of platform limitation

#### CMakeUserPresets.json

- **Current location**: `CMakeUserPresets.json` (project root)
- **Changes required**:
  1. Add `profiling-release` preset to `configurePresets` array:
     ```json
     {
         "name": "profiling-release",
         "inherits": "conan-release",
         "displayName": "Release with Profiling",
         "description": "Release build with debug symbols for profiling (use: conan install . -o \"&:enable_profiling=True\")",
         "cacheVariables": {
             "ENABLE_PROFILING": "ON"
         }
     }
     ```
- **Backward compatibility**: New preset does not affect existing presets
- **Rationale**:
  - Provides convenient single-command profiling builds
  - Inherits from Release for realistic performance baseline
  - Follows naming pattern of `coverage-debug`

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| conanfile.py option | CMakeLists.txt option | Conan → CMake variable | ENABLE_PROFILING passed via CMakeToolchain |
| profiling-release preset | conan-release preset | CMake preset inheritance | Inherits Release configuration + adds profiling flag |
| profile-instruments.sh | msd_sim_bench | Shell script execution | Profiles benchmark executables |
| profile-instruments.sh | msd_sim_test | Shell script execution | Profiles test executables |
| profile-instruments.sh | msd_exe | Shell script execution | Profiles main application |
| xctrace | Instruments.app | Native macOS toolchain | xctrace generates .trace files opened by Instruments GUI |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| None | N/A | No impact | Profiling is an optional build mode, no test changes needed |

Profiling support does not modify test functionality. Tests continue to run normally with or without profiling enabled.

### New Tests Required

#### Unit Tests

No unit tests required. Profiling infrastructure is validated through manual execution.

#### Integration Tests

No integration tests required. Profiling is a developer tool for performance analysis, not production functionality.

### Validation Approach

| Component | Validation Method | Success Criteria |
|-----------|------------------|------------------|
| Profiling build | Manual build | Executables compile with -g -O2 flags |
| profile-instruments.sh | Manual execution | Script launches xctrace and generates .trace file |
| xctrace integration | Open .trace in Instruments | Call graph and timing data visible in Instruments GUI |
| Symbol resolution | Instruments call tree | Function names (not addresses) appear in call tree |

## Build Instructions

### Building with Profiling Support

```bash
# Install dependencies (same as Release build, profiling is compiler flags only)
conan install . --build=missing -s build_type=Release -o "&:enable_profiling=True"

# Configure with profiling preset
cmake --preset profiling-release

# Build all targets (includes benchmarks if enabled separately)
cmake --build --preset conan-release

# Build specific executable
cmake --build --preset conan-release --target msd_sim_bench
```

### Profiling Workflow

#### Option 1: Helper Script (Recommended)

```bash
# Profile a benchmark
./scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench

# Profile with Allocations template
./scripts/profile-instruments.sh ./build/Release/release/msd_sim_bench "Allocations"

# Open generated trace file
open profile_20260108_143000.trace
```

#### Option 2: Direct xctrace Usage

```bash
# Time Profiler (CPU profiling)
xctrace record --template "Time Profiler" \
    --output profile.trace \
    --launch -- ./build/Release/release/msd_sim_bench

# Allocations (Memory profiling)
xctrace record --template "Allocations" \
    --output allocations.trace \
    --launch -- ./build/Release/release/msd_sim_bench

# Open trace file in Instruments GUI
open profile.trace
```

#### Option 3: Instruments GUI

```bash
# Launch Instruments with executable
open -a Instruments ./build/Release/release/msd_sim_bench

# Then manually:
# 1. Choose "Time Profiler" or "Allocations" template
# 2. Click record button
# 3. Analyze results in GUI
```

## Open Questions

### Design Decisions (Human Input Needed)

1. **Profiling output directory organization**
   - Option A: Output .trace files to current directory (xctrace default behavior)
   - Option B: Create dedicated `profiling_results/` directory similar to `benchmark_results/`
   - Recommendation: Option A initially (simpler, follows xctrace convention). Option B if organization becomes necessary.

2. **Additional Instruments templates**
   - Option A: Support only Time Profiler template initially
   - Option B: Support both Time Profiler and Allocations templates
   - Recommendation: Option B (minimal additional complexity, high value for memory profiling)

3. **Profiling flags vs. separate build type**
   - Option A: Use ENABLE_PROFILING flag with Release build (current design)
   - Option B: Create new CMake build type "RelWithDebInfo"
   - Recommendation: Option A (RelWithDebInfo is a standard CMake build type, but ENABLE_PROFILING provides explicit opt-in control consistent with project patterns)

4. **Symbol stripping policy**
   - Option A: Never strip symbols from profiling builds
   - Option B: Strip symbols from install targets only, keep in build directory
   - Recommendation: Option A (profiling builds are local development only, not for distribution)

### Prototype Required

1. **Validate xctrace symbol resolution**
   - Uncertainty: Does `-g -O2` provide sufficient symbol information for Instruments?
   - Validation: Build msd_sim_bench with profiling flags, profile with xctrace, verify function names appear in call tree
   - Acceptance: Function names visible in Instruments call tree (not just memory addresses)

2. **Verify codesigning requirements**
   - Uncertainty: Does xctrace require executables to be codesigned?
   - Validation: Run xctrace on locally-built executable without explicit codesigning
   - Acceptance: xctrace profiles successfully without codesigning errors

### Requirements Clarification

1. **Profiling scope**: Should profiling be limited to Release builds, or also support Debug?
   - Assumption: Release builds only for realistic performance measurements. Debug profiling rarely useful.

2. **CI integration**: Should profiling infrastructure integrate with CI?
   - Assumption: No CI integration. Profiling is a local development tool for performance investigation.

3. **Baseline profiling results**: Should we track baseline profiling results like benchmarks?
   - Assumption: No baseline tracking initially. Profiling is for ad-hoc investigation, not regression detection. Use benchmarks for regression detection.

4. **Windows/Linux profiling support**: Should we add cross-platform profiling later?
   - Assumption: macOS-only initially. Linux can use `perf`, Windows can use Visual Studio Profiler, but these are separate tickets.

## Platform Considerations

### macOS Requirements

| Requirement | Minimum Version | Rationale |
|-------------|----------------|-----------|
| macOS | 12.0+ (Monterey) | Full xctrace CLI functionality |
| Xcode Command Line Tools | 13.0+ | Provides xctrace |
| Apple Clang | 13.0+ | Supports -g -O2 with good symbol generation |

### Xcode Instruments Templates

| Template | Use Case | Overhead | Data Collected |
|----------|----------|----------|----------------|
| Time Profiler | CPU profiling, hotspot identification | Low (~5%) | Call tree, CPU time per function |
| Allocations | Memory profiling, leak detection | Medium (~15%) | Allocation call stacks, heap growth |
| Leaks | Memory leak detection | Low (~5%) | Leak traces, reference cycles |
| System Trace | Low-level system calls | High (~30%) | syscalls, context switches, I/O |

Initial implementation focuses on Time Profiler (most common use case) with Allocations support.

### Symbol Resolution

Instruments requires debug symbols to map addresses to function names. The `-g` flag provides DWARF debug information without impacting runtime performance. This enables:

- **Call tree navigation**: Function names instead of addresses
- **Source line attribution**: Map CPU time to specific source lines
- **Inlining visibility**: See which functions were inlined by optimizer

## Future Enhancements

### Additional Platform Support

- **Linux**: Add `perf` script wrapper for CPU profiling
- **Windows**: Add Visual Studio Profiler integration

### Advanced Profiling Features

- **Flame graph generation**: Convert xctrace output to FlameGraph format
- **Diff profiling**: Compare two .trace files for regression analysis
- **Automated profiling**: Profile specific benchmarks as part of CI

### Integration with Benchmark Infrastructure

- **Benchmark + Profile**: Run benchmarks under profiler automatically
- **Hotspot detection**: Identify bottlenecks in slow benchmarks
- **Profile-guided optimization**: Use profiling data for compiler optimization

### Profiling-Specific Builds

- **Frame pointer preservation**: Add `-fno-omit-frame-pointer` for better stack traces
- **No inlining mode**: Add `-fno-inline` option for clearer call trees
- **Custom allocator tracing**: Instrument memory allocations for allocation profiling

## Implementation Guidance

### Phase 1: Build System Integration (Conan + CMake)
1. Modify `conanfile.py` to add `enable_profiling` option and pass to CMake
2. Modify root `CMakeLists.txt` to add `ENABLE_PROFILING` option with Apple-specific flags
3. Verify profiling build compiles with `-g -O2` on macOS

### Phase 2: CMake Preset Configuration
1. Add `profiling-release` preset to `CMakeUserPresets.json`
2. Verify preset inherits from `conan-release` correctly
3. Test preset builds executables with profiling flags

### Phase 3: Helper Script Development
1. Create `scripts/profile-instruments.sh` with validation functions
2. Implement xctrace invocation with template selection
3. Test script profiles msd_sim_bench successfully
4. Verify .trace file opens in Instruments GUI with function names

### Phase 4: Documentation
1. Update `CLAUDE.md` with profiling build instructions
2. Add profiling workflow examples (helper script, xctrace, GUI)
3. Document how to interpret Instruments results
4. Add troubleshooting section for common issues

## References

- Apple Instruments Documentation: https://developer.apple.com/documentation/xcode/instruments
- xctrace man page: `man xctrace` (macOS 12.0+)
- Apple Developer: Profiling Your App: https://developer.apple.com/documentation/xcode/improving-your-app-s-performance
- DWARF Debug Information: https://dwarfstd.org/

---

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-08
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Script name `profile-instruments.sh` follows existing `run_benchmarks.sh` pattern. CMake preset `profiling-release` follows `coverage-debug` pattern. Conan option `enable_profiling` consistent with `enable_benchmarks`, `enable_coverage`. |
| Namespace organization | N/A | No C++ code introduced - only build system and shell script. |
| File structure | ✓ | Script in `scripts/` matches existing organization. Design docs in `docs/designs/0012_add_macos_profiling_support/` follow convention. |
| Dependency direction | ✓ | Clean dependency flow: helper script → xctrace → executables. No circular dependencies. Profiling flags applied via CMake options. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | N/A | No C++ code introduced. |
| Smart pointer appropriateness | N/A | No C++ code introduced. |
| Value/reference semantics | N/A | No C++ code introduced. |
| Rule of 0/3/5 | N/A | No C++ code introduced. |
| Const correctness | N/A | No C++ code introduced. |
| Exception safety | N/A | No C++ code introduced. |
| Initialization | N/A | No C++ code introduced. |
| Return values | N/A | No C++ code introduced. |

**Note**: This design modifies build system configuration only. No C++ interfaces or classes are added, so C++ design quality criteria are not applicable.

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | No header changes. Compiler flags applied globally via `add_compile_options()`. |
| Template complexity | N/A | No templates involved. |
| Memory strategy | ✓ | No memory management changes. Debug symbols add disk space overhead but not runtime memory. |
| Thread safety | N/A | No threading concerns. |
| Build integration | ✓ | Follows existing pattern from benchmark and coverage options. Conan option → CMake variable → compiler flags. Clean integration. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Profiling is completely isolated. Optional build mode, no impact on production code. |
| Mockable dependencies | N/A | No dependencies to mock. Profiling is a developer tool. |
| Observable state | ✓ | Success observable via: (1) executables build with -g -O2, (2) xctrace generates .trace files, (3) Instruments GUI shows function names. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Symbol resolution may fail with -O2 optimizations | Technical | Low | Medium | Validate with prototype: build executable with -g -O2, profile with xctrace, verify function names appear in Instruments. | Yes |
| R2 | xctrace may require codesigning for local executables | Technical | Medium | Low | Prototype validates xctrace works on unsigned local builds. Codesigning typically only required for system-level profiling. | Yes |
| R3 | Profiling flags may conflict with other build modes | Technical | Low | Low | Design applies flags only when ENABLE_PROFILING=ON. Mutually exclusive with coverage (different goals). Benchmark mode compatible. | No |
| R4 | Script validation functions may have macOS version edge cases | Maintenance | Low | Low | Script includes validation for Xcode tools and macOS version. Edge cases handled with clear error messages. | No |

### Prototype Guidance

#### Prototype P1: Validate Symbol Resolution with -g -O2

**Risk addressed**: R1  
**Question to answer**: Do executables built with `-g -O2` provide sufficient debug symbols for Instruments to display function names and source lines?

**Success criteria**:
- Executable builds successfully with `-g -O2` flags
- xctrace generates .trace file without errors
- Instruments GUI displays function names (not addresses) in call tree
- Source line attribution visible for ConvexHull methods

**Prototype approach**:
```
Location: Local build (no prototype directory needed)
Type: Manual validation with existing msd_sim_bench

Steps:
1. Build msd_sim_bench with profiling flags:
   conan install . --build=missing -s build_type=Release -o "&:enable_profiling=True"
   cmake --preset profiling-release -DENABLE_PROFILING=ON
   cmake --build --preset conan-release --target msd_sim_bench

2. Profile with xctrace:
   xctrace record --template "Time Profiler" \
       --output test_profile.trace \
       --launch -- ./build/Release/release/msd_sim_bench

3. Open trace in Instruments:
   open test_profile.trace

4. Verify call tree shows:
   - Function names: "ConvexHull::signedDistance", "ConvexHull::contains"
   - Not: "0x10a4f3c20", "0x10a4f3d40" (raw addresses)
   - Source file annotations visible in Instruments source view
```

**Time box**: 30 minutes

**If prototype fails**:
- Investigate additional debug flags: `-fno-omit-frame-pointer`, `-gdwarf-4`
- Fallback: Use RelWithDebInfo CMake build type (standard -g -O2 -DNDEBUG)
- Ultimate fallback: Document GUI-only workflow, defer CLI scripting

#### Prototype P2: Verify xctrace Works Without Codesigning

**Risk addressed**: R2  
**Question to answer**: Can xctrace profile locally-built executables without explicit codesigning?

**Success criteria**:
- xctrace launches executable without codesigning errors
- Profiling data collected successfully
- No System Integrity Protection (SIP) warnings

**Prototype approach**:
```
Location: Local build (no prototype directory needed)
Type: Manual validation with msd_sim_bench

Steps:
1. Build msd_sim_bench (from P1)
2. Verify executable is not codesigned:
   codesign -dvv ./build/Release/release/msd_sim_bench
   (Expect: "code object is not signed" or similar)

3. Run xctrace without errors:
   xctrace record --template "Time Profiler" \
       --output unsigned_test.trace \
       --launch -- ./build/Release/release/msd_sim_bench

4. Verify no codesigning errors in output
5. Verify .trace file generated and can be opened
```

**Time box**: 15 minutes

**If prototype fails**:
- Document codesigning requirement in CLAUDE.md
- Add codesigning step to build instructions (ad-hoc signature acceptable)
- Provide fallback: `codesign -s - <executable>` (ad-hoc signature)

### Summary

This design is **APPROVED** with two quick validation prototypes. The design demonstrates excellent adherence to project patterns and provides comprehensive profiling infrastructure for macOS development:

**Strengths:**
1. **Follows established patterns**: Mirrors benchmark infrastructure (Conan option, CMake option, helper script)
2. **Platform-appropriate tooling**: Uses native Xcode Instruments rather than third-party profilers
3. **Zero impact when disabled**: Default OFF, no dependencies added, no behavior changes
4. **Practical workflows**: Helper script reduces friction, supports both CLI and GUI workflows
5. **Comprehensive documentation**: Clear build instructions, multiple profiling options, platform requirements
6. **Clean build integration**: No conflicts with existing build modes (coverage, benchmarks, tests)

**Design quality indicators:**
- No architectural violations or circular dependencies
- Clear separation: profiling is a build mode, not a code change
- Appropriate use of compiler flags: `-g -O2` is industry standard for profiling
- Pragmatic helper script: validates inputs, generates timestamped outputs
- Platform-aware: macOS-only with clear warnings on other platforms

**Prototypes required**: Two quick validations (30 min + 15 min = 45 min total) to confirm:
1. Symbol resolution works with optimization flags
2. xctrace works without codesigning (expected to pass)

Both prototypes use existing infrastructure and have clear fallback strategies if issues arise. The design is fundamentally sound and ready for prototype validation.

**Next steps:**
1. Human reviews this approval
2. Execute P1 and P2 prototypes (45 minutes total)
3. Document prototype results
4. Proceed to implementation phase if prototypes pass
5. Revise design if prototypes reveal issues (unlikely)
