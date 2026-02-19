# Ticket 0074: Fix and Complete Devcontainer Setup

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Type**: Infrastructure
**Priority**: Medium
**Created**: 2026-02-18
**Generate Tutorial**: No

---

## Summary

The existing `.devcontainer/` configuration has several issues that prevent it from working end-to-end for full development (build, test, and GUI). This ticket tracks fixes to make the devcontainer a one-click setup for new developers.

---

## Problem

The current devcontainer has the following issues:

1. **Conan profile mismatch**: `conan-profile` specifies `compiler.version=11` but the Dockerfile installs `gcc-14`
2. **Local Conan recipes not registered**: `conan install .` fails because local recipes (sdl, qhull, cpp_sqlite, etc.) in `conan/` are not published to the container's Conan cache
3. **Python environment not set up**: `python/setup.sh` is never run automatically
4. **Missing apt packages**: `doxygen` and `lcov` are not installed, so documentation and coverage targets are unavailable
5. **Mount ordering bug**: The `~/.conan2` host bind mount overwrites the conan-profile mount; also causes cross-platform cache conflicts (macOS arm64 Conan binaries vs Linux arm64 have different ABIs)
6. **No X11 forwarding**: The GUI app (`msd_exe`) cannot open a window in the container
7. **No automated first-build**: Developer must manually run multiple steps after container creation

---

## Acceptance Criteria

- [ ] AC1: Container builds successfully from `.devcontainer/Dockerfile`
- [ ] AC2: `g++-14 --version`, `conan --version`, `cmake --version`, `python3 --version` all work in container
- [ ] AC3: `conan install . --build=missing -s build_type=Debug` succeeds without manual recipe registration
- [ ] AC4: `cmake --preset conan-debug && cmake --build --preset debug-sim-only` compiles successfully
- [ ] AC5: `./build/Debug/debug/msd_sim_test` passes all tests
- [ ] AC6: `doxygen` and `lcov` are available in the container
- [ ] AC7: GUI app opens a window when X11 forwarding is configured on host (XQuartz on macOS)
- [ ] AC8: Python venv is set up and MCP servers can be started

---

## Proposed Changes

### `.devcontainer/conan-profile`
- Change `compiler.version=11` → `compiler.version=14` to match installed gcc

### `.devcontainer/Dockerfile`
- Add `doxygen lcov` to the apt-get install line

### `.devcontainer/devcontainer.json`
- **Remove** both `mounts` entries (conan-profile and `~/.conan2` host bind mount). Let the container manage its own Conan cache to avoid macOS/Linux binary conflicts.
- **Add** X11 forwarding: mount `/tmp/.X11-unix` and set `DISPLAY` env var
- **Add** `postCreateCommand` → `bash .devcontainer/setup.sh` (runs once after container creation)
- **Keep** `postStartCommand` → `bash .devcontainer/install-npm.sh` (idempotent, runs each start)
- **Fix** duplicate `cmake.configureOnOpen` settings (remove non-nested one)
- **Remove** deprecated `terminal.integrated.shell.linux` key

### `.devcontainer/setup.sh` (new file)
Post-create script that automates first-time setup:
1. Register all local Conan recipes (`conan create conan/sqlite3 ...`, `conan create conan/sdl ...`, etc.)
2. Run `conan install . --build=missing -s build_type=Debug`
3. Run `python/setup.sh` for Python environment
4. Run `cmake --preset conan-debug` to configure the build

---

## Testing

1. Rebuild devcontainer from VS Code command palette ("Dev Containers: Rebuild Container")
2. Verify tool versions in container terminal (AC2)
3. Build and run sim tests (AC4, AC5)
4. Verify `doxygen --version` and `lcov --version` work (AC6)
5. Test GUI with XQuartz on macOS host (AC7)
6. Verify `python/setup.sh` output and MCP server startup (AC8)

---

## Files

| File | Action |
|------|--------|
| `.devcontainer/conan-profile` | Edit compiler version |
| `.devcontainer/Dockerfile` | Add doxygen, lcov to apt |
| `.devcontainer/devcontainer.json` | Fix mounts, add X11, add postCreateCommand, clean up settings |
| `.devcontainer/setup.sh` | **New** — automated first-build script |
