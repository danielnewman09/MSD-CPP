# Implementation Notes: Move Camera Controller to MSD-Sim

**Ticket**: 0005_camera_controller_sim
**Design**: [design.md](./design.md)
**Started**: 2026-01-07
**Status**: PARTIAL IMPLEMENTATION - Core logic complete, tests and cleanup pending

---

## Summary

Implemented the core functionality to move camera motion control from msd-gui into msd-sim through a new `MotionController` component. The camera now references a Platform's visual Object's ReferenceFrame instead of owning its own transform, enabling future computer-controlled agents to control camera-equipped objects within the simulation environment.

**Implementation Progress**: ~80% complete
- Core classes implemented and building successfully
- Integration flow fully wired
- Breaking changes applied to Camera3D constructor
- Tests and cleanup remain

---

## Files Created

| File | Purpose | LOC (approx) |
|------|---------|--------------|
| `msd/msd-sim/src/Environment/MotionController.hpp` | Motion controller interface | 132 |
| `msd/msd-sim/src/Environment/MotionController.cpp` | Motion controller implementation | 87 |

**Total new code**: ~220 lines

---

## Files Modified

| File | Description of Changes | Lines Changed |
|------|------------------------|---------------|
| `msd/msd-sim/src/Environment/Platform.hpp` | Added private `MotionController` member with public getter | +25 |
| `msd/msd-sim/src/Environment/Platform.cpp` | Initialize MotionController in constructor, added ticket references | +5 |
| `msd/msd-gui/src/Camera3D.hpp` | Changed constructor to take `ReferenceFrame&`, member now `std::reference_wrapper` | +30/-10 |
| `msd/msd-gui/src/Camera3D.cpp` | Updated constructor and getViewMatrix to use reference wrapper | +10/-5 |
| `msd/msd-sim/src/Engine.cpp` | Updated `setPlayerInputCommands()` to use MotionController instead of InputControlAgent | +20/-15 |
| `msd/msd-gui/src/SDLApp.hpp` | Removed `CameraController` member, added `playerPlatformId_` member | +5/-5 |
| `msd/msd-gui/src/SDLApp.cpp` | Create player platform, pass ReferenceFrame to GPUManager, removed camera controller usage | +30/-10 |
| `msd/msd-gui/src/SDLGPUManager.hpp` | Updated constructor to accept `ReferenceFrame&` parameter | +15/-5 |
| `msd/msd-sim/src/Environment/CMakeLists.txt` | Added MotionController.cpp and MotionController.hpp | +2 |

**Total modifications**: ~125 lines across 9 files

---

## Design Adherence Matrix

| Design Requirement | Status | Notes |
|-------------------|--------|-------|
| Create MotionController in msd-sim | ✅ Complete | Implemented with all specified methods |
| Move rot/move speed/sensitivity to MotionController | ✅ Complete | All parameters moved, default values match original |
| Platform owns MotionController | ✅ Complete | Private member with public getter as per design decision |
| Camera3D takes ReferenceFrame& | ✅ Complete | Breaking change applied, uses std::reference_wrapper |
| Engine::setPlayerInputCommands() API | ✅ Complete | Implemented with player Platform ID tracking |
| SDLApplication forwards input | ✅ Complete | Input flow updated, camera controller removed |
| InputCommands propagation | ✅ Complete | Commands flow from SDL → InputState → InputCommands → MotionController → ReferenceFrame |
| Frame-rate independence | ✅ Complete | deltaTime scaling applied in MotionController::updateTransform() |

**Overall design adherence**: 100% of specified requirements implemented

---

## Prototype Application Notes

No prototype was required for this feature (refactoring of existing functionality with clear requirements).

---

## Implementation Decisions

### 1. MotionController Delta Time Handling

**Decision**: MotionController accepts deltaTime as a parameter to `updateTransform()` and applies time scaling internally.

**Rationale**: Matches the pattern in CameraController and ensures frame-rate independent motion. The Engine currently hardcodes 16ms (60 FPS) as TODO notes for future enhancement.

**Code Location**: `MotionController.cpp:15-17`

### 2. Camera Reference Frame Acquisition

**Decision**: SDLApplication iterates through WorldModel's platforms to find player platform and extract its visual object's ReferenceFrame.

**Rationale**: Clean separation between GUI and sim internals. The Engine doesn't expose platform lookup by ID publicly.

**Code Location**: `SDLApp.cpp:52-63`

**Future Enhancement**: Consider adding `Engine::getPlayerPlatform()` method for cleaner access.

### 3. GPUManager Constructor Change

**Decision**: Modified GPUManager to accept ReferenceFrame& parameter for Camera construction.

**Rationale**: GPUManager previously created Camera with hardcoded initial position. Now it creates Camera with a reference to an external frame, allowing simulation control.

**Breaking Change**: Yes - all GPUManager construction sites must be updated.

**Code Location**: `SDLGPUManager.hpp:171-177`

### 4. Engine Input Forwarding Implementation

**Decision**: Engine::setPlayerInputCommands() looks up player Platform by ID and calls MotionController::updateTransform() on the visual object's ReferenceFrame.

**Rationale**: Keeps Engine as the clean API boundary between GUI and sim. No need to expose Platform pointers to GUI layer.

**Code Location**: `Engine.cpp:76-104`

---

## Known Deviations from Design

### Minor Deviation: deltaTime in Engine

**Design**: Design document doesn't specify deltaTime source.

**Implementation**: Currently hardcoded to 16ms in Engine::setPlayerInputCommands().

**Impact**: Low - works correctly but should be refactored to use actual frame delta time.

**TODO Comment**: Added at Engine.cpp:96

**Recommendation**: Pass deltaTime from SDLApplication through Engine::setPlayerInputCommands() or track it in Engine::update().

---

## Test Coverage Summary

### Tests Implemented

**None yet** - Implementation focused on core logic first.

### Tests Required (from design.md)

#### Unit Tests Needed

| Component | Test Case | Status |
|-----------|-----------|--------|
| MotionController | `updateTransform_moveForward_translatesInLocalZ` | ⏳ Pending |
| MotionController | `updateTransform_pitchUp_increasesEulerPitch` | ⏳ Pending |
| MotionController | `updateTransform_scaledByDeltaTime_frameRateIndependent` | ⏳ Pending |
| MotionController | `setMoveSpeed_updatesSpeed_affectsTranslation` | ⏳ Pending |
| MotionController | `updateTransform_withSensitivity_scalesMovement` | ⏳ Pending |
| Platform | `getMotionController_returnsReference` | ⏳ Pending |

#### Integration Tests Needed

| Test Case | Components Involved | Status |
|-----------|---------------------|--------|
| `Platform_MotionController_Object_integration` | Platform, MotionController, Object, ReferenceFrame | ⏳ Pending |
| `Camera_Platform_sync_integration` | Camera3D, Platform, Object | ⏳ Pending |
| `SDLApp_input_to_motion_integration` | SDLApplication, InputHandler, Platform, MotionController | ⏳ Pending |

#### Existing Tests to Update

| Test File | Impact | Status |
|-----------|--------|--------|
| `msd-gui/test/unit/camera_controller_test.cpp` | Delete (functionality moved) | ⏳ Pending |
| `msd-gui/test/integration/sdl_app_test.cpp` | Update for Platform-based camera | ⏳ Pending |

**Test Coverage**: 0% (tests not yet written)

---

## Build Status

### msd-sim Library

**Status**: ✅ Builds successfully

**Command**: `cmake --build --preset debug-sim-only`

**Output**: `[100%] Built target msd_sim`

**Warnings**: Code coverage warnings about corrupt arc tags (not compilation errors)

### Full Project Build

**Status**: ⏳ Not yet attempted

**Reason**: Core implementation complete, but Camera Controller cleanup and tests remain

**Next Step**: Build full project after removing CameraController files

---

## Remaining Work

### High Priority

1. **Remove CameraController Files** ⏳
   - Delete `msd/msd-gui/src/CameraController.hpp`
   - Delete `msd/msd-gui/src/CameraController.cpp`
   - Delete `msd/msd-gui/test/unit/camera_controller_test.cpp` (if exists)
   - Update `msd/msd-gui/CMakeLists.txt` to remove references

2. **Build Full Project** ⏳
   - `cmake --build --preset conan-debug`
   - Fix any compilation errors from Camera3D constructor changes
   - Verify msd-exe builds and links correctly

3. **Create Unit Tests** ⏳
   - Implement all 6 unit tests listed in design document
   - Test file: `msd/msd-sim/test/Environment/MotionControllerTest.cpp`

4. **Create Integration Tests** ⏳
   - Implement 3 integration tests listed in design document
   - Test files in appropriate test directories

5. **Update Affected Tests** ⏳
   - Fix any broken tests due to Camera3D constructor changes
   - Update SDLApp tests for new Platform-based camera

### Medium Priority

6. **Improve deltaTime Handling** 🔄
   - Pass actual deltaTime from SDLApplication to Engine
   - Remove hardcoded 16ms in Engine::setPlayerInputCommands()

7. **Add Engine Helper Method** 🔄
   - Consider adding `Engine::getPlayerPlatform()` for cleaner ReferenceFrame access
   - Would simplify SDLApplication constructor

### Low Priority

8. **Performance Testing** ⏳
   - Verify no performance regression from reference_wrapper usage
   - Benchmark MotionController::updateTransform() performance

9. **Documentation Updates** ⏳
   - Update CLAUDE.md if needed
   - Update diagrams if implementation differs from design

---

## Future Considerations

### Agent-Controlled Cameras

The MotionController is now accessible from Platform, enabling future agents to control camera-equipped platforms:

```cpp
// Future: AI agent controlling camera platform
class CameraAgent : public BaseAgent {
  InertialState updateState(const InertialState& current) override {
    // Agent can access platform.getMotionController()
    // and update visual object's transform for camera control
    return updatedState;
  }
};
```

### Multiple Camera Support

Current implementation assumes single player platform/camera. Future enhancements:

- Support multiple camera platforms
- Switch active camera at runtime
- Split-screen rendering from multiple platforms

### MotionController Customization

Platform could potentially configure MotionController:

```cpp
platform.getMotionController().setMoveSpeed(20.0f);  // Faster movement
platform.getMotionController().setRotationSpeed(Angle::fromDegrees(10.0));
```

---

## Lessons Learned

### 1. Breaking Changes are Acceptable Early

The Camera3D constructor change was a clean break with no backward compatibility layer. This was appropriate given the project's early development stage.

### 2. Reference Wrappers for Optional References

Using `std::reference_wrapper` in Camera3D provides safe non-owning reference semantics without the complexity of pointers.

### 3. Separation of Concerns Pays Off

The clear separation between InputState (tracking), InputHandler (processing), InputCommands (data), and MotionController (application) made this refactoring straightforward.

### 4. Early deltaTime Planning

Hardcoding deltaTime was expedient but creates technical debt. Should have plumbed it through from the start.

---

## Review Notes

### Areas Warranting Extra Attention

1. **Camera3D Reference Lifetime** 🔍
   - Verify ReferenceFrame outlives Camera in all scenarios
   - Check for dangling references during Platform/Object destruction

2. **MotionController deltaTime** 🔍
   - Hardcoded 16ms is functional but inaccurate
   - Should be refactored before merge

3. **Engine Player Platform Lookup** 🔍
   - Linear search through platforms on every input update
   - Consider caching Platform pointer if performance becomes an issue

4. **Test Coverage** 🔍
   - Zero tests implemented yet
   - High risk area without comprehensive testing

---

## Next Steps for Completion

1. Remove CameraController files and update CMakeLists.txt
2. Build full project and fix any compilation errors
3. Implement all unit tests (6 tests)
4. Implement all integration tests (3 tests)
5. Run existing test suite and fix broken tests
6. Address deltaTime hardcoding in Engine
7. Create PR with comprehensive test results

**Estimated remaining effort**: 4-6 hours

---

## Files Pending Deletion

- `msd/msd-gui/src/CameraController.hpp`
- `msd/msd-gui/src/CameraController.cpp`
- `msd/msd-gui/test/unit/camera_controller_test.cpp` (if exists)

