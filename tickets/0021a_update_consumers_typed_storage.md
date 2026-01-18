# Feature Ticket: Update Consumers to Use Typed Storage APIs

## Status
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-11
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim (Engine), msd-gui (SDLApp)
- **Depends On**: 0021_worldmodel_asset_refactor (Phase 1 must be complete)
- **Blocks**: 0022_remove_deprecated_worldmodel_components (Phase 3)

---

## Summary

Update all consumers of deprecated `Object` APIs to use the new typed storage APIs introduced in ticket 0021. This is Phase 2 of the three-phase migration strategy:

- **Phase 1** (0021 - COMPLETE): Add typed storage (`AssetEnvironment`, `AssetInertial`), deprecate Object class
- **Phase 2** (THIS TICKET): Update all consumers to use new typed storage APIs
- **Phase 3** (0022 - BLOCKED): Remove deprecated code

## Motivation

Ticket 0022 cannot proceed until all consumers have been migrated away from deprecated APIs. The following deprecated APIs are still actively used in production code:

| File | Line | Deprecated Usage |
|------|------|------------------|
| `Engine.cpp` | 44-45 | `worldModel_.spawnObject(Object::createInertial(...))` |
| `Engine.cpp` | 135-136 | `Object::createGraphical(...)` and `worldModel_.spawnObject(...)` |
| `Engine.cpp` | 144 | `worldModel_.getObject(objIndex)` |
| `SDLApp.cpp` | 302 | `Object::createGraphical(*asset, randomFrame, r, g, b)` |

These must be updated before the deprecated code can be safely removed.

## Requirements

### Functional Requirements

1. **Engine::spawnInertialObject()** shall delegate to `Engine::spawnInertialAsset()` internally
   - Current: Uses `Object::createInertial()` and `worldModel_.spawnObject()`
   - Target: Call `spawnInertialAsset()` with same parameters

2. **Engine::spawnPlayerPlatform()** shall use new typed storage APIs
   - Current: Uses `Object::createGraphical()`, `worldModel_.spawnObject()`, `worldModel_.getObject()`
   - Target: Use `AssetInertial` or design alternative for Platform visual binding
   - Note: Platform currently stores a reference to the visual Object; this needs redesign

3. **SDLApp::spawnObject()** shall not use deprecated `Object` class
   - Current: Uses `Object::createGraphical()` for mock rendering objects
   - Target: Either use `AssetEnvironment` or create a rendering-only solution
   - Note: `mockObjects_` is used for GUI-only mock rendering, not physics simulation

4. All existing tests must pass without using deprecated APIs
5. All existing simulation behavior must be preserved

### Non-Functional Requirements
- **Build**: Project must compile without deprecation warnings after migration
- **Tests**: All tests must pass
- **No Regressions**: Simulation and rendering behavior must be unchanged

## Constraints
- Must not change the behavior of existing functionality
- Must not modify the deprecated APIs themselves (leave for ticket 0022)
- Should complete before ticket 0022 can proceed

## Acceptance Criteria
- [ ] `Engine::spawnInertialObject()` no longer uses `Object::createInertial()` or `spawnObject()`
- [ ] `Engine::spawnPlayerPlatform()` no longer uses deprecated Object APIs
- [ ] `SDLApp::spawnObject()` no longer uses `Object::createGraphical()`
- [ ] All tests pass
- [ ] No deprecation warnings in build output
- [ ] Simulation behavior unchanged
- [ ] Rendering behavior unchanged

---

## Analysis of Current Usage

### 1. Engine::spawnInertialObject() — Simple Delegation

**Current Implementation** (Engine.cpp:28-46):
```cpp
void Engine::spawnInertialObject(const std::string& assetName,
                                  const Coordinate& position,
                                  const EulerAngles& orientation)
{
  ReferenceFrame objectFrame{position, orientation};
  const auto& asset = assetRegistry_.getAsset(assetName);

  if (!asset.has_value()) {
    throw std::runtime_error("Could not spawn object with name: " + assetName +
                             ". It was not found in the asset registry");
  }

  worldModel_.spawnObject(
    Object::createInertial(asset->get(), objectFrame, 1.0));
}
```

**Migration Path**:
Simply delegate to `spawnInertialAsset()` which already exists and uses the new typed storage:
```cpp
void Engine::spawnInertialObject(const std::string& assetName,
                                  const Coordinate& position,
                                  const EulerAngles& orientation)
{
  spawnInertialAsset(assetName, position, orientation, 1.0);
}
```

### 2. Engine::spawnPlayerPlatform() — Requires Design Decision

**Current Implementation** (Engine.cpp:120-150):
```cpp
uint32_t Engine::spawnPlayerPlatform(const std::string& assetName,
                                     const Coordinate& position,
                                     const EulerAngles& orientation)
{
  ReferenceFrame objectFrame{position, orientation};
  const auto& asset = assetRegistry_.getAsset(assetName);

  if (!asset.has_value()) {
    throw std::runtime_error("Could not spawn player platform with asset: " +
                             assetName + ". Asset not found in registry");
  }

  auto object = Object::createGraphical(asset->get(), objectFrame);
  size_t objIndex = worldModel_.spawnObject(std::move(object));

  uint32_t platformId = worldModel_.getNextPlatformId();
  Platform platform{platformId};
  platform.setAgent(std::make_unique<InputControlAgent>());
  platform.getState().position = position;
  platform.getState().angularPosition = orientation;
  platform.setVisualObject(worldModel_.getObject(objIndex));

  worldModel_.addPlatform(std::move(platform));
  playerPlatformId_ = platformId;

  return platformId;
}
```

**Issue**: Platform currently stores a reference to an Object via `setVisualObject()`.

**Migration Options**:

**Option A**: Platform stores reference to `AssetInertial` instead
- Platform gains physics capability (may be overkill for pure visual binding)
- Requires modifying Platform class

**Option B**: Platform stores visual data directly (position, asset reference)
- Platform tracks its own visual state
- Rendering queries Platform directly instead of through Object

**Option C**: Create rendering-only storage in WorldModel
- Add `std::vector<RenderableAsset>` for visual-only objects
- Similar to AssetEnvironment but without collision hull
- Keeps physics and rendering concerns separated

**Recommended**: Option B aligns with ticket 0021's note that `Object::createGraphical()` was "Not replaced (rendering is msd-gpu responsibility)". Platform should own its visual state.

### 3. SDLApp::spawnObject() — Mock Rendering Objects

**Current Implementation** (SDLApp.cpp:280-315):
```cpp
void SDLApplication::spawnObject(const std::string& geometryType)
{
  // ... random position/orientation/color generation ...

  mockObjects_.push_back(
    msd_sim::Object::createGraphical(*asset, randomFrame, r, g, b));

  SDL_Log("Spawned %s at (...)", ...);
}
```

**Issue**: Uses Object for mock rendering in the GUI layer.

**Migration Options**:

**Option A**: Use AssetEnvironment for mock objects
- Treats them as static environment objects
- But no collision hull is needed for pure rendering

**Option B**: Create a simple rendering-only struct
- Define a local struct in msd-gui that holds what's needed for rendering
- `RenderableObject { ReferenceFrame frame; Color color; GeometryType type; }`

**Option C**: Store raw data for GPU rendering
- SDLApp directly stores instance data for GPUManager
- Already using `GPUManager::addInstance()` pattern

**Recommended**: Option C - the mockObjects_ vector appears to be for GUI mock testing. The GPUManager already has instance management. Could store asset references or directly manage GPU instances.

**Human Feedback**AssetEnvironment should be used. We expect collisionhull to be needed for all `AssetEnvironment` objects in the future.

---

## Implementation Checklist

### Phase 1: Engine::spawnInertialObject()
- [ ] Modify `spawnInertialObject()` to call `spawnInertialAsset()` internally
- [ ] Verify existing tests still pass
- [ ] Verify simulation spawning behavior unchanged

### Phase 2: SDLApp::spawnObject()
- [ ] Analyze how mockObjects_ is used (rendering pipeline)
- [ ] Design alternative storage for visual-only mock objects
- [ ] Implement replacement storage
- [ ] Update spawnObject() to use new approach
- [ ] Update rendering code that iterates mockObjects_
- [ ] Verify rendering behavior unchanged

### Phase 3: Engine::spawnPlayerPlatform()
- [ ] Modify Platform class to store visual state directly
- [ ] Update Platform::setVisualObject() or replace with alternative
- [ ] Update Engine::spawnPlayerPlatform() to use new approach
- [ ] Update any code that queries Platform's visual object
- [ ] Verify player platform behavior unchanged

### Phase 4: Verification
- [ ] Build project: `cmake --build --preset conan-debug`
- [ ] Run tests: `ctest --preset conan-debug`
- [ ] Verify no deprecation warnings in build output
- [ ] Manual testing of simulation and rendering

---

## References

### Related Code
- `msd/msd-sim/src/Engine.cpp:28-46` — spawnInertialObject() uses deprecated APIs
- `msd/msd-sim/src/Engine.cpp:120-150` — spawnPlayerPlatform() uses deprecated APIs
- `msd/msd-gui/src/SDLApp.cpp:280-315` — spawnObject() uses deprecated APIs
- `msd/msd-sim/src/Environment/Platform.hpp` — Platform class with setVisualObject()

### Related Tickets
- `tickets/0021_worldmodel_asset_refactor.md` — Parent ticket (Phase 1)
- `tickets/0022_remove_deprecated_worldmodel_components.md` — Blocked ticket (Phase 3)

### Design Documents
- `docs/designs/worldmodel-asset-refactor/design.md` — Original refactor design

---

## Workflow Log

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Modified**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}
