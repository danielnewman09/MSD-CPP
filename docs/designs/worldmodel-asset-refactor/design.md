# Design: WorldModel Asset Refactor

## Summary

This design refactors the `WorldModel` class to deprecate the unified `Object` class in favor of specialized asset types (`AssetEnvironment` and `AssetInertial`), and consolidates boundary handling into a single `ConvexHull`. This provides compile-time type safety, eliminates the need for runtime type checking and index caches, and creates a cleaner separation between static environment objects, dynamic physics objects, and simulation boundaries.

## Architecture Changes

### PlantUML Diagram

See: [`./worldmodel-asset-refactor.puml`](./worldmodel-asset-refactor.puml)

### New Components

#### CollisionIterator

**REMOVED**: This component is no longer part of the design. Collision detection will iterate over moving objects (`AssetInertial`) directly, as static objects (`AssetEnvironment`) and boundary (`ConvexHull`) do not move and can be handled separately in collision routines.

### Modified Components

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:

  1. **Add typed storage vectors**:
     ```cpp
     private:
         std::vector<AssetEnvironment> environmentAssets_;
         std::vector<AssetInertial> inertialAssets_;
         std::optional<ConvexHull> simulationBoundary_;
     ```

  2. **Add environment asset management methods**:
     ```cpp
     public:
         size_t addEnvironmentAsset(AssetEnvironment&& asset);
         const AssetEnvironment& getEnvironmentAsset(size_t index) const;
         AssetEnvironment& getEnvironmentAsset(size_t index);
         const std::vector<AssetEnvironment>& getEnvironmentAssets() const;
         std::vector<AssetEnvironment>& getEnvironmentAssets();
         void removeEnvironmentAsset(size_t index);
         size_t getEnvironmentAssetCount() const;
     ```

  3. **Add inertial asset management methods**:
     ```cpp
     public:
         size_t addInertialAsset(AssetInertial&& asset);
         const AssetInertial& getInertialAsset(size_t index) const;
         AssetInertial& getInertialAsset(size_t index);
         const std::vector<AssetInertial>& getInertialAssets() const;
         std::vector<AssetInertial>& getInertialAssets();
         void removeInertialAsset(size_t index);
         size_t getInertialAssetCount() const;
     ```

  4. **Add boundary management methods**:
     ```cpp
     public:
         void setBoundary(ConvexHull boundary);
         const std::optional<ConvexHull>& getBoundary() const;
         std::optional<ConvexHull>& getBoundary();
         bool hasBoundary() const;
         void clearBoundary();
     ```

  5. **Update internal physics/collision methods**:
     ```cpp
     private:
         void updatePhysics(double dt);      // Iterate inertialAssets_ directly
         void updateCollisions();            // Iterate inertialAssets_ for moving objects
     ```

  7. **Deprecate old members** (Phase 1 - keep for compatibility):
     ```cpp
     private:
         [[deprecated("Use environmentAssets_/inertialAssets_ instead")]]
         std::vector<Object> objects_;

         [[deprecated("No longer needed with typed storage")]]
         std::vector<size_t> physicsObjectIndices_;

         [[deprecated("No longer needed with typed storage")]]
         std::vector<size_t> collisionObjectIndices_;

         [[deprecated("No longer needed with typed storage")]]
         std::vector<size_t> renderObjectIndices_;
     ```

- **Backward compatibility**:
  - Phase 1: Keep `objects_` vector and index caches, mark as deprecated
  - Phase 2: Update all consumers to use new typed methods
  - Phase 3: Remove deprecated members in a breaking change release

#### Engine

- **Current location**: `msd/msd-sim/src/Engine.hpp`, `Engine.cpp`
- **Changes required**:

  1. **Update `spawnInertialObject`** to use new typed storage:
     ```cpp
     void spawnInertialObject(const std::string& assetName,
                              const Coordinate& position,
                              const EulerAngles& orientation) {
         // Get collision geometry from asset registry
         auto asset = assetRegistry_.getAsset(assetName);
         if (!asset) { throw std::invalid_argument("Asset not found"); }

         auto collisionGeom = asset->get().getCollisionGeometry();
         if (!collisionGeom) { throw std::invalid_argument("No collision geometry"); }

         // Create AssetInertial with geometry and frame
         ReferenceFrame frame{position};
         frame.setRotation(orientation);

         // Create CollisionGeometry by value and move into AssetInertial
         msd_assets::CollisionGeometry geom{collisionGeom->get()};

         AssetInertial inertialAsset{std::move(geom), 1.0, frame};  // Default mass

         worldModel_.addInertialAsset(std::move(inertialAsset));
     }
     ```

  2. **Add method for environment assets**:
     ```cpp
     void spawnEnvironmentAsset(const std::string& assetName,
                                const Coordinate& position,
                                const EulerAngles& orientation);
     ```

  3. **Add method for setting simulation boundary**:
     ```cpp
     void setSimulationBoundary(const ConvexHull& boundary);
     void setSimulationBoundary(const std::vector<Coordinate>& boundaryPoints);
     ```

- **Backward compatibility**: Existing `spawnInertialObject` signature preserved, implementation updated

#### AssetPhysical (minor update)

- **Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetPhysical.hpp`
- **Changes required**:
  1. **Ensure value semantics for geometry storage**:
     ```cpp
     protected:
         msd_assets::CollisionGeometry visualGeometry_;
         ConvexHull collisionHull_;
         ReferenceFrame referenceFrame_;
     ```

  2. **Update constructor to accept rvalue reference for geometry**:
     ```cpp
     public:
         explicit AssetPhysical(msd_assets::CollisionGeometry&& geometry,
                                const ReferenceFrame& frame = ReferenceFrame());
     ```

  3. **Ensure copy/move semantics are explicit**:
     ```cpp
     public:
         AssetPhysical(const AssetPhysical&) = default;
         AssetPhysical& operator=(const AssetPhysical&) = default;
         AssetPhysical(AssetPhysical&&) noexcept = default;
         AssetPhysical& operator=(AssetPhysical&&) noexcept = default;
     ```

  4. **Deprecate color members** (if present - rendering is msd-gpu responsibility):
     ```cpp
     [[deprecated("Render color is msd-gpu responsibility")]]
     void setColor(float r, float g, float b);

     [[deprecated("Render color is msd-gpu responsibility")]]
     std::array<float, 3> getColor() const;
     ```

- **Backward compatibility**: Geometry storage changes are internal; deprecate any color APIs

#### AssetEnvironment (minor update)

- **Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp`
- **Changes required**:
  1. **Add public value-based constructor**:
     ```cpp
     public:
         // Primary constructor for WorldModel storage
         explicit AssetEnvironment(msd_assets::CollisionGeometry&& geometry,
                                   const ReferenceFrame& frame = ReferenceFrame());
     ```

  2. **Deprecate existing factory method**:
     ```cpp
     [[deprecated("Use public constructor instead")]]
     static std::shared_ptr<AssetEnvironment> create(
         std::shared_ptr<msd_assets::Geometry> geometry,
         const ReferenceFrame& frame = ReferenceFrame());
     ```

- **Backward compatibility**: Existing `create()` method preserved but deprecated

#### AssetInertial (minor update)

- **Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp`
- **Changes required**:
  1. **Update constructor to accept rvalue reference for geometry**:
     ```cpp
     public:
         explicit AssetInertial(msd_assets::CollisionGeometry&& geometry,
                                double mass,
                                const ReferenceFrame& frame = ReferenceFrame());
     ```

  2. **Ensure move semantics work correctly** with DynamicState
  3. **Add explicit special member functions**:
     ```cpp
     public:
         AssetInertial(const AssetInertial&) = default;
         AssetInertial& operator=(const AssetInertial&) = default;
         AssetInertial(AssetInertial&&) noexcept = default;
         AssetInertial& operator=(AssetInertial&&) noexcept = default;
     ```

- **Backward compatibility**: Constructor signature change is breaking; document migration path

### Deprecated Components

#### Object

- **Current location**: `msd/msd-sim/src/Environment/Object.hpp`, `Object.cpp`
- **Deprecation strategy**:

  Phase 1 (this design):
  - Mark class with `[[deprecated("Use AssetEnvironment/AssetInertial instead")]]`
  - Keep for backward compatibility
  - Document migration path in class comments

  Phase 2 (future):
  - Remove usage from WorldModel
  - Remove usage from Engine

  Phase 3 (future breaking change):
  - Delete Object class entirely
  - Delete associated test files

- **Migration mapping**:
  | Object Factory Method | Replacement |
  |-----------------------|-------------|
  | `Object::createInertial()` | `AssetInertial` constructor |
  | `Object::createEnvironmental()` | `AssetEnvironment::create()` |
  | `Object::createBoundary()` | `WorldModel::setBoundary()` |
  | `Object::createGraphical()` | See Open Questions |

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| `WorldModel` (typed storage) | `AssetEnvironment` | Composition | Owns environment assets by value |
| `WorldModel` (typed storage) | `AssetInertial` | Composition | Owns inertial assets by value |
| `WorldModel` (typed storage) | `ConvexHull` | Optional composition | Single boundary hull |
| `Engine` | `WorldModel` (typed storage) | Uses | Spawns assets via new methods |
| `Platform` | `AssetInertial` | Reference | Platform delegates to AssetInertial |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|-----------------|
| `test/Environment/WorldModelTest.cpp` | All Object-based tests | Deprecated | Add parallel tests using new typed storage |
| `test/Environment/ObjectTest.cpp` | All tests | Deprecated | Keep until Object removed |
| `test/Physics/ConvexHullTest.cpp` | None | No impact | No changes needed |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| `WorldModel` | `addEnvironmentAsset_storesAsset` | Environment assets are stored and retrievable |
| `WorldModel` | `addInertialAsset_storesAsset` | Inertial assets are stored and retrievable |
| `WorldModel` | `setBoundary_storesBoundary` | Single boundary is set correctly |
| `WorldModel` | `setBoundary_replacesPrevious` | Setting new boundary replaces old one |
| `WorldModel` | `clearBoundary_removesBoundary` | Boundary is cleared |
| `WorldModel` | `removeEnvironmentAsset_removesAsset` | Environment assets can be removed |
| `WorldModel` | `removeInertialAsset_removesAsset` | Inertial assets can be removed |
| `WorldModel` | `getInertialAssets_iteratesMovingObjects` | Direct iteration over moving objects |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `Engine_spawnInertialAsset_addsToWorld` | Engine, WorldModel, AssetInertial | Engine correctly creates and stores inertial assets with move semantics |
| `Engine_spawnEnvironmentAsset_addsToWorld` | Engine, WorldModel, AssetEnvironment | Engine correctly creates and stores environment assets |
| `WorldModel_update_integratesInertialPhysics` | WorldModel, AssetInertial, DynamicState | Physics update works with new typed storage |
| `WorldModel_update_detectsCollisions` | WorldModel, AssetInertial, ConvexHull | Collision detection iterates over moving objects |
| `Platform_delegatesToAssetInertial` | Platform, AssetInertial | Platform delegates state queries to AssetInertial |

#### Benchmark Tests (if performance-critical)

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| `WorldModel` | `BM_IterateInertialAssets` | Direct iteration over inertial vector | Faster than index-based iteration |
| `WorldModel` | `BM_IterateEnvironmentAssets` | Direct iteration over environment vector | Faster than index-based iteration |

## Open Questions

### Design Decisions (Human Input Needed)

1. **How should graphical-only objects be handled?**
   - Option A: Add `AssetGraphical` class
     - Pros: Maintains support for pure visual objects, follows same pattern as other assets
     - Cons: Adds another class, may encourage non-physical decoration which is outside sim scope
   - Option B: Remove support for graphical-only objects
     - Pros: Simplifies design, forces all visible objects to have collision
     - Cons: Cannot represent purely decorative elements (skyboxes, particle effects)
   - Option C: Use existing `msd_assets::Asset` directly with a separate render list
     - Pros: Reuses existing class, clean separation of concerns
     - Cons: Inconsistent with other asset storage patterns, requires separate render management
   - **Recommendation**: Option B (remove support) if the simulation does not need purely decorative objects. If decorative objects are needed, Option C keeps rendering concerns separate from physics simulation.

2. **Should the boundary support multiple ConvexHulls for complex boundaries?**
   - Option A: Single `ConvexHull` (current design)
     - Pros: Simpler implementation, clear semantics, matches typical simulation bounds
     - Cons: Cannot represent L-shaped or complex boundary regions
   - Option B: `std::vector<ConvexHull>` for compound boundaries
     - Pros: Supports arbitrary boundary shapes, more flexible
     - Cons: More complex collision detection, unclear semantics for "inside boundary"
   - **Recommendation**: Option A (single hull). Complex boundaries can be represented as multiple `AssetEnvironment` objects with appropriate collision hulls. The single boundary hull represents the outer simulation volume.

3. **What happens to Platform legacy support?**
   - **Resolution**: Platform will delegate to AssetInertial. Platform should NOT contain an `InertialState` independent of `AssetInertial`'s state. The Platform class will store a reference or index to an `AssetInertial` object and delegate all state queries and updates to that asset. This maintains unified storage while allowing Platform to act as a controller/agent wrapper.

### Prototype Required

**REMOVED**: No prototypes required. CollisionIterator has been removed from the design, and direct vector iteration is a well-understood pattern with predictable performance characteristics.

### Requirements Clarification

**RESOLVED**: All requirements have been clarified through human feedback.

1. **Render color storage location**: All references to render color in `msd-sim` are deprecated. Render color and texture are the responsibility of the `msd-gpu` library. Any existing color APIs in `AssetPhysical` or derived classes should be marked `[[deprecated]]`.

2. **Asset ownership model**: WorldModel uses value semantics (`std::vector<AssetInertial>`, `std::vector<AssetEnvironment>`). Geometry is stored by value in `AssetPhysical`, and assets are moved into WorldModel vectors using move semantics.

---

## Appendix: Current vs. New Storage Comparison

### Current Architecture (Object-based)

```cpp
// WorldModel storage
std::vector<Object> objects_;
std::vector<size_t> physicsObjectIndices_;   // Indices into objects_
std::vector<size_t> collisionObjectIndices_; // Indices into objects_
std::vector<size_t> renderObjectIndices_;    // Indices into objects_

// Iteration pattern
for (size_t idx : worldModel.getPhysicsObjectIndices()) {
    Object& obj = worldModel.getObject(idx);
    if (obj.hasPhysics()) {  // Runtime check (always true, but...)
        obj.getPhysics().applyForce(...);
    }
}
```

**Problems**:
- Runtime type checking even when type is known
- Index caches must be maintained on add/remove
- Object stores optional components that waste memory
- Type enum checked at runtime for iteration

### New Architecture (Typed storage)

```cpp
// WorldModel storage
std::vector<AssetEnvironment> environmentAssets_;
std::vector<AssetInertial> inertialAssets_;
std::optional<ConvexHull> simulationBoundary_;

// Iteration pattern - physics update (only moving objects)
for (auto& asset : worldModel.getInertialAssets()) {
    asset.getDynamicState().applyForce(...);  // No runtime check
}

// Collision detection - iterate moving objects against static environment
for (auto& movingAsset : worldModel.getInertialAssets()) {
    for (const auto& staticAsset : worldModel.getEnvironmentAssets()) {
        // Check collision between movingAsset and staticAsset
    }
    if (worldModel.hasBoundary()) {
        // Check collision with boundary
    }
}
```

**Benefits**:
- Compile-time type safety
- No index caches needed
- No wasted memory on optional components
- Direct vector iteration (cache-friendly)
- Clear separation of concerns

---

## Design Review — Initial Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-11
**Status**: REVISION_REQUESTED
**Iteration**: 0 of 1

### Issues Requiring Revision

| ID | Issue | Category | Required Change |
|----|-------|----------|-----------------|
| I1 | AssetPhysical ownership model conflicts with value semantics goal | C++ Quality / Architectural Fit | Remove shared_ptr from AssetPhysical, use value storage for geometry |
| I2 | AssetEnvironment factory pattern incompatible with value semantics | C++ Quality | Add value-based constructor alongside factory methods |
| I3 | Missing color storage location decision impacts multiple components | Architectural Fit | Clarify where render color is stored after Object deprecation |
| I4 | CollisionIterator complexity and performance overhead unclear | Feasibility | Add prototype requirement for iteration performance validation |
| I5 | Engine spawn methods use shared_ptr creation but WorldModel uses value storage | Architectural Fit | Align Engine spawn implementation with value-based WorldModel storage |
| I6 | Platform migration path underspecified | Architectural Fit | Clarify Platform's relationship to AssetInertial with concrete design |
| I7 | Boundary return type inconsistent with CLAUDE.md guidelines | C++ Quality | Use plain optional reference instead of reference_wrapper |

### Revision Instructions for Architect

The following changes must be made before final review:

1. **Issue I1 - AssetPhysical shared_ptr conflict**: The design proposes value semantics (`std::vector<AssetInertial>`) for WorldModel storage, but `AssetPhysical` currently uses `std::shared_ptr<msd_assets::CollisionGeometry>` for geometry. This creates an ownership mismatch. **Required change**: Modify AssetPhysical to store geometry by value (`msd_assets::CollisionGeometry visualGeometry_`) instead of shared_ptr. If memory concerns exist for large geometry, document the trade-off and prototype the memory impact. The current codebase shows `AssetPhysical` already stores geometry by value, so ensure the design reflects this accurately.

**Human Response** I agree with the required change

2. **Issue I2 - AssetEnvironment factory incompatibility**: The design shows `AssetEnvironment::create()` returning `shared_ptr<AssetEnvironment>`, which is incompatible with storing `AssetEnvironment` by value in `std::vector<AssetEnvironment>`. **Required change**: Add a public value-based constructor to `AssetEnvironment` that can be used with `emplace_back()`, or modify the factory to support creating values directly (e.g., `AssetEnvironment::createValue()` returning by value). Document which construction path is preferred for WorldModel usage.

**Human Response** I agree with the required change. Let's use the public constructor and deprecate the factory method

3. **Issue I3 - Color storage location**: The design mentions "deprecate render color - currently in use in Object" in human decisions but doesn't specify the migration path. The design proposes adding color to `AssetPhysical`, but this couples rendering concerns to physics. **Required change**: Either (A) add color to AssetPhysical with clear justification for coupling rendering to physics layer, or (B) specify that color should be managed outside the asset classes (e.g., in a separate render component). Provide rationale for the chosen approach.

**Human Response**All references to render color in the msd-sim library should be deprecated because we store simply the vertex geometry for physics simulation. Responsibility for render color and texture are in the msd-gpu library

4. **Issue I4 - CollisionIterator performance**: The design proposes a `CollisionIterator` that unifies iteration over heterogeneous types (`AssetEnvironment`, `AssetInertial`, `ConvexHull` boundary). This adds complexity and potential performance overhead compared to direct vector iteration. **Required change**: Add a prototype requirement (P1) to validate that `CollisionIterator` overhead is acceptable (<10% slower than separate typed iterations). Specify measurable success criteria (e.g., "Iterate 100 environment + 100 inertial + 1 boundary in <X microseconds").

**Human Response**Do not implement CollisionIterator. We will only need to iterate over moving objects (`AssetInertial`) since environmental and boundary objects are by definition static.

5. **Issue I5 - Engine spawn method mismatch**: The `Engine::spawnInertialObject` method in the design creates `AssetInertial` with `std::make_shared<msd_assets::CollisionGeometry>`, but WorldModel's `addInertialAsset` expects move semantics for value types. **Required change**: Update the Engine example to use value-based construction: `AssetInertial inertialAsset{*collisionGeom, 1.0, frame}` (dereferencing the shared_ptr to copy geometry), or justify why shared_ptr is necessary. Ensure consistency with the value semantics design goal.

**Human Response**Modify the design to simply create a `CollisionGeometry` object and construct the `AssetInertial` using move semantics. This means altering the function signature for the constructor of `AssetInertial` to accept an rvalue reference to the collision geometry rather than a shared pointer

6. **Issue I6 - Platform migration underspecified**: The design recommends "Platform should reference an AssetInertial rather than containing its own state" but doesn't provide concrete details on how this works. **Required change**: Specify whether Platform stores `std::reference_wrapper<AssetInertial>`, `size_t` index, or some other reference mechanism. Clarify how Platform's `InertialState` relates to AssetInertial's state (does Platform delegate to AssetInertial, or maintain separate state?). Show concrete code example of the proposed relationship.

**Human Response**Platform should not contain an `InertialState` independent of `AssetInertial`'s state. Platform should delegate to `AssetInertial`

7. **Issue I7 - Boundary return type non-standard**: The design uses `std::optional<std::reference_wrapper<const ConvexHull>>` for `getBoundary()`, but CLAUDE.md states: "Prefer plain references (`const T&` or `T&`) for non-owning access" and "Use `std::optional<std::reference_wrapper<const T>>` ONLY for returning non-owning references that are truly optional". While boundary is optional, the return type should follow project conventions more closely. **Required change**: Consider returning `std::optional<ConvexHull>&` (reference to the optional itself) which is cleaner and allows direct access via `.value()` without unwrapping reference_wrapper. If reference_wrapper is required, document the specific reason why direct optional reference won't work.

**Human response**I agree with the required change

### Items Passing Review (No Changes Needed)

- **Naming conventions**: Class names (`CollisionIterator`, `WorldModel`) and method names (`addEnvironmentAsset`, `getInertialAssets`) follow PascalCase/camelCase conventions correctly
- **Namespace organization**: Components remain in `msd_sim` namespace with appropriate submodules
- **File structure**: Proposed layout follows `msd/msd-sim/src/Environment/` and `msd/msd-sim/src/Physics/RigidBody/` patterns
- **Dependency direction**: No cycles introduced; CollisionIterator depends on WorldModel (correct direction)
- **Rule of 0/3/5**: Explicit `= default` declarations proposed for AssetPhysical, AssetInertial are correct
- **Const correctness**: Accessor methods properly const-qualified
- **Test organization**: Proposed test structure mirrors source layout correctly
- **Deprecation strategy**: Phased approach (mark deprecated → update consumers → remove) is sound
- **Backward compatibility**: Phase 1 maintains Object for compatibility (good gradual migration)

---

## Architect Revision Notes

**Date**: 2026-01-11
**Responding to**: Design Review — Initial Assessment

### Changes Made

| Issue ID | Original | Revised | Rationale |
|----------|----------|---------|-----------|
| I1 | AssetPhysical stored geometry via `shared_ptr<msd_assets::CollisionGeometry>` | AssetPhysical stores geometry by value: `msd_assets::CollisionGeometry visualGeometry_` | Aligns with value semantics goal for WorldModel storage; eliminates ownership complexity |
| I2 | AssetEnvironment used factory method `create()` returning `shared_ptr` | Added public constructor accepting `CollisionGeometry&&`; deprecated factory method | Enables direct construction with `emplace_back()` for value-based WorldModel storage |
| I3 | Design proposed adding color to AssetPhysical | Deprecated all render color APIs in msd-sim classes | Render color is msd-gpu library responsibility; keeps physics layer decoupled from rendering |
| I4 | Design included CollisionIterator for unified iteration | Removed CollisionIterator entirely | Only need to iterate moving objects (AssetInertial); static objects (AssetEnvironment, boundary) handled separately |
| I5 | Engine spawn used `make_shared` to create geometry, passed shared_ptr to AssetInertial | Engine creates `CollisionGeometry` by value, uses `std::move()` to pass to AssetInertial constructor | Consistent with value semantics; AssetInertial constructor accepts `CollisionGeometry&&` |
| I6 | Platform migration path was underspecified | Platform delegates to AssetInertial; does NOT maintain separate `InertialState` | Unified storage; Platform acts as controller/agent wrapper referencing AssetInertial |
| I7 | getBoundary() returned `std::optional<std::reference_wrapper<const ConvexHull>>` | getBoundary() returns `const std::optional<ConvexHull>&` | Simpler, follows project conventions for optional references |

### Diagram Updates

- Removed `CollisionIterator` class and all associated relationships
- Updated `WorldModel::getBoundary()` return type to `const std::optional<ConvexHull>&`
- Removed `getAllCollidables()` method from WorldModel
- Updated AssetPhysical to show value-based geometry storage (`CollisionGeometry visualGeometry_`)
- Added deprecation markers for AssetEnvironment factory method
- Updated iteration pattern notes to show collision detection (iterate moving objects against static environment)
- Removed CollisionIterator from architecture benefits note

### Unchanged (Per Reviewer Guidance)

- Naming conventions (PascalCase classes, camelCase methods, snake_case_ members)
- Namespace organization (msd_sim with appropriate submodules)
- File structure following existing patterns
- Dependency direction (no cycles)
- Rule of 0/3/5 explicit `= default` declarations
- Const correctness in accessor methods
- Test organization mirroring source layout
- Deprecation strategy (phased approach)
- Backward compatibility (Phase 1 maintains Object for compatibility)

---

## Design Review — Final Assessment

**Reviewer**: Design Review Agent
**Date**: 2026-01-11
**Status**: APPROVED
**Iteration**: 1 of 1

### Issue Resolution Verification

| Issue ID | Requested Change | Resolution | Status |
|----------|------------------|------------|--------|
| I1 | AssetPhysical stores geometry by value | Design updated to show `msd_assets::CollisionGeometry visualGeometry_` (value storage). PlantUML diagram correctly reflects value composition. Current codebase already implements this. | RESOLVED |
| I2 | AssetEnvironment public constructor, deprecate factory | Design adds public constructor accepting `CollisionGeometry&&`. Factory method marked `[[deprecated]]`. PlantUML shows both constructor and deprecated factory. | RESOLVED |
| I3 | Deprecate all render color APIs | Design adds deprecation markers to color methods with message "Render color is msd-gpu responsibility". Requirements clarification section documents this decision. | RESOLVED |
| I4 | Remove CollisionIterator | Design explicitly states "REMOVED" at top of CollisionIterator section. PlantUML removes all CollisionIterator references. Iteration patterns show direct typed vector iteration. | RESOLVED |
| I5 | Engine uses move semantics for CollisionGeometry | Design shows `CollisionGeometry geom{...}; AssetInertial{std::move(geom), ...}`. AssetInertial constructor accepts `CollisionGeometry&&`. | RESOLVED |
| I6 | Platform delegates to AssetInertial | Integration table shows delegation relationship. Test case `Platform_delegatesToAssetInertial` added. Open Questions section documents resolution. | RESOLVED |
| I7 | getBoundary() returns `const std::optional<ConvexHull>&` | Design shows correct return type. PlantUML updated. Simpler than reference_wrapper pattern. | RESOLVED |

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | PASS | Class names (AssetEnvironment, AssetInertial), methods (addEnvironmentAsset, getBoundary) follow PascalCase/camelCase correctly |
| Namespace organization | PASS | Components remain in `msd_sim` namespace with Physics submodule where appropriate |
| File structure | PASS | Proposed layout follows existing `msd/msd-sim/src/Environment/` and `msd/msd-sim/src/Physics/RigidBody/` patterns |
| Dependency direction | PASS | No cycles introduced. WorldModel owns assets, Engine uses WorldModel (correct direction) |

#### C++ Design Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | PASS | Geometry stored by value in AssetPhysical, assets stored by value in WorldModel vectors |
| Smart pointer appropriateness | PASS | Removed shared_ptr for geometry (value semantics). No unnecessary smart pointers in new design |
| Value/reference semantics | PASS | Assets stored by value in vectors. Move semantics for construction. Plain references for non-owning access |
| Rule of 0/3/5 | PASS | Explicit `= default` declarations for copy/move/destructor on AssetPhysical and AssetInertial |
| Const correctness | PASS | Accessor methods properly const-qualified. `const std::optional<ConvexHull>&` return type correct |
| Exception safety | PASS | Constructor validation throws on invalid arguments. Basic exception guarantee for storage operations |

#### Feasibility
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | PASS | No circular dependencies introduced. Forward declarations used where appropriate |
| Template complexity | PASS | No complex template metaprogramming. Uses standard `std::vector`, `std::optional` |
| Memory strategy | PASS | Clear ownership: WorldModel owns assets by value in vectors. CollisionGeometry moved into assets |
| Thread safety | PASS | Single-threaded simulation assumption documented. Consistent with existing WorldModel design |
| Build integration | PASS | No new dependencies. Uses existing msd-assets types (CollisionGeometry). Fits existing CMake structure |

#### Testability
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | PASS | Assets constructible independently. WorldModel testable with mock assets |
| Mockable dependencies | PASS | CollisionGeometry can be constructed in tests from point vectors |
| Observable state | PASS | Typed storage vectors accessible via getters. Asset state inspectable |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Platform refactor to delegate to AssetInertial may have unintended side effects on existing MotionController/Agent integration | Integration | Low | Medium | Phase the Platform change separately, maintain backward compatibility via optional InertialState fallback during transition | No |
| R2 | Removing Object type may break external consumers (msd-gui rendering) | Maintenance | Low | Medium | Phased deprecation strategy (Phase 1-2-3) gives time to update consumers | No |

### Summary

All 7 issues from the initial review have been properly addressed per the human's feedback. The revised design:

1. **Correctly implements value semantics** - Geometry stored by value, assets stored in typed vectors, move semantics for construction
2. **Cleanly separates concerns** - Render color deprecated in msd-sim (msd-gpu responsibility), CollisionIterator removed (direct iteration sufficient)
3. **Follows project conventions** - Return types, naming, const correctness all align with CLAUDE.md standards
4. **Maintains feasibility** - No prototype required since direct vector iteration is well-understood

The design is **APPROVED** for implementation. No blocking issues remain. Minor risks (R1, R2) are mitigated by the phased deprecation strategy already documented in the design.

### Recommended Implementation Order

1. **Phase 1a**: Add typed storage to WorldModel (environmentAssets_, inertialAssets_, simulationBoundary_) with new add/get methods
2. **Phase 1b**: Add value-based constructor to AssetEnvironment, deprecate factory method
3. **Phase 1c**: Update Engine spawn methods to use move semantics with CollisionGeometry
4. **Phase 2**: Update physics/collision loops to use typed iteration
5. **Phase 3**: Deprecate Object class and index caches
6. **Future**: Platform delegation refactor (can be done independently)

---
