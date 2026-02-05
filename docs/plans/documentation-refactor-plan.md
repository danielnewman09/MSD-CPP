# Documentation Refactoring Plan

> Goal: Eliminate redundancy between CLAUDE.md files and MCP-queryable information while preserving essential architectural context.

## Executive Summary

With the new MCP codebase server, much of what's currently in CLAUDE.md files can now be queried programmatically from the Doxygen-generated SQLite database. This plan outlines how to:

1. Move API-level documentation into Doxygen comments (queryable via MCP)
2. Slim down CLAUDE.md to focus on architectural context (not queryable)
3. Establish clear guidelines for what goes where

---

## Phase 1: Audit and Categorize

### Files to Review (20 CLAUDE.md files)

**Root Level:**
- [ ] `CLAUDE.md` (project root)
- [ ] `msd/CLAUDE.md` (library overview)
- [ ] `analysis/CLAUDE.md`

**Library Level:**
- [ ] `msd/msd-assets/CLAUDE.md`
- [ ] `msd/msd-transfer/CLAUDE.md`
- [ ] `msd/msd-utils/CLAUDE.md`
- [ ] `msd/msd-gui/CLAUDE.md`
- [ ] `msd/msd-exe/CLAUDE.md`
- [ ] `msd/msd-asset-gen/CLAUDE.md`

**msd-sim Submodules:**
- [ ] `msd/msd-sim/CLAUDE.md`
- [ ] `msd/msd-sim/src/Agent/CLAUDE.md`
- [ ] `msd/msd-sim/src/Environment/CLAUDE.md`
- [ ] `msd/msd-sim/src/Utils/CLAUDE.md`
- [ ] `msd/msd-sim/src/Physics/CLAUDE.md`
- [ ] `msd/msd-sim/src/Physics/Collision/CLAUDE.md`
- [ ] `msd/msd-sim/src/Physics/Constraints/CLAUDE.md`
- [ ] `msd/msd-sim/src/Physics/Constraints/ECOS/CLAUDE.md`
- [ ] `msd/msd-sim/src/Physics/Integration/CLAUDE.md`
- [ ] `msd/msd-sim/src/Physics/PotentialEnergy/CLAUDE.md`
- [ ] `msd/msd-sim/src/Physics/RigidBody/CLAUDE.md`

---

## Phase 2: Define What Stays vs. Goes

### Content That STAYS in CLAUDE.md (NOT queryable via MCP)

1. **Architectural Diagrams**
   - ASCII diagrams showing component relationships
   - Layer hierarchies (e.g., Engine → WorldModel → Object)
   - Data flow diagrams

2. **Design Rationale ("Why" documentation)**
   - Why specific algorithms were chosen (GJK vs SAT, Mirtich vs tetrahedron decomposition)
   - Why certain patterns are used (quaternions vs Euler angles)
   - Trade-offs considered and rejected alternatives

3. **Cross-Cutting Conventions**
   - Coordinate system (right-handed, X=forward, Y=starboard, Z=up)
   - Unit conventions (SI units throughout)
   - Error handling philosophy (when to use optional vs exceptions)
   - Memory management patterns (value semantics, ownership rules)
   - Thread safety guarantees

4. **Historical Context and Breaking Changes**
   - Why components were refactored (CollisionResponse → ContactConstraint)
   - Migration guidance for breaking changes
   - Evolution of state vector (13→14 components)

5. **Performance Characteristics**
   - Algorithmic complexity analysis
   - FLOP counts for critical operations
   - Benchmark references and optimization guidance

6. **Integration Patterns**
   - How subsystems work together (WorldModel::updatePhysics flow)
   - Order of operations in physics pipeline
   - Constraint solver integration points

7. **Future Work / Limitations**
   - Known limitations (no friction, O(n²) broadphase)
   - Planned enhancements with priority
   - Extension points for future development

8. **Mathematical Reference**
   - Core equations (Minkowski difference, Jacobians)
   - Derivation references (Mirtich 1996, van den Bergen 2001)

9. **AI Assistant Navigation Hints**
   - "Start here" guidance
   - Reading order recommendations
   - Cross-references to related documentation

### Content That MOVES to Doxygen Comments (queryable via MCP)

1. **Class/Struct Descriptions**
   ```cpp
   // Before (in CLAUDE.md):
   // "CollisionHandler orchestrates GJK and EPA algorithms..."

   // After (in source code):
   /**
    * @brief Orchestrates GJK and EPA algorithms for collision detection.
    *
    * Provides a unified collision detection interface that:
    * - Runs GJK to detect intersection
    * - If collision detected, runs EPA to compute contact info
    * - Returns std::optional to indicate collision presence
    *
    * @see GJK
    * @see EPA
    * @see CollisionResult
    */
   class CollisionHandler { ... };
   ```

2. **Method Documentation**
   ```cpp
   // Move method descriptions to Doxygen:
   /**
    * @brief Check for collision between two physical assets.
    *
    * Runs GJK to detect intersection. If collision detected, runs EPA
    * to compute penetration depth, contact normal, and contact manifold.
    *
    * @param assetA First physical asset
    * @param assetB Second physical asset
    * @return std::nullopt if no collision, CollisionResult if collision
    *
    * @threadsafe Stateless after construction - safe for concurrent calls
    */
   std::optional<CollisionResult> checkCollision(
       const AssetPhysical& assetA,
       const AssetPhysical& assetB) const;
   ```

3. **Parameter Documentation**
   - Move parameter descriptions to `@param` tags
   - Move return value descriptions to `@return` tags
   - Move exception documentation to `@throws` tags

4. **Usage Examples** (short ones)
   - Simple usage examples can go in `@code` blocks
   - Complex integration examples stay in CLAUDE.md

5. **Thread Safety Notes**
   - Add `@threadsafe` or `@notthreadsafe` custom tags

### Content to REMOVE (redundant)

1. **File Path Lists**
   - "Location: `../CollisionHandler.hpp`" — queryable via MCP
   - Entire "Key Interfaces" sections that just list signatures

2. **Class/Method Signature Listings**
   - Code blocks showing public interface — queryable via `get_class_members`

3. **Simple "What This Does" Descriptions**
   - One-liner class purposes that duplicate Doxygen `@brief`

4. **Inheritance Hierarchies**
   - Simple inheritance trees — queryable via `get_class_hierarchy`

5. **File Organization Trees**
   - Directory structure listings — queryable via file tools

---

## Phase 3: Execution Plan

### Step 1: Enhance Doxygen Comments (Before slimming CLAUDE.md)

For each component with significant CLAUDE.md documentation:

1. Read current CLAUDE.md documentation
2. Identify API-level content that should be in Doxygen
3. Add/enhance Doxygen comments in source files:
   - Class-level `@brief` and `@details`
   - Method-level documentation with `@param`, `@return`, `@throws`
   - `@see` cross-references
   - `@threadsafe` / `@notthreadsafe` tags
4. Regenerate database: `cmake --build --preset doxygen-db`
5. Verify information is queryable via MCP

**Priority Order** (by documentation volume):
1. `msd-sim/src/Physics/Collision/` — GJK, EPA, CollisionHandler, CollisionResult
2. `msd-sim/src/Physics/RigidBody/` — ConvexHull, AssetInertial, InertialState
3. `msd-sim/src/Physics/Constraints/` — ConstraintSolver, ContactConstraint
4. `msd-assets/` — AssetRegistry, Geometry, GeometryFactory
5. Remaining modules...

### Step 2: Slim Down CLAUDE.md Files

For each CLAUDE.md file:

1. Remove sections now covered by Doxygen:
   - "Key Interfaces" code blocks
   - Simple class descriptions
   - Parameter listings
   - File location references

2. Restructure to focus on:
   - **Overview**: 2-3 sentence purpose (architecture focus)
   - **Architecture Diagram**: Visual component relationships
   - **Design Rationale**: Why decisions were made
   - **Cross-Cutting Concerns**: Conventions and patterns
   - **Integration**: How subsystem fits in larger pipeline
   - **Future Work**: Limitations and planned enhancements
   - **References**: External papers, related tickets

3. Add MCP usage hints:
   ```markdown
   ## Querying This Module

   Use MCP tools for API details:
   - `find_class ConvexHull` — class definition and methods
   - `get_class_members CollisionHandler` — full API surface
   - `search_documentation collision` — find collision-related docs
   ```

### Step 3: Add Custom Doxygen Tags

Add to `Doxyfile.in`:
```
ALIASES += "threadsafe=@par Thread Safety:^^ Thread-safe."
ALIASES += "notthreadsafe=@par Thread Safety:^^ Not thread-safe."
ALIASES += "rationale=@par Design Rationale:^^"
ALIASES += "performance=@par Performance:^^"
```

### Step 4: Update MCP Server Documentation

Update `CLAUDE.md` (root) with guidance on using MCP vs CLAUDE.md:
- MCP for: "What is the API?", "What methods does X have?", "What calls Y?"
- CLAUDE.md for: "Why was it designed this way?", "How does it fit together?"

---

## Phase 4: Validation

### Success Criteria

1. **No Loss of Information**
   - All API documentation accessible via MCP queries
   - All architectural rationale preserved in CLAUDE.md

2. **Reduced File Sizes**
   - Target: 50% reduction in CLAUDE.md word count
   - Focus: Remove redundant listings, keep prose rationale

3. **Clear Separation**
   - MCP answers "what" questions
   - CLAUDE.md answers "why" and "how" questions

4. **Doxygen Quality**
   - All public classes have `@brief`
   - All public methods have `@param`, `@return`, `@throws` as needed
   - Cross-references via `@see`

### Testing

1. Run sample MCP queries to verify API info is accessible
2. Review slimmed CLAUDE.md for architectural completeness
3. Have AI assistant navigate codebase using both sources

---

## Estimated Effort

| Phase | Tasks | Files | Estimate |
|-------|-------|-------|----------|
| Phase 1 | Audit | 20 CLAUDE.md | 1 hour |
| Phase 2 | Define rules | — | Already done |
| Phase 3a | Enhance Doxygen (msd-sim/Physics) | ~30 headers | 3-4 hours |
| Phase 3a | Enhance Doxygen (other modules) | ~20 headers | 2 hours |
| Phase 3b | Slim CLAUDE.md files | 20 files | 2-3 hours |
| Phase 4 | Validation | — | 1 hour |

**Total: ~10-12 hours**

---

## Example: Before/After

### Collision/CLAUDE.md (Before)
```markdown
### CollisionHandler

**Location**: `../CollisionHandler.hpp`, `../CollisionHandler.cpp`
**Introduced**: [Ticket: 0027a_expanding_polytope_algorithm](...)

#### Purpose
Orchestrates GJK and EPA algorithms to provide a unified collision detection interface.
Returns `std::optional<CollisionResult>` where `std::nullopt` indicates no collision.

#### Key Interfaces
```cpp
class CollisionHandler {
public:
  explicit CollisionHandler(double epsilon = 1e-6);
  std::optional<CollisionResult> checkCollision(
      const AssetPhysical& assetA,
      const AssetPhysical& assetB) const;
};
```

#### Usage Example
```cpp
CollisionHandler collisionHandler{1e-6};
auto result = collisionHandler.checkCollision(assetA, assetB);
if (result) {
  // Collision detected
}
```

#### Thread Safety
**Stateless after construction** — Safe to call from multiple threads.
```

### Collision/CLAUDE.md (After)
```markdown
### CollisionHandler

Orchestrates the GJK→EPA pipeline for narrow-phase collision detection.

**Design Rationale**: Separated from WorldModel to enable independent testing and future
broadphase integration. Returns `std::optional` rather than boolean+output because collision
state is inherently optional—most pairs don't collide.

**Integration**: Called by `WorldModel::updateCollisions()` for each AABB-overlapping pair.
Results are passed to `ContactConstraintFactory` to create solver-ready constraints.

> Query `get_class_members CollisionHandler` for full API details.
```

### CollisionHandler.hpp (After - Enhanced Doxygen)
```cpp
/**
 * @brief Orchestrates GJK and EPA algorithms for collision detection.
 *
 * Provides a unified collision detection interface that:
 * - Runs GJK to detect intersection
 * - If collision detected, runs EPA to compute contact info
 * - Returns std::optional to indicate collision presence
 *
 * This abstraction allows future enhancements (broadphase, CCD) without
 * changing callers.
 *
 * @see GJK
 * @see EPA
 * @see CollisionResult
 * @see ContactConstraintFactory
 *
 * @ticket 0027a_expanding_polytope_algorithm
 * @threadsafe Stateless after construction - safe for concurrent calls
 */
class CollisionHandler {
public:
  /**
   * @brief Construct a collision handler.
   * @param epsilon Numerical tolerance for convergence (default: 1e-6)
   */
  explicit CollisionHandler(double epsilon = 1e-6);

  /**
   * @brief Check for collision between two physical assets.
   *
   * Runs GJK to detect intersection. If collision detected, runs EPA
   * to compute penetration depth, contact normal, and contact manifold.
   *
   * @param assetA First physical asset (includes collision hull and transform)
   * @param assetB Second physical asset
   * @return std::nullopt if no collision, CollisionResult with manifold if collision
   *
   * @code
   * CollisionHandler handler{1e-6};
   * auto result = handler.checkCollision(assetA, assetB);
   * if (result) {
   *   // Process collision...
   * }
   * @endcode
   */
  std::optional<CollisionResult> checkCollision(
      const AssetPhysical& assetA,
      const AssetPhysical& assetB) const;
};
```

---

## Next Steps

1. **Approve this plan** — Review and adjust priorities as needed
2. **Start with highest-value modules** — msd-sim/Physics/Collision first
3. **Iterate** — Enhance Doxygen, regenerate DB, verify MCP queries, then slim CLAUDE.md
4. **Validate** — Test AI assistant navigation with new documentation structure
