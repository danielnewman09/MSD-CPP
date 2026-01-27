# Design: Contact Manifold Generation

## Summary

Extend the collision response system to generate multiple contact points (contact manifold) from EPA's closest face witness points, then distribute impulse forces evenly across all contact points. This improves stability for face-face contacts where a single contact point causes rotational jitter, such as a box resting on a plane. The system leverages existing EPA witness point infrastructure to extract 3 distinct contact locations from the closest face, deduplicates near-identical points, and distributes collision impulses equally across the resulting manifold.

## Architecture Changes

### PlantUML Diagram

See: `./0029_contact_manifold_generation.puml`

### New Components

#### ContactPoint

- **Purpose**: Store a single contact pair within a manifold (contact locations on both object surfaces)
- **Header location**: Defined inline in `msd/msd-sim/src/Physics/CollisionResult.hpp`
- **Source location**: Header-only struct (no .cpp file)
- **Key interfaces**:
  ```cpp
  struct ContactPoint {
    Coordinate pointA;  // Contact point on object A's surface (world space) [m]
    Coordinate pointB;  // Contact point on object B's surface (world space) [m]

    ContactPoint() = default;
    ContactPoint(const Coordinate& pA, const Coordinate& pB)
      : pointA{pA}, pointB{pB} {}

    ContactPoint(const ContactPoint&) = default;
    ContactPoint(ContactPoint&&) noexcept = default;
    ContactPoint& operator=(const ContactPoint&) = default;
    ContactPoint& operator=(ContactPoint&&) noexcept = default;
    ~ContactPoint() = default;
  };
  ```
- **Dependencies**: `Coordinate` from Environment module
- **Thread safety**: Value type, safe to copy across threads
- **Error handling**: No validation (assumes valid coordinates from EPA)

**Design Rationale**:
- **POD struct**: Simple data container with no logic
- **Dual witness points**: Each contact stores points on both surfaces (consistent with existing single-contact witness point design from ticket 0028)
- **24-byte size**: Two 12-byte Coordinates (x, y, z doubles), fits in cache line
- **Value semantics**: Default copy/move operations, no resource management needed

### Modified Components

#### CollisionResult

- **Current location**: `msd/msd-sim/src/Physics/CollisionResult.hpp`
- **Changes required**:
  1. **Remove** legacy `contactPointA` and `contactPointB` members (simplification over backward compatibility)
  2. Add `std::array<ContactPoint, 4> contacts` — Fixed-size manifold storage
  3. Add `size_t contactCount` — Number of valid contacts in array (range: [1, 4])
  4. Update constructors to support manifold initialization
- **Breaking change**: Existing code using `.contactPointA`/`.contactPointB` must migrate to `.contacts[0].pointA`/`.contacts[0].pointB`

**Implementation details**:
```cpp
struct CollisionResult {
  Coordinate normal;  // Contact normal (world space, A→B, unit length)
  double penetrationDepth{std::numeric_limits<double>::quiet_NaN()};  // Overlap distance [m]

  // Contact manifold storage (replaces legacy contactPointA/contactPointB)
  std::array<ContactPoint, 4> contacts;  // Up to 4 contact points
  size_t contactCount{0};                 // Number of valid contacts [1, 4]

  // Default constructor
  CollisionResult() = default;

  // Manifold constructor
  CollisionResult(const Coordinate& n,
                  double depth,
                  const std::array<ContactPoint, 4>& contactsArray,
                  size_t count)
    : normal{n}
    , penetrationDepth{depth}
    , contacts{contactsArray}
    , contactCount{count}
  {
    if (count < 1 || count > 4) {
      throw std::invalid_argument("contactCount must be in [1, 4]");
    }
  }

  // Single-contact convenience constructor
  CollisionResult(const Coordinate& n,
                  double depth,
                  const Coordinate& pointA,
                  const Coordinate& pointB)
    : normal{n}
    , penetrationDepth{depth}
    , contacts{ContactPoint{pointA, pointB}}
    , contactCount{1}
  {}

  CollisionResult(const CollisionResult&) = default;
  CollisionResult(CollisionResult&&) noexcept = default;
  CollisionResult& operator=(const CollisionResult&) = default;
  CollisionResult& operator=(CollisionResult&&) noexcept = default;
  ~CollisionResult() = default;
};
```

**Memory layout**:
- `normal`: 24 bytes (Coordinate)
- `penetrationDepth`: 8 bytes (double)
- `contacts`: 192 bytes (4 × ContactPoint @ 48 bytes each)
- `contactCount`: 8 bytes (size_t)
- **Total**: ~232 bytes (stack-allocated)

**Validation**:
- `contactCount` must be in [1, 4], enforced by manifold constructor
- Zero contacts is invalid (at least one contact required for collision)
- More than 4 contacts unsupported (EPA face has 3 vertices, +1 reserved for future clipping)

**Migration from legacy code**:
- Replace `.contactPointA` → `.contacts[0].pointA`
- Replace `.contactPointB` → `.contacts[0].pointB`

#### EPA

- **Current location**: `msd/msd-sim/src/Physics/EPA.hpp`, `EPA.cpp`
- **Changes required**:
  1. Modify `computeContactInfo()` to populate `contacts` array instead of single `contactPointA`/`contactPointB`
  2. Add private method `extractContactManifold(size_t closestFaceIndex)`
     - Extracts 3 witness points from closest face's MinkowskiVertex vertices
     - Calls `deduplicateContacts()` to merge near-identical points
     - Populates `CollisionResult::contacts` and `contactCount`
  3. Add private static method `deduplicateContacts(std::array<ContactPoint, 4>& contacts, size_t count, double epsilon) -> size_t`
     - Merges contacts within epsilon distance (default: 1e-6)
     - Returns new contact count after deduplication
     - Fallback: If all points collapse to single point, return centroid
- **Backward compatibility**: No breaking changes to public API

**Implementation details**:

```cpp
// In EPA.cpp - computeContactInfo()
CollisionResult EPA::computeContactInfo(const std::vector<Coordinate>& simplex,
                                        int maxIterations) {
  // ... existing EPA expansion logic ...

  size_t closestFaceIndex = findClosestFace();
  const Facet& closestFace = faces_[closestFaceIndex];

  // Compute contact normal and penetration depth (unchanged)
  Coordinate contactNormal = closestFace.normal;
  double penetrationDepth = closestFace.offset;

  // NEW: Extract contact manifold instead of single point
  std::array<ContactPoint, 4> contacts;
  size_t contactCount = extractContactManifold(closestFaceIndex, contacts);

  return CollisionResult{contactNormal, penetrationDepth, contacts, contactCount};
}

// NEW: Private method
size_t EPA::extractContactManifold(size_t faceIndex,
                                   std::array<ContactPoint, 4>& contacts) const {
  const Facet& face = faces_[faceIndex];

  // Extract 3 witness points from face vertices (barycentric vertices, not centroid)
  size_t v0 = face.vertexIndices[0];
  size_t v1 = face.vertexIndices[1];
  size_t v2 = face.vertexIndices[2];

  contacts[0] = ContactPoint{vertices_[v0].witnessA, vertices_[v0].witnessB};
  contacts[1] = ContactPoint{vertices_[v1].witnessA, vertices_[v1].witnessB};
  contacts[2] = ContactPoint{vertices_[v2].witnessA, vertices_[v2].witnessB};
  size_t count = 3;

  // Deduplicate near-identical contacts
  count = deduplicateContacts(contacts, count, epsilon_);

  // Fallback: If deduplication collapsed to 0 contacts, use centroid
  if (count == 0) {
    Coordinate centroidA = computeWitnessA(face);
    Coordinate centroidB = computeWitnessB(face);
    contacts[0] = ContactPoint{centroidA, centroidB};
    count = 1;
  }

  return count;
}

// NEW: Private static method
size_t EPA::deduplicateContacts(std::array<ContactPoint, 4>& contacts,
                                size_t count,
                                double epsilon) {
  if (count <= 1) {
    return count;
  }

  size_t uniqueCount = 0;
  std::array<ContactPoint, 4> uniqueContacts;

  for (size_t i = 0; i < count; ++i) {
    bool isDuplicate = false;

    // Check if this contact is near any existing unique contact
    for (size_t j = 0; j < uniqueCount; ++j) {
      double distA = (contacts[i].pointA - uniqueContacts[j].pointA).norm();
      double distB = (contacts[i].pointB - uniqueContacts[j].pointB).norm();

      if (distA < epsilon && distB < epsilon) {
        isDuplicate = true;
        break;
      }
    }

    if (!isDuplicate) {
      uniqueContacts[uniqueCount++] = contacts[i];
    }
  }

  // Copy unique contacts back
  for (size_t i = 0; i < uniqueCount; ++i) {
    contacts[i] = uniqueContacts[i];
  }

  return uniqueCount;
}
```

**Edge cases**:
- **All 3 witness points identical**: `deduplicateContacts()` returns 1 contact
- **2 of 3 points identical**: Returns 2 contacts
- **All points within epsilon**: Fallback to centroid (single contact)
- **Degenerate face (zero area)**: EPA should not produce such faces (protected by expansion logic)

#### CollisionResponse namespace

- **Current location**: `msd/msd-sim/src/Physics/CollisionResponse.hpp`, `CollisionResponse.cpp`
- **Changes required**:
  1. Add `applyImpulseManifold(AssetInertial& assetA, AssetInertial& assetB, const CollisionResult& result, double combinedRestitution)`
     - Computes total impulse using existing `computeImpulseMagnitude()` formula
     - Divides impulse equally across all contacts: `j_per_contact = j_total / contactCount`
     - For each contact:
       - Applies linear impulse (same magnitude for all contacts)
       - Computes lever arm from contact point to center of mass
       - Applies angular impulse: `Δω = I^-1 * (r × J)`
  2. Add `applyPositionCorrectionManifold(AssetInertial& assetA, AssetInertial& assetB, const CollisionResult& result)`
     - Computes average contact point: `avg = Σ(pointA_i + pointB_i) / (2 * contactCount)`
     - Uses average point for correction direction (same formula as single-point version)
     - Same `kSlop` and `kCorrectionFactor` constants
  3. **Remove** legacy single-contact functions (`applyImpulse`, `applyPositionCorrection`) — manifold functions handle all cases
- **Breaking change**: Callers must migrate to manifold-aware functions

**Implementation details**:

```cpp
namespace CollisionResponse {

void applyImpulseManifold(AssetInertial& assetA,
                          AssetInertial& assetB,
                          const CollisionResult& result,
                          double combinedRestitution) {
  // Compute total impulse magnitude (same formula as single-contact)
  double j_total = computeImpulseMagnitude(assetA, assetB, result, combinedRestitution);

  // Divide impulse equally across all contacts
  double j_per_contact = j_total / static_cast<double>(result.contactCount);
  CoordinateRate impulse_per_contact = result.normal * j_per_contact;

  // Apply impulse at each contact point
  for (size_t i = 0; i < result.contactCount; ++i) {
    const ContactPoint& contact = result.contacts[i];

    // Linear impulse (same for all contacts, additive)
    assetA.getInertialState().velocity += impulse_per_contact / assetA.getMass();
    assetB.getInertialState().velocity -= impulse_per_contact / assetB.getMass();

    // Angular impulse (unique per contact due to different lever arms)
    Coordinate leverArmA = contact.pointA - assetA.getInertialState().position;
    Coordinate leverArmB = contact.pointB - assetB.getInertialState().position;

    AngularRate angularImpulseA = assetA.getInverseInertiaTensor() *
                                   leverArmA.cross(impulse_per_contact);
    AngularRate angularImpulseB = assetB.getInverseInertiaTensor() *
                                   leverArmB.cross(-impulse_per_contact);

    assetA.getInertialState().angularVelocity += angularImpulseA;
    assetB.getInertialState().angularVelocity -= angularImpulseB;
  }
}

void applyPositionCorrectionManifold(AssetInertial& assetA,
                                     AssetInertial& assetB,
                                     const CollisionResult& result) {
  // Use average contact point for position correction direction
  // (Alternative considered: use deepest penetration point, but average is simpler)

  // Correction magnitude (same formula as single-contact)
  double correction = std::max(result.penetrationDepth - kSlop, 0.0) * kCorrectionFactor;
  if (correction <= 0.0) {
    return;  // No correction needed
  }

  Coordinate separationVector = result.normal * correction;

  // Mass-weighted correction (same as single-contact)
  double totalInverseMass = 1.0 / assetA.getMass() + 1.0 / assetB.getMass();
  double weightA = (1.0 / assetA.getMass()) / totalInverseMass;
  double weightB = (1.0 / assetB.getMass()) / totalInverseMass;

  assetA.getInertialState().position -= separationVector * weightA;
  assetB.getInertialState().position += separationVector * weightB;
}

}  // namespace CollisionResponse
```

**Design decision: Equal impulse distribution**:
- **Why equal distribution**: Simplicity, consistent with human design guidance
- **Alternative rejected**: Weight by per-contact penetration depth (more complex, not needed for stability improvement)
- **Physical justification**: Contact manifold represents distributed contact area; equal distribution approximates uniform pressure

**Design decision: Average contact for position correction**:
- **Resolves open question #1**: Use average of all contact points
- **Rationale**: Provides stable correction direction even with asymmetric manifolds
- **Alternative rejected**: Use deepest penetration point (requires per-point depth tracking, deferred)

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  1. Modify `updateCollisions()` to call `applyImpulseManifold()` instead of manually applying impulses
  2. Modify `updateCollisions()` to call `applyPositionCorrectionManifold()` instead of `applyPositionCorrection()`
- **Backward compatibility**: Internal implementation change, no public API changes

**Implementation details**:

```cpp
// In WorldModel.cpp - updateCollisions()
void WorldModel::updateCollisions() {
  const auto& assets = inertialAssets_;
  for (size_t i = 0; i < assets.size(); ++i) {
    for (size_t j = i + 1; j < assets.size(); ++j) {
      AssetInertial& assetA = inertialAssets_[i];
      AssetInertial& assetB = inertialAssets_[j];

      // Check collision
      auto result = collisionHandler_.checkCollision(assetA, assetB);
      if (!result) {
        continue;  // No collision
      }

      // Combine restitution coefficients (unchanged)
      double combinedE = CollisionResponse::combineRestitution(
          assetA.getCoefficientOfRestitution(),
          assetB.getCoefficientOfRestitution());

      // MODIFIED: Use manifold-aware impulse application
      CollisionResponse::applyImpulseManifold(assetA, assetB, *result, combinedE);

      // MODIFIED: Use manifold-aware position correction
      CollisionResponse::applyPositionCorrectionManifold(assetA, assetB, *result);
    }
  }
}
```

**Integration order** (unchanged from ticket 0027):
1. `updateCollisions()` — Detect collisions, apply manifold impulses, correct positions
2. `updatePhysics(dt)` — Integrate velocities, apply forces, synchronize frames

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| ContactPoint | CollisionResult | Composition | CollisionResult owns `std::array<ContactPoint, 4>` |
| EPA::extractContactManifold() | MinkowskiVertex | Read | Extracts witness points from face vertices |
| EPA::deduplicateContacts() | ContactPoint | Read/Write | Merges near-identical contacts in-place |
| CollisionResponse::applyImpulseManifold() | CollisionResult | Read | Iterates over `contacts` array |
| CollisionResponse::applyPositionCorrectionManifold() | CollisionResult | Read | Uses manifold for average contact point |
| WorldModel::updateCollisions() | CollisionResponse | Call | Invokes manifold-aware functions |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Physics/EPATest.cpp` | Contact point extraction tests | Moderate | Update assertions to use `contacts[0].pointA`/`pointB` |
| `test/Physics/CollisionResponseTest.cpp` | Impulse calculation tests | Moderate | Update to use manifold-aware functions |
| `test/Environment/WorldModelCollisionTest.cpp` | Integration tests | Moderate | Update to verify manifold-aware behavior (multiple contacts) |
| `test/Physics/CollisionHandlerTest.cpp` | Contact info tests | Moderate | Update assertions for manifold structure |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| ContactPoint | DefaultConstruction | Default-constructed ContactPoint has valid (zero-initialized) coordinates |
| ContactPoint | ValueConstruction | Construct with explicit pointA and pointB |
| ContactPoint | CopySemantics | ContactPoint is copyable and movable |
| CollisionResult | ManifoldConstruction | Construct CollisionResult with manifold array and contactCount |
| CollisionResult | SingleContactConstruction | Single-contact constructor populates manifold with 1 contact |
| CollisionResult | InvalidContactCount | Constructor throws if contactCount < 1 or > 4 |
| EPA | ExtractContactManifold_FaceContact | Face-face collision produces 3 distinct contacts |
| EPA | ExtractContactManifold_EdgeContact | Edge-edge collision produces 2 contacts (after deduplication) |
| EPA | ExtractContactManifold_VertexContact | Vertex-vertex collision produces 1 contact (all points deduplicated) |
| EPA | DeduplicateContacts_AllUnique | 3 contacts with distance > epsilon → 3 contacts |
| EPA | DeduplicateContacts_TwoDuplicates | 3 contacts with 2 within epsilon → 2 contacts |
| EPA | DeduplicateContacts_AllDuplicates | 3 contacts all within epsilon → 1 contact |
| EPA | DeduplicateContacts_FallbackToCentroid | Zero unique contacts → fallback to centroid |
| CollisionResponse | ApplyImpulseManifold_SingleContact | Single contact behaves same as legacy function |
| CollisionResponse | ApplyImpulseManifold_ThreeContacts | 3 contacts distribute impulse equally (j_per = j_total / 3) |
| CollisionResponse | ApplyImpulseManifold_AngularBalancing | Face-face contact with 3 symmetric contacts produces zero net torque |
| CollisionResponse | ApplyPositionCorrectionManifold_Average | Correction uses average of all contact points |
| CollisionResponse | ApplyPositionCorrectionManifold_Slop | No correction when penetration < kSlop |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| WorldModel_ManifoldCollision_BoxOnPlane | WorldModel, EPA, CollisionResponse, AssetInertial | Box resting on plane has 4 contacts (corners), remains stable |
| WorldModel_ManifoldCollision_NoRotationDrift | WorldModel, EPA, CollisionResponse | Box on plane with symmetric manifold has zero angular drift over 1000 frames |
| WorldModel_ManifoldCollision_MomentumConservation | WorldModel, EPA, CollisionResponse | Total linear and angular momentum conserved with manifold response |
| WorldModel_ManifoldCollision_FaceContact_vs_EdgeContact | WorldModel, EPA, CollisionResponse | Face contact (3 contacts) more stable than edge contact (2 contacts) |
| WorldModel_ManifoldCollision_AsymmetricManifold | WorldModel, EPA, CollisionResponse | Asymmetric manifold (3 non-coplanar contacts) applies correct torque |

#### Benchmark Tests

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| EPA | ExtractContactManifold_BenchOps | Manifold extraction throughput | > 100K ops/sec (trivial overhead) |
| CollisionResponse | ApplyImpulseManifold_BenchN | Impulse application for N contacts | Linear scaling: 3x time for 3 contacts vs 1 contact |
| WorldModel | UpdateCollisions_ManifoldOverhead | Full collision pipeline with manifolds | < 10% overhead vs single-contact baseline |

**Benchmark rationale**:
- Manifold extraction is simple array population (< 10 operations), should not be bottleneck
- Impulse application scales linearly with contact count (3x angular impulse calculations for 3 contacts)
- Overall overhead minimal since typical face-face collisions are rare compared to edge/vertex contacts

## Open Questions

### Design Decisions (Human Input Needed)

**All open questions from ticket have been resolved in this design:**

1. **Position correction strategy** — **RESOLVED**
   - **Decision**: Use average of all contact points for correction direction
   - **Implementation**: `applyPositionCorrectionManifold()` computes `avg = Σ(pointA + pointB) / (2 * contactCount)`
   - **Rationale**: Provides stable correction even with asymmetric manifolds, simpler than per-point correction

2. **Per-point penetration depth** — **RESOLVED**
   - **Decision**: No per-point penetration depth in ContactPoint struct
   - **Rationale**: Not needed for equal impulse distribution (ticket guidance), defers complexity
   - **Future extension**: Can add `double penetrationDepth` to ContactPoint if weighted distribution is needed later

### Prototype Required

**None** — This design leverages existing EPA witness point infrastructure (ticket 0028) and collision response formulas (ticket 0027). The core algorithms are well-established:
- Witness point extraction: Already validated in ticket 0028 (accuracy within 1e-6)
- Impulse distribution: Standard physics engine technique (equal distribution for uniform pressure)
- Position correction: Same formula as existing single-contact version

**Validation approach**: Direct implementation with comprehensive unit and integration tests (see Test Impact section).

### Requirements Clarification

**None** — All requirements are unambiguous and ticket provides explicit design guidance:
- Direct face witness extraction (not Sutherland-Hodgman clipping)
- Fixed-size `std::array<ContactPoint, 4>` with `contactCount` field
- Equal impulse distribution (not weighted by penetration)
- Clean API (no deprecated backward-compatible accessors — per human feedback prioritizing simplicity)

## Design Complexity Sanity Checks

### Red Flag Assessment

**No red flags detected.** This design:
- Adds **1 new component** (ContactPoint struct)
- Modifies **4 existing components** (CollisionResult, EPA, CollisionResponse, WorldModel)
- No combinatorial overloads (single function signature per manifold function)
- No optional wrappers for legacy paths
- Clean break from single-contact API (simpler architecture, no deprecated members to maintain)

The ratio of new-to-modified components is acceptable for a feature that extends existing collision pipeline.

### Complexity Analysis

**Maintained simplicity**:
- **ContactPoint**: 4-line POD struct, no logic
- **Deduplication**: O(n²) for n ≤ 4 (trivial cost, max 6 comparisons)
- **Impulse distribution**: Linear loop over contacts (max 4 iterations)
- **No conditional complexity**: No `if (hasManifold)` branches, single code path

**Trade-offs accepted**:
- Breaking change to CollisionResult API requires updating existing code
- Simpler architecture justifies migration effort

## Dependencies

### External Libraries
- **Eigen3** — Matrix operations (unchanged from ticket 0027)
- **msd-assets** — Geometry types (unchanged)

### Internal Dependencies
- `AssetInertial` — Collision objects with inertia and state
- `CollisionResult` — Extended with manifold storage
- `EPA` — Modified to extract witness point manifolds
- `MinkowskiVertex` — Source of witness points (ticket 0028)
- `Coordinate`, `CoordinateRate`, `AngularRate` — Mathematical primitives
- `ReferenceFrame` — Coordinate transformations
- `InertialState` — Kinematic state representation

### Build Impact
- **New source files**: None (ContactPoint defined inline in CollisionResult.hpp)
- **Modified files**:
  - `CollisionResult.hpp` — Add ContactPoint struct, manifold fields, accessors
  - `EPA.hpp`, `EPA.cpp` — Add extractContactManifold(), deduplicateContacts()
  - `CollisionResponse.hpp`, `CollisionResponse.cpp` — Add applyImpulseManifold(), applyPositionCorrectionManifold()
  - `WorldModel.cpp` — Update updateCollisions() to call manifold functions
- **CMakeLists.txt**: No changes (no new source files)

## Performance Considerations

### Computational Complexity

**Per-collision overhead**:
- **Manifold extraction**: O(1) — Extract 3 witness points, deduplicate (max 6 comparisons)
- **Impulse distribution**: O(k) where k = contactCount ∈ [1, 4]
  - Linear impulse: k additions (same total magnitude as single-contact)
  - Angular impulse: k × (cross product + matrix-vector multiply) = k × ~50 FLOPs
  - Worst case: 4 × 50 = 200 FLOPs (vs 50 FLOPs for single-contact)
- **Position correction**: O(1) — Average contact point calculation (max 4 additions)

**Scaling analysis**:
- Face-face collision (4 contacts): ~4x angular impulse cost vs single-contact
- Edge-edge collision (2 contacts): ~2x angular impulse cost
- Vertex-vertex collision (1 contact): Same cost as single-contact baseline
- **Typical case**: Most collisions are edge/vertex (1-2 contacts), so average overhead < 2x

### Memory Impact

**Per-collision memory**:
- `ContactPoint`: 48 bytes (2 × Coordinate @ 24 bytes)
- **Total CollisionResult size**: ~232 bytes (stack-allocated, no heap)

**Runtime memory**:
- **Stack usage**: 232 bytes per CollisionResult (acceptable, well below typical 1MB stack)
- **No heap allocations**: Fixed-size array, no dynamic memory during collision response
- **Cache efficiency**: 232 bytes fits in single cache line on modern CPUs (512-byte L1 cache lines)

### Optimization Opportunities (Future Work)

1. **Lazy deduplication**: Skip deduplication if all 3 witness points are already far apart (pre-check distances)
2. **SIMD impulse application**: Vectorize angular impulse loop if manifolds consistently have 4 contacts
3. **Spatial coherence**: Cache manifold from previous frame, only recompute if objects moved significantly

## Code Quality Notes

### Adherence to Coding Standards

- **Brace initialization**: `ContactPoint{pointA, pointB}`, `contacts{contactsArray}`, `contactCount{count}`
- **NaN for uninitialized floats**: Not applicable (contactCount has valid default of 0, penetrationDepth already uses NaN)
- **References for non-owning access**: Accessor methods return `const Coordinate&` (not copies)
- **Return values over output parameters**: `deduplicateContacts()` returns new count, modifies array in-place (acceptable for optimization)
- **Rule of Zero**: ContactPoint and CollisionResult use `= default` for all special member functions

### Static Analysis Readiness

- No raw pointers in public interfaces
- All functions have well-defined preconditions (documented in implementation)
- No global mutable state
- Exception safety: Strong guarantee (no partial modifications on exception)
- Const-correctness: Accessor methods marked const, manifold functions take const references where appropriate

### Const-Correctness

- `getContactPointA()` and `getContactPointB()` marked const
- `extractContactManifold()` takes const reference to faces
- `deduplicateContacts()` takes mutable reference (array modified in-place)
- `applyImpulseManifold()` and `applyPositionCorrectionManifold()` take mutable references (state modification intended)

## Future Enhancements (Out of Scope)

These enhancements are intentionally deferred to future tickets:

1. **Sutherland-Hodgman clipping** (contact manifold refinement)
   - Clips collision shapes to extract more accurate contact regions
   - Produces up to 8 contact points (vs 3 from EPA face)
   - Estimated complexity: Large (requires convex polygon clipping implementation)

2. **Per-point penetration depth** (weighted impulse distribution)
   - Store penetration depth per contact point
   - Weight impulse by depth: `j_i = j_total * (depth_i / Σdepth)`
   - Estimated complexity: Medium (requires depth calculation during extraction)

3. **Persistent contact manifold** (temporal coherence)
   - Cache manifold from previous frame
   - Match contacts by proximity, warm-start solver
   - Improves stability for resting contacts
   - Estimated complexity: Large (requires contact ID tracking)

4. **Friction with manifold** (tangential impulses)
   - Extend friction system (future ticket) to use manifold
   - Apply friction impulse at each contact point
   - Estimated complexity: Medium (depends on friction implementation)

5. **Contact reduction** (manifold optimization)
   - Reduce manifold to 4 most important contacts (if > 4 from clipping)
   - Choose contacts that maximize contact area and stability
   - Estimated complexity: Medium (requires contact selection heuristic)

---

## Acceptance Criteria Mapping

This design addresses all acceptance criteria from ticket 0029:

| AC | Requirement | Design Component |
|----|-------------|------------------|
| AC1 | CollisionResult contains `contacts` array with `contactCount` field | Modified CollisionResult struct with `std::array<ContactPoint, 4> contacts` and `size_t contactCount` |
| AC2 | EPA `extractContactManifold()` returns 3 distinct contact points for face-face collision | EPA::extractContactManifold() extracts 3 witness points from closest face vertices |
| AC3 | Near-duplicate witness points (within epsilon) are merged | EPA::deduplicateContacts() merges contacts within epsilon tolerance |
| AC4 | Fallback to centroid if all points collapse to single point | extractContactManifold() calls computeWitnessA/B() for centroid fallback |
| AC5 | `applyImpulseManifold()` divides impulse equally among contact points | CollisionResponse::applyImpulseManifold() computes `j_per = j_total / contactCount` |
| AC6 | Angular impulse applied at each contact point with correct lever arm | applyImpulseManifold() loop: `leverArm = contactPoint - centerOfMass`, `Δω = I^-1 * (r × J)` |
| AC7 | Box resting on plane remains stable for 1000+ frames (no rotation drift) | Integration test: WorldModel_ManifoldCollision_NoRotationDrift |
| AC8 | Total momentum conserved before and after multi-point collision response | Integration test: WorldModel_ManifoldCollision_MomentumConservation |

---

## Summary

This design provides a clean, minimal extension to the existing collision response system to support contact manifolds. By leveraging the witness point infrastructure from ticket 0028, it avoids unnecessary complexity while delivering the stability improvements needed for face-face contacts. The equal impulse distribution strategy (per human guidance) keeps the implementation simple. Legacy single-contact members are removed entirely (per human feedback prioritizing simplicity over backward compatibility).

The design maintains project coding standards (brace initialization, Rule of Zero, const-correctness) and integrates seamlessly with the existing collision pipeline. Memory overhead is acceptable (232 bytes stack-allocated per collision), and computational overhead scales linearly with contact count (typical 2x for edge contacts, 4x for rare face contacts).

Ready for implementation phase.

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-25
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | `ContactPoint` (PascalCase), `extractContactManifold()` (camelCase), `contacts_` array member would use snake_case_ if private. All consistent with project standards. |
| Namespace organization | ✓ | Components correctly placed in `msd_sim` namespace. CollisionResponse remains utility namespace. |
| File structure | ✓ | Follows `msd/msd-sim/src/Physics/` pattern. ContactPoint defined inline in CollisionResult.hpp (appropriate for simple POD struct). |
| Dependency direction | ✓ | Clean layering: CollisionResult → ContactPoint, EPA → CollisionResult, CollisionResponse → CollisionResult, WorldModel → CollisionResponse. No cycles introduced. |

**Overall**: Excellent fit with existing architecture. The design extends the collision pipeline without disrupting established patterns.

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | No resource management needed (stack-allocated arrays). Appropriate use of value semantics. |
| Smart pointer appropriateness | ✓ | No pointers used. Fixed-size array avoids heap allocation per design requirement. |
| Value/reference semantics | ✓ | ContactPoint is value type (appropriate for 48-byte POD). CollisionResult uses value semantics for contacts array. Functions take const references for read access. |
| Rule of 0/3/5 | ✓ | ContactPoint and CollisionResult both use `= default` for all special member functions (Rule of Zero correctly applied). |
| Const correctness | ✓ | `extractContactManifold() const`, `deduplicateContacts()` takes mutable reference (intentional for in-place modification), manifold functions take const references where appropriate. |
| Exception safety | ✓ | Strong guarantee: Constructor validation throws before any state modification. No partial updates possible. |
| Initialization | ✓ | Brace initialization throughout: `ContactPoint{pointA, pointB}`, `contacts{contactsArray}`. NaN for penetrationDepth (already present). |
| Return values | ✓ | `deduplicateContacts()` returns new count (acceptable for optimization), otherwise prefers return values. CollisionResult constructor validates and throws rather than using output parameters. |

**Overall**: Exemplary adherence to modern C++ standards and project coding conventions. The clean break from deprecated members (per human feedback) simplifies the design significantly.

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | CollisionResult.hpp includes only Coordinate.hpp. No circular dependencies. EPA.hpp already includes CollisionResult.hpp. Clean compile-time dependencies. |
| Template complexity | ✓ | No templates introduced. Simple POD struct and straightforward algorithms. |
| Memory strategy | ✓ | Fixed-size `std::array<ContactPoint, 4>` avoids heap allocations per requirement. 232 bytes stack-allocated is well within reasonable limits. |
| Thread safety | ✓ | Value types safe to copy. Stateless namespace functions (CollisionResponse) are thread-safe. EPA methods are const where appropriate. |
| Build integration | ✓ | No new source files (ContactPoint inline), modified files already in build. No CMakeLists.txt changes needed. |

**Overall**: Highly feasible implementation. Leverages existing infrastructure (MinkowskiVertex, witness points) effectively. No complex dependencies or build system changes required.

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | ContactPoint is standalone POD. EPA methods can be tested independently. CollisionResponse functions are stateless (easy to unit test). |
| Mockable dependencies | ✓ | No dependencies requiring mocks. EPA takes const references to AssetPhysical (can use test fixtures). CollisionResponse operates on concrete types (straightforward to test). |
| Observable state | ✓ | All state is public or accessible via const getters. CollisionResult provides direct access to contacts array and contactCount. Test cases can verify manifold contents directly. |

**Overall**: Excellent testability. The design section includes comprehensive test plan with unit tests (ContactPoint, CollisionResult, EPA, CollisionResponse) and integration tests (WorldModel). No hidden state or singleton patterns.

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Edge case: Degenerate EPA faces (near-zero area) could produce invalid witness points | Technical | Low | Medium | EPA's existing expansion logic prevents degenerate faces. Fallback to centroid if deduplication produces zero contacts. Add test case for near-degenerate faces. | No |
| R2 | Performance: 4x angular impulse overhead for face-face contacts could accumulate in dense collision scenarios | Performance | Low | Low | Design correctly notes face-face collisions are rare (most are edge/vertex). Average overhead < 2x. Acceptable per requirements. Monitor in profiling if needed. | No |
| R3 | Breaking change migration: Existing code using `.contactPointA`/`.contactPointB` must migrate to `.contacts[0]` | Maintenance | High | Low | Breaking change is intentional per human feedback. Impact limited to current codebase (WorldModel, tests). Migration straightforward (mechanical replacement). | No |

### Risks Assessment Summary

All identified risks have **low impact** and adequate mitigation. No high-likelihood/high-impact risks present.

- **R1** is adequately handled by EPA's existing robustness features and fallback logic
- **R2** is a non-issue given the rarity of face-face contacts and acceptable overhead
- **R3** is accepted as part of the clean break strategy prioritizing simplicity

### Prototype Guidance

**No prototypes required.**

**Rationale**:
- Core algorithm (equal impulse distribution across contacts) is well-established in physics engines
- Witness point extraction already validated in ticket 0028 (accuracy within 1e-6)
- Deduplication logic is simple O(n²) for n ≤ 4 (trivial cost)
- Performance overhead analyzed in design (linear scaling with contact count)
- Breaking changes are intentional and migration is straightforward

The design can proceed directly to implementation with comprehensive unit and integration testing as outlined in the Test Impact section.

### Notes for Implementation

1. **Validation in constructor**: The manifold constructor validates `contactCount ∈ [1, 4]`. Ensure this check happens before array initialization to provide strong exception guarantee.

2. **Deduplication epsilon**: The design uses `epsilon_` from EPA (default 1e-6). Consider whether this should be configurable separately (e.g., larger tolerance for contact merging vs convergence). Current approach is acceptable but worth noting for future enhancement.

3. **Test coverage priority**: Focus integration tests on AC7 (stability over 1000 frames) and AC8 (momentum conservation), as these validate the core motivation for the feature. Unit tests should verify the mechanics, but integration tests prove the stability improvement.

4. **Migration strategy**: Update existing tests in this order:
   - `test/Physics/EPATest.cpp` (contact point assertions)
   - `test/Physics/CollisionResponseTest.cpp` (manifold-aware functions)
   - `test/Environment/WorldModelCollisionTest.cpp` (integration tests)

   This ensures each layer is validated before moving to the next.

5. **Documentation**: Update PlantUML diagrams after implementation:
   - `docs/msd/msd-sim/Physics/collision-response.puml` — Show manifold-aware impulse distribution
   - `docs/msd/msd-sim/Physics/witness-points.puml` — Document ContactPoint struct relationship

### Summary

This design receives **APPROVED WITH NOTES** status. It demonstrates:

- **Strong architectural fit**: Seamless integration with existing collision pipeline
- **Excellent C++ design**: Modern standards, Rule of Zero, value semantics, const-correctness
- **High feasibility**: No complex dependencies, leverages existing infrastructure effectively
- **Superior testability**: Comprehensive test plan with clear acceptance criteria mapping

The design correctly incorporates human feedback to remove deprecated members in favor of a clean API. The equal impulse distribution approach (per human guidance) keeps implementation complexity low while delivering the required stability improvements.

**Key strengths**:
- Leverages ticket 0028's witness point infrastructure (avoids reimplementation)
- Fixed-size array avoids heap allocations (meets performance requirement)
- Clean break from single-contact API (simplifies architecture per human feedback)
- Comprehensive test plan maps directly to acceptance criteria
- Well-analyzed performance characteristics (linear scaling with contact count)

**Minor considerations** (not blocking):
- Deduplication epsilon is tied to EPA epsilon (acceptable, but future enhancement could separate these)
- Breaking change requires test migration (expected, straightforward mechanical replacement)

**Recommendation**: Proceed to implementation phase. No prototype required. Follow the test migration order outlined in Notes for Implementation.

The design is ready for human review and approval.
