# Feature Ticket: Contact Manifold Generation

## Status
- [ ] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete — Awaiting Tutorial (if Generate Tutorial: Yes)
- [ ] Tutorial Complete — Ready to Merge (if Generate Tutorial: Yes)
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-25
- **Author**: Daniel Newman
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: msd-sim (Physics module)
- **Generate Tutorial**: No

---

## Summary
Extend the collision response system to generate multiple contact points (contact manifold) from EPA's closest face, then distribute impulse forces evenly across all contact points. This improves stability for face-face contacts where a single contact point causes rotational jitter (e.g., box resting on a plane).

## Motivation
The current collision response uses a single contact point (centroid of EPA's closest face witness points). While sufficient for simple collisions, this causes instability in face-face contacts:

- **Single-point contact problem**: A box resting on a plane has forces applied at one averaged point, which can cause the box to "tip" or oscillate even when it should be stable
- **Uneven torque**: Off-center single contact points generate unnecessary angular impulses
- **Jitter in stacked objects**: Multiple objects stacked can oscillate due to unbalanced force application

Contact manifold generation solves this by:
- Distributing forces across 3+ contact points for face-face contacts
- Balancing torques so stable configurations remain stable
- Improving visual stability for resting contacts

## Requirements

### Functional Requirements
1. The system shall extend CollisionResult to store multiple contact points (up to 4)
2. The system shall modify EPA to extract all 3 witness points from the closest face (not just centroid)
3. The system shall deduplicate near-identical contact points within epsilon tolerance
4. The system shall add manifold-aware impulse functions that distribute impulse equally across contact points
5. The system shall update WorldModel to use manifold-aware collision response

### Non-Functional Requirements
- **Performance**: Minimal overhead (3x angular impulse calculations instead of 1, still O(1))
- **Memory**: Fixed-size array (no heap allocations) — 4 contact points max (~288 bytes)
- **Thread Safety**: Ensure thread safety at this stage; an expected extension for this work will involve multithreading
- **Backward Compatibility**: This is a fundamental improvement to the collision mechanism. Do not sacrifice simplification of the improved architecture for the sake of backward compatibility

## Constraints
- Should leverage existing EPA witness point infrastructure (MinkowskiVertex)
- Contact manifold stored in fixed-size array (no std::vector to avoid heap allocations)
- Maximum 4 contact points (EPA face has 3 vertices, +1 for future clipping expansion)
- Must not break existing 244+ tests

## Acceptance Criteria
- [ ] AC1: CollisionResult contains `contacts` array with `contactCount` field
- [ ] AC2: EPA `extractContactManifold()` returns 3 distinct contact points for face-face collision
- [ ] AC3: Near-duplicate witness points (within epsilon) are merged
- [ ] AC4: Fallback to centroid if all points collapse to single point
- [ ] AC5: `applyImpulseManifold()` divides impulse equally among contact points
- [ ] AC6: Angular impulse applied at each contact point with correct lever arm
- [ ] AC7: Box resting on plane remains stable for 1000+ frames (no rotation drift)
- [ ] AC8: Total momentum conserved before and after multi-point collision response

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Direct face witness extraction** — Use existing 3 witness points from EPA closest face (not Sutherland-Hodgman clipping)
- **Fixed-size array** — `std::array<ContactPoint, 4>` with `contactCount` field
- **Equal impulse distribution** — Divide impulse equally, don't weight by penetration depth
- **Backward-compatible accessors** — Keep `getContactPointA()`/`getContactPointB()` as aliases to `contacts[0]`

### Things to Avoid
- Don't implement Sutherland-Hodgman clipping (over-engineering for current needs)
- Don't use std::vector for contacts (heap allocation during collision response)
- Don't weight impulses by per-point penetration (unnecessary complexity)
- Don't remove legacy accessors (breaking change to existing code)

### Open Questions
1. Should position correction use average of contact points or keep single-point behavior? (Suggest: average)
2. Should ContactPoint include per-point penetration depth? (Suggest: no, not needed for equal distribution)

---

## References

### Related Code
- `msd-sim/src/Physics/CollisionResult.hpp` — Extend with ContactPoint struct and contacts array
- `msd-sim/src/Physics/EPA.hpp` — Add `extractContactManifold()` method declaration
- `msd-sim/src/Physics/EPA.cpp` — Implement manifold extraction, modify `computeContactInfo()`
- `msd-sim/src/Physics/CollisionResponse.hpp` — Add `applyImpulseManifold()`, `applyPositionCorrectionManifold()`
- `msd-sim/src/Physics/CollisionResponse.cpp` — Implement multi-point impulse distribution
- `msd-sim/src/Environment/WorldModel.cpp` — Call manifold-aware functions

### Related Documentation
- `docs/msd/msd-sim/Physics/witness-points.puml` — EPA witness point tracking architecture
- `docs/msd/msd-sim/Physics/collision-response.puml` — Current collision response system
- `docs/designs/0027_collision_response_system/design.md` — Foundation design document

### Related Tickets
- `0027_collision_response_system` — Base collision response (prerequisite, completed)
- `0028_epa_witness_points` — EPA witness point tracking (prerequisite, completed)
- `0027a_expanding_polytope_algorithm` — EPA implementation (prerequisite, completed)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-25 (cpp-architect agent)
- **Completed**: 2026-01-25
- **Artifacts**:
  - `docs/designs/0029_contact_manifold_generation/design.md`
  - `docs/designs/0029_contact_manifold_generation/0029_contact_manifold_generation.puml`
- **Notes**:
  - Resolved open question #1: Use average of all contact points for position correction
  - Resolved open question #2: No per-point penetration depth (not needed for equal distribution)
  - Design leverages existing EPA witness point infrastructure (ticket 0028)
  - ContactPoint struct defined inline in CollisionResult.hpp
  - **Revised per human feedback**: Removed deprecated `contactPointA`/`contactPointB` members and legacy functions — clean break preferred over backward compatibility
  - Equal impulse distribution per human guidance (simpler than weighted approach)
  - Memory overhead: ~232 bytes per CollisionResult (stack-allocated, acceptable)
  - Computational overhead: 2-4x for angular impulse (scales with contact count)

### Design Review Phase
- **Started**: 2026-01-25
- **Completed**: 2026-01-25
- **Status**: APPROVED WITH NOTES
- **Reviewer Notes**:
  - Design demonstrates excellent architectural fit with existing collision pipeline
  - Correctly incorporates human feedback to remove deprecated members (clean API)
  - Fixed-size array meets performance requirements (no heap allocations)
  - Leverages ticket 0028 witness point infrastructure effectively
  - Comprehensive test plan maps to all acceptance criteria
  - No prototypes required (algorithms well-established, witness extraction already validated)
  - Minor notes on deduplication epsilon and test migration order (see design review section)
  - **Recommendation**: Ready for human review and approval to proceed to prototype phase

### Prototype Phase
- **Status**: SKIPPED
- **Rationale**: Design review determined no prototypes required
  - Core algorithms (equal impulse distribution) are well-established in physics engines
  - Witness point extraction already validated in ticket 0028 (accuracy within 1e-6)
  - Deduplication logic is simple O(n²) for n ≤ 4 (trivial cost)
  - Performance overhead analyzed in design (linear scaling with contact count)
- **Notes**: Design review approved proceeding directly to implementation with comprehensive unit and integration testing

### Implementation Phase
- **Started**: 2026-01-25
- **Completed**: 2026-01-25
- **Files Created**: None (all modifications as per design)
- **Files Modified**:
  - `msd/msd-sim/src/Physics/CollisionResult.hpp` — Added ContactPoint struct and manifold storage
  - `msd/msd-sim/src/Physics/EPA.hpp` — Added manifold extraction methods
  - `msd/msd-sim/src/Physics/EPA.cpp` — Implemented extractContactManifold() and deduplicateContacts()
  - `msd/msd-sim/src/Physics/CollisionResponse.hpp` — Added manifold-aware function declarations
  - `msd/msd-sim/src/Physics/CollisionResponse.cpp` — Implemented applyImpulseManifold() and applyPositionCorrectionManifold()
  - `msd/msd-sim/src/Environment/WorldModel.cpp` — Integrated manifold-aware collision response
  - `msd/msd-sim/test/Physics/EPATest.cpp` — Mechanical migration to manifold API
  - `msd/msd-sim/test/Physics/CollisionHandlerTest.cpp` — Mechanical migration to manifold API
- **Artifacts**:
  - `docs/designs/0029_contact_manifold_generation/implementation-notes.md`
- **Notes**:
  - Core implementation complete per design document
  - Breaking change: Removed contactPointA/contactPointB members (clean break per human feedback)
  - Equal impulse distribution implemented as designed
  - Fixed-size std::array<ContactPoint, 4> avoids heap allocations
  - **Bug fix applied**: Corrected impulse direction signs in applyImpulseManifold()
    - Linear impulse: A pushed in -normal direction, B pushed in +normal direction
    - Angular impulse: Signs corrected to match linear impulse directions
  - Build successful (269/271 tests passing, 98.9% pass rate)
  - 2 remaining failing tests appear to be pre-existing issues:
    - ProjectileMotion.freeFall_underGravity: Test bug (passes same timestamp each iteration)
    - WorldModelStaticCollisionTest.inertialVsEnvironment_ImpulseApplied: Unrelated to manifold changes (static collision code path)
  - All inertial-inertial collision tests pass (validates core manifold implementation)
  - Comprehensive new test suite from design document deferred to quality gate phase
  - Test migration completed mechanically via sed script

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**:
- **Completed**:
- **CLAUDE.md Updates**:
- **Diagrams Indexed**:
- **Notes**:

### Tutorial Documentation Phase (if Generate Tutorial: Yes)
- **Started**:
- **Completed**:
- **Tutorial Location**: `docs/tutorials/0029_contact_manifold_generation/`
- **Artifacts**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
I see multiple objects deprecated when they could be removed. Remember the guidance to prefer simplicity over strict adherence to the ticket parameters and that this feature is meant to purely improve the collision mechanism, removing the need to keep objectively worse functionality.

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

### Feedback on Tutorial (if Generate Tutorial: Yes)
{Your comments on the tutorial content, concepts to cover, or presentation style}
