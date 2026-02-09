# Documentation Sync Summary

## Feature: 0047_face_contact_manifold_generation
**Date**: 2026-02-09
**Target Library**: msd-sim (Collision and Environment modules)

## Diagrams Synchronized

No new diagrams were created for this feature. The changes are algorithmic additions to existing CollisionHandler and WorldModel components.

### Updated Documentation Files
| File | Changes |
|------|---------|
| `msd/msd-sim/src/Physics/Collision/CLAUDE.md` | Added "SAT Fallback for EPA Validation" section explaining why SAT validation is needed and how it works |
| `msd/msd-sim/src/Environment/CLAUDE.md` | Added "Update Order" subsection to WorldModel documentation explaining gravity pre-apply sequence |

## CLAUDE.md Updates

### Sections Added

**Collision/CLAUDE.md**:
- "SAT Fallback for EPA Validation" section (before "Why std::optional<CollisionResult>?")
  - Explains EPA failure mode at zero/near-zero penetration
  - Documents SAT validation logic (10× threshold)
  - References `computeSATMinPenetration()` and `buildSATContact()` methods

**Environment/CLAUDE.md**:
- "Update Order" subsection in WorldModel component (after "Usage Example")
  - Documents 6-step update sequence
  - Explains gravity pre-apply rationale (standard Box2D/Bullet approach)
  - Documents restitution-gravity coupling trade-off and future work (ticket 0051)

### Sections Modified
None. All changes are additive.

### Diagrams Index
No new diagrams added. The feature does not introduce new architectural components requiring new diagrams.

## Key Files Changed

| File | Purpose |
|------|---------|
| `msd/msd-sim/src/Physics/Collision/CollisionHandler.hpp` | Added SATResult struct, computeSATMinPenetration(), buildSATContact() methods |
| `msd/msd-sim/src/Physics/Collision/CollisionHandler.cpp` | Implemented SAT fallback logic in checkCollision() |
| `msd/msd-sim/src/Environment/WorldModel.cpp` | Modified update() to pre-apply gravity before collision solving |
| `msd/msd-sim/test/Physics/Collision/ManifoldDiagnosticTest.cpp` | Added diagnostic tests for manifold generation |

## Verification

- [x] All diagram links verified (no new diagrams added)
- [x] CLAUDE.md formatting consistent with existing style
- [x] No broken references
- [x] Documentation accurately reflects implementation changes

## Notes

This feature was primarily an algorithmic fix rather than an architectural change:
1. **SAT fallback**: Catches EPA failures at shallow penetration by validating results against ground-truth SAT computation
2. **Gravity pre-apply**: Architectural change to update order to match industry-standard physics engines

The documentation updates focus on explaining the *why* (rationale) and *when* (zero-penetration failure mode) rather than duplicating implementation details already visible in code comments.

**Follow-on work**: Ticket 0051 will address the restitution-gravity coupling introduced by gravity pre-apply using velocity-bias threading.
