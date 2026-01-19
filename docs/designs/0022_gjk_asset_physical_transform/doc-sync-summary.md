# Documentation Sync Summary

## Feature: 0022_gjk_asset_physical_transform
**Date**: 2026-01-18
**Target Library**: msd-sim (Physics module)

## Diagrams Synchronized

### Copied/Created
| Source | Destination | Changes |
|--------|-------------|---------|
| `docs/designs/0022_gjk_asset_physical_transform/0022_gjk_asset_physical_transform.puml` | `docs/msd/msd-sim/Physics/gjk-asset-physical.puml` | Removed "new/modified" highlighting, cleaned up comments for stable library documentation, updated title from "0022_gjk_asset_physical_transform" to "gjk-asset-physical", added performance and memory notes |

### Updated
No existing diagrams were updated. The new `gjk-asset-physical.puml` diagram is a standalone component diagram.

## CLAUDE.md Updates

### Sections Added
**msd/msd-sim/CLAUDE.md**:
- Added "Recent Architectural Changes" section documenting the GJK refactor

### Sections Modified
**msd/msd-sim/CLAUDE.md**:
- **Physics Module summary**: Updated GJK component description to mention AssetPhysical transform support, added AssetPhysical to key components list
- **Diagrams Index**: Updated gjk.puml reference to gjk-asset-physical.puml with updated description

**msd/msd-sim/src/Physics/CLAUDE.md**:
- **GJK section**: Complete rewrite to document new AssetPhysical-based API, transformation pipeline, performance characteristics, breaking changes, and migration guidance
- **ConvexHull section**: Removed references to intersects() method in key features, key interfaces, and usage examples
- **Getting Help section**: Updated collision detection guidance to reference GJK with AssetPhysical instead of ConvexHull::intersects()

### Diagrams Index
Added new entry:
- `gjk-asset-physical.puml` — GJK collision detection with AssetPhysical transforms

## Verification

- [x] All diagram links verified
- [x] CLAUDE.md formatting consistent
- [x] No broken references
- [x] Library documentation structure complete
- [x] Breaking changes clearly documented
- [x] Migration guidance provided
- [x] Performance characteristics documented

## Notes

### Breaking Change Documentation
This feature is a breaking change that removes the old ConvexHull-only GJK interface in favor of a unified AssetPhysical-based interface. All documentation has been updated to:
1. Clearly mark the breaking changes with ticket reference
2. Provide migration guidance (wrap ConvexHull in AssetPhysical with identity ReferenceFrame)
3. Document the rationale (simpler API, correct abstraction level for physics simulation)

### Performance Documentation
The documentation includes validated performance characteristics from prototype testing:
- < 2% overhead compared to identity transform baseline
- No heap allocations during collision detection
- Memory overhead: 16 bytes for two AssetPhysical references

### Transformation Pipeline Documentation
The documentation clearly explains the on-the-fly transformation strategy:
1. Transform search direction from world to local space (globalToLocalRelative - rotation only)
2. Find support vertex in local hull space
3. Transform support vertex from local to world space (localToGlobal - rotation + translation)
4. Construct simplex in world space

This is documented in both the Physics/CLAUDE.md detailed section and in the PlantUML diagram notes.

### ConvexHull Cleanup
All references to the removed `ConvexHull::intersects()` method have been removed from documentation:
- Removed from key features list
- Removed from key interfaces code block
- Removed from usage examples
- Updated "Getting Help" section to reference GJK with AssetPhysical

### Recent Architectural Changes Section
Added a new "Recent Architectural Changes" section to msd/msd-sim/CLAUDE.md to track major architectural modifications. This follows the pattern established in other library documentation and provides a historical record of breaking changes.

## Files Modified

1. `docs/msd/msd-sim/Physics/gjk-asset-physical.puml` (created)
2. `msd/msd-sim/CLAUDE.md` (updated)
3. `msd/msd-sim/src/Physics/CLAUDE.md` (updated)
4. `docs/designs/0022_gjk_asset_physical_transform/doc-sync-summary.md` (this file)
