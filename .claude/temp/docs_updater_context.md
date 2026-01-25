# Context for Documentation Update: 0028_epa_witness_points

## Feature Summary
EPA Witness Points for Accurate Torque Calculation - Extends the EPA implementation to track witness points (actual contact locations on each colliding object's surface) to enable accurate torque calculation in collision response.

## Target Library
msd-sim (Physics module)

## Key Changes
- Added SupportResult struct and supportMinkowskiWithWitness() function to SupportFunction
- Added MinkowskiVertex struct to EPA
- Breaking change to CollisionResult: contactPoint → contactPointA + contactPointB
- EPA now tracks witness points throughout polytope expansion
- Witness points extracted using barycentric interpolation

## Files Modified
- msd-sim/src/Physics/SupportFunction.hpp
- msd-sim/src/Physics/SupportFunction.cpp
- msd-sim/src/Physics/EPA.hpp
- msd-sim/src/Physics/EPA.cpp
- msd-sim/src/Physics/CollisionResult.hpp
- msd-sim/test/Physics/EPATest.cpp
- msd-sim/test/Physics/CollisionHandlerTest.cpp

## Design Artifacts
- Design document: docs/designs/0028_epa_witness_points/design.md
- PlantUML diagram: docs/designs/0028_epa_witness_points/0028_epa_witness_points.puml
- Implementation notes: docs/designs/0028_epa_witness_points/implementation-notes.md

## Implementation Status
- Implementation review: APPROVED
- All tests passing (219/219)
- Ready for documentation update

## Task
Update documentation to reflect the witness point tracking implementation:
1. Sync diagrams from docs/designs/0028_epa_witness_points/ to docs/msd/msd-sim/Physics/
2. Update msd/msd-sim/src/Physics/CLAUDE.md to document SupportResult, MinkowskiVertex, and CollisionResult changes
3. Update docs/msd/msd-sim/Physics/ diagrams to reflect the new components
4. Create doc-sync-summary.md
5. Ensure all diagram links are valid
