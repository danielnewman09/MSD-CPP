# Architect Revision Request

## Mode
**REVISION MODE** - Revising existing design based on design reviewer feedback

## Design Document Location
`/Users/danielnewman/Documents/GitHub/MSD-CPP/docs/designs/0024_angular_coordinate/design.md`

## Context
The design-reviewer agent has reviewed the initial design for ticket 0024_angular_coordinate and returned status **REVISION_REQUESTED** (iteration 0 of 1). The design document has been updated with the review feedback appended.

## Human Feedback Incorporated
The human provided specific feedback in `docs/designs/0024_angular_coordinate/Design_notes.md` that MUST be addressed:

1. Remove `toCoordinate()` and `fromCoordinate()` methods from both AngularCoordinate and AngularRate
2. Remove `pitchRaw()`, `rollRaw()`, `yawRaw()` - just return normalized values
3. Remove `toRate()` and `toAngularCoordinate()` conversions between the two classes
4. Remove `EulerAngles` entirely NOW (not deprecate), replacing it with `AngularCoordinate` everywhere

## Issues to Address
The design reviewer identified four issues (I1-I4) that need revision. Please read the design document review section for full details on each issue.

## Instructions
1. Read the design document at `/Users/danielnewman/Documents/GitHub/MSD-CPP/docs/designs/0024_angular_coordinate/design.md`
2. Address ONLY the four issues (I1-I4) identified in the review
3. Do NOT modify parts of the design that passed review
4. Update the PlantUML diagram to reflect the changes (especially removing EulerAngles)
5. Document all changes in a "Revision Notes" section
6. Append your revision completion marker to the design document

## Key Changes Required
- API simplification: Remove unnecessary conversion methods
- API cleanup: Remove raw accessors, keep only normalized accessors
- Semantic correctness: Remove cross-type conversions that don't make sense
- Complete EulerAngles removal: This is now a clean-break approach, not a deprecation path
