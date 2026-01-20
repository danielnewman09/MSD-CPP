# Architect Revision Request

## Mode
**REVISION MODE** - Address reviewer feedback for ticket 0023a_force_application_scaffolding

## Context
The design review returned status `REVISION_REQUESTED` with three critical issues that must be addressed before final approval.

## Ticket Reference
`tickets/0023a_force_application_scaffolding.md`

## Current Design Document
`docs/designs/0023a_force_application_scaffolding/design.md`

## Current PlantUML Diagram
`docs/designs/0023a_force_application_scaffolding/0023a_force_application_scaffolding.puml`

## Issues to Address (from Design Review)

### Issue I1 - Type Inconsistency for Angular Quantities (Critical)
**Problem**: The design document specifies inconsistent types for `InertialState::angularVelocity` and `angularAcceleration`:
- Design document lines 19, 54, 76 specify `Coordinate`
- Ticket acceptance criteria lines 115-116 specify `Eigen::Vector3d`
- PlantUML diagram lines 14-15 specify `Coordinate`

**Impact**: Creates confusion and would lead to compilation errors

**Required Change**:
- Use `Coordinate` consistently (it inherits from `Eigen::Vector3d` and aligns with project patterns)
- Update ALL occurrences in:
  - InertialState struct specification
  - AssetInertial method signatures (lines 135, 179, 186 in design.md)
  - Migration guide
  - PlantUML diagram (already correct, verify consistency)
- The ticket acceptance criteria is a reference only; the design document is authoritative
- Provide helper functions in the `InertialState` struct which convert the `Coordinate` representations to `EulerAngles`

**Rationale**: `Coordinate` is preferred because:
1. It inherits from `Eigen::Vector3d`, providing all vector operations
2. It's already used for linear quantities (position, velocity, acceleration)
3. It maintains consistency across the `InertialState` struct
4. Project patterns favor using existing domain types over raw Eigen types

### Issue I2 - Invalid Initialization Syntax
**Problem**: Line 161 in design.md shows:
```cpp
Coordinate accumulatedTorque_{Coordinate::Zero()};
```

But `Coordinate` does not have a static `Zero()` method (that's `Eigen::Vector3d::Zero()`).

**Impact**: Would cause compilation error

**Required Change**:
- Use brace initialization per project standards: `Coordinate accumulatedTorque_{0.0, 0.0, 0.0};`
- This is consistent with `accumulatedForce_` initialization on line 160
- Also check line 195 in placeholder implementations for similar issues

### Issue I3 - WorldModel Constructor Contradiction
**Problem**: The design shows contradictory requirements:
- Line 245: `const Coordinate gravity_` (const member)
- Line 213: `setGravity()` method specified
- Line 233: Constructor `WorldModel(const Coordinate& gravity)` shown

**Impact**: Cannot have both a const member AND a setter method - this is logically inconsistent

**Required Change**:
2. Remove `setGravity()` method
3. Update constructor specification to show both default constructor and parametric constructor
4. Clarify that gravity is set at construction time and cannot be modified later

**Alternative Approach** (if human prefers immutability):
1. Remove `setGravity()` method entirely
2. Keep `gravity_` as const
3. Require initialization via constructor only
4. Update ticket requirements

**Recommendation**: Follow the ticket requirements (keep `setGravity()`), remove const qualifier.

## Items Passing Review (DO NOT MODIFY)

The following aspects passed review and should remain unchanged:
- File structure and namespace organization
- Naming conventions (PascalCase, camelCase, snake_case_)
- Dependencies structure (no circular dependencies)
- Breaking change documentation and migration guide
- RAII compliance and memory management patterns
- Const correctness in accessors
- Rule of Zero for special member functions
- Return value patterns (prefer values over output params)
- Placeholder implementation strategy
- TODO comment formatting
- Test requirements (12 unit tests, 2 integration tests)
- Test strategy focusing on API contracts

## Output Requirements

1. Update `docs/designs/0023a_force_application_scaffolding/design.md`:
   - Fix all three issues
   - Add "Architect Revision Notes" section documenting changes
   - Do NOT modify sections that passed review

2. Update `docs/designs/0023a_force_application_scaffolding/0023a_force_application_scaffolding.puml`:
   - Verify type consistency (should already be correct for I1)
   - Update if any architectural changes were made

3. Preserve the existing design review section (don't remove it)

## Success Criteria

The revision is complete when:
1. All three issues are resolved with consistent, compilable code
2. Type usage is uniform throughout (Coordinate for angular velocity/acceleration)
3. Initialization syntax follows project standards
4. WorldModel gravity configuration is logically consistent (non-const with setter)
5. Revision notes document all changes made
6. PlantUML diagram matches the updated design