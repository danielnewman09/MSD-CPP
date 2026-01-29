# Feature Ticket: lagrangian_quaternion_physics

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review (SKIPPED per human decision)
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [x] Approved — Ready to Merge
- [x] Documentation Complete — Awaiting Tutorial (if Generate Tutorial: Yes)
- [x] Tutorial Complete — Ready to Merge (if Generate Tutorial: Yes)
- [x] Merged / Complete

## Metadata
- **Created**: 2026-01-28
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: Large
- **Target Component(s)**: msd-sim (Physics module, Environment module)
- **Generate Tutorial**: No

---

## Summary
Implement Lagrangian mechanics with quaternion 7-state representation (X, Q, Ẋ, Q̇) replacing Euler angle-based orientation, with general potential energy abstraction and Lagrange multiplier constraints. This eliminates gimbal lock singularities and provides a mathematically rigorous constraint-based physics framework.

## Motivation
The current physics implementation uses Euler angles for orientation representation, which suffers from gimbal lock singularities when pitch approaches ±90°. This makes the simulation numerically unstable for certain spacecraft attitudes and prevents robust simulation of arbitrary orientations.

Additionally, implementing Lagrangian mechanics with a general potential energy framework will:
- Provide a mathematically rigorous foundation for constraint-based physics
- Enable future extensions for general force fields, tidal effects, and multi-body dynamics
- Improve numerical stability through quaternion normalization constraints
- Establish a cleaner separation between kinematics (state representation) and dynamics (forces/constraints)

User impact: Enables simulation of spacecraft in any orientation without numerical instabilities, providing a foundation for more complex multi-body scenarios.

## Requirements

### Functional Requirements
1. The system shall refactor `InertialState` to store quaternion-based state (X, Q, Ẋ, Q̇) with 14 total components
2. The system shall implement a `PotentialEnergy` interface for abstracting potential energy fields
3. The system shall provide a `GravityPotential` implementation for uniform gravitational fields
4. The system shall implement `QuaternionConstraint` using Lagrange multipliers with Baumgarte stabilization to maintain |Q|=1
5. The system shall update `WorldModel::updatePhysics()` for quaternion-based numerical integration
6. The system shall provide deprecated accessor methods for backward compatibility with Euler angle representations
7. The system shall implement conversion utilities between Q̇ (quaternion derivative) and ω (angular velocity)

### Non-Functional Requirements
- **Performance**: Quaternion integration overhead must not exceed 10% compared to current Euler integration
- **Numerical Stability**: Quaternion constraint must maintain |Q|=1 to within 1e-10 tolerance over extended simulations
- **Thread Safety**: No new thread safety requirements (single-threaded simulation loop)
- **Backward Compatibility**: NONE

## Constraints
- Must preserve current WorldModel API surface for external consumers
- Quaternion normalization constraint requires careful tuning of Baumgarte stabilization parameters
- Collision response integration is OUT OF SCOPE for this ticket (will be addressed separately)

## Acceptance Criteria
- [x] AC1: Q̇ ↔ ω conversion round-trips correctly (within 1e-10 tolerance)
- [x] AC2: Quaternion constraint maintains |Q|=1 over 10000 integration steps (error < 1e-10)
- [x] AC3: Free-fall test matches analytical solution z = z₀ - ½gt² (within 1e-6 tolerance)
- [x] AC4: No gimbal lock at 90° pitch (quaternion stays valid, no NaN propagation)
- [x] AC5: GravityPotential produces correct force F = m*g pointing in -Z direction

---

## Design Decisions (Human Input)

### Preferred Approaches
- Use quaternion representation (w, x, y, z) with scalar-first ordering consistent with industry standards
- Implement Baumgarte stabilization for quaternion constraint with α, β parameters for drift correction
- Use Lagrangian mechanics formulation: L = T - V where T is kinetic energy and V is potential energy
- Store quaternion derivative Q̇ directly in state rather than converting to/from ω at every step

### Things to Avoid
- Do not use Euler angles internally (only provide as deprecated accessor for compatibility)
- Avoid naive quaternion integration without constraint enforcement (drift accumulates quickly)
- Do not hard-code gravity in physics update loop - must go through PotentialEnergy abstraction

### Open Questions
- What Baumgarte stabilization parameters (α, β) provide optimal stability vs. performance?
- Should we support multiple simultaneous PotentialEnergy instances (e.g., gravity + tidal forces)?
- How should deprecated Euler accessors behave when conversion is ambiguous?
- Should InertialState store both Q and normalized Q_normalized, or normalize on-access?

---

## References

### Related Code
- `msd/msd-sim/src/Environment/InertialState.hpp` — Current state representation using Euler angles
- `msd/msd-sim/src/Environment/WorldModel.hpp` — Physics update loop that will be refactored
- `msd/msd-sim/src/Environment/ReferenceFrame.hpp` — Coordinate transformations that may need quaternion support

### Related Documentation
- Lagrangian mechanics: Goldstein, Classical Mechanics (3rd ed.), Chapter 1-2
- Quaternion kinematics: Kuipers, Quaternions and Rotation Sequences (1999)
- Baumgarte stabilization: Baumgarte, "Stabilization of constraints and integrals of motion" (1972)
- Constraint-based dynamics: Witkin & Baraff, "Physically Based Modeling" SIGGRAPH course notes

### Related Tickets
- Future multi-body dynamics will depend on this quaternion foundation

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Artifacts**:
  - `docs/designs/0030_lagrangian_quaternion_physics/design.md`
  - `docs/designs/0030_lagrangian_quaternion_physics/0030_lagrangian_quaternion_physics.puml`
- **Notes**: Design focuses exclusively on stable gravity integration with quaternion state representation. Collision response integration explicitly OUT OF SCOPE per ticket requirements. Three new components: PotentialEnergy (abstract interface), GravityPotential (uniform field implementation), QuaternionConstraint (Lagrange multipliers with Baumgarte stabilization). Modified components: InertialState (quaternion orientation + Q̇), ReferenceFrame (quaternion storage), WorldModel (potential energy container + refactored physics integration). Open questions documented for Baumgarte parameters (α, β), multiple potential energy support, and Euler angle deprecation strategy.

### Design Review Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Status**: APPROVED WITH NOTES
- **Reviewer Notes**: Design demonstrates excellent architectural quality and strong alignment with project coding standards. Perfect adherence to CLAUDE.md memory management, smart pointer usage, and Rule of Five conventions. Clean PotentialEnergy abstraction provides extensibility. All components designed for isolated unit testing. Two prototypes REQUIRED before implementation: P1 (Baumgarte parameter tuning, 2 hours) to determine optimal α, β and validate 1e-10 constraint tolerance; P2 (performance validation, 1.5 hours) to confirm < 10% overhead requirement. Breaking changes well-mitigated through deprecated accessors and phased implementation. Collision response integration deferred per ticket scope is acceptable.

### Prototype Phase
- **Started**: N/A
- **Completed**: N/A (SKIPPED)
- **Prototypes**: None — Human decided to skip prototyping and proceed directly to implementation
- **Artifacts**: None
- **Notes**: Baumgarte parameters will use literature defaults (α=10, β=10). Performance validation will occur during implementation testing.

### Implementation Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Files Created**:
  - `msd-sim/src/Physics/Integration/Integrator.hpp` — Abstract interface for numerical integration
  - `msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp/cpp` — Symplectic integrator
  - `msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp` — Abstract interface for environmental potentials
  - `msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp/cpp` — Uniform gravity implementation
  - `msd-sim/src/Physics/Constraints/QuaternionConstraint.hpp/cpp` — Baumgarte stabilization
  - `msd-sim/src/Physics/RigidBody/InertialState.cpp` — ω ↔ Q̇ conversion utilities
- **Files Modified**:
  - `msd-sim/src/Physics/RigidBody/InertialState.hpp` — Quaternion orientation + quaternionRate
  - `msd-sim/src/Physics/RigidBody/AssetInertial.hpp/cpp` — Added QuaternionConstraint ownership
  - `msd-sim/src/Environment/ReferenceFrame.hpp/cpp` — Added setQuaternion(), getQuaternion()
  - `msd-sim/src/Environment/WorldModel.hpp/cpp` — Integrator + potential energies + refactored physics
  - `msd-sim/src/Agent/InputControlAgent.cpp` — Updated for quaternion API
  - `msd-sim/src/Physics/CollisionResponse.cpp` — Updated for quaternion API
  - Multiple test files — Updated for new quaternion state API
- **Test Results**: 319/320 tests pass (1 unrelated EPA witness point test)
- **Artifacts**: None (no separate implementation-notes.md needed)
- **Notes**:
  - Fixed critical bug in ω ↔ Q̇ conversion (Eigen uses x,y,z,w coefficient order, not w,x,y,z)
  - Fixed test timing issues (world.update requires incrementing timestamps)
  - Fixed GravityPotential::computeEnergy() sign convention (V = -m*g·r for positive energy above origin)
  - Remaining EPA test failure is unrelated to quaternion physics (collision geometry corner case)

### Implementation Review Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **Status**: PASSED
- **Reviewer Notes**: All 5 acceptance criteria validated with unit tests in `test/Physics/Integration/QuaternionPhysicsTest.cpp`. Tests cover:
  - AC1: 5 tests for Q̇ ↔ ω round-trip conversion with identity, rotated, and arbitrary quaternions
  - AC2: 3 tests for quaternion constraint maintenance over 10000 steps and enforcement behavior
  - AC3: 2 tests for free-fall position and velocity matching analytical solutions
  - AC4: 2 tests for gimbal lock avoidance at 90° pitch and continuous rotation through singularity
  - AC5: 4 tests for gravity force magnitude, torque (zero), energy, and orientation independence

### Documentation Update Phase
- **Started**: 2026-01-28
- **Completed**: 2026-01-28
- **CLAUDE.md Updates**:
  - `msd-sim/CLAUDE.md` — Added "Lagrangian Quaternion Physics" to Recent Architectural Changes, indexed design diagram
  - `msd-sim/src/Physics/CLAUDE.md` — Added 6 new components to Core Components table with full documentation: Integrator, SemiImplicitEulerIntegrator, PotentialEnergy, GravityPotential, QuaternionConstraint
  - `msd-sim/src/Environment/CLAUDE.md` — Updated InertialState and ReferenceFrame documentation for quaternion changes with migration guide
- **Diagrams Indexed**:
  - `0030_lagrangian_quaternion_physics.puml` referenced in msd-sim/CLAUDE.md Diagrams Index
  - Design diagram remains in `docs/designs/0030_lagrangian_quaternion_physics/` (not copied to `docs/msd/msd-sim/` due to cross-cutting nature)
- **Artifacts**:
  - `docs/designs/0030_lagrangian_quaternion_physics/doc-sync-summary.md` — Documentation sync summary
- **Notes**: Feature diagram kept in design folder rather than copied to library diagrams because it shows cross-cutting integration across Physics and Environment modules (InertialState, ReferenceFrame, WorldModel) rather than isolated subsystem. All new components fully documented with purpose, interfaces, usage examples, thread safety, error handling, memory management, and design rationale. Breaking changes clearly marked with migration guidance.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design

**Question 1**: Why are the potential energies and constraint in the world model? It makes more sense to me that they would be in the `AssetInertial`

**Question 2**: Can we (simply) break out the integration to have a simple `Integrator` class that steps all of the elements in the world model forward in time? This should help keep things more manageable in the future

**Resolution** (2026-01-28): Design revised to address both points:
1. **QuaternionConstraint moved to AssetInertial** — Each asset owns its own constraint since each has its own quaternion state. This provides better encapsulation.
2. **Integrator abstraction added** — New `Integrator` interface with `SemiImplicitEulerIntegrator` implementation. WorldModel owns the integrator and delegates integration math to it. Makes it easy to swap integrators (Euler → RK4 → Verlet) and improves testability.
3. **Environmental potentials remain in WorldModel** — Global forces like gravity are properties of the environment and apply uniformly to all objects, so WorldModel ownership makes sense. Per-object forces (thrusters, springs) would belong in AssetInertial (future extension).

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
