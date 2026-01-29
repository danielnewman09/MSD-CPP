# Design: Generalized Lagrange Multiplier Constraint System

## Summary

This design introduces a generalized constraint framework using Lagrange multipliers that enables users to define arbitrary constraints by providing constraint function data (evaluation, Jacobian, Baumgarte parameters). The system enables building a constraint library where each constraint type provides the mathematical information and a unified solver handles enforcement. This replaces the current hard-coded `QuaternionConstraint` implementation with an extensible architecture that supports equality constraints (bilateral), inequality constraints (unilateral), and multi-object constraints (joints, contacts).

**Key benefit**: New constraint types (joints, contacts, limits) can be added by implementing the `Constraint` interface without modifying the solver or integration infrastructure.

## Architecture Changes

### PlantUML Diagram
See: `./0031_generalized_lagrange_constraints.puml`

### New Components

#### Constraint (Abstract Base Class)
- **Purpose**: Abstract interface defining the mathematical requirements for constraint definitions
- **Header location**: `msd/msd-sim/src/Physics/Constraints/Constraint.hpp`
- **Source location**: Header-only (pure virtual interface)
- **Key interfaces**:
  ```cpp
  class Constraint {
  public:
      virtual ~Constraint() = default;

      // Number of scalar constraint equations
      virtual int dimension() const = 0;

      // Evaluate constraint function C(q, t)
      // Returns vector of constraint violations
      virtual Eigen::VectorXd evaluate(
          const InertialState& state,
          double time) const = 0;

      // Compute constraint Jacobian J = ∂C/∂q
      // Returns (dimension x state_dim) matrix
      virtual Eigen::MatrixXd jacobian(
          const InertialState& state,
          double time) const = 0;

      // Compute constraint time derivative ∂C/∂t (optional, default: zero)
      virtual Eigen::VectorXd partialTimeDerivative(
          const InertialState& state,
          double time) const;

      // Baumgarte stabilization parameters
      virtual double alpha() const { return 10.0; }
      virtual double beta() const { return 10.0; }

      // Constraint type for debugging/logging
      virtual std::string typeName() const = 0;

  protected:
      Constraint() = default;
      Constraint(const Constraint&) = default;
      Constraint& operator=(const Constraint&) = default;
      Constraint(Constraint&&) noexcept = default;
      Constraint& operator=(Constraint&&) noexcept = default;
  };
  ```
- **Dependencies**: Eigen3, InertialState
- **Thread safety**: Implementations must be thread-safe for concurrent evaluation (read-only after construction)
- **Error handling**: Implementations may throw `std::invalid_argument` for invalid state or parameters

#### BilateralConstraint (Abstract Subclass)
- **Purpose**: Specialization for equality constraints C(q, t) = 0 with unrestricted Lagrange multipliers
- **Header location**: `msd/msd-sim/src/Physics/Constraints/BilateralConstraint.hpp`
- **Source location**: Header-only (pure virtual interface)
- **Key interfaces**:
  ```cpp
  class BilateralConstraint : public Constraint {
  public:
      virtual ~BilateralConstraint() = default;

      // No additional interface beyond Constraint
      // Semantic marker for bilateral (equality) constraints

  protected:
      BilateralConstraint() = default;
      BilateralConstraint(const BilateralConstraint&) = default;
      BilateralConstraint& operator=(const BilateralConstraint&) = default;
      BilateralConstraint(BilateralConstraint&&) noexcept = default;
      BilateralConstraint& operator=(BilateralConstraint&&) noexcept = default;
  };
  ```
- **Dependencies**: Constraint
- **Thread safety**: Same as Constraint
- **Error handling**: Same as Constraint
- **Rationale**: Provides semantic separation between equality and inequality constraints, enabling future solver optimizations

#### UnilateralConstraint (Abstract Subclass)
- **Purpose**: Specialization for inequality constraints C(q, t) ≥ 0 with complementarity conditions
- **Header location**: `msd/msd-sim/src/Physics/Constraints/UnilateralConstraint.hpp`
- **Source location**: Header-only (pure virtual interface)
- **Key interfaces**:
  ```cpp
  class UnilateralConstraint : public Constraint {
  public:
      virtual ~UnilateralConstraint() = default;

      // Check if constraint is active (C ≈ 0)
      // Inactive constraints are not included in solver
      virtual bool isActive(const InertialState& state,
                           double time) const = 0;

  protected:
      UnilateralConstraint() = default;
      UnilateralConstraint(const UnilateralConstraint&) = default;
      UnilateralConstraint& operator=(const UnilateralConstraint&) = default;
      UnilateralConstraint(UnilateralConstraint&&) noexcept = default;
      UnilateralConstraint& operator=(UnilateralConstraint&&) noexcept = default;
  };
  ```
- **Dependencies**: Constraint
- **Thread safety**: Same as Constraint
- **Error handling**: Same as Constraint
- **Rationale**: Supports contact constraints and limits that are conditionally active based on state

#### UnitQuaternionConstraint (Concrete Implementation)
- **Purpose**: Migrated implementation of quaternion normalization constraint using new framework
- **Header location**: `msd/msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.cpp`
- **Key interfaces**:
  ```cpp
  class UnitQuaternionConstraint : public BilateralConstraint {
  public:
      explicit UnitQuaternionConstraint(double alpha = 10.0, double beta = 10.0);
      ~UnitQuaternionConstraint() override = default;

      // Constraint interface
      int dimension() const override;  // Returns 1
      Eigen::VectorXd evaluate(const InertialState& state, double time) const override;
      Eigen::MatrixXd jacobian(const InertialState& state, double time) const override;
      Eigen::VectorXd partialTimeDerivative(const InertialState& state, double time) const override;
      std::string typeName() const override;  // Returns "UnitQuaternionConstraint"

      // Baumgarte parameters
      double alpha() const override;
      double beta() const override;
      void setAlpha(double alpha);
      void setBeta(double beta);

      // Rule of Five
      UnitQuaternionConstraint(const UnitQuaternionConstraint&) = default;
      UnitQuaternionConstraint& operator=(const UnitQuaternionConstraint&) = default;
      UnitQuaternionConstraint(UnitQuaternionConstraint&&) noexcept = default;
      UnitQuaternionConstraint& operator=(UnitQuaternionConstraint&&) noexcept = default;

  private:
      double alpha_{10.0};
      double beta_{10.0};
  };
  ```
- **Dependencies**: BilateralConstraint, InertialState, Eigen3
- **Thread safety**: Thread-safe for concurrent evaluation after construction
- **Error handling**: No exceptions (all quaternion operations numerically stable)
- **Mathematical formulation**:
  - Constraint: `C(Q) = Q^T·Q - 1 = 0`
  - Jacobian: `J = 2·Q^T` (1×4 row vector w.r.t quaternion components)
  - Time derivative: `∂C/∂t = 0` (time-independent)

#### DistanceConstraint (Concrete Implementation)
- **Purpose**: Example constraint enforcing fixed distance between two points
- **Header location**: `msd/msd-sim/src/Physics/Constraints/DistanceConstraint.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/DistanceConstraint.cpp`
- **Key interfaces**:
  ```cpp
  class DistanceConstraint : public BilateralConstraint {
  public:
      explicit DistanceConstraint(double targetDistance,
                                  double alpha = 10.0,
                                  double beta = 10.0);
      ~DistanceConstraint() override = default;

      // Constraint interface
      int dimension() const override;  // Returns 1
      Eigen::VectorXd evaluate(const InertialState& state, double time) const override;
      Eigen::MatrixXd jacobian(const InertialState& state, double time) const override;
      Eigen::VectorXd partialTimeDerivative(const InertialState& state, double time) const override;
      std::string typeName() const override;

      // Baumgarte parameters
      double alpha() const override;
      double beta() const override;

      // Distance configuration
      double getTargetDistance() const;

      // Rule of Five
      DistanceConstraint(const DistanceConstraint&) = default;
      DistanceConstraint& operator=(const DistanceConstraint&) = default;
      DistanceConstraint(DistanceConstraint&&) noexcept = default;
      DistanceConstraint& operator=(DistanceConstraint&&) noexcept = default;

  private:
      double targetDistance_{std::numeric_limits<double>::quiet_NaN()};
      double alpha_{10.0};
      double beta_{10.0};
  };
  ```
- **Dependencies**: BilateralConstraint, InertialState, Eigen3
- **Thread safety**: Thread-safe for concurrent evaluation after construction
- **Error handling**: Throws `std::invalid_argument` if `targetDistance <= 0`
- **Mathematical formulation**:
  - Constraint: `C(X) = |X|² - d² = 0` (where X is position, d is target distance from origin)
  - Jacobian: `J = 2·X^T` (1×3 row vector w.r.t position components)
  - Time derivative: `∂C/∂t = 0` (time-independent)
- **Note**: This is a simplified single-object constraint for demonstration. Multi-object distance constraints require design extension in future ticket.

#### ConstraintSolver
- **Purpose**: Computes Lagrange multipliers for arbitrary constraint systems using direct linear solve
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.cpp`
- **Key interfaces**:
  ```cpp
  class ConstraintSolver {
  public:
      struct SolveResult {
          Eigen::VectorXd lambdas;              // Lagrange multipliers
          Coordinate linearConstraintForce;     // Net linear force
          Coordinate angularConstraintForce;    // Net angular torque
          bool converged{false};
          double conditionNumber{std::numeric_limits<double>::quiet_NaN()};

          SolveResult() = default;
          SolveResult(const Eigen::VectorXd& l,
                     const Coordinate& linForce,
                     const Coordinate& angForce,
                     bool conv,
                     double cond);
      };

      explicit ConstraintSolver(double epsilon = 1e-10);
      ~ConstraintSolver() = default;

      // Solve constraint system for Lagrange multipliers
      SolveResult solve(
          const std::vector<Constraint*>& constraints,
          InertialState& state,
          const Coordinate& externalForce,
          const Coordinate& externalTorque,
          double mass,
          const Eigen::Matrix3d& inverseInertia,
          double dt);

      // Rule of Five
      ConstraintSolver(const ConstraintSolver&) = default;
      ConstraintSolver& operator=(const ConstraintSolver&) = default;
      ConstraintSolver(ConstraintSolver&&) noexcept = default;
      ConstraintSolver& operator=(ConstraintSolver&&) noexcept = default;

  private:
      double epsilon_{1e-10};  // Numerical tolerance for convergence

      // Internal helpers
      Eigen::MatrixXd assembleConstraintMatrix(
          const std::vector<Constraint*>& constraints,
          const InertialState& state,
          double time,
          const Eigen::Matrix3d& inverseInertia) const;

      Eigen::VectorXd assembleRHS(
          const std::vector<Constraint*>& constraints,
          const InertialState& state,
          const Coordinate& externalForce,
          const Coordinate& externalTorque,
          double mass,
          const Eigen::Matrix3d& inverseInertia,
          double time,
          double dt) const;

      std::pair<Coordinate, Coordinate> extractConstraintForces(
          const Eigen::VectorXd& lambdas,
          const std::vector<Constraint*>& constraints,
          const InertialState& state,
          double time) const;
  };
  ```
- **Dependencies**: Constraint, InertialState, Eigen3
- **Thread safety**: Thread-safe (stateless solver, mutable matrices are local)
- **Error handling**:
  - Returns `converged = false` if matrix is singular/ill-conditioned
  - Logs warning if condition number > 1e12
  - Does not throw exceptions (graceful degradation)
- **Algorithm**:
  1. Assemble constraint Jacobian `J` by stacking all constraint Jacobians
  2. Compute mass matrix inverse `M^-1` (block diagonal: [m^-1·I₃, I^-1])
  3. Form constraint matrix: `A = J·M^-1·J^T`
  4. Build RHS: `b = -J·M^-1·F_ext - J̇·q̇ - α·C - β·Ċ`
  5. Solve `A·λ = b` using Eigen LLT decomposition (positive definite assumption)
  6. Extract forces: `F_constraint = J^T·λ`
- **Performance**: O(n³) where n = total constraint dimension. Suitable for n < 100 (typical: 1-10 constraints per object).

#### SolveResult (Struct)
- **Purpose**: Return value from ConstraintSolver containing Lagrange multipliers and constraint forces
- **Header location**: `msd/msd-sim/src/Physics/Constraints/ConstraintSolver.hpp`
- **Source location**: Header-only (POD struct)
- **Key interfaces**: See `ConstraintSolver::SolveResult` above
- **Dependencies**: Eigen3, Coordinate
- **Thread safety**: Value type, safe to copy across threads
- **Memory management**: Stack-allocated struct
- **Rationale**: Provides diagnostic information (condition number, convergence) alongside forces for solver health monitoring

### Modified Components

#### Integrator (Abstract Interface)
- **Current location**: `msd/msd-sim/src/Physics/Integration/Integrator.hpp`
- **Changes required**:
  1. Replace `QuaternionConstraint&` parameter with `std::vector<Constraint*>&` in `step()` signature
  2. Update documentation to reflect generalized constraint support
- **Backward compatibility**: Breaking change to interface signature
- **Modified interface**:
  ```cpp
  virtual void step(InertialState& state,
                    const Coordinate& force,
                    const Coordinate& torque,
                    double mass,
                    const Eigen::Matrix3d& inverseInertia,
                    const std::vector<Constraint*>& constraints,  // CHANGED
                    double dt) = 0;
  ```
- **Rationale**: Enables passing arbitrary constraint collections instead of single hard-coded constraint

#### SemiImplicitEulerIntegrator (Concrete Implementation)
- **Current location**: `msd/msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp`, `SemiImplicitEulerIntegrator.cpp`
- **Changes required**:
  1. Add `ConstraintSolver` member
  2. Update `step()` implementation to use `ConstraintSolver::solve()`
  3. Apply constraint forces from solver result
  4. Remove direct `QuaternionConstraint::enforceConstraint()` call
- **Backward compatibility**: Breaking change (signature change), but implementation change is transparent
- **Modified implementation**:
  ```cpp
  class SemiImplicitEulerIntegrator : public Integrator {
  public:
      SemiImplicitEulerIntegrator();
      explicit SemiImplicitEulerIntegrator(ConstraintSolver solver);
      ~SemiImplicitEulerIntegrator() override = default;

      void step(InertialState& state,
                const Coordinate& force,
                const Coordinate& torque,
                double mass,
                const Eigen::Matrix3d& inverseInertia,
                const std::vector<Constraint*>& constraints,  // CHANGED
                double dt) override;

      // Rule of Five
      SemiImplicitEulerIntegrator(const SemiImplicitEulerIntegrator&) = default;
      SemiImplicitEulerIntegrator& operator=(const SemiImplicitEulerIntegrator&) = default;
      SemiImplicitEulerIntegrator(SemiImplicitEulerIntegrator&&) noexcept = default;
      SemiImplicitEulerIntegrator& operator=(SemiImplicitEulerIntegrator&&) noexcept = default;

  private:
      ConstraintSolver solver_;  // NEW
  };
  ```
- **Integration algorithm update**:
  ```
  1. Compute unconstrained accelerations:
     a_free = F_ext / m
     α_free = I^-1 * τ_ext

  2. Solve constraint system:
     result = solver_.solve(constraints, state, F_ext, τ_ext, m, I^-1, dt)

  3. Apply constraint forces:
     a_total = a_free + result.linearConstraintForce / m
     α_total = α_free + I^-1 * result.angularConstraintForce

  4. Semi-implicit Euler update:
     v_new = v_old + a_total * dt
     x_new = x_old + v_new * dt
     ω_new = ω_old + α_total * dt
     Q̇_new = omegaToQuaternionRate(ω_new, Q_old)
     Q_new = Q_old + Q̇_new * dt
     normalize(Q_new)  // Implicit constraint enforcement
  ```

#### AssetInertial
- **Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp`
- **Changes required**:
  1. Replace `QuaternionConstraint` member with `std::vector<std::unique_ptr<Constraint>>`
  2. Add constraint management methods: `addConstraint()`, `removeConstraint()`, `getConstraints()`, `clearConstraints()`
  3. Constructor adds default `UnitQuaternionConstraint` automatically
- **Backward compatibility**: Breaking change (member type changed), but public interface is additive
- **Modified interface**:
  ```cpp
  class AssetInertial {
  public:
      // ... existing interface unchanged ...

      // NEW: Constraint management
      void addConstraint(std::unique_ptr<Constraint> constraint);
      void removeConstraint(size_t index);
      std::vector<Constraint*> getConstraints();
      std::vector<const Constraint*> getConstraints() const;
      void clearConstraints();

  private:
      // ... existing members unchanged ...
      std::vector<std::unique_ptr<Constraint>> constraints_;  // NEW (replaces quaternionConstraint_)
  };
  ```
- **Default behavior**: Constructor automatically adds `UnitQuaternionConstraint` to preserve existing behavior
- **Ownership**: AssetInertial owns constraints via `std::unique_ptr`, provides raw pointers for integration

#### WorldModel
- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  1. Update `updatePhysics()` to gather constraints from each asset and pass to integrator
  2. No new members or public interface changes
- **Backward compatibility**: Internal implementation change only
- **Modified implementation**:
  ```cpp
  void WorldModel::updatePhysics(double dt) {
      for (auto& asset : inertialAssets_) {
          // Accumulate external forces
          Coordinate netForce = computePotentialForces(asset);
          Coordinate netTorque = computePotentialTorques(asset);

          // Get constraints from asset
          std::vector<Constraint*> constraints = asset.getConstraints();  // NEW

          // Integrate with constraints
          integrator_->step(asset.getInertialState(),
                           netForce,
                           netTorque,
                           asset.getMass(),
                           asset.getInverseInertiaTensor(),
                           constraints,  // CHANGED
                           dt);

          // Synchronize ReferenceFrame
          syncReferenceFrame(asset);
      }
  }
  ```

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| Constraint | InertialState | Read-only dependency | Constraint implementations read state for evaluation and Jacobian computation |
| BilateralConstraint | Constraint | Inheritance | Pure interface extension, no additional dependencies |
| UnilateralConstraint | Constraint | Inheritance | Adds `isActive()` method for conditional enforcement |
| UnitQuaternionConstraint | BilateralConstraint | Inheritance | Concrete implementation, drop-in replacement for `QuaternionConstraint` |
| DistanceConstraint | BilateralConstraint | Inheritance | Example implementation demonstrating framework usage |
| ConstraintSolver | Constraint | Uses | Solver calls `evaluate()`, `jacobian()`, etc. on constraint collection |
| SemiImplicitEulerIntegrator | ConstraintSolver | Composition | Integrator owns solver, delegates constraint solving |
| AssetInertial | Constraint | Ownership | AssetInertial owns constraints via `std::unique_ptr`, provides raw pointers |
| WorldModel | Integrator | Delegation | WorldModel calls integrator with constraint collection from assets |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Physics/Integration/QuaternionPhysicsTest.cpp` | All 16 test cases | Breaking change | Update to use `UnitQuaternionConstraint` and constraint vector |
| `test/Physics/RigidBody/AssetInertialTest.cpp` | Constraint-related tests | Breaking change | Replace `QuaternionConstraint` with `UnitQuaternionConstraint` |
| `test/Environment/WorldModelTest.cpp` | Physics integration tests | Internal change | Verify behavior unchanged (should pass without modification) |

**Migration strategy for existing tests**:
```cpp
// Old (Ticket 0030)
QuaternionConstraint constraint{10.0, 10.0};
integrator.step(state, force, torque, mass, invInertia, constraint, dt);

// New (Ticket 0031)
auto constraint = std::make_unique<UnitQuaternionConstraint>(10.0, 10.0);
std::vector<Constraint*> constraints{constraint.get()};
integrator.step(state, force, torque, mass, invInertia, constraints, dt);
```

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| Constraint | Interface contract | Abstract interface is well-formed, default implementations work |
| BilateralConstraint | Semantic interface | Inheritance hierarchy correct, no additional logic |
| UnilateralConstraint | Activity detection | `isActive()` interface correct |
| UnitQuaternionConstraint | Dimension | Returns 1 |
| UnitQuaternionConstraint | Evaluate | `C(Q) = Q^T·Q - 1` computed correctly for unit and non-unit quaternions |
| UnitQuaternionConstraint | Jacobian | `J = 2·Q^T` computed correctly, matches analytical derivative |
| UnitQuaternionConstraint | PartialTimeDerivative | Returns zero vector (time-independent constraint) |
| UnitQuaternionConstraint | Baumgarte parameters | `alpha()`, `beta()`, `setAlpha()`, `setBeta()` work correctly |
| UnitQuaternionConstraint | Migration from QuaternionConstraint | Equivalent behavior to old implementation |
| DistanceConstraint | Dimension | Returns 1 |
| DistanceConstraint | Evaluate | `C(X) = |X|² - d²` computed correctly |
| DistanceConstraint | Jacobian | `J = 2·X^T` computed correctly |
| DistanceConstraint | Invalid target distance | Throws `std::invalid_argument` for distance ≤ 0 |
| ConstraintSolver | Empty constraint set | Returns zero forces, converged = true |
| ConstraintSolver | Single constraint (quaternion) | Computes correct Lagrange multiplier and forces |
| ConstraintSolver | Multiple constraints | Correctly stacks Jacobians and solves system |
| ConstraintSolver | Singular matrix | Returns `converged = false`, handles gracefully |
| ConstraintSolver | Condition number | Reports accurate condition number for health monitoring |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| Constraint enforcement during integration | SemiImplicitEulerIntegrator, UnitQuaternionConstraint, ConstraintSolver | Quaternion remains normalized within tolerance over many steps |
| Multiple constraints per object | SemiImplicitEulerIntegrator, UnitQuaternionConstraint, DistanceConstraint, ConstraintSolver | Multiple constraints are correctly solved and enforced simultaneously |
| AssetInertial constraint management | AssetInertial, UnitQuaternionConstraint | Add/remove/get/clear constraints work correctly |
| WorldModel physics integration | WorldModel, AssetInertial, SemiImplicitEulerIntegrator, UnitQuaternionConstraint | Constraints gathered from assets and passed to integrator correctly |
| Backward compatibility | All integration tests from Ticket 0030 | Behavior identical to previous implementation |

#### Benchmark Tests (if performance-critical)

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| ConstraintSolver | Solve 1 constraint (quaternion) | Time to solve single constraint system | < 10 μs (debug), < 1 μs (release) |
| ConstraintSolver | Solve 5 constraints | Time to solve multi-constraint system | < 50 μs (debug), < 5 μs (release) |
| ConstraintSolver | Jacobian assembly | Time to assemble constraint matrix | < 5 μs per constraint |
| UnitQuaternionConstraint | Evaluate + Jacobian | Time to compute constraint data | < 1 μs |
| SemiImplicitEulerIntegrator | Integration with constraints | End-to-end integration step time | < 5% overhead vs. no constraints |

**Rationale for benchmarks**: Constraint solving happens every timestep for every object, so performance is critical. Benchmarks ensure O(n³) direct solve remains tractable for typical constraint counts (n < 10).

## Open Questions

### Design Decisions (Human Input Needed)

1. **Multi-object constraint architecture**
   - **Question**: How should constraints involving multiple objects (joints, contacts) be represented?
   - **Option A**: Constraint stores references to multiple InertialState objects
     - Pros: Direct access to all relevant state
     - Cons: Ownership becomes complex, harder to manage lifetimes
   - **Option B**: Constraint receives concatenated state vector of all involved objects
     - Pros: Cleaner interface, solver doesn't care about object boundaries
     - Cons: Requires WorldModel to assemble multi-object state, more complex integration
   - **Option C**: Defer multi-object constraints to future ticket, implement only single-object constraints now
     - Pros: Simplifies initial implementation, proves architecture with simpler cases
     - Cons: Delays key use cases (joints, contacts)
   - **Recommendation**: Option C for this ticket. Implement single-object constraint framework, design multi-object extension in separate ticket (0032). This allows validating the core architecture before adding complexity.

2. **Unilateral constraint solver algorithm**
   - **Question**: Should this ticket implement complementarity solver for unilateral constraints, or only direct solver for bilateral?
   - **Option A**: Implement only bilateral constraint solver (direct LLT solve)
     - Pros: Simpler, faster to implement, sufficient for initial constraints (quaternion, distance)
     - Cons: Cannot enforce contact constraints or limits (future use cases)
   - **Option B**: Implement both bilateral and unilateral solvers (add projected Gauss-Seidel)
     - Pros: Complete framework, enables contact constraints immediately
     - Cons: Significantly more complex, harder to validate, increases ticket scope
   - **Option C**: Implement bilateral solver, add `UnilateralConstraint` interface for future extensibility
     - Pros: Interface defines contract, but implementation deferred until needed
     - Cons: Incomplete framework, requires follow-up ticket
   - **Recommendation**: Option C for this ticket. Define `UnilateralConstraint` interface with `isActive()` method, but `ConstraintSolver` only handles bilateral constraints initially. Follow-up ticket (0033) can add complementarity solver when contact constraints are designed.

3. **Jacobian computation fallback**
   - **Question**: Should the framework support automatic numerical Jacobian computation as fallback for complex constraints?
   - **Option A**: Require all constraints to provide analytical Jacobians
     - Pros: Enforces efficiency and accuracy, simpler framework
     - Cons: Higher barrier to entry for constraint authors
   - **Option B**: Provide numerical Jacobian utility as fallback
     - Pros: Easier to implement new constraints, good for prototyping
     - Cons: Performance penalty, less accurate, requires tuning finite difference step size
   - **Recommendation**: Option A for this ticket. Analytical Jacobians are feasible for all planned constraints (quaternion, distance, joints). Numerical Jacobian support can be added in future ticket if complex constraints emerge.

4. **Constraint force logging/debugging**
   - **Question**: Should `SolveResult` include per-constraint force breakdown for debugging?
   - **Option A**: Return only net force (current design)
     - Pros: Simpler interface, minimal allocation
     - Cons: Harder to debug constraint interactions
   - **Option B**: Add optional `std::vector<Coordinate> perConstraintForces` field
     - Pros: Better diagnostics, easier debugging
     - Cons: Heap allocation per solve, larger return value
   - **Recommendation**: Option A for initial implementation. Constraint force breakdown can be added via separate debug API in future if needed (e.g., `solveDebug()` that returns extended result).

### Prototype Required

1. **Constraint matrix conditioning with typical constraint combinations**
   - **Uncertainty**: Will typical constraint combinations (quaternion + distance + joint) produce well-conditioned constraint matrices?
   - **Validation approach**: Prototype with 5-10 constraints on single object, measure condition number
   - **Acceptance criteria**: Condition number < 1e12 for typical configurations
   - **Risk mitigation**: If poorly conditioned, may need regularization or iterative solver

2. **Performance overhead of constraint framework vs. hard-coded QuaternionConstraint**
   - **Uncertainty**: Does virtual function overhead and dynamic constraint vector impact performance?
   - **Validation approach**: Benchmark old vs. new implementation with single quaternion constraint
   - **Acceptance criteria**: < 10% performance regression compared to hard-coded implementation
   - **Risk mitigation**: If overhead is significant, consider constraint batching or template-based dispatch

### Requirements Clarification

1. **Default constraints on AssetInertial**
   - **Ambiguity**: Should `UnitQuaternionConstraint` be added automatically to every `AssetInertial`, or require explicit addition?
   - **Impact**: Affects API ergonomics and backward compatibility
   - **Recommendation**: Add automatically in constructor to preserve existing behavior (quaternion normalization always active). Provide `clearConstraints()` for advanced users who want to manage constraints manually.

2. **Constraint ownership for multi-object constraints**
   - **Ambiguity**: When a constraint involves multiple objects (future), which object owns it?
   - **Impact**: Affects memory management and lifetime guarantees
   - **Recommendation**: Defer decision to multi-object constraint ticket (0032). Current single-object constraints are clearly owned by the object they constrain.

3. **Time-varying constraints**
   - **Ambiguity**: Should constraints support time-varying target values (e.g., actuated joints with target angle)?
   - **Impact**: Affects constraint interface design
   - **Recommendation**: `evaluate()` and `jacobian()` already receive `time` parameter, so time-varying constraints are supported by interface. Defer implementation to future ticket when actuator constraints are designed (0034).

## Implementation Notes

### Migration Path from QuaternionConstraint

To minimize disruption, the migration will follow this sequence:

1. **Phase 1**: Implement new constraint framework alongside existing `QuaternionConstraint`
   - Add `Constraint`, `BilateralConstraint`, `UnilateralConstraint` interfaces
   - Implement `UnitQuaternionConstraint` as new class
   - Implement `ConstraintSolver`
   - Keep `QuaternionConstraint` unchanged

2. **Phase 2**: Update integration infrastructure
   - Modify `Integrator` interface to accept `std::vector<Constraint*>`
   - Update `SemiImplicitEulerIntegrator` to use `ConstraintSolver`
   - Update `AssetInertial` to own constraint vector with default `UnitQuaternionConstraint`
   - Update `WorldModel::updatePhysics()` to gather constraints

3. **Phase 3**: Remove deprecated `QuaternionConstraint`
   - Delete `QuaternionConstraint.hpp` and `QuaternionConstraint.cpp`
   - Update all tests to use `UnitQuaternionConstraint`
   - Update documentation and CLAUDE.md files

### Constraint Jacobian Dimensionality

The Jacobian matrix returned by `jacobian()` must have dimensions `(constraint_dimension × state_dimension)`. For the current 14-component state vector `(X, Q, Ẋ, Q̇)`, the state dimension is:
- Position: 3 components (X)
- Quaternion: 4 components (Q)
- **Total position-level: 7 components**

For velocity-level constraints, the Jacobian operates on the 7-component velocity vector `(Ẋ, Q̇)`.

**Example Jacobian dimensions**:
- `UnitQuaternionConstraint`: 1 × 7 (constraint acts on quaternion components)
- `DistanceConstraint`: 1 × 7 (constraint acts on position components)
- Multi-constraint system with 3 constraints: 3 × 7 (stacked Jacobians)

### Memory Pre-allocation Strategy

To minimize heap allocations in the hot path (`ConstraintSolver::solve()` called every timestep):

1. **Constraint vector**: Passed by reference, no allocation in solver
2. **Jacobian assembly**: Allocate `Eigen::MatrixXd` on stack (VLA not used, Eigen handles allocation)
3. **Constraint matrix `A = J·M⁻¹·Jᵀ`**: Allocate on stack for typical sizes (< 10 constraints)
4. **Future optimization**: Add `preallocate(maxConstraints)` method if profiling shows allocation overhead

### Numerical Stability Considerations

1. **Matrix conditioning**: `ConstraintSolver` computes condition number and logs warning if > 1e12
2. **Singularity handling**: Returns `converged = false` if LLT decomposition fails (matrix not positive definite)
3. **Regularization**: Future enhancement if conditioning issues arise (add small diagonal term `A + εI`)
4. **Iterative solver**: Future enhancement for large constraint systems (n > 100)

## Dependencies

- **Ticket 0030**: Lagrangian quaternion physics (provides `InertialState`, `Integrator`, quaternion-based state)
- **Eigen3**: Matrix operations, linear algebra
- **C++20**: Required for project-wide standards

## Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| **Constraint matrix singular/ill-conditioned** | Solver fails, constraints not enforced | Medium | Return `converged = false`, add regularization if needed, log condition number for diagnostics |
| **Performance overhead of virtual dispatch** | Physics loop slowed by constraint framework | Low | Benchmark old vs. new, accept < 10% overhead, consider template dispatch if significant |
| **Complexity of multi-object constraints** | Deferred design may need framework changes | Medium | Design single-object constraints first, validate architecture, extend carefully in follow-up ticket |
| **Breaking changes disrupt existing tests** | Test suite requires significant rework | High | Provide clear migration guide, update tests systematically, validate behavior equivalence |
| **Solver convergence failures in practice** | Constraints violated during simulation | Low | Monitor condition number, log solver failures, add fallback to projection method if needed |

## Future Extensions

The following features are explicitly deferred to future tickets:

1. **Multi-object constraints** (Ticket 0032)
   - Joint constraints (ball-socket, revolute, prismatic)
   - Contact constraints between colliding objects
   - Design constraint ownership and multi-object state assembly

2. **Unilateral constraint solver** (Ticket 0033)
   - Projected Gauss-Seidel algorithm for complementarity
   - Contact non-penetration constraints
   - Joint angle limits

3. **Actuator constraints** (Ticket 0034)
   - Motor constraints with target velocities
   - Time-varying constraint targets
   - PID control integration

4. **Solver optimizations** (Ticket 0035)
   - Sparse matrix support for large systems
   - Iterative solvers (CG, GMRES)
   - Warm starting (use previous λ as initial guess)
   - Constraint batching for parallel solving

5. **Friction constraints** (Ticket 0036)
   - Coulomb friction via complementarity
   - Friction cone approximations
   - Tangent space parameterization

6. **Soft constraints** (Future)
   - Spring-damper approximations for compliant constraints
   - Penalty-based enforcement
   - Constraint violation penalties in cost function

## References

- Baraff, D. (1996). "Linear-Time Dynamics using Lagrange Multipliers" - Constraint solving algorithm
- Cline, M. B. (2002). "Rigid Body Simulation with Contact and Constraints" - Baumgarte stabilization
- Bullet Physics Library: `btSequentialImpulseConstraintSolver` - Production constraint solver architecture
- ODE (Open Dynamics Engine): Constraint solving architecture - Design patterns for extensible constraints
- Witkin, A., & Baraff, D. (2001). "Physically Based Modeling: Principles and Practice" - Lagrange multiplier theory

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-01-28
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | Pass | `Constraint`, `BilateralConstraint`, `UnilateralConstraint`, `ConstraintSolver`, `SolveResult` follow PascalCase for classes. Method names (`evaluate`, `jacobian`, `dimension`) follow camelCase. Member variables (`alpha_`, `beta_`, `epsilon_`) use snake_case with trailing underscore. |
| Namespace organization | Pass | All components reside in `msd_sim` namespace, consistent with existing Physics module. Sub-organization under `Physics::Constraints` is appropriate. |
| File structure | Pass | Proposed locations (`msd/msd-sim/src/Physics/Constraints/*.hpp`) match existing patterns (`QuaternionConstraint.hpp` already exists there). |
| Dependency direction | Pass | Design correctly layers: `Constraint` <- `BilateralConstraint`/`UnilateralConstraint` <- concrete implementations. `ConstraintSolver` depends on `Constraint` interface (not vice versa). `SemiImplicitEulerIntegrator` composes `ConstraintSolver`. No circular dependencies identified. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | Pass | `AssetInertial` owns constraints via `std::vector<std::unique_ptr<Constraint>>`, ensuring proper cleanup. `ConstraintSolver` is stateless after construction. |
| Smart pointer appropriateness | Pass | Design uses `std::unique_ptr` for exclusive ownership (AssetInertial owns constraints), raw pointers for non-owning access (`std::vector<Constraint*>` passed to integrator). Follows CLAUDE.md guideline of avoiding `std::shared_ptr`. |
| Value/reference semantics | Pass | `SolveResult` uses value semantics (return struct). Constraints passed by raw pointer for non-owning iteration. `InertialState` passed by reference for mutation. |
| Rule of 0/3/5 | Pass | All classes declare Rule of Five with `= default` explicitly. `Constraint` base class has protected copy/move operations (appropriate for polymorphic base). |
| Const correctness | Pass | `evaluate()`, `jacobian()`, `partialTimeDerivative()`, `alpha()`, `beta()`, `typeName()`, `dimension()` are all `const`. `ConstraintSolver::solve()` receives `constraints` by const reference. Minor note: `InertialState&` parameter in `solve()` is mutable (intentional for state updates). |
| Exception safety | Pass | Design specifies graceful degradation: `ConstraintSolver` returns `converged = false` rather than throwing on singular matrices. Concrete constraints may throw `std::invalid_argument` for invalid parameters (documented). |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | Pass | Forward declarations can be used for `InertialState` and `Coordinate` in constraint headers. `Constraint.hpp` depends only on Eigen and forward-declared types. No circular header issues. |
| Template complexity | Pass | No template metaprogramming. Polymorphism via virtual functions is appropriate for runtime constraint composition. |
| Memory strategy | Pass | Stack allocation for typical constraint counts. `Eigen::MatrixXd` handles dynamic sizing. Design notes pre-allocation strategy for hot path. |
| Thread safety | Pass | Design correctly marks implementations as "thread-safe for concurrent evaluation after construction" (read-only). `ConstraintSolver` is stateless (local matrix allocations). |
| Build integration | Pass | New files follow existing CMake patterns. `Constraints/CMakeLists.txt` already exists from Ticket 0030. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | Pass | `Constraint` interface can be tested with mock `InertialState`. `ConstraintSolver` can be tested with mock constraints. `UnitQuaternionConstraint` can be unit-tested independently. |
| Mockable dependencies | Pass | All dependencies are interfaces or value types. `Constraint*` vector enables test doubles. |
| Observable state | Pass | `SolveResult` exposes all solver state (lambdas, forces, convergence, condition number). Constraint violation can be queried via `evaluate()`. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Jacobian dimension mismatch between constraint output and InertialState size | Technical | Medium | High | Document state vector mapping clearly (7 position-level DOFs). Add runtime dimension check in ConstraintSolver. | No |
| R2 | LLT decomposition fails for non-positive-definite constraint matrices | Technical | Medium | Medium | Design already specifies: return `converged = false`, log warning. May need regularization fallback. | Yes |
| R3 | Virtual function overhead in hot path | Performance | Low | Low | Benchmarks specified. Virtual dispatch cost is ~2-5ns per call, negligible vs O(n^3) solve. | Yes |
| R4 | Breaking change to Integrator interface disrupts test suite | Maintenance | High | Medium | Migration guide provided. Systematic test updates required. | No |
| R5 | DistanceConstraint formulation incomplete for single-object use | Technical | Low | Low | Design acknowledges this is simplified demonstration. Full multi-object design deferred to Ticket 0032. | No |

### Prototype Guidance

#### Prototype P1: Constraint Matrix Conditioning

**Risk addressed**: R2
**Question to answer**: Do typical constraint combinations (quaternion + distance) produce well-conditioned constraint matrices, and does LLT decomposition handle edge cases gracefully?

**Success criteria**:
- Condition number < 1e12 for quaternion-only constraint
- Condition number < 1e12 for quaternion + distance constraint
- `converged = false` returned correctly when matrix is singular
- No NaN propagation from ill-conditioned solve

**Prototype approach**:
```
Location: prototypes/0031_generalized_lagrange_constraints/p1_conditioning/
Type: Catch2 test harness

Steps:
1. Create mock InertialState with typical values
2. Assemble constraint matrices for 1, 2, 5 constraint scenarios
3. Compute condition numbers using Eigen's JacobiSVD
4. Test LLT solve with deliberately singular matrix
5. Verify graceful failure returns converged=false
```

**Time box**: 1 hour

**If prototype fails**:
- Add diagonal regularization term `A + epsilon*I` before solve
- Fallback to iterative solver (deferred to Ticket 0035)

#### Prototype P2: Virtual Function Overhead

**Risk addressed**: R3
**Question to answer**: What is the performance overhead of the generalized constraint framework compared to the hard-coded QuaternionConstraint implementation?

**Success criteria**:
- Less than 10% performance regression for single quaternion constraint
- Less than 5 microseconds overhead per constraint solve (release build)

**Prototype approach**:
```
Location: prototypes/0031_generalized_lagrange_constraints/p2_overhead/
Type: Google Benchmark harness

Steps:
1. Benchmark existing QuaternionConstraint::enforceConstraint() call
2. Implement minimal Constraint interface wrapper
3. Benchmark equivalent path through ConstraintSolver
4. Compare results and compute overhead percentage
```

**Time box**: 30 minutes

**If prototype fails**:
- Consider constraint batching to amortize virtual call overhead
- Template-based dispatch for hot path (future optimization ticket)

### Notes

1. **Open Question Resolution Recommended**: The design identifies 4 open questions requiring human input. Recommend Option C for multi-object constraints (defer to Ticket 0032), Option C for unilateral solver (define interface, defer implementation), Option A for Jacobians (require analytical), and Option A for force logging (minimal interface first). These are reasonable defaults that can proceed without blocking.

2. **Jacobian Dimensionality Clarity**: The design states "7-component position-level state" but the Jacobian discussion mentions both position (7) and velocity (7) levels. Recommend clarifying that `jacobian()` returns partial derivatives w.r.t. position-level DOFs only (X: 3, Q: 4 = 7), not velocity-level. This is consistent with holonomic constraint formulation but should be explicit.

3. **Migration Test Strategy**: The breaking change to `Integrator::step()` signature affects 16+ existing tests. The migration guide provided is clear. Recommend creating a temporary compatibility shim during transition if phased rollout is needed.

4. **SolveResult Memory**: `Eigen::VectorXd lambdas` involves heap allocation. For hot path optimization, consider fixed-size `Eigen::Vector<double, MaxConstraints>` if typical constraint count is known. Deferred to future ticket per design notes.

5. **QuaternionConstraint Deprecation**: The existing `QuaternionConstraint` class should be marked `[[deprecated]]` before removal in Phase 3, allowing downstream code to migrate gracefully.

### Summary

The design is well-structured and demonstrates strong alignment with project coding standards and existing physics architecture. The constraint hierarchy (Constraint -> Bilateral/Unilateral -> concrete implementations) is clean and extensible. The `ConstraintSolver` approach using direct LLT solve is appropriate for the expected constraint counts (1-10 per object). Breaking changes are documented with clear migration paths.

Two prototypes are recommended to validate matrix conditioning (P1) and performance overhead (P2) before implementation. Both have clear success criteria and reasonable time boxes. The risks are well-mitigated by the design's error handling strategy (graceful degradation via `converged = false`).

**Verdict**: Approved with notes. Proceed to prototype phase after addressing open questions with stakeholder input.
