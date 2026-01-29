# Design: Lagrangian Quaternion Physics

## Summary

This design replaces the Euler angle-based orientation representation in `InertialState` with a quaternion-based 7-state representation (X, Q, Ẋ, Q̇) and implements Lagrangian mechanics with a general potential energy abstraction. The quaternion constraint is maintained via Lagrange multipliers with Baumgarte stabilization to ensure numerical stability. This design focuses **ONLY** on stable integration under gravity — collision response integration is explicitly OUT OF SCOPE and will be addressed in a separate ticket.

The primary benefits are:
1. **Elimination of gimbal lock** — quaternions have no singularities at any orientation
2. **Mathematical rigor** — Lagrangian formulation provides clear separation between kinematics and dynamics
3. **Extensibility** — potential energy abstraction enables future force fields (tidal, magnetic, etc.)
4. **Numerical stability** — constraint enforcement maintains |Q|=1 to within 1e-10 tolerance

## Architecture Changes

### PlantUML Diagram
See: `./0030_lagrangian_quaternion_physics.puml`

### Key Architectural Decisions

Based on human feedback, the architecture follows these principles:

1. **QuaternionConstraint owned by AssetInertial** — Each asset has its own quaternion state, so each asset owns its own constraint. This provides better encapsulation.

2. **Integrator abstraction** — A separate `Integrator` class handles numerical integration, making it easy to swap integration schemes (Euler → RK4 → Verlet) and improving testability.

3. **Environmental potentials in WorldModel** — Global forces like gravity are properties of the environment and apply uniformly to all objects. WorldModel owns these.

4. **Per-object forces in AssetInertial** — Future per-object forces (thrusters, springs) would naturally belong in AssetInertial.

### New Components

#### Integrator (Abstract Interface)

- **Purpose**: Abstract interface for numerical integration schemes, enabling swappable integrators
- **Header location**: `msd/msd-sim/src/Physics/Integration/Integrator.hpp`
- **Source location**: Header-only (pure abstract interface)
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Abstract interface for numerical integration of rigid body dynamics
   *
   * Decouples the integration scheme from the physics computation, enabling:
   * - Swappable integrators (Euler, RK4, Verlet, etc.)
   * - Isolated testing of integration math
   * - WorldModel as pure orchestrator
   *
   * Thread safety: Implementations should be stateless and thread-safe
   */
  class Integrator {
  public:
    virtual ~Integrator() = default;

    /**
     * @brief Integrate state forward by one timestep
     * @param state Current inertial state (modified in place)
     * @param force Net force in world frame [N]
     * @param torque Net torque in world frame [N·m]
     * @param mass Object mass [kg]
     * @param inverseInertia Inverse inertia tensor in body frame [1/(kg·m²)]
     * @param constraint Quaternion constraint for normalization
     * @param dt Timestep [s]
     */
    virtual void step(InertialState& state,
                      const Coordinate& force,
                      const Coordinate& torque,
                      double mass,
                      const Eigen::Matrix3d& inverseInertia,
                      QuaternionConstraint& constraint,
                      double dt) = 0;

  protected:
    Integrator() = default;
    Integrator(const Integrator&) = default;
    Integrator& operator=(const Integrator&) = default;
    Integrator(Integrator&&) noexcept = default;
    Integrator& operator=(Integrator&&) noexcept = default;
  };

  }  // namespace msd_sim
  ```
- **Dependencies**:
  - `InertialState.hpp` — State representation
  - `Coordinate.hpp` — Force/torque vectors
  - `QuaternionConstraint.hpp` — Constraint enforcement
  - `Eigen3` — Matrix operations
- **Thread safety**: Stateless interface (thread-safe)

#### SemiImplicitEulerIntegrator

- **Purpose**: Semi-implicit Euler integration (symplectic, energy-conserving for Hamiltonian systems)
- **Header location**: `msd/msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.cpp`
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Semi-implicit Euler integrator (symplectic)
   *
   * Integration order:
   * 1. Update velocities: v_new = v_old + a * dt
   * 2. Update positions: x_new = x_old + v_new * dt (uses NEW velocity)
   * 3. Enforce quaternion constraint with Baumgarte stabilization
   *
   * Properties:
   * - First-order accurate
   * - Symplectic (preserves phase space volume)
   * - Better energy conservation than explicit Euler
   * - Simple and computationally efficient
   */
  class SemiImplicitEulerIntegrator : public Integrator {
  public:
    SemiImplicitEulerIntegrator() = default;
    ~SemiImplicitEulerIntegrator() override = default;

    void step(InertialState& state,
              const Coordinate& force,
              const Coordinate& torque,
              double mass,
              const Eigen::Matrix3d& inverseInertia,
              QuaternionConstraint& constraint,
              double dt) override;

    // Rule of Five
    SemiImplicitEulerIntegrator(const SemiImplicitEulerIntegrator&) = default;
    SemiImplicitEulerIntegrator& operator=(const SemiImplicitEulerIntegrator&) = default;
    SemiImplicitEulerIntegrator(SemiImplicitEulerIntegrator&&) noexcept = default;
    SemiImplicitEulerIntegrator& operator=(SemiImplicitEulerIntegrator&&) noexcept = default;
  };

  }  // namespace msd_sim
  ```
- **Dependencies**:
  - `Integrator.hpp` — Base interface
  - `InertialState.hpp` — State manipulation
  - `QuaternionConstraint.hpp` — Constraint enforcement
- **Thread safety**: Stateless (thread-safe)
- **Error handling**: No exceptions (all operations numerically stable)

#### PotentialEnergy (Abstract Interface)

- **Purpose**: Abstract interface for environmental potential energy fields in Lagrangian mechanics formulation
- **Header location**: `msd/msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp`
- **Source location**: Header-only (pure abstract interface)
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Abstract interface for environmental potential energy fields
   *
   * This interface enables extensible potential energy computation for rigid body dynamics.
   * Implementations compute generalized forces from energy gradients.
   *
   * Lagrangian formulation: L = T - V where T is kinetic energy, V is potential energy
   * Generalized forces: F = -∂V/∂X  (linear force from position gradient)
   *                     τ = -∂V/∂Q  (torque from orientation gradient)
   *
   * Note: These are ENVIRONMENTAL potentials (gravity, magnetic fields) that apply
   * uniformly to all objects. Per-object forces (thrusters, springs) would be
   * handled separately in AssetInertial.
   *
   * Thread safety: Read-only methods after construction (thread-safe)
   */
  class PotentialEnergy {
  public:
    virtual ~PotentialEnergy() = default;

    /**
     * @brief Compute linear force from potential energy gradient
     * @param state Current inertial state
     * @param mass Object mass [kg]
     * @return Generalized force F = -∂V/∂X [N]
     */
    virtual Coordinate computeForce(const InertialState& state, double mass) const = 0;

    /**
     * @brief Compute torque from potential energy gradient
     * @param state Current inertial state
     * @param inertia Inertia tensor in world frame [kg·m²]
     * @return Generalized torque τ = -∂V/∂Q [N·m]
     */
    virtual Coordinate computeTorque(const InertialState& state,
                                     const Eigen::Matrix3d& inertia) const = 0;

    /**
     * @brief Compute potential energy
     * @param state Current inertial state
     * @param mass Object mass [kg]
     * @return Potential energy V [J]
     */
    virtual double computeEnergy(const InertialState& state, double mass) const = 0;

  protected:
    PotentialEnergy() = default;
    PotentialEnergy(const PotentialEnergy&) = default;
    PotentialEnergy& operator=(const PotentialEnergy&) = default;
    PotentialEnergy(PotentialEnergy&&) noexcept = default;
    PotentialEnergy& operator=(PotentialEnergy&&) noexcept = default;
  };

  }  // namespace msd_sim
  ```
- **Dependencies**:
  - `msd-sim/src/Physics/RigidBody/InertialState.hpp` — State representation
  - `msd-sim/src/Environment/Coordinate.hpp` — Force/torque vectors
  - `Eigen3` — Matrix operations
- **Thread safety**: Read-only interface after construction (implementations must be thread-safe)
- **Error handling**: Implementations may throw for invalid state (e.g., negative mass)

#### GravityPotential

- **Purpose**: Implements uniform gravitational field potential energy
- **Header location**: `msd/msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp`
- **Source location**: `msd/msd-sim/src/Physics/PotentialEnergy/GravityPotential.cpp`
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Uniform gravitational field potential energy
   *
   * Implements V = m * g * z where z is the vertical position component.
   * The gravitational vector g is typically (0, 0, -9.81) in z-up convention.
   *
   * Uniform gravity produces constant force F = m*g but no torque (orientation-independent).
   */
  class GravityPotential : public PotentialEnergy {
  public:
    /**
     * @brief Construct gravitational field with specified acceleration vector
     * @param gravityVector Gravitational acceleration [m/s²], e.g. (0, 0, -9.81)
     */
    explicit GravityPotential(const Coordinate& gravityVector);

    ~GravityPotential() override = default;

    // PotentialEnergy interface implementation
    CoordinateRate computeForce(const InertialState& state, double mass) const override;
    AngularRate computeTorque(const InertialState& state,
                              const Eigen::Matrix3d& inertia) const override;
    double computeEnergy(const InertialState& state, double mass) const override;

    // Gravity configuration
    void setGravity(const Coordinate& gravityVector);
    const Coordinate& getGravity() const;

    // Rule of Five
    GravityPotential(const GravityPotential&) = default;
    GravityPotential& operator=(const GravityPotential&) = default;
    GravityPotential(GravityPotential&&) noexcept = default;
    GravityPotential& operator=(GravityPotential&&) noexcept = default;

  private:
    Coordinate g_{0.0, 0.0, -9.81};  // Gravitational acceleration [m/s²]
  };

  }  // namespace msd_sim
  ```
- **Dependencies**:
  - `PotentialEnergy.hpp` — Base interface
  - `msd-sim/src/Environment/Coordinate.hpp` — Vector representation
  - `msd-sim/src/Environment/InertialState.hpp` — State access
- **Thread safety**: Immutable after construction (setGravity() should only be called during initialization)
- **Error handling**: No exceptions (gravity vector can be arbitrary, including zero for free-space dynamics)

#### QuaternionConstraint

- **Purpose**: Enforces unit quaternion constraint via Lagrange multipliers with Baumgarte stabilization
- **Header location**: `msd/msd-sim/src/Physics/Constraints/QuaternionConstraint.hpp`
- **Source location**: `msd/msd-sim/src/Physics/Constraints/QuaternionConstraint.cpp`
- **Ownership**: Owned by `AssetInertial` (each asset has its own quaternion state, so each owns its own constraint)
- **Key interfaces**:
  ```cpp
  namespace msd_sim {

  /**
   * @brief Enforces unit quaternion constraint via Lagrange multipliers
   *
   * Maintains the constraint g(Q) = QᵀQ - 1 = 0 using Baumgarte stabilization
   * to correct drift accumulated during numerical integration.
   *
   * Constraint equations:
   * - Position: g(Q) = QᵀQ - 1 = 0
   * - Velocity: ġ = 2QᵀQ̇ = 0  (Q̇ ⊥ Q)
   *
   * Baumgarte stabilization computes Lagrange multiplier as:
   *   λ = -α * g - β * ġ
   * where α, β > 0 are tuning parameters.
   *
   * Ownership: Each AssetInertial owns its own QuaternionConstraint instance,
   * since the constraint operates on per-object quaternion state.
   *
   * Thread safety: Not thread-safe (modifies quaternion state)
   */
  class QuaternionConstraint {
  public:
    /**
     * @brief Construct constraint with Baumgarte parameters
     * @param alpha Position error gain (default: 10.0)
     * @param beta Velocity error gain (default: 10.0)
     */
    explicit QuaternionConstraint(double alpha = 10.0, double beta = 10.0);

    ~QuaternionConstraint() = default;

    /**
     * @brief Enforce unit quaternion constraint with Baumgarte stabilization
     *
     * Modifies Q and Qdot to satisfy:
     * 1. Normalize Q to unit length
     * 2. Project Qdot onto tangent space (perpendicular to Q)
     * 3. Apply Baumgarte correction to reduce drift
     *
     * @param Q Quaternion to constrain (modified in place)
     * @param Qdot Quaternion rate (modified in place)
     */
    void enforceConstraint(Eigen::Quaterniond& Q, Eigen::Vector4d& Qdot);

    /**
     * @brief Compute constraint force for dynamics integration
     * @param Q Current quaternion
     * @param Qdot Current quaternion rate
     * @return Constraint force F_c = G^T * λ where G = ∂g/∂Q
     */
    Eigen::Vector4d computeConstraintForce(const Eigen::Quaterniond& Q,
                                           const Eigen::Vector4d& Qdot) const;

    // Parameter configuration
    void setAlpha(double alpha);
    void setBeta(double beta);
    double getAlpha() const;
    double getBeta() const;

    // Constraint violation queries (for diagnostics)
    double positionViolation(const Eigen::Quaterniond& Q) const;
    double velocityViolation(const Eigen::Quaterniond& Q,
                            const Eigen::Vector4d& Qdot) const;

    // Rule of Five
    QuaternionConstraint(const QuaternionConstraint&) = default;
    QuaternionConstraint& operator=(const QuaternionConstraint&) = default;
    QuaternionConstraint(QuaternionConstraint&&) noexcept = default;
    QuaternionConstraint& operator=(QuaternionConstraint&&) noexcept = default;

  private:
    double alpha_{10.0};  // Position error gain
    double beta_{10.0};   // Velocity error gain
  };

  }  // namespace msd_sim
  ```
- **Dependencies**:
  - `Eigen3` — Quaternion and vector operations
- **Thread safety**: Not thread-safe (modifies state during constraint enforcement)
- **Error handling**: No exceptions (all quaternion operations are numerically stable)

### Modified Components

#### InertialState

- **Current location**: `msd/msd-sim/src/Physics/RigidBody/InertialState.hpp`
- **Changes required**:
  1. Replace `AngularCoordinate orientation` with `Eigen::Quaterniond orientation`
  2. Add `Eigen::Vector4d quaternionRate` member for Q̇
  3. Add conversion utility methods:
     - `AngularRate quaternionRateToOmega() const` — Convert Q̇ → ω using ω = 2 * Q̄ ⊗ Q̇
     - `static Eigen::Vector4d omegaToQuaternionRate(const AngularRate& omega, const Eigen::Quaterniond& Q)` — Convert ω → Q̇ using Q̇ = ½ * Q ⊗ [0, ω]
  4. Add deprecated method `AngularCoordinate getEulerAngles() const` for backward compatibility
- **Backward compatibility**:
  - **BREAKING CHANGE**: `orientation` type changed from `AngularCoordinate` to `Eigen::Quaterniond`
  - **BREAKING CHANGE**: Added `quaternionRate` member (state size increased from 13 to 14 components)
  - Deprecated accessor `getEulerAngles()` provides Euler angle conversion via quaternion for legacy code
  - Migration: Replace `state.orientation.yaw()` with `state.getEulerAngles().yaw()` or migrate to quaternions

#### ReferenceFrame

- **Current location**: `msd/msd-sim/src/Environment/ReferenceFrame.hpp`, `ReferenceFrame.cpp`
- **Changes required**:
  1. Replace `AngularCoordinate angular_` with `Eigen::Quaterniond quaternion_` member
  2. Add new constructor: `ReferenceFrame(const Coordinate& origin, const Eigen::Quaterniond& quaternion)`
  3. Add new methods:
     - `void setQuaternion(const Eigen::Quaterniond& quat)`
     - `Eigen::Quaterniond getQuaternion() const`
  4. Mark existing methods as deprecated:
     - `ReferenceFrame(const Coordinate& origin, const AngularCoordinate& angular)` [[deprecated]]
     - `void setRotation(const AngularCoordinate& angular)` [[deprecated]]
     - `AngularCoordinate getAngularCoordinate() const` [[deprecated]]
  5. Update `updateRotationMatrix()` to compute from quaternion: `rotation_ = quaternion_.toRotationMatrix()`
- **Backward compatibility**:
  - **BREAKING CHANGE**: Internal storage changed from `AngularCoordinate` to `Eigen::Quaterniond`
  - Deprecated methods provide Euler angle interface via quaternion conversion
  - Transform methods (`globalToLocal`, `localToGlobal`) unchanged (work with rotation matrix)
  - Migration: Replace `ReferenceFrame(origin, AngularCoordinate{p, r, y})` with `ReferenceFrame(origin, Eigen::Quaterniond(...))`

#### AssetInertial

- **Current location**: `msd/msd-sim/src/Physics/RigidBody/AssetInertial.hpp`, `AssetInertial.cpp`
- **Changes required**:
  1. Add member: `QuaternionConstraint quaternionConstraint_{10.0, 10.0}`
  2. Add accessor: `QuaternionConstraint& getQuaternionConstraint()`
  3. Add accessor: `const QuaternionConstraint& getQuaternionConstraint() const`
- **Rationale**: Each asset has its own quaternion state (Q, Q̇), so each asset should own its own constraint. This provides better encapsulation than having WorldModel manage a shared constraint.
- **Backward compatibility**: Non-breaking (additive change)

#### WorldModel

- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `WorldModel.cpp`
- **Changes required**:
  1. Add member: `std::vector<std::unique_ptr<PotentialEnergy>> potentialEnergies_`
  2. Add member: `std::unique_ptr<Integrator> integrator_`
  3. Add methods:
     - `void addPotentialEnergy(std::unique_ptr<PotentialEnergy> energy)`
     - `void clearPotentialEnergies()`
     - `void setIntegrator(std::unique_ptr<Integrator> integrator)`
  4. **Major refactoring of `updatePhysics(double dt)` method**:

     **New integration order** (WorldModel as orchestrator, Integrator handles math):
     ```cpp
     void WorldModel::updatePhysics(double dt) {
       for (auto& asset : inertialAssets_) {
         // Step 1: Compute generalized forces from environmental potentials
         Coordinate netForce{0, 0, 0};
         Coordinate netTorque{0, 0, 0};

         for (const auto& potential : potentialEnergies_) {
           netForce += potential->computeForce(asset.getInertialState(), asset.getMass());
           netTorque += potential->computeTorque(asset.getInertialState(), asset.getInertiaTensor());
         }

         // Step 2: Delegate integration to Integrator
         // (Integrator handles: velocity update, position update, constraint enforcement)
         integrator_->step(
             asset.getInertialState(),
             netForce,
             netTorque,
             asset.getMass(),
             asset.getInverseInertiaTensor(),
             asset.getQuaternionConstraint(),  // Each asset owns its constraint
             dt
         );

         // Step 3: Synchronize ReferenceFrame
         ReferenceFrame& frame = asset.getReferenceFrame();
         frame.setOrigin(asset.getInertialState().position);
         frame.setQuaternion(asset.getInertialState().orientation);
       }
     }
     ```
  5. **Remove hardcoded gravity application** — replaced by `GravityPotential` in `potentialEnergies_` vector
  6. Constructor initialization:
     - Add default `GravityPotential` to `potentialEnergies_`
     - Initialize `integrator_` with `std::make_unique<SemiImplicitEulerIntegrator>()`
- **Backward compatibility**:
  - **BREAKING CHANGE**: Gravity is no longer hardcoded in `updatePhysics()`
  - **BREAKING CHANGE**: `gravity_` member should be removed (or marked deprecated)
  - Migration: WorldModel constructor should initialize with default `GravityPotential{Coordinate{0, 0, -9.81}}`
  - External code that modifies `gravity_` directly must migrate to `clearPotentialEnergies()` + `addPotentialEnergy(std::make_unique<GravityPotential>(...))`

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| Integrator | WorldModel | Owned by unique_ptr | WorldModel owns integrator, can swap implementations |
| SemiImplicitEulerIntegrator | WorldModel | Default implementation | Created during WorldModel construction |
| PotentialEnergy | WorldModel | Owned by container | WorldModel owns vector of `std::unique_ptr<PotentialEnergy>` |
| GravityPotential | WorldModel | Instantiated on construction | Default gravity potential added during WorldModel construction |
| QuaternionConstraint | AssetInertial | Owned by value | Each AssetInertial owns its own constraint |
| Integrator | QuaternionConstraint | Uses reference | Integrator calls constraint enforcement via passed reference |
| PotentialEnergy | AssetInertial | Reads state | Potential energies query `InertialState` via const reference |
| InertialState (modified) | AssetInertial | Owned by value | AssetInertial continues to own `InertialState` |
| ReferenceFrame (modified) | AssetInertial | Owned by value | AssetInertial continues to own `ReferenceFrame` |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/Environment/ReferenceFrameTest.cpp` | All transformation tests | **BROKEN** | Update to use quaternion constructors instead of `AngularCoordinate` |
| `test/Physics/InertialCalculationsTest.cpp` | Inertia tensor tests | Unchanged | No changes (operates on `ConvexHull` directly, not `InertialState`) |
| `test/Environment/WorldModelTest.cpp` | Physics integration tests | **BROKEN** | Update to use `Eigen::Quaterniond` for orientation, verify quaternion constraint |
| `test/Physics/AssetInertialTest.cpp` | Force application tests | **MODERATE** | Verify angular dynamics with quaternion state representation |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| GravityPotential | `computeForce_uniform_field` | Force equals `m * g` for all positions |
| GravityPotential | `computeTorque_uniform_field` | Torque is zero (uniform field independent of orientation) |
| GravityPotential | `computeEnergy_vertical_position` | Energy equals `m * g * z` |
| SemiImplicitEulerIntegrator | `step_linear_motion` | Position follows `x = x₀ + v*dt` for constant velocity |
| SemiImplicitEulerIntegrator | `step_linear_acceleration` | Velocity and position correctly update under constant force |
| SemiImplicitEulerIntegrator | `step_angular_motion` | Quaternion correctly integrates under constant torque |
| SemiImplicitEulerIntegrator | `step_calls_constraint` | Constraint enforcement is invoked after integration |
| QuaternionConstraint | `enforceConstraint_normalizes_quaternion` | `|Q|² = 1` after enforcement within 1e-10 tolerance |
| QuaternionConstraint | `enforceConstraint_projects_qdot` | `Q̇ ⊥ Q` (2Q^TQ̇ = 0) after enforcement within 1e-10 tolerance |
| QuaternionConstraint | `baumgarte_stabilization_reduces_drift` | Position violation decreases over multiple enforcement calls |
| QuaternionConstraint | `computeConstraintForce_magnitude` | Constraint force proportional to violation magnitude |
| InertialState | `quaternionRateToOmega_conversion` | Round-trip conversion Q̇→ω→Q̇ preserves value within 1e-10 |
| InertialState | `omegaToQuaternionRate_conversion` | Round-trip conversion ω→Q̇→ω preserves value within 1e-10 |
| InertialState | `getEulerAngles_deprecated_accessor` | Deprecated method returns correct Euler angles from quaternion |
| ReferenceFrame | `setQuaternion_updates_rotation_matrix` | Rotation matrix matches quaternion.toRotationMatrix() |
| ReferenceFrame | `quaternion_backward_compatibility` | Deprecated `setRotation(AngularCoordinate)` correctly converts to quaternion |
| AssetInertial | `owns_quaternion_constraint` | Each asset has its own constraint instance |
| AssetInertial | `constraint_parameters_configurable` | Can modify α, β via asset's constraint accessor |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `free_fall_under_gravity` | WorldModel, SemiImplicitEulerIntegrator, GravityPotential, AssetInertial | z-position follows `z = z₀ - ½gt²` within 1e-6 tolerance over 1000 steps |
| `no_gimbal_lock_at_90deg_pitch` | WorldModel, SemiImplicitEulerIntegrator, QuaternionConstraint, InertialState | Quaternion remains valid (no NaN) when orientation approaches singularities that would break Euler angles |
| `quaternion_constraint_stability` | WorldModel, SemiImplicitEulerIntegrator, QuaternionConstraint | `|Q| = 1` maintained over 10000 integration steps with error < 1e-10 |
| `energy_conservation_in_vacuum` | WorldModel, SemiImplicitEulerIntegrator, GravityPotential | Total energy `E = T + V` conserved within 1% over 5000 steps in gravity-free environment |
| `multiple_potential_energies` | WorldModel, PotentialEnergy | Multiple potentials (e.g., gravity + future tidal) produce additive forces |
| `swappable_integrator` | WorldModel, Integrator | WorldModel correctly uses injected integrator implementation |

#### Benchmark Tests (if performance-critical)

| Component | Benchmark Case | What It Measures | Baseline Expectation |
|-----------|----------------|------------------|----------------------|
| WorldModel | `physics_integration_quaternion_vs_euler` | Time per physics update with quaternion constraint | < 10% overhead vs current Euler integration |
| QuaternionConstraint | `constraint_enforcement_cost` | Time per `enforceConstraint()` call | < 100 nanoseconds per call (negligible) |
| InertialState | `quaternion_omega_conversion_overhead` | Time for Q̇↔ω conversions | < 50 nanoseconds per conversion |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Baumgarte Stabilization Parameters**
   - Question: What values of α and β provide optimal stability vs. performance trade-off?
   - Option A: Use literature defaults (α=10, β=10) — Pros: Standard values, well-tested. Cons: May not be optimal for this timestep/application
   - Option B: Prototype different values (α=5-20, β=5-20) — Pros: Empirically tuned for this sim. Cons: Requires prototyping phase
   - Option C: Make parameters configurable via WorldModel API — Pros: Maximum flexibility. Cons: More complex API
   - Recommendation: Option A for initial implementation, Option C for future enhancement if instability observed

2. **Multiple Potential Energy Support**
   - Question: Should WorldModel support multiple simultaneous PotentialEnergy instances initially?
   - Option A: Single PotentialEnergy instance — Pros: Simpler implementation, sufficient for gravity-only requirement. Cons: Limits extensibility
   - Option B: Vector of PotentialEnergy instances — Pros: Fully extensible, enables future tidal/magnetic fields. Cons: Slightly more complex
   - Recommendation: Option B — minimal complexity increase, enables future-proofing for known use cases (tidal forces in multi-body scenarios)

3. **Euler Angle Deprecation Strategy**
   - Question: How should deprecated Euler angle accessors behave when conversion is ambiguous?
   - Option A: Use Eigen's default Euler angle extraction (ZYX convention) — Pros: Simple, consistent. Cons: No gimbal lock handling
   - Option B: Detect near-gimbal-lock and return approximate Euler angles with warning — Pros: More robust. Cons: Complexity, warning infrastructure needed
   - Recommendation: Option A — gimbal lock is Euler angles' fundamental limitation; users should migrate to quaternions

4. **InertialState Storage Strategy**
   - Question: Should InertialState store both Q and Q_normalized, or normalize on-access?
   - Option A: Store only Q, normalize on-access — Pros: Smaller memory footprint. Cons: Performance overhead on every read
   - Option B: Store Q and normalized Q_normalized — Pros: Fast access. Cons: 32 extra bytes per object, requires synchronization
   - Option C: Store only Q, enforce constraint in integration loop — Pros: Minimal memory, correctness guaranteed by physics. Cons: Slightly more complex integration
   - Recommendation: Option C — constraint enforcement in integration loop is required anyway for Baumgarte stabilization, no need for redundant storage

### Prototype Required

1. **Baumgarte Parameter Tuning**
   - Uncertainty: Optimal α, β values for numerical stability with semi-implicit Euler integration
   - Prototype approach:
     - Implement physics loop with configurable α, β
     - Run 10000-step simulations with varying timesteps (0.001s - 0.1s)
     - Measure quaternion constraint violation drift over time
     - Test corner cases: high angular velocity, near-gimbal-lock initial conditions
   - Success criteria: Find (α, β) pair that maintains `|Q| - 1| < 1e-10` over 10000 steps for dt=0.016s

2. **Performance Overhead Validation**
   - Uncertainty: Whether quaternion integration overhead exceeds 10% requirement
   - Prototype approach:
     - Benchmark current Euler-based `updatePhysics()` with 100 objects
     - Benchmark quaternion-based `updatePhysics()` with 100 objects
     - Measure per-step wall-clock time over 1000 iterations
   - Success criteria: Quaternion integration < 110% of Euler integration time

### Requirements Clarification

1. **Collision Response Integration Scope**
   - Ambiguity: Ticket states collision response integration is OUT OF SCOPE, but existing `updateCollisions()` modifies velocities
   - Question: Should existing collision response be temporarily disabled, or should it continue to work with the new quaternion state?
   - Clarification needed: Define behavior of collision response during transition period before collision integration ticket

2. **Backward Compatibility Requirements**
   - Ambiguity: "Must preserve current WorldModel API surface for external consumers"
   - Question: Are there external consumers outside the MSD-CPP repository? If so, which APIs are considered public vs internal?
   - Clarification needed: Identify which breaking changes are acceptable vs which require migration paths

3. **Euler Angle Deprecation Timeline**
   - Ambiguity: Deprecated methods provide backward compatibility, but no removal timeline specified
   - Question: When should deprecated Euler angle methods be removed? Next major version? Never?
   - Clarification needed: Define deprecation policy and removal timeline

## Notes

### Mathematical Background

#### Quaternion Kinematics

Quaternions represent orientation as a 4-component unit vector Q = [w, x, y, z]^T where w² + x² + y² + z² = 1.

**Quaternion Rate Equation**:
```
Q̇ = ½ * Q ⊗ [0, ω]
```
where ω is the angular velocity vector in world frame and ⊗ denotes quaternion multiplication.

**Inverse Conversion** (Q̇ → ω):
```
ω = 2 * Q̄ ⊗ Q̇
```
where Q̄ is the quaternion conjugate.

#### Lagrangian Mechanics Formulation

**Lagrangian**: L = T - V
- T = ½m|Ẋ|² + ½ωᵀIω (kinetic energy)
- V = V(X, Q) (potential energy)

**Generalized Forces**:
- F = -∂V/∂X (linear force from position gradient)
- τ = -∂V/∂Q (torque from orientation gradient, converted to ω space)

#### Constraint Enforcement

**Position Constraint**: g(Q) = QᵀQ - 1 = 0

**Velocity Constraint**: ġ = 2QᵀQ̇ = 0

**Baumgarte Stabilization**:
```
λ = -α * g - β * ġ
F_constraint = Gᵀ * λ where G = ∂g/∂Q = 2Qᵀ
```

The Baumgarte method adds feedback terms proportional to constraint violation (α) and violation rate (β) to exponentially reduce drift over time.

### Design Rationale

#### Why Quaternions Over Euler Angles?

1. **No Gimbal Lock**: Euler angles have singularities at pitch = ±90° where the Jacobian matrix becomes singular. Quaternions have no singularities.

2. **Numerical Stability**: Quaternion normalization is a single scalar constraint. Euler angle rate transformation requires trigonometric functions that amplify numerical errors near singularities.

3. **Interpolation**: Quaternion SLERP (spherical linear interpolation) provides smooth orientation interpolation. Euler angle interpolation can pass through gimbal lock.

4. **Industry Standard**: Modern physics engines (PhysX, Bullet, ODE) use quaternions for orientation representation.

#### Why Lagrange Multipliers Over Direct Normalization?

1. **Physical Correctness**: Lagrange multipliers represent constraint forces in the system's dynamics. Direct normalization is a kinematic correction that violates energy conservation.

2. **Baumgarte Stabilization**: Combining Lagrange multipliers with Baumgarte feedback provides exponential convergence to the constraint manifold, eliminating accumulated drift.

3. **Extensibility**: The constraint framework generalizes to additional constraints (e.g., fixed joints, hinges) in future multi-body dynamics.

#### Why Potential Energy Abstraction?

1. **Separation of Concerns**: Physics integration logic (WorldModel) decoupled from force computation logic (PotentialEnergy implementations).

2. **Extensibility**: New force fields (tidal, magnetic, atmospheric drag) can be added without modifying WorldModel.

3. **Testability**: Potential energy implementations can be unit tested independently of the physics loop.

4. **Lagrangian Framework**: Potential energy abstraction aligns with Lagrangian mechanics formulation, enabling energy-based analysis (conservation, stability).

### Implementation Sequencing

Recommended implementation order to minimize risk:

1. **Phase 1: InertialState Refactoring** (Low risk)
   - Add `Eigen::Quaterniond orientation` and `Eigen::Vector4d quaternionRate`
   - Implement Q̇↔ω conversion utilities
   - Add unit tests for conversions
   - Mark `AngularCoordinate orientation` as deprecated
   - No changes to WorldModel yet (physics still uses Euler angles internally)

2. **Phase 2: Potential Energy Abstraction** (Medium risk)
   - Create `PotentialEnergy` interface and `GravityPotential` implementation
   - Add `potentialEnergies_` vector to WorldModel
   - Unit test GravityPotential in isolation
   - No changes to `updatePhysics()` yet

3. **Phase 3: Quaternion Constraint System** (Medium risk)
   - Implement `QuaternionConstraint` class
   - Unit test constraint enforcement and Baumgarte stabilization
   - Prototype to determine optimal α, β parameters
   - No integration into WorldModel yet

4. **Phase 4: Physics Integration Update** (High risk)
   - Refactor `WorldModel::updatePhysics()` to use quaternions and potential energies
   - Apply quaternion constraint in integration loop
   - Integration tests for gravity, gimbal lock avoidance, energy conservation
   - Benchmark performance vs. Euler-based integration

5. **Phase 5: ReferenceFrame Update** (Low risk)
   - Update ReferenceFrame to use `Eigen::Quaterniond` internally
   - Mark AngularCoordinate methods as deprecated
   - Update all transformation tests

This sequencing allows for incremental integration with testing at each phase boundary.

### Performance Considerations

**Quaternion Operations Cost**:
- Quaternion multiplication: ~16 FLOPs
- Quaternion normalization: ~8 FLOPs (sqrt + 4 divisions)
- Euler angle rate Jacobian: ~24 FLOPs + 6 trig function calls

**Expected Performance**:
- Quaternion integration: ~40 FLOPs per object per step
- Euler integration with Jacobian: ~100+ FLOPs per object per step
- **Prediction**: Quaternion approach should be **faster** than current Euler implementation despite added constraint enforcement

**Memory Overhead**:
- Quaternion: 4 doubles = 32 bytes
- Quaternion rate: 4 doubles = 32 bytes
- Total added per object: 64 bytes
- For 1000 objects: ~64 KB (negligible for modern systems)

### Future Extensions

This design enables future enhancements:

1. **Tidal Forces**: Implement `TidalPotential` with orientation-dependent energy V(X, Q)
2. **Magnetic Torques**: Implement `MagneticPotential` for spacecraft attitude control
3. **Atmospheric Drag**: Implement `DragPotential` with velocity-dependent dissipation
4. **Multi-Body Constraints**: Generalize `QuaternionConstraint` to holonomic and non-holonomic constraints
5. **Implicit Integration**: Upgrade from semi-implicit Euler to implicit methods (backward Euler, trapezoidal) using constraint Jacobians
6. **Contact Constraints**: Integrate collision response as Lagrangian constraints (frictionless and friction cones)

All of these extensions fit naturally into the Lagrangian mechanics framework established by this design.

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
| Naming conventions | ✓ | Classes use PascalCase (`PotentialEnergy`, `GravityPotential`, `QuaternionConstraint`), methods use camelCase (`computeForce`, `enforceConstraint`), members use snake_case_ (`alpha_`, `beta_`, `g_`). Fully consistent with project standards. |
| Namespace organization | ✓ | New components correctly placed in `msd_sim` namespace. Directory structure follows established patterns: `Physics/PotentialEnergy/`, `Physics/Constraints/`. |
| File structure | ✓ | Headers in `src/Physics/`, sources in corresponding `.cpp` files. Matches existing pattern (`src/Physics/CollisionResponse.hpp`, `src/Physics/RigidBody/AssetInertial.hpp`). Pure abstract interface (`PotentialEnergy`) is header-only, concrete implementations have `.cpp` files. |
| Dependency direction | ✓ | Clean dependency flow: `WorldModel` → `PotentialEnergy` (interface) ← `GravityPotential` (impl). `QuaternionConstraint` has no dependencies on simulation (only Eigen). No circular dependencies. Respects msd-sim layering. |

#### C++ Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| RAII usage | ✓ | All classes use compiler-generated destructors with proper cleanup. `QuaternionConstraint` has no resources to manage. `WorldModel::potentialEnergies_` uses `std::unique_ptr` for automatic cleanup. |
| Smart pointer appropriateness | ✓ | **Excellent adherence to CLAUDE.md standards**: `WorldModel` owns `std::vector<std::unique_ptr<PotentialEnergy>>` for exclusive ownership transfer. `PotentialEnergy` interface takes `const InertialState&` (non-owning reference) as required. No `shared_ptr` used. Perfect alignment with project memory management conventions. |
| Value/reference semantics | ✓ | `QuaternionConstraint` stored by value in `WorldModel` (lightweight, 16 bytes). `InertialState` passed by const reference to avoid copies. `Eigen::Quaterniond` and `Eigen::Vector4d` used appropriately as value types. |
| Rule of 0/3/5 | ✓ | All classes explicitly declare Rule of Five with `= default` (`PotentialEnergy` protected copy/move, `GravityPotential` public defaulted, `QuaternionConstraint` public defaulted). No manual implementations where compiler can generate correct behavior. Perfect adherence to CLAUDE.md guidance. |
| Const correctness | ✓ | All query methods marked const (`computeForce`, `computeTorque`, `computeEnergy`, `getGravity`, `positionViolation`). Mutation methods not const (`enforceConstraint`, `setAlpha`, `setBeta`, `setGravity`). `PotentialEnergy` interface read-only after construction per thread safety requirements. |
| Exception safety | ✓ | Exception guarantees documented in design notes. `GravityPotential` no-throw (arbitrary gravity vectors valid including zero). `QuaternionConstraint` no-throw (quaternion ops numerically stable). `PotentialEnergy` implementations may throw for invalid state (documented). Strong exception safety for WorldModel operations (unique_ptr provides automatic cleanup on throw). |
| Initialization | ✓ | All member variables initialized with brace initialization: `alpha_{10.0}`, `beta_{10.0}`, `g_{0.0, 0.0, -9.81}`. Uninitialized quaternion uses `Eigen::Quaterniond` default constructor (identity). Consistent with CLAUDE.md standard. |
| Return values | ✓ | Design prefers returning values over output parameters: `computeForce()` returns `CoordinateRate`, `computeConstraintForce()` returns `Eigen::Vector4d`, `positionViolation()` returns `double`. No unnecessary output parameters. Aligns with CLAUDE.md guidance. |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Header dependencies | ✓ | No circular dependencies. Forward declaration possible for `InertialState` in `PotentialEnergy.hpp` if needed. `QuaternionConstraint` only depends on Eigen (external). Clean dependency graph. |
| Template complexity | ✓ | No templates used. All interfaces use concrete types. Maintains simplicity. |
| Memory strategy | ✓ | Clear ownership: WorldModel owns potentials via unique_ptr, constraint by value. No shared ownership complexity. Estimated memory: ~50 bytes per GravityPotential, 16 bytes for QuaternionConstraint, ~16 bytes per unique_ptr in vector. Negligible for typical simulations. |
| Thread safety | ✓ | Thread safety requirements clearly documented. `PotentialEnergy` read-only after construction (thread-safe). `QuaternionConstraint::enforceConstraint()` mutates state (not thread-safe, but called sequentially in single-threaded physics loop). No problematic global state or singletons. |
| Build integration | ✓ | New source files integrate cleanly into existing CMake structure. `PotentialEnergy.hpp` header-only (no build changes). `GravityPotential.cpp` and `QuaternionConstraint.cpp` add to `msd_sim` library sources. No new external dependencies beyond existing Eigen. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | All classes can be instantiated independently: `GravityPotential` constructible with gravity vector, `QuaternionConstraint` constructible with α/β parameters, no required WorldModel context for testing. Perfect for unit testing. |
| Mockable dependencies | ✓ | `PotentialEnergy` is pure abstract interface, trivially mockable for WorldModel testing. `QuaternionConstraint` has no dependencies (only Eigen), easily testable in isolation. `GravityPotential` has no dependencies beyond InertialState parameter. |
| Observable state | ✓ | All classes provide query methods for state validation: `getGravity()`, `getAlpha()`, `getBeta()`, `positionViolation()`, `velocityViolation()`. Constraint violations observable for test assertions. Computed forces/torques/energies returned as observable values. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Baumgarte stabilization parameters (α=10, β=10) may not provide optimal stability for the specific timestep and integration method used in this simulation | Performance | Medium | Medium | Start with literature defaults (α=10, β=10). Add diagnostic tests measuring constraint violation drift over 10000 steps. Make parameters configurable via `QuaternionConstraint` constructor if tuning needed. Prototype to determine optimal values. | Yes |
| R2 | Quaternion integration overhead may exceed 10% performance requirement compared to current Euler integration | Performance | Low | Medium | Design already optimizes by storing Q̇ directly in state (avoids repeated ω→Q̇ conversion). Constraint enforcement is O(1) with ~40 FLOPs vs Euler Jacobian ~100+ FLOPs. Prototype benchmark validates performance meets requirement. | Yes |
| R3 | Conversion between Q̇ and ω may accumulate numerical errors over long simulations if not properly normalized | Technical | Low | High | `QuaternionConstraint::enforceConstraint()` normalizes Q and projects Q̇ every integration step, preventing drift accumulation. Use Baumgarte feedback to exponentially reduce constraint violations. Validation test: maintain |Q|=1 within 1e-10 over 10000 steps. | No |
| R4 | Breaking changes to `InertialState` and `ReferenceFrame` will require extensive migration of existing code and tests | Integration | High | Medium | Design provides deprecated accessor methods (`getEulerAngles()`, deprecated `ReferenceFrame` constructors) for backward compatibility. Migration can be incremental: deprecation warnings guide updates. Implementation sequencing (Phase 1-5) minimizes risk by testing each component before integration. | No |
| R5 | Collision response integration deferred to separate ticket may cause temporary inconsistency where angular dynamics work under gravity but not collisions | Integration | Medium | Low | Explicitly documented as OUT OF SCOPE per ticket requirements. Collision response system (ticket 0027) already exists with angular impulse support. Future ticket will integrate quaternion state with existing collision response (minimal changes, since collision response uses witness points and lever arms, not Euler angles). No blocking dependency. | No |

### Prototype Guidance

#### Prototype P1: Baumgarte Parameter Tuning

**Risk addressed**: R1
**Question to answer**: What values of α and β maintain quaternion constraint |Q|=1 within 1e-10 tolerance over 10000 integration steps with dt=0.016s (60 FPS)?

**Success criteria**:
- Constraint violation |Q²-1| < 1e-10 after 10000 steps (160 seconds simulation)
- No oscillatory instability (violation should decrease or remain bounded, not grow)
- Parameters work across different initial conditions (identity quaternion, 45° rotations, 90° pitch, high angular velocity)
- Performance overhead < 5% compared to no constraint enforcement (baseline)

**Prototype approach**:
```
Location: prototypes/0030_lagrangian_quaternion_physics/p1_baumgarte_tuning/
Type: Standalone executable with parameterized α, β

Steps:
1. Implement simplified physics loop with quaternion integration
   - Semi-implicit Euler: Q_new = Q_old + Qdot * dt
   - Apply QuaternionConstraint::enforceConstraint(Q, Qdot) each step
   - Log constraint violation: |Q.squaredNorm() - 1|
2. Test parameter ranges:
   - α ∈ {5, 10, 15, 20, 25}
   - β ∈ {5, 10, 15, 20, 25}
   - Timesteps dt ∈ {0.001, 0.008, 0.016, 0.033, 0.1}
3. Test corner cases:
   - Identity quaternion with zero angular velocity
   - 90° pitch (near-gimbal-lock for Euler angles)
   - High angular velocity (10 rad/s)
   - Rapidly changing angular acceleration (oscillating torque)
4. Measure performance:
   - Time 10000 iterations with constraint vs without
   - Target: < 5% overhead
5. Analyze results:
   - Plot constraint violation vs iteration for each (α, β) pair
   - Identify parameter combinations meeting 1e-10 tolerance
   - Select default values balancing stability and performance
```

**Time box**: 2 hours

**If prototype fails**:
- If no (α, β) pair meets 1e-10 tolerance: Relax tolerance to 1e-8 (still acceptable for double precision)
- If performance exceeds 5%: Profile constraint enforcement, optimize Eigen operations, consider normalization-only (no Baumgarte) as fallback
- If instability detected: Investigate integration method (consider backward Euler or trapezoidal for constraint enforcement step)

#### Prototype P2: Performance Overhead Validation

**Risk addressed**: R2
**Question to answer**: Does quaternion-based integration with constraint enforcement execute within 110% of current Euler-based integration time?

**Success criteria**:
- Quaternion integration (100 objects, 1000 steps) completes in < 110% time of Euler integration
- Per-step timing variance < 10% (consistent performance)
- Memory overhead < 100 KB for 100 objects (64 bytes/object * 100 = 6.4 KB baseline)

**Prototype approach**:
```
Location: prototypes/0030_lagrangian_quaternion_physics/p2_performance_validation/
Type: Benchmark executable using Google Benchmark (if available) or manual timing

Steps:
1. Implement baseline (current Euler integration):
   - Read current WorldModel::updatePhysics() implementation
   - Extract physics loop: F=ma, torque, Euler angle integration
   - Benchmark 100 objects over 1000 steps
   - Record per-step min/mean/max time
2. Implement quaternion integration:
   - Same physics loop structure
   - Replace Euler angle integration with:
     * ω → Q̇ conversion: Qdot = 0.5 * Q ⊗ [0, ω]
     * Quaternion integration: Q_new = Q_old + Qdot * dt
     * Constraint enforcement: enforceConstraint(Q, Qdot)
     * Q̇ → ω conversion (if needed for output)
   - Benchmark identical scenario
3. Memory profiling:
   - Measure heap allocations during physics loop (Valgrind massif or Instruments)
   - Verify no unexpected allocations in hot path
   - Confirm memory overhead matches theoretical 64 bytes/object
4. Comparison:
   - Compute speedup ratio: time_quaternion / time_euler
   - Verify ratio < 1.10 (quaternion at most 10% slower)
   - Profile hotspots if ratio exceeds threshold
```

**Time box**: 1.5 hours

**If prototype fails**:
- If overhead > 10%: Profile to identify bottleneck (likely Q̇↔ω conversion or constraint enforcement)
  - Optimization 1: Store ω directly, skip Q̇→ω conversion for most operations
  - Optimization 2: Batch constraint enforcement (vectorize across objects)
  - Optimization 3: Reduce constraint enforcement frequency (every N steps)
- If memory overhead excessive: Check for unexpected heap allocations, use stack-based temporaries

### Required Revisions

None. Design is ready for prototyping phase.

### Blocking Issues

None.

### Summary

The design demonstrates excellent architectural quality and strong alignment with project coding standards. The quaternion-based state representation with Lagrangian mechanics provides a mathematically rigorous foundation for eliminating gimbal lock and enabling future extensibility.

**Strengths**:
1. **Exceptional adherence to CLAUDE.md standards**: Smart pointer usage, memory management, Rule of Five, initialization, and return values all follow project conventions perfectly
2. **Clean architecture**: PotentialEnergy abstraction provides clear separation of concerns and extensibility
3. **Testability**: All components designed for isolated unit testing with observable state
4. **Well-documented risks**: Breaking changes acknowledged with migration strategies
5. **Performance-conscious**: Direct Q̇ storage and optimized constraint enforcement minimize overhead

**Notes for Implementation**:
1. **Prototype P1 (Baumgarte tuning) is REQUIRED** before implementation to determine optimal α, β parameters and validate 1e-10 constraint tolerance is achievable
2. **Prototype P2 (performance validation) is REQUIRED** to confirm quaternion integration meets the < 10% overhead requirement stated in non-functional requirements
3. Breaking changes to `InertialState` and `ReferenceFrame` are significant but well-mitigated through deprecated accessors and incremental implementation phases
4. Collision response integration deferred per ticket scope is acceptable; existing collision response system (ticket 0027) already supports angular impulses via witness points, making future integration straightforward

**Recommended Next Steps**:
1. Execute Prototype P1 to determine optimal Baumgarte parameters (α, β) — 2 hours
2. Execute Prototype P2 to validate performance requirement — 1.5 hours
3. Proceed to implementation following phased sequencing (Phase 1-5) outlined in design
4. Update ticket status to "Design Approved — Ready for Prototype"

The design is **APPROVED** for prototyping. Once prototypes validate parameter choices and performance requirements, implementation can proceed with high confidence.
