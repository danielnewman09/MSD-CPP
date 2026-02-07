# Ticket 0039a: Energy Tracking Diagnostic Infrastructure

## Status
- [x] Draft
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Assignee**: TBD
**Created**: 2026-02-05
**Updated**: 2026-02-06
**Generate Tutorial**: No
**Parent Ticket**: [0039_collision_energy_stabilization_debug](0039_collision_energy_stabilization_debug.md)
**Depends On**: [0038_simulation_data_recorder](0038_simulation_data_recorder.md) ✅ Merged
**Type**: Infrastructure

---

## Overview

This ticket creates the energy tracking infrastructure required to investigate the collision energy stabilization bug. The DataRecorder infrastructure (ticket 0038) provides the persistence layer; this ticket adds energy computation and recording.

**Key Insight**: The DataRecorder already handles frame timestamping, per-body state recording, SQLite persistence, background threading, and opt-in enablement. This ticket focuses on:
1. Computing energy values correctly (especially world-frame rotational KE)
2. Extending records to include energy data
3. Adding energy change detection for anomaly identification

---

## Foundation: What DataRecorder Provides

The following infrastructure from ticket 0038 is already complete:

| Capability | Implementation | Status |
|------------|----------------|--------|
| Frame timestamping | `SimulationFrameRecord` | ✅ Complete |
| Per-body kinematic state | `InertialStateRecord` | ✅ Complete |
| Position, velocity, orientation, angular velocity | Fields in `InertialStateRecord` | ✅ Complete |
| SQLite persistence | `DataRecorder` with background thread | ✅ Complete |
| Opt-in recording | `WorldModel::enableRecording()` | ✅ Complete |
| Thread-safe buffering | Double-buffer DAOs | ✅ Complete |
| Configurable flush interval | `DataRecorder::Config` | ✅ Complete |

---

## Requirements

### R1: Energy Tracker Utility

Create an `EnergyTracker` class that computes total mechanical energy for rigid body systems.

#### R1.1: Linear Kinetic Energy
```
KE_linear = ½ m v²
```
Where `v` is the center-of-mass velocity magnitude.

#### R1.2: Rotational Kinetic Energy (CRITICAL)
```
KE_rot = ½ ω^T I_world ω
```
Where:
- `ω` is angular velocity in world frame
- `I_world = R I_body R^T` is the inertia tensor transformed to world frame
- `R` is the rotation matrix from body to world frame

**WARNING**: Using body-frame inertia tensor with world-frame angular velocity produces chaotic-looking energy even when physics are correct. This is a common diagnostic pitfall.

#### R1.3: Gravitational Potential Energy
```
PE = -m (g · r)
```
Where:
- `g` is the gravity vector (e.g., `(0, 0, -9.81)` m/s²)
- `r` is the center of mass position
- The dot product supports arbitrary gravity directions

#### R1.4: Total Mechanical Energy
```
E_total = KE_linear + KE_rot + PE
```

### R2: Energy Record Extension

Extend the data recording to include computed energy values.

#### R2.1: Create EnergyRecord

Create a new transfer record to store per-body energy values:

```cpp
struct EnergyRecord : public cpp_sqlite::BaseTransferObject {
  uint32_t body_id;           // Identifier for the rigid body
  double linear_ke;           // Linear kinetic energy [J]
  double rotational_ke;       // Rotational kinetic energy [J]
  double potential_e;         // Gravitational potential energy [J]
  double total_e;             // Total mechanical energy [J]
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

**Rationale**: Separate record (vs extending `InertialStateRecord`) keeps energy diagnostics cleanly separated and allows recording energy without modifying the core kinematic record schema.

#### R2.2: Body Identification

Add `body_id` field to enable tracking individual bodies across frames. This should be a stable identifier (not vector index) that persists across the simulation.

### R3: Energy Change Detection

#### R3.1: System Energy Record

Create a per-frame system-level energy summary:

```cpp
struct SystemEnergyRecord : public cpp_sqlite::BaseTransferObject {
  double total_linear_ke;     // Sum of all body linear KE
  double total_rotational_ke; // Sum of all body rotational KE
  double total_potential_e;   // Sum of all body PE
  double total_system_e;      // Total system energy
  double delta_e;             // Change from previous frame
  bool energy_injection;      // True if ΔE > ε (anomaly flag)
  bool collision_active;      // True if any collision this frame
  cpp_sqlite::ForeignKey<SimulationFrameRecord> frame;
};
```

#### R3.2: Anomaly Detection Threshold

Flag frames where energy increased beyond numerical tolerance:
- Relative tolerance: `ε = 1e-6 * |E_total|`
- Absolute tolerance: `ε = 1e-9 J` (for near-zero energy states)
- Use whichever is larger

#### R3.3: Collision Correlation

The `collision_active` flag enables post-hoc queries like:
```sql
SELECT * FROM SystemEnergyRecord
WHERE energy_injection = 1 AND collision_active = 1;
```

### R4: Integration with DataRecorder

#### R4.1: Extend WorldModel::recordCurrentFrame()

After recording kinematic state, compute and record energy:

```cpp
void WorldModel::recordCurrentFrame() {
  // ... existing kinematic recording ...

  // Compute and record energy for each body
  auto& energyDAO = dataRecorder_->getDAO<msd_transfer::EnergyRecord>();
  for (const auto& asset : inertialAssets_) {
    auto energy = EnergyTracker::computeBodyEnergy(
      asset.getInertialState(),
      asset.getMass(),
      asset.getBodyInertia(),
      gravityVector_
    );
    energyDAO.addToBuffer(energy.toRecord(frameId, asset.getId()));
  }

  // Compute and record system energy summary
  auto& sysEnergyDAO = dataRecorder_->getDAO<msd_transfer::SystemEnergyRecord>();
  auto sysEnergy = EnergyTracker::computeSystemEnergy(inertialAssets_, gravityVector_);
  sysEnergyDAO.addToBuffer(sysEnergy.toRecord(frameId, previousSystemEnergy_));
  previousSystemEnergy_ = sysEnergy;
}
```

---

## Deferred Requirements

The following requirements from the original ticket are **deferred** to follow-up tickets if root cause analysis requires them:

| Requirement | Reason for Deferral |
|-------------|---------------------|
| R2.3: Per-Contact State | Requires solver instrumentation; add if energy injection correlates with contacts |
| R2.4: Force Audit Trail | Deep solver instrumentation; add if energy injection source unclear |
| R2.5: Solver State | Iteration counts, residuals; add if solver convergence suspected |
| R2.6: Pre/Post-Solve Energy | Requires collision pipeline hooks; add to isolate integrator vs solver |
| R4.4: Ring Buffer Mode | SQLite handles large datasets; optimize only if I/O becomes bottleneck |

These can be added incrementally in tickets 0039b-0039e as investigation proceeds.

---

## Implementation Approach

### EnergyTracker Class

```cpp
// msd-sim/src/Diagnostics/EnergyTracker.hpp

namespace msd_sim {

class EnergyTracker {
public:
  struct BodyEnergy {
    double linearKE{0.0};
    double rotationalKE{0.0};
    double potentialE{0.0};

    double total() const { return linearKE + rotationalKE + potentialE; }

    msd_transfer::EnergyRecord toRecord(uint32_t frameId, uint32_t bodyId) const;
  };

  struct SystemEnergy {
    double totalLinearKE{0.0};
    double totalRotationalKE{0.0};
    double totalPotentialE{0.0};

    double total() const {
      return totalLinearKE + totalRotationalKE + totalPotentialE;
    }

    msd_transfer::SystemEnergyRecord toRecord(
      uint32_t frameId,
      const SystemEnergy& previous,
      bool collisionActive) const;
  };

  /// Compute energy for a single rigid body
  /// @param state Current kinematic state
  /// @param mass Body mass [kg]
  /// @param bodyInertia Inertia tensor in body frame [kg·m²]
  /// @param gravity Gravity vector in world frame [m/s²]
  static BodyEnergy computeBodyEnergy(
    const InertialState& state,
    double mass,
    const Eigen::Matrix3d& bodyInertia,
    const Eigen::Vector3d& gravity);

  /// Compute total system energy across all bodies
  static SystemEnergy computeSystemEnergy(
    std::span<const AssetInertial> bodies,
    const Eigen::Vector3d& gravity);

  /// Check if energy change exceeds tolerance (anomaly detection)
  static bool isEnergyInjection(
    double currentEnergy,
    double previousEnergy,
    double relativeTolerance = 1e-6,
    double absoluteTolerance = 1e-9);
};

} // namespace msd_sim
```

### Key Implementation Detail: World-Frame Rotational KE

```cpp
BodyEnergy EnergyTracker::computeBodyEnergy(
    const InertialState& state,
    double mass,
    const Eigen::Matrix3d& bodyInertia,
    const Eigen::Vector3d& gravity) {

  BodyEnergy result;

  // Linear KE: ½mv²
  Eigen::Vector3d velocity = state.getVelocity();
  result.linearKE = 0.5 * mass * velocity.squaredNorm();

  // Rotational KE: ½ω^T I_world ω
  // CRITICAL: Transform inertia to world frame
  Eigen::Matrix3d R = state.getOrientation().toRotationMatrix();
  Eigen::Matrix3d I_world = R * bodyInertia * R.transpose();
  Eigen::Vector3d omega = state.getAngularVelocity();
  result.rotationalKE = 0.5 * omega.transpose() * I_world * omega;

  // Potential energy: -m(g·r)
  Eigen::Vector3d position = state.getPosition();
  result.potentialE = -mass * gravity.dot(position);

  return result;
}
```

---

## Test Plan

### Unit Tests

```cpp
// Verify linear KE computation
TEST(EnergyTracker, LinearKE_MovingSphere)
TEST(EnergyTracker, LinearKE_StationaryBody_ReturnsZero)

// Verify rotational KE with world-frame tensor (CRITICAL)
TEST(EnergyTracker, RotationalKE_SpinningCube_AxisAligned)
TEST(EnergyTracker, RotationalKE_SpinningCube_Tilted)
TEST(EnergyTracker, RotationalKE_ConsistentAcrossOrientations)

// Verify PE computation
TEST(EnergyTracker, PotentialEnergy_HeightProportional)
TEST(EnergyTracker, PotentialEnergy_ArbitraryGravityDirection)

// Verify total energy conservation (no collision)
TEST(EnergyTracker, FreeFall_TotalEnergyConstant)
TEST(EnergyTracker, Tumbling_TotalEnergyConstant)

// Verify anomaly detection
TEST(EnergyTracker, IsEnergyInjection_DetectsIncrease)
TEST(EnergyTracker, IsEnergyInjection_IgnoresNumericalNoise)
TEST(EnergyTracker, IsEnergyInjection_AllowsEnergyDecrease)
```

### Integration Tests

```cpp
// Verify energy recording with DataRecorder
TEST(EnergyRecording, RecordsEnergyPerBody)
TEST(EnergyRecording, RecordsSystemEnergySummary)
TEST(EnergyRecording, FlagsEnergyInjection)

// Verify SQLite queryability
TEST(EnergyRecording, QueryEnergyAnomalies)
```

---

## Acceptance Criteria

1. [ ] **AC1**: `EnergyTracker` class implemented with correct world-frame rotational KE
2. [ ] **AC2**: `EnergyRecord` transfer object created and registered
3. [ ] **AC3**: `SystemEnergyRecord` transfer object created and registered
4. [ ] **AC4**: `WorldModel::recordCurrentFrame()` extended to record energy data
5. [ ] **AC5**: Energy injection detection (ΔE > ε flagging) implemented
6. [ ] **AC6**: All unit tests pass
7. [ ] **AC7**: Free-fall test shows constant total energy (validates baseline)
8. [ ] **AC8**: Tumbling rigid body test shows constant total energy

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd-sim/src/Diagnostics/EnergyTracker.hpp` | Energy computation utility |
| `msd-sim/src/Diagnostics/EnergyTracker.cpp` | Implementation |
| `msd-transfer/src/EnergyRecord.hpp` | Per-body energy transfer object |
| `msd-transfer/src/SystemEnergyRecord.hpp` | System energy summary transfer object |
| `msd-sim/test/Diagnostics/EnergyTrackerTest.cpp` | Unit tests |

### Modified Files
| File | Change |
|------|--------|
| `msd-sim/CMakeLists.txt` | Add Diagnostics directory |
| `msd-transfer/src/Records.hpp` | Include new record headers |
| `msd-sim/src/Environment/WorldModel.cpp` | Extend `recordCurrentFrame()` |

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-05
- **Notes**: Subticket split from parent 0039. Focuses on diagnostic infrastructure required before investigation can proceed.

### Gemini Review (2026-02-05)
**Status: Approved with Modifications**

Key changes incorporated:
1. **R1.3**: Changed PE formula from `mgh` to dot-product form `-m(g·r)` for arbitrary gravity support
2. **R2.5**: Added solver state logging (iteration count, residual, max penetration, warm-start λ)
3. **R2.6**: Added explicit Pre-Solve vs Post-Solve energy tracking requirement
4. **R4.4**: Added ring buffer mode for triggered logging (reduces I/O, avoids Heisenbugs)
5. **Option B**: Changed from simple hooks to Observer Pattern (`IPhysicsDebugListener`)

### Scope Revision (2026-02-06)
**Status: Simplified based on DataRecorder foundation**

The DataRecorder infrastructure (ticket 0038) was merged, providing:
- Frame timestamping (`SimulationFrameRecord`)
- Per-body kinematic state (`InertialStateRecord`)
- SQLite persistence with background threading
- Opt-in enablement via `WorldModel::enableRecording()`

**Revised scope**:
- **Kept**: R1 (EnergyTracker utility), R3 (Energy change detection)
- **Simplified**: R2 (Create new EnergyRecord instead of CSV logging)
- **Removed**: R4 (CSV/JSON output - SQLite is superior for analysis)
- **Deferred**: Contact/Force/Solver logging to follow-up tickets (0039b-0039e)

This reduces implementation effort while maintaining the critical capability: detecting when and where energy is injected during collision resolution.

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

