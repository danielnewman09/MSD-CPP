# Ticket 0038a: Energy Tracking Diagnostic Infrastructure

## Status
- [x] Draft
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Documentation Complete
- [ ] Merged / Complete

**Current Phase**: Draft
**Assignee**: TBD
**Created**: 2026-02-05
**Generate Tutorial**: No
**Parent Ticket**: [0038_collision_energy_stabilization_debug](0038_collision_energy_stabilization_debug.md)
**Type**: Infrastructure

---

## Overview

This ticket creates the diagnostic infrastructure required to investigate the collision energy stabilization bug. Without accurate energy measurement and logging, debugging is blind guesswork.

**Blocking**: All subsequent 0038x tickets depend on this infrastructure.

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

**Note**: The formula `PE = mgh` is a simplification assuming gravity aligned with a coordinate axis. Use the dot product form for general support.

#### R1.4: Total Mechanical Energy
```
E_total = KE_linear + KE_rot + PE
```

### R2: Per-Frame Logging Infrastructure

Create a logging system that captures collision pipeline state at each simulation frame.

#### R2.1: Frame Header
- Frame number
- Simulation time
- Timestep (dt)

#### R2.2: Per-Body State
For each rigid body:
- Body ID
- Position (x, y, z)
- Velocity (vx, vy, vz)
- Orientation (quaternion: w, x, y, z)
- Angular velocity (ωx, ωy, ωz)
- Linear KE, Rotational KE, PE, Total E

#### R2.3: Per-Contact State
For each active contact constraint:
- Contact ID
- Body A index, Body B index
- Contact normal (nx, ny, nz)
- Contact point A (world frame)
- Contact point B (world frame)
- Penetration depth
- Pre-solve relative normal velocity
- Post-solve relative normal velocity
- Lagrange multiplier (λ)

#### R2.4: Force Audit Trail
For each body receiving constraint forces:
- Body ID
- Applied linear force (fx, fy, fz)
- Applied torque (τx, τy, τz)
- Pre-apply velocity
- Post-apply velocity
- Expected Δv (from impulse/mass)
- Actual Δv

#### R2.5: Solver State (NEW - from Gemini review)
For diagnosing solver-related energy injection:
- Solver iteration count (how many iterations before convergence/cap)
- Final velocity residual (convergence quality)
- Max penetration depth per frame
- Warm-start initial λ values (if using cached impulses)

#### R2.6: Pre-Solve vs Post-Solve Energy (NEW - from Gemini review)
**Critical for isolating integrator vs solver bugs:**
- Energy **Pre-Solve**: After external force integration, before constraint solving
- Energy **Post-Solve**: After constraint forces applied
- Energy delta during solve phase specifically

### R3: Energy Change Detection

#### R3.1: Frame-to-Frame Delta
Compute and log:
```
ΔE = E_total(frame N) - E_total(frame N-1)
```

#### R3.2: Energy Injection Flag
Flag frames where `ΔE > ε` (energy increased beyond numerical tolerance).
- Suggested tolerance: `ε = 1e-6 * E_total` (relative) or `ε = 1e-9` (absolute)

#### R3.3: Collision Frame Correlation
Mark whether each frame had active collisions, to correlate energy changes with collision events.

### R4: Output Format

#### R4.1: CSV Export
Primary output should be CSV for easy analysis in spreadsheets or Python/MATLAB.

Example structure:
```csv
frame,time,body_id,KE_linear,KE_rot,PE,E_total,delta_E,collision_frame
0,0.000,0,0.0,0.0,9.81,9.81,0.0,false
1,0.016,0,0.157,0.0,9.653,9.81,0.0,false
...
```

#### R4.2: Contact Log (Separate File)
```csv
frame,contact_id,body_a,body_b,nx,ny,nz,penetration,v_rel_pre,v_rel_post,lambda
10,0,0,1,0.0,0.0,1.0,0.001,-2.5,0.5,15.3
```

#### R4.3: Optional JSON Format
For programmatic analysis, provide JSON output option with nested structure.

#### R4.4: Ring Buffer Mode (NEW - from Gemini review)
To avoid I/O bottlenecks that alter timing (Heisenbugs):
- Store last N frames (e.g., 60) in a circular memory buffer
- Only write to disk when energy anomaly (ΔE > ε) is detected
- Captures the moments leading up to instability without generating gigabytes of CSVs
- Optional body ID filter ("watch list") to reduce data volume

---

## Design Considerations

### Integration Points

The energy tracker should integrate with `WorldModel::update()` or `CollisionPipeline::execute()` to capture state at the appropriate points:

1. **Pre-collision**: State after external force integration, before collision detection
2. **Post-collision**: State after constraint forces applied

### Performance

Diagnostic logging should be:
- **Opt-in**: Disabled by default, enabled via flag or compile-time option
- **Low overhead when disabled**: No virtual calls or branching in hot path
- **Acceptable overhead when enabled**: Logging during debugging is acceptable

### Thread Safety

If simulation is multi-threaded, ensure logging is thread-safe or document single-threaded requirement.

---

## Implementation Approach

### Option A: Standalone Utility Class (Recommended)

```cpp
class EnergyTracker {
public:
  struct BodyEnergy {
    double linearKE;
    double rotationalKE;
    double potentialE;
    double total() const { return linearKE + rotationalKE + potentialE; }
  };

  // Compute energy for a single body
  static BodyEnergy computeBodyEnergy(
    const InertialState& state,
    double mass,
    const Eigen::Matrix3d& bodyInertia,
    double gravity = 9.81,
    double referenceHeight = 0.0);

  // Compute total system energy
  static double computeSystemEnergy(
    std::span<const AssetInertial> bodies,
    double gravity = 9.81);
};
```

### Option B: Observer Pattern Integration (Recommended by Gemini)

Use an Observer/Listener interface to decouple debug logic from core physics:

```cpp
class IPhysicsDebugListener {
public:
  virtual ~IPhysicsDebugListener() = default;
  virtual void onPreSolve(const SystemState& state) = 0;
  virtual void onPostSolve(const SystemState& state) = 0;
  virtual void onContactProcessed(const ContactInfo& contact) = 0;
};

class CollisionPipeline {
public:
  void setDebugListener(std::shared_ptr<IPhysicsDebugListener> listener);
  // ...
};

// EnergyTracker implements IPhysicsDebugListener
class EnergyTracker : public IPhysicsDebugListener {
  // ...
};
```

**Advantages**:
- Keeps debug logic decoupled from core physics math
- Avoids modifying solver loop to extract λ values (which can introduce bugs)
- Easy to swap different diagnostic implementations

### Recommendation

Start with Option A (standalone utility) for flexibility in tests. Use Option B (Observer Pattern) if inline pre/post-solve logging proves necessary for root cause identification.

---

## Test Plan

### Unit Tests

```cpp
// Verify linear KE computation
TEST(EnergyTracker, LinearKE_MovingSphere)
TEST(EnergyTracker, LinearKE_StationaryBody_ReturnsZero)

// Verify rotational KE with world-frame tensor
TEST(EnergyTracker, RotationalKE_SpinningCube_AxisAligned)
TEST(EnergyTracker, RotationalKE_SpinningCube_Tilted)
TEST(EnergyTracker, RotationalKE_ConsistentAcrossOrientations)

// Verify PE computation
TEST(EnergyTracker, PotentialEnergy_HeightProportional)

// Verify total energy conservation (no collision)
TEST(EnergyTracker, FreeFall_TotalEnergyConstant)
```

### Integration Tests

```cpp
// Verify logging output format
TEST(DiagnosticLogger, CSV_OutputFormat_Valid)
TEST(DiagnosticLogger, ContactLog_CapturesAllContacts)

// Verify energy change detection
TEST(EnergyTracker, DetectsEnergyIncrease_WhenPresent)
TEST(EnergyTracker, NoFalsePositives_NumericalNoise)
```

---

## Acceptance Criteria

1. [ ] **AC1**: `EnergyTracker` class implemented with correct world-frame rotational KE
2. [ ] **AC2**: Per-frame CSV logging of body energies implemented
3. [ ] **AC3**: Per-contact CSV logging implemented
4. [ ] **AC4**: Energy injection detection (ΔE > ε flagging) implemented
5. [ ] **AC5**: All unit tests pass
6. [ ] **AC6**: Free-fall test shows constant total energy (validates baseline)
7. [ ] **AC7**: Documentation of output format and usage

---

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `msd-sim/src/Diagnostics/EnergyTracker.hpp` | Energy computation utility |
| `msd-sim/src/Diagnostics/EnergyTracker.cpp` | Implementation |
| `msd-sim/src/Diagnostics/DiagnosticLogger.hpp` | CSV/JSON logging |
| `msd-sim/src/Diagnostics/DiagnosticLogger.cpp` | Implementation |
| `msd-sim/test/Diagnostics/EnergyTrackerTest.cpp` | Unit tests |

### Modified Files
| File | Change |
|------|--------|
| `msd-sim/CMakeLists.txt` | Add Diagnostics directory |

---

## Estimated Effort

- Energy computation: ~2 hours
- Logging infrastructure: ~3 hours
- Testing: ~2 hours
- Documentation: ~1 hour

**Total**: ~8 hours

---

## Workflow Log

### Draft Phase
- **Created**: 2026-02-05
- **Notes**: Subticket split from parent 0038. Focuses on diagnostic infrastructure required before investigation can proceed.

### Gemini Review (2026-02-05)
**Status: Approved with Modifications**

Key changes incorporated:
1. **R1.3**: Changed PE formula from `mgh` to dot-product form `-m(g·r)` for arbitrary gravity support
2. **R2.5**: Added solver state logging (iteration count, residual, max penetration, warm-start λ)
3. **R2.6**: Added explicit Pre-Solve vs Post-Solve energy tracking requirement
4. **R4.4**: Added ring buffer mode for triggered logging (reduces I/O, avoids Heisenbugs)
5. **Option B**: Changed from simple hooks to Observer Pattern (`IPhysicsDebugListener`)

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

