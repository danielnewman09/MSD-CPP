# Design: Tilted Drop Rotation Test Coverage

## Summary

Add a systematic test suite (`TiltedDropTest.cpp`) covering cubes dropped with initial rotation about X, Y, and combined X+Y axes at three tilt magnitudes each (5°, 15°, 30°). Each test asserts that post-bounce rotation stays predominantly in the expected axis, with cross-axis angular velocity bounded relative to the dominant axis. This provides regression coverage for the asymmetric decoupling fix in ticket 0084 and establishes a baseline for rotation axis fidelity under the Block PGS solver.

## Architecture Changes

### PlantUML Diagram

See: `./0084a-tilted-drop-rotation-tests.puml`

### New Components

No new production code components are introduced. This ticket adds only a new test file.

#### TiltedDropTest.cpp

- **Purpose**: Systematic regression suite for rotation axis isolation after tilted cube drop and bounce
- **Location**: `msd/msd-sim/test/Physics/Collision/TiltedDropTest.cpp`
- **Registered in**: `msd/msd-sim/test/Physics/Collision/CMakeLists.txt`
- **Fixture**: Inherits from `ReplayEnabledTest` via a local typedef class `TiltedDropTest`
- **Helper functions** (anonymous namespace):
  - `centerZForTilt(double tiltRad)` — computes cube center Z so lowest corner is at floor level
  - `totalEnergy(const AssetInertial&)` — inline KE (linear + rotational) + PE energy computation (same pattern as FrictionSlidingTest)
  - `dominantAxis(omegaX, omegaY, omegaZ)` — returns the index (0/1/2) of the largest absolute angular velocity component

### Modified Components

#### CMakeLists.txt (test/Physics/Collision/)

- **Current location**: `msd/msd-sim/test/Physics/Collision/CMakeLists.txt`
- **Change**: Add `${CMAKE_CURRENT_SOURCE_DIR}/TiltedDropTest.cpp` to `target_sources`
- **Backward compatibility**: Additive only — no existing tests modified

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---|---|---|---|
| TiltedDropTest.cpp | ReplayEnabledTest | Inheritance via test fixture class | Same fixture used by all collision tests |
| TiltedDropTest.cpp | spawnInertial | Direct call | Spawn cube with mass=1.0, restitution=0.5, friction=0.5 |
| TiltedDropTest.cpp | spawnEnvironment | Direct call | Floor slab at z=-50 |
| CMakeLists.txt | msd_sim_test target | target_sources addition | New .cpp registered with test binary |

## Test Structure

### Setup Pattern

Each test follows this pattern (matching RotationDampingTest.cpp):

```cpp
// 1. Spawn floor
spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

// 2. Compute tilt quaternion
double const tiltRad = kTiltDeg * M_PI / 180.0;
Eigen::Quaterniond q{Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()}};

// 3. Position cube: lowest corner at z=0 (single-axis formula)
// For X-tilt: center_z = 0.5*cos(theta) + 0.5*sin(theta)
double const centerZ = 0.5 * std::cos(tiltRad) + 0.5 * std::sin(tiltRad);

// 4. Spawn cube with target physics
const auto& cube = spawnInertial("unit_cube",
                                  Coordinate{0.0, 0.0, centerZ},
                                  1.0,   // mass (kg)
                                  0.5,   // restitution
                                  0.5);  // friction
uint32_t cubeId = cube.getInstanceId();

// 5. Manually set orientation (same as RockingCube test)
world().getObject(cubeId).getInertialState().orientation = q;

// 6. Record initial energy
double const initialEnergy = computeTotalEnergy(cube);
```

### Assertion Pattern — Single-Axis Tests

After 200 frames:

1. **No NaN** — position.z() must be finite
2. **No energy growth** — total energy <= initial * 1.1 (10% tolerance for bounce transients)
3. **Dominant axis** — the tilt axis angular velocity must exceed both other axes
4. **Cross-axis bound** — spurious axes must be < `kCrossAxisRatio` (0.5) times the dominant axis

```cpp
const double omegaX = ..., omegaY = ..., omegaZ = ...;
const double dominant = std::max({std::abs(omegaX), std::abs(omegaY), std::abs(omegaZ)});

// For an X-tilt test:
EXPECT_GE(std::abs(omegaX), dominant * 0.5)
  << "X-tilt should produce dominant rotation about X";
EXPECT_LE(std::abs(omegaY), dominant * 0.5)
  << "Y rotation should be subdominant for X-tilt";
EXPECT_LE(std::abs(omegaZ), dominant * 0.5)
  << "Z rotation should be subdominant for X-tilt";
```

Note: The cross-axis threshold (0.5) is intentionally loose for the first pass. If the solver is clean, observed ratios will be near 0. If the asymmetric decoupling bug reappears, large-tilt tests will show ratios near or above 1.0.

### Assertion Pattern — Combined Tests

For X+Y combined tilts:

1. **No NaN** — same as above
2. **No energy growth** — same as above
3. **Both axes active** — both |omegaX| and |omegaY| exceed a minimum threshold (dominant * 0.1)
4. **Z suppressed** — |omegaZ| < dominant * 0.5

### Test Matrix

| Test Name | Tilt Axis | Angle | Key Assertion |
|---|---|---|---|
| TiltedDrop_X_Small_DominantAxisX | X | 5° | omegaX dominant, omegaY/Z subdominant |
| TiltedDrop_X_Medium_DominantAxisX | X | 15° | omegaX dominant, omegaY/Z subdominant |
| TiltedDrop_X_Large_DominantAxisX | X | 30° | omegaX dominant, omegaY/Z subdominant |
| TiltedDrop_Y_Small_DominantAxisY | Y | 5° | omegaY dominant, omegaX/Z subdominant |
| TiltedDrop_Y_Medium_DominantAxisY | Y | 15° | omegaY dominant, omegaX/Z subdominant |
| TiltedDrop_Y_Large_DominantAxisY | Y | 30° | omegaY dominant, omegaX/Z subdominant |
| TiltedDrop_XY_Small_BothAxesActive | X+Y | 5°+5° | both omegaX and omegaY active, omegaZ subdominant |
| TiltedDrop_XY_Medium_BothAxesActive | X+Y | 15°+15° | both omegaX and omegaY active, omegaZ subdominant |
| TiltedDrop_XY_Large_BothAxesActive | X+Y | 30°+30° | both omegaX and omegaY active, omegaZ subdominant |

## Test Impact

### Existing Tests Affected

None — no existing test files are modified.

### New Tests Required

#### 9 New Tests in TiltedDropTest.cpp

| Test Case | What It Validates |
|---|---|
| TiltedDrop_X_Small_DominantAxisX | 5° X-tilt → rotation stays about X after bounce |
| TiltedDrop_X_Medium_DominantAxisX | 15° X-tilt → rotation stays about X after bounce |
| TiltedDrop_X_Large_DominantAxisX | 30° X-tilt → rotation stays about X after bounce |
| TiltedDrop_Y_Small_DominantAxisY | 5° Y-tilt → rotation stays about Y after bounce |
| TiltedDrop_Y_Medium_DominantAxisY | 15° Y-tilt → rotation stays about Y after bounce |
| TiltedDrop_Y_Large_DominantAxisY | 30° Y-tilt → rotation stays about Y after bounce |
| TiltedDrop_XY_Small_BothAxesActive | 5°+5° → both X and Y active, Z suppressed |
| TiltedDrop_XY_Medium_BothAxesActive | 15°+15° → both X and Y active, Z suppressed |
| TiltedDrop_XY_Large_BothAxesActive | 30°+30° → both X and Y active, Z suppressed |

## Open Questions

None. All design guidance is provided in the ticket. The threshold values (0.5 cross-axis ratio, 10% energy tolerance) are calibration values that can be tightened once baseline behavior is observed.

## Physics Notes

### Center-Z Calculation for Single-Axis Tilt

For a unit cube (half-side = 0.5) tilted by angle θ about the X-axis, the lowest point
in world space is at z_center − (0.5·cos θ + 0.5·sin θ). Setting z_lowest = 0:

    z_center = 0.5·cos θ + 0.5·sin θ

This formula appears in both RockingCube and TiltedCubeSettles tests and is correct
for single-axis tilts up to 45°.

### Center-Z Calculation for Combined X+Y Tilt

For equal X+Y tilts, the lowest corner is further down. The safe approach is to use
the half-diagonal: z_center = 0.5·√3 ≈ 0.866. This ensures the cube starts just
above the floor for any combined tilt ≤ 45° per axis.

### Energy Measurement

Uses the inline helper from FrictionSlidingTest.cpp:

    E = ½mv² + ½ωᵀIω + mgh

Applied per-asset (not via EnergyTracker) for simplicity.

### Simulation Length

200 frames (~3.3 s at 60 FPS). Sufficient for:
- Cube to fall, bounce, and settle into primary rotation
- Post-bounce rotation to stabilize (3–5 frames after contact)
- Not so long that rotation fully damps to zero (which would make axis assertions vacuous)
