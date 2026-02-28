// Ticket: 0084a_tilted_drop_rotation_tests
//
// PURPOSE: Systematic regression suite for rotation axis isolation after a
// tilted cube is dropped and bounces on the floor. Tests assert that
// post-bounce angular velocity stays predominantly in the tilt axis and
// does not develop significant spurious rotation about the orthogonal axes.
//
// This covers the rotational coupling path exercised by the asymmetric
// decoupling fix in ticket 0084. Any regression in BlockPGS K_nt handling
// should cause cross-axis angular velocity to exceed the threshold below.
//
// TEST MATRIX:
//   Single-axis X:  5°, 15°, 30° tilts → rotation stays about X
//   Single-axis Y:  5°, 15°, 30° tilts → rotation stays about Y
//   Combined X+Y:   5°+5°, 15°+15°, 30°+30° → both X and Y active, Z suppressed
//
// PHYSICS NOTES:
//   - Cube: 1x1x1 m, mass = 1 kg
//   - Floor: static slab at z = -50 (surface at z = 0)
//   - Gravity: g = 9.81 m/s^2 downward
//   - dt = 0.016 s per frame (60 FPS)
//   - restitution = 0.5, friction = 0.5
//   - Drop height: ~4m above floor to build sufficient angular momentum at bounce
//   - 200 frames (~3.3 s): enough for first bounce to occur and rotation to develop
//
// MEASUREMENT STRATEGY:
//   Rotation is measured at peak angular velocity during the simulation
//   (not at the final frame, where damping may reduce omega to near-zero).
//   Peak omega and its axis composition are tracked across all frames.
//   The peak frame is typically within 1-5 frames of the first floor contact.
//
// THRESHOLD RATIONALE:
//   kCrossAxisRatio = 0.5: if any cross-axis omega exceeds 50% of the dominant
//   axis component, the solver is introducing spurious coupling. With a healthy
//   solver, observed ratios are expected near zero for clean single-axis tilts.

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

// ============================================================================
// Constants and helpers
// ============================================================================

namespace
{

/// Threshold: cross-axis omega must be below this fraction of the dominant axis
/// at the peak rotation frame (first bounce). For a pure single-axis tilt, the
/// cross-axis component should be near zero (numerical noise only). A 5%
/// threshold catches the uneven contact normal bug where asymmetric normal
/// directions at first contact inject spurious cross-axis rotation.
constexpr double kCrossAxisRatio = 0.05;

/// Total number of frames to simulate (200 frames = ~3.3 s)
constexpr int kSimFrames = 200;

/// Drop height offset above lowest-corner clearance.
/// The cube is placed with its lowest corner at z=0 (touching floor), then
/// elevated by this amount so it falls and builds angular momentum at contact.
constexpr double kDropHeight = 4.0;  // meters above floor surface

/// Compute total system energy using EnergyTracker with gravity potential
double computeSystemEnergy(const WorldModel& world)
{
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));

  auto sysEnergy = EnergyTracker::computeSystemEnergy(
    world.getInertialAssets(), potentials);
  return sysEnergy.total();
}

/// Z-center for a unit cube tilted by theta about a single axis (X or Y),
/// positioned so the lowest corner is at z=0, then elevated by dropHeight.
double centerZForSingleAxisTilt(double thetaRad, double dropHeight)
{
  return 0.5 * std::cos(thetaRad) + 0.5 * std::sin(thetaRad) + dropHeight;
}

/// Z-center for a unit cube tilted about combined X+Y axes, using the
/// conservative half-diagonal to ensure clearance for any equal tilt <= 45°.
double centerZForCombinedTilt(double dropHeight)
{
  // half-diagonal of unit cube = sqrt(3)/2 ≈ 0.866
  constexpr double kHalfDiag = 0.8660;
  return kHalfDiag + dropHeight;
}

struct PeakOmega
{
  double x{};
  double y{};
  double z{};
  double norm{};
  int frame{};
};

}  // anonymous namespace

// ============================================================================
// Test fixture
// ============================================================================

class TiltedDropTest : public ReplayEnabledTest
{
};

// ============================================================================
// Single-axis X tilts
// Assertion: peak post-bounce rotation predominantly about X; Y and Z subdominant
// ============================================================================

// ---------------------------------------------------------------------------
// X 5 degrees (small tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_X_Small_DominantAxisX)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 5.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;
  const double centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);

  Eigen::Quaterniond q{Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()}};

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,   // mass (kg)
                                   0.5,   // restitution
                                   0.5);  // friction
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop X " << kTiltDeg << " deg (peak frame "
            << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  ASSERT_GT(peak.norm, 1e-6) << "Peak angular velocity too small to analyze";

  EXPECT_GE(peak.x, peak.norm * kCrossAxisRatio)
    << "X-tilt should produce dominant rotation about X axis at peak. "
    << "omegaX=" << peak.x << " norm=" << peak.norm;

  EXPECT_LE(peak.y, peak.norm * kCrossAxisRatio)
    << "Spurious Y rotation should be subdominant for X-tilt. "
    << "omegaY=" << peak.y << " norm=" << peak.norm;

  EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
    << "Spurious Z rotation should be subdominant for X-tilt. "
    << "omegaZ=" << peak.z << " norm=" << peak.norm;
}

// ---------------------------------------------------------------------------
// X 15 degrees (medium tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_X_Medium_DominantAxisX)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 15.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;
  const double centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);

  Eigen::Quaterniond q{Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()}};

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop X " << kTiltDeg << " deg (peak frame "
            << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  ASSERT_GT(peak.norm, 1e-6) << "Peak angular velocity too small to analyze";

  EXPECT_GE(peak.x, peak.norm * kCrossAxisRatio)
    << "X-tilt should produce dominant rotation about X axis at peak. "
    << "omegaX=" << peak.x << " norm=" << peak.norm;

  EXPECT_LE(peak.y, peak.norm * kCrossAxisRatio)
    << "Spurious Y rotation should be subdominant for X-tilt. "
    << "omegaY=" << peak.y << " norm=" << peak.norm;

  EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
    << "Spurious Z rotation should be subdominant for X-tilt. "
    << "omegaZ=" << peak.z << " norm=" << peak.norm;
}

// ---------------------------------------------------------------------------
// X 30 degrees (large tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_X_Large_DominantAxisX)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 30.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;
  const double centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);

  Eigen::Quaterniond q{Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()}};

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop X " << kTiltDeg << " deg (peak frame "
            << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  ASSERT_GT(peak.norm, 1e-6) << "Peak angular velocity too small to analyze";

  EXPECT_GE(peak.x, peak.norm * kCrossAxisRatio)
    << "X-tilt should produce dominant rotation about X axis at peak. "
    << "omegaX=" << peak.x << " norm=" << peak.norm;

  EXPECT_LE(peak.y, peak.norm * kCrossAxisRatio)
    << "Spurious Y rotation should be subdominant for X-tilt. "
    << "omegaY=" << peak.y << " norm=" << peak.norm;

  EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
    << "Spurious Z rotation should be subdominant for X-tilt. "
    << "omegaZ=" << peak.z << " norm=" << peak.norm;
}

// ============================================================================
// Single-axis Y tilts
// Assertion: peak post-bounce rotation predominantly about Y; X and Z subdominant
// ============================================================================

// ---------------------------------------------------------------------------
// Y 5 degrees (small tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_Y_Small_DominantAxisY)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 5.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;
  const double centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);

  Eigen::Quaterniond q{Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()}};

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop Y " << kTiltDeg << " deg (peak frame "
            << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  ASSERT_GT(peak.norm, 1e-6) << "Peak angular velocity too small to analyze";

  EXPECT_GE(peak.y, peak.norm * kCrossAxisRatio)
    << "Y-tilt should produce dominant rotation about Y axis at peak. "
    << "omegaY=" << peak.y << " norm=" << peak.norm;

  EXPECT_LE(peak.x, peak.norm * kCrossAxisRatio)
    << "Spurious X rotation should be subdominant for Y-tilt. "
    << "omegaX=" << peak.x << " norm=" << peak.norm;

  EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
    << "Spurious Z rotation should be subdominant for Y-tilt. "
    << "omegaZ=" << peak.z << " norm=" << peak.norm;
}

// ---------------------------------------------------------------------------
// Y 15 degrees (medium tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_Y_Medium_DominantAxisY)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 15.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;
  const double centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);

  Eigen::Quaterniond q{Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()}};

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop Y " << kTiltDeg << " deg (peak frame "
            << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  ASSERT_GT(peak.norm, 1e-6) << "Peak angular velocity too small to analyze";

  EXPECT_GE(peak.y, peak.norm * kCrossAxisRatio)
    << "Y-tilt should produce dominant rotation about Y axis at peak. "
    << "omegaY=" << peak.y << " norm=" << peak.norm;

  EXPECT_LE(peak.x, peak.norm * kCrossAxisRatio)
    << "Spurious X rotation should be subdominant for Y-tilt. "
    << "omegaX=" << peak.x << " norm=" << peak.norm;

  EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
    << "Spurious Z rotation should be subdominant for Y-tilt. "
    << "omegaZ=" << peak.z << " norm=" << peak.norm;
}

// ---------------------------------------------------------------------------
// Y 30 degrees (large tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_Y_Large_DominantAxisY)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 30.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;
  const double centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);

  Eigen::Quaterniond q{Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()}};

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop Y " << kTiltDeg << " deg (peak frame "
            << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  ASSERT_GT(peak.norm, 1e-6) << "Peak angular velocity too small to analyze";

  EXPECT_GE(peak.y, peak.norm * kCrossAxisRatio)
    << "Y-tilt should produce dominant rotation about Y axis at peak. "
    << "omegaY=" << peak.y << " norm=" << peak.norm;

  EXPECT_LE(peak.x, peak.norm * kCrossAxisRatio)
    << "Spurious X rotation should be subdominant for Y-tilt. "
    << "omegaX=" << peak.x << " norm=" << peak.norm;

  EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
    << "Spurious Z rotation should be subdominant for Y-tilt. "
    << "omegaZ=" << peak.z << " norm=" << peak.norm;
}

// ============================================================================
// Combined X+Y tilts
// Assertion: both X and Y rotation active at peak; Z subdominant
// ============================================================================

// ---------------------------------------------------------------------------
// X+Y 5 degrees each (small combined tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_XY_Small_BothAxesActive)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 5.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;

  // Combined X+Y tilt: compose rotations
  Eigen::Quaterniond q =
    Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()};

  const double centerZ = centerZForCombinedTilt(kDropHeight);

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop XY " << kTiltDeg << "+" << kTiltDeg
            << " deg (peak frame " << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  ASSERT_GT(peak.norm, 1e-6) << "Peak angular velocity too small to analyze";

  // Both X and Y should be active (each >= 10% of total norm)
  constexpr double kBothActiveRatio = 0.10;
  EXPECT_GE(peak.x, peak.norm * kBothActiveRatio)
    << "Combined X+Y tilt should produce X-axis rotation at peak. "
    << "omegaX=" << peak.x << " norm=" << peak.norm;

  EXPECT_GE(peak.y, peak.norm * kBothActiveRatio)
    << "Combined X+Y tilt should produce Y-axis rotation at peak. "
    << "omegaY=" << peak.y << " norm=" << peak.norm;

  // Spurious Z rotation should be subdominant
  EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
    << "Spurious Z rotation should be subdominant for X+Y tilt. "
    << "omegaZ=" << peak.z << " norm=" << peak.norm;
}

// ---------------------------------------------------------------------------
// X+Y 15 degrees each (medium combined tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_XY_Medium_BothAxesActive)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 15.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;

  Eigen::Quaterniond q =
    Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()};

  const double centerZ = centerZForCombinedTilt(kDropHeight);

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop XY " << kTiltDeg << "+" << kTiltDeg
            << " deg (peak frame " << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  if (!nanDetected && peak.norm > 1e-6)
  {
    constexpr double kBothActiveRatio = 0.10;
    EXPECT_GE(peak.x, peak.norm * kBothActiveRatio)
      << "Combined X+Y tilt should produce X-axis rotation at peak. "
      << "omegaX=" << peak.x << " norm=" << peak.norm;

    EXPECT_GE(peak.y, peak.norm * kBothActiveRatio)
      << "Combined X+Y tilt should produce Y-axis rotation at peak. "
      << "omegaY=" << peak.y << " norm=" << peak.norm;

    EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
      << "Spurious Z rotation should be subdominant for X+Y tilt. "
      << "omegaZ=" << peak.z << " norm=" << peak.norm;
  }
}

// ---------------------------------------------------------------------------
// X+Y 30 degrees each (large combined tilt)
// ---------------------------------------------------------------------------
TEST_F(TiltedDropTest, TiltedDrop_XY_Large_BothAxesActive)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  constexpr double kTiltDeg = 30.0;
  const double tiltRad = kTiltDeg * M_PI / 180.0;

  Eigen::Quaterniond q =
    Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()};

  const double centerZ = centerZForCombinedTilt(kDropHeight);

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, centerZ},
                                   1.0,
                                   0.5,
                                   0.5);
  const uint32_t cubeId = cube.getInstanceId();
  world().getObject(cubeId).getInertialState().orientation = q;

  const double initialEnergy = computeSystemEnergy(world());
  double maxEnergy = initialEnergy;
  PeakOmega peak{};
  bool nanDetected = false;

  for (int frame = 1; frame <= kSimFrames; ++frame)
  {
    step(1);

    const auto& state = world().getObject(cubeId).getInertialState();
    if (std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxEnergy = std::max(maxEnergy, computeSystemEnergy(world()));

    const double ox = std::abs(state.getAngularVelocity().x());
    const double oy = std::abs(state.getAngularVelocity().y());
    const double oz = std::abs(state.getAngularVelocity().z());
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (norm > peak.norm)
    {
      peak = {ox, oy, oz, norm, frame};
    }
  }

  ASSERT_FALSE(nanDetected) << "NaN detected in position";

  ASSERT_LE(maxEnergy, initialEnergy * 1.10)
    << "Energy must not grow beyond 10% tolerance. initial=" << initialEnergy
    << " max=" << maxEnergy;

  std::cout << "\n=== TiltedDrop XY " << kTiltDeg << "+" << kTiltDeg
            << " deg (peak frame " << peak.frame << ") ===\n";
  std::cout << "omega: X=" << peak.x << " Y=" << peak.y << " Z=" << peak.z
            << " (norm=" << peak.norm << ")\n";

  if (!nanDetected && peak.norm > 1e-6)
  {
    constexpr double kBothActiveRatio = 0.10;
    EXPECT_GE(peak.x, peak.norm * kBothActiveRatio)
      << "Combined X+Y tilt should produce X-axis rotation at peak. "
      << "omegaX=" << peak.x << " norm=" << peak.norm;

    EXPECT_GE(peak.y, peak.norm * kBothActiveRatio)
      << "Combined X+Y tilt should produce Y-axis rotation at peak. "
      << "omegaY=" << peak.y << " norm=" << peak.norm;

    EXPECT_LE(peak.z, peak.norm * kCrossAxisRatio)
      << "Spurious Z rotation should be subdominant for X+Y tilt. "
      << "omegaZ=" << peak.z << " norm=" << peak.norm;
  }
}
