// Ticket: 0084a_tilted_drop_rotation_tests
//
// PURPOSE: Per-bounce regression suite for rotation axis isolation after a
// tilted cube is dropped and bounces on the floor. Tests detect individual
// bounce events and assert full state snapshots at each bounce, catching:
//   - Cross-axis angular velocity injection from uneven contact normals
//   - Lateral displacement drift off the expected tilt plane
//   - Energy growth beyond physical tolerance
//   - Cross-axis accumulation worsening across successive bounces
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
//   - Drop height: ~4m above floor surface
//   - 200 frames (~3.3 s): enough for multiple bounces
//
// MEASUREMENT STRATEGY:
//   Bounce events are detected by vertical velocity sign reversal (negative
//   to non-negative) when the downward speed exceeds kBounceVzThreshold.
//   At each bounce, a full state snapshot is captured: angular velocity,
//   position, lateral displacement from spawn, and system energy.
//   Per-bounce assertions replace the old aggregate peak-omega metric.

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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
// Constants and types
// ============================================================================

namespace
{

// --- Thresholds ---

/// Cross-axis omega must be below this fraction of the dominant axis
constexpr double kCrossAxisRatio = 0.05;

/// Angular velocity change threshold (rad/s per frame) for contact detection.
/// During free flight there are no external torques on the CoM, so omega is
/// constant. Any delta above this threshold indicates a contact impulse.
constexpr double kOmegaChangeThreshold = 0.5;

/// Cross-axis displacement must be below this fraction of lateral norm
constexpr double kDisplacementCrossRatio = 0.20;

/// Energy must not grow beyond this multiple of initial energy
constexpr double kEnergyGrowthTolerance = 1.10;

/// For XY: each axis must contribute at least this fraction of omega norm
constexpr double kBothActiveRatio = 0.10;

/// Cross-axis ratio shouldn't grow between first and last bounce by more than this
constexpr double kCrossAxisAccumulationMargin = 0.05;

/// Tighter Z-axis threshold for XY tests: wrong-corner contacts from excessive
/// penetration inject Z-axis rotation that the general kCrossAxisRatio misses.
constexpr double kXYZAxisRatio = 0.02;

/// For equal-angle XY tilts, omega_x and omega_y should be nearly equal.
/// Asymmetry beyond this fraction of omega_norm indicates solver bias.
constexpr double kXYSymmetryTolerance = 0.15;

/// Minimum number of contact events expected in the simulation
constexpr int kMinBounces = 2;

// --- Regime-specific constants ---

/// Frictionless: no friction torques → tighter cross-axis threshold
constexpr double kFrictionlessCrossAxisRatio = 0.02;

/// Elastic: energy must be conserved within tight tolerance
constexpr double kElasticEnergyTolerance = 1.02;

/// Inelastic: only 1 contact event required (no rebound)
constexpr int kInelasticMinBounces = 1;

/// Maximum number of contact events to capture before stopping.
/// Increased to capture sub-bounce pairs (initial corner + follow-on).
constexpr int kMaxBounces = 6;

/// Total number of frames to simulate
constexpr int kSimFrames = 200;

/// Drop height offset above lowest-corner clearance (meters)
constexpr double kDropHeight = 4.0;

// --- Types ---

enum class TiltAxis
{
  X,
  Y,
  XY
};

struct TiltConfig
{
  TiltAxis axis;
  double tiltDeg;
  std::string label;
};

struct SceneSetup
{
  uint32_t cubeId;
  Coordinate initialPosition;
  double initialEnergy;
};

struct BounceSnapshot
{
  int frame;
  double omegaX;       // |omega_x|
  double omegaY;       // |omega_y|
  double omegaZ;       // |omega_z|
  double omegaNorm;
  double signedOmegaX;  // signed omega_x (for sign consistency checks)
  double signedOmegaY;  // signed omega_y
  double signedOmegaZ;  // signed omega_z
  double deltaOmegaNorm;  // |delta_omega| that triggered detection
  double posX;
  double posY;
  double posZ;
  double displacementX;
  double displacementY;
  double energy;
};

struct BounceResult
{
  std::vector<BounceSnapshot> bounces;
  bool nanDetected{false};
};

// --- Free helper functions ---

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

/// Z-center for a unit cube tilted by theta about a single axis
double centerZForSingleAxisTilt(double thetaRad, double dropHeight)
{
  return 0.5 * std::cos(thetaRad) + 0.5 * std::sin(thetaRad) + dropHeight;
}

/// Z-center for a unit cube tilted about combined X+Y axes
double centerZForCombinedTilt(double dropHeight)
{
  constexpr double kHalfDiag = 0.8660;  // sqrt(3)/2
  return kHalfDiag + dropHeight;
}

/// Compute cross-axis ratio for a single-axis bounce snapshot
double crossAxisRatio(const BounceSnapshot& snap, TiltAxis axis)
{
  if (snap.omegaNorm < 1e-6) return 0.0;

  if (axis == TiltAxis::X)
  {
    return std::max(snap.omegaY, snap.omegaZ) / snap.omegaNorm;
  }
  else
  {
    return std::max(snap.omegaX, snap.omegaZ) / snap.omegaNorm;
  }
}

}  // anonymous namespace

// ============================================================================
// Test fixture with helper methods
// ============================================================================

class TiltedDropTest : public ReplayEnabledTest
{
protected:
  /// Set up a tilted drop scene: floor + cube with given tilt config
  SceneSetup setupScene(const TiltConfig& config,
                        double restitution = 0.5, double friction = 0.5)
  {
    spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

    const double tiltRad = config.tiltDeg * M_PI / 180.0;

    Eigen::Quaterniond q;
    double centerZ{};

    switch (config.axis)
    {
      case TiltAxis::X:
        q = Eigen::Quaterniond{
          Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()}};
        centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);
        break;
      case TiltAxis::Y:
        q = Eigen::Quaterniond{
          Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()}};
        centerZ = centerZForSingleAxisTilt(tiltRad, kDropHeight);
        break;
      case TiltAxis::XY:
        q = Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitX()} *
            Eigen::AngleAxisd{tiltRad, Eigen::Vector3d::UnitY()};
        centerZ = centerZForCombinedTilt(kDropHeight);
        break;
    }

    const Coordinate spawnPos{0.0, 0.0, centerZ};
    const auto& cube = spawnInertial("unit_cube", spawnPos, 1.0, restitution, friction);
    const uint32_t cubeId = cube.getInstanceId();
    world().getObject(cubeId).getInertialState().orientation = q;

    const double initialEnergy = computeSystemEnergy(world());

    return SceneSetup{cubeId, spawnPos, initialEnergy};
  }

  /// Capture a BounceSnapshot from the current simulation state
  BounceSnapshot captureSnapshot(int frame, uint32_t cubeId,
                                 const Coordinate& spawnPos,
                                 double deltaOmegaNorm)
  {
    const auto& state = world().getObject(cubeId).getInertialState();

    const double sox = state.getAngularVelocity().x();
    const double soy = state.getAngularVelocity().y();
    const double soz = state.getAngularVelocity().z();
    const double ox = std::abs(sox);
    const double oy = std::abs(soy);
    const double oz = std::abs(soz);
    const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);

    BounceSnapshot snap{};
    snap.frame = frame;
    snap.omegaX = ox;
    snap.omegaY = oy;
    snap.omegaZ = oz;
    snap.omegaNorm = norm;
    snap.signedOmegaX = sox;
    snap.signedOmegaY = soy;
    snap.signedOmegaZ = soz;
    snap.deltaOmegaNorm = deltaOmegaNorm;
    snap.posX = state.position.x();
    snap.posY = state.position.y();
    snap.posZ = state.position.z();
    snap.displacementX = state.position.x() - spawnPos.x();
    snap.displacementY = state.position.y() - spawnPos.y();
    snap.energy = computeSystemEnergy(world());
    return snap;
  }

  /// Run simulation and detect contact events via angular velocity discontinuity.
  ///
  /// During free flight there are no external torques on the CoM, so angular
  /// velocity is constant. Any frame-to-frame change in omega exceeding
  /// kOmegaChangeThreshold indicates a contact impulse was applied. This
  /// naturally captures both the initial corner impact and rapid follow-on
  /// wrong-corner contacts as separate events.
  BounceResult detectBounces(uint32_t cubeId, const Coordinate& spawnPos)
  {
    BounceResult result{};
    Eigen::Vector3d prevOmega = Eigen::Vector3d::Zero();

    for (int frame = 1; frame <= kSimFrames; ++frame)
    {
      step(1);

      const auto& state = world().getObject(cubeId).getInertialState();

      if (std::isnan(state.position.z()))
      {
        result.nanDetected = true;
        return result;
      }

      const Eigen::Vector3d currOmega{
        state.getAngularVelocity().x(),
        state.getAngularVelocity().y(),
        state.getAngularVelocity().z()};

      const double deltaOmegaNorm = (currOmega - prevOmega).norm();

      if (deltaOmegaNorm > kOmegaChangeThreshold)
      {
        result.bounces.push_back(
          captureSnapshot(frame, cubeId, spawnPos, deltaOmegaNorm));

        if (static_cast<int>(result.bounces.size()) >= kMaxBounces)
        {
          return result;
        }
      }

      prevOmega = currOmega;
    }

    return result;
  }

  /// Print diagnostic output for all bounces
  static void printBounces(const std::vector<BounceSnapshot>& bounces,
                           const std::string& label)
  {
    std::cout << "\n=== " << label << " (" << bounces.size()
              << " bounces) ===\n";
    for (size_t i = 0; i < bounces.size(); ++i)
    {
      const auto& s = bounces[i];
      std::cout << "  Contact " << i << " (frame " << s.frame
                << ", dOmega=" << s.deltaOmegaNorm << "): "
                << "omega=[" << s.omegaX << ", " << s.omegaY << ", "
                << s.omegaZ << "] norm=" << s.omegaNorm
                << "  signed=[" << s.signedOmegaX << ", " << s.signedOmegaY
                << ", " << s.signedOmegaZ << "]"
                << "  pos=[" << s.posX << ", " << s.posY << ", " << s.posZ
                << "]  dx=" << s.displacementX << " dy=" << s.displacementY
                << "  E=" << s.energy << "\n";
    }
  }

  /// Assert single-axis bounce: dominant omega on expected axis, cross-axis suppressed
  static void assertSingleAxisBounce(const BounceSnapshot& snap, int bounceIndex,
                                     TiltAxis axis, double initialEnergy,
                                     const std::string& label)
  {
    ASSERT_GT(snap.omegaNorm, 1e-6)
      << label << " bounce " << bounceIndex
      << ": omega norm too small to analyze";

    double dominant{};
    double cross1{};
    double cross2{};
    std::string dominantName;
    std::string cross1Name;
    std::string cross2Name;

    if (axis == TiltAxis::X)
    {
      dominant = snap.omegaX;  dominantName = "X";
      cross1 = snap.omegaY;   cross1Name = "Y";
      cross2 = snap.omegaZ;   cross2Name = "Z";
    }
    else
    {
      dominant = snap.omegaY;  dominantName = "Y";
      cross1 = snap.omegaX;   cross1Name = "X";
      cross2 = snap.omegaZ;   cross2Name = "Z";
    }

    EXPECT_GE(dominant, snap.omegaNorm * kCrossAxisRatio)
      << label << " bounce " << bounceIndex
      << ": " << dominantName << "-tilt should produce dominant rotation about "
      << dominantName << ". omega" << dominantName << "=" << dominant
      << " norm=" << snap.omegaNorm;

    EXPECT_LE(cross1, snap.omegaNorm * kCrossAxisRatio)
      << label << " bounce " << bounceIndex
      << ": spurious " << cross1Name << " rotation. omega" << cross1Name << "="
      << cross1 << " norm=" << snap.omegaNorm;

    EXPECT_LE(cross2, snap.omegaNorm * kCrossAxisRatio)
      << label << " bounce " << bounceIndex
      << ": spurious " << cross2Name << " rotation. omega" << cross2Name << "="
      << cross2 << " norm=" << snap.omegaNorm;

    // Lateral displacement check
    const double lateralNorm =
      std::sqrt(snap.displacementX * snap.displacementX +
                snap.displacementY * snap.displacementY);
    if (lateralNorm > 1e-4)
    {
      // For X-tilt, displacement should be in Y (cross-axis displacement is X)
      // For Y-tilt, displacement should be in X (cross-axis displacement is Y)
      double crossDisplacement = (axis == TiltAxis::X)
                                   ? std::abs(snap.displacementX)
                                   : std::abs(snap.displacementY);
      EXPECT_LT(crossDisplacement, lateralNorm * kDisplacementCrossRatio)
        << label << " bounce " << bounceIndex
        << ": cross-axis displacement too large. cross=" << crossDisplacement
        << " lateralNorm=" << lateralNorm;
    }

    // Energy check
    EXPECT_LE(snap.energy, initialEnergy * kEnergyGrowthTolerance)
      << label << " bounce " << bounceIndex
      << ": energy growth beyond tolerance. E=" << snap.energy
      << " initial=" << initialEnergy;
  }

  /// Assert combined XY bounce: both X and Y active, Z suppressed
  static void assertCombinedAxisBounce(const BounceSnapshot& snap,
                                       int bounceIndex, double initialEnergy,
                                       const std::string& label)
  {
    ASSERT_GT(snap.omegaNorm, 1e-6)
      << label << " bounce " << bounceIndex
      << ": omega norm too small to analyze";

    EXPECT_GE(snap.omegaX, snap.omegaNorm * kBothActiveRatio)
      << label << " bounce " << bounceIndex
      << ": X-axis should be active for XY tilt. omegaX=" << snap.omegaX
      << " norm=" << snap.omegaNorm;

    EXPECT_GE(snap.omegaY, snap.omegaNorm * kBothActiveRatio)
      << label << " bounce " << bounceIndex
      << ": Y-axis should be active for XY tilt. omegaY=" << snap.omegaY
      << " norm=" << snap.omegaNorm;

    // Equal-angle XY tilt should produce symmetric omega_x ≈ omega_y
    const double omegaAsymmetry = std::abs(snap.omegaX - snap.omegaY);
    EXPECT_LE(omegaAsymmetry, snap.omegaNorm * kXYSymmetryTolerance)
      << label << " bounce " << bounceIndex
      << ": omega_x and omega_y should be nearly equal for equal-angle XY tilt. "
      << "omegaX=" << snap.omegaX << " omegaY=" << snap.omegaY
      << " asymmetry=" << omegaAsymmetry << " norm=" << snap.omegaNorm;

    // Tighter Z threshold: wrong-corner contacts from excessive penetration
    // inject Z-axis rotation that the general cross-axis threshold misses
    EXPECT_LE(snap.omegaZ, snap.omegaNorm * kXYZAxisRatio)
      << label << " bounce " << bounceIndex
      << ": spurious Z rotation for XY tilt (tight threshold). omegaZ="
      << snap.omegaZ << " norm=" << snap.omegaNorm
      << " ratio=" << (snap.omegaNorm > 1e-6 ? snap.omegaZ / snap.omegaNorm : 0.0);

    EXPECT_LE(snap.energy, initialEnergy * kEnergyGrowthTolerance)
      << label << " bounce " << bounceIndex
      << ": energy growth beyond tolerance. E=" << snap.energy
      << " initial=" << initialEnergy;
  }

  /// Assert frictionless single-axis bounce: tighter cross-axis since no friction torques
  static void assertFrictionlessSingleAxisBounce(const BounceSnapshot& snap,
                                                  int bounceIndex, TiltAxis axis,
                                                  double initialEnergy,
                                                  const std::string& label)
  {
    ASSERT_GT(snap.omegaNorm, 1e-6)
      << label << " bounce " << bounceIndex
      << ": omega norm too small to analyze";

    double dominant{};
    double cross1{};
    double cross2{};
    std::string dominantName;
    std::string cross1Name;
    std::string cross2Name;

    if (axis == TiltAxis::X)
    {
      dominant = snap.omegaX;  dominantName = "X";
      cross1 = snap.omegaY;   cross1Name = "Y";
      cross2 = snap.omegaZ;   cross2Name = "Z";
    }
    else
    {
      dominant = snap.omegaY;  dominantName = "Y";
      cross1 = snap.omegaX;   cross1Name = "X";
      cross2 = snap.omegaZ;   cross2Name = "Z";
    }

    EXPECT_GE(dominant, snap.omegaNorm * kFrictionlessCrossAxisRatio)
      << label << " bounce " << bounceIndex
      << ": " << dominantName << "-tilt should produce dominant rotation about "
      << dominantName;

    EXPECT_LE(cross1, snap.omegaNorm * kFrictionlessCrossAxisRatio)
      << label << " bounce " << bounceIndex
      << ": spurious " << cross1Name << " rotation (frictionless). omega"
      << cross1Name << "=" << cross1 << " norm=" << snap.omegaNorm;

    EXPECT_LE(cross2, snap.omegaNorm * kFrictionlessCrossAxisRatio)
      << label << " bounce " << bounceIndex
      << ": spurious " << cross2Name << " rotation (frictionless). omega"
      << cross2Name << "=" << cross2 << " norm=" << snap.omegaNorm;

    // Near-zero lateral displacement without friction
    EXPECT_LT(std::abs(snap.displacementX), 0.05)
      << label << " bounce " << bounceIndex
      << ": lateral X displacement should be near-zero without friction";
    EXPECT_LT(std::abs(snap.displacementY), 0.05)
      << label << " bounce " << bounceIndex
      << ": lateral Y displacement should be near-zero without friction";

    EXPECT_LE(snap.energy, initialEnergy * kEnergyGrowthTolerance)
      << label << " bounce " << bounceIndex
      << ": energy growth beyond tolerance";
  }

  /// Assert frictionless combined-axis bounce: both axes active, tight Z suppression
  static void assertFrictionlessCombinedAxisBounce(const BounceSnapshot& snap,
                                                    int bounceIndex,
                                                    double initialEnergy,
                                                    const std::string& label)
  {
    ASSERT_GT(snap.omegaNorm, 1e-6)
      << label << " bounce " << bounceIndex
      << ": omega norm too small to analyze";

    EXPECT_GE(snap.omegaX, snap.omegaNorm * kBothActiveRatio)
      << label << " bounce " << bounceIndex
      << ": X-axis should be active for XY tilt (frictionless)";

    EXPECT_GE(snap.omegaY, snap.omegaNorm * kBothActiveRatio)
      << label << " bounce " << bounceIndex
      << ": Y-axis should be active for XY tilt (frictionless)";

    // Equal-angle XY tilt should produce symmetric omega_x ≈ omega_y
    const double omegaAsymmetry = std::abs(snap.omegaX - snap.omegaY);
    EXPECT_LE(omegaAsymmetry, snap.omegaNorm * kXYSymmetryTolerance)
      << label << " bounce " << bounceIndex
      << ": omega_x and omega_y should be nearly equal for equal-angle XY tilt. "
      << "omegaX=" << snap.omegaX << " omegaY=" << snap.omegaY
      << " asymmetry=" << omegaAsymmetry << " norm=" << snap.omegaNorm;

    EXPECT_LE(snap.omegaZ, snap.omegaNorm * kFrictionlessCrossAxisRatio)
      << label << " bounce " << bounceIndex
      << ": spurious Z rotation for XY tilt (frictionless, tight threshold). omegaZ="
      << snap.omegaZ << " norm=" << snap.omegaNorm;

    // Near-zero lateral displacement without friction
    EXPECT_LT(std::abs(snap.displacementX), 0.05)
      << label << " bounce " << bounceIndex
      << ": lateral X displacement should be near-zero without friction";
    EXPECT_LT(std::abs(snap.displacementY), 0.05)
      << label << " bounce " << bounceIndex
      << ": lateral Y displacement should be near-zero without friction";

    EXPECT_LE(snap.energy, initialEnergy * kEnergyGrowthTolerance)
      << label << " bounce " << bounceIndex
      << ": energy growth beyond tolerance (frictionless)";
  }

  /// Assert omega sign consistency across bounces for XY tests.
  /// Wrong-corner contacts from excessive penetration flip the rotation
  /// direction, causing sign reversals in omega_x or omega_y between bounces.
  static void assertOmegaSignConsistency(
    const std::vector<BounceSnapshot>& bounces, const std::string& label)
  {
    ASSERT_GE(bounces.size(), 2u)
      << label << ": need at least 2 contact events for sign consistency check";

    const auto& first = bounces.front();

    // Only check sign if the first bounce has significant omega in that axis
    const bool xSignificant = std::abs(first.signedOmegaX) > 0.1;
    const bool ySignificant = std::abs(first.signedOmegaY) > 0.1;

    for (size_t i = 1; i < bounces.size(); ++i)
    {
      const auto& snap = bounces[i];

      if (xSignificant && std::abs(snap.signedOmegaX) > 0.1)
      {
        const bool sameSign =
          (first.signedOmegaX > 0) == (snap.signedOmegaX > 0);
        EXPECT_TRUE(sameSign)
          << label << " bounce " << i
          << ": omega_x sign flipped from bounce 0. "
          << "first=" << first.signedOmegaX
          << " current=" << snap.signedOmegaX;
      }

      if (ySignificant && std::abs(snap.signedOmegaY) > 0.1)
      {
        const bool sameSign =
          (first.signedOmegaY > 0) == (snap.signedOmegaY > 0);
        EXPECT_TRUE(sameSign)
          << label << " bounce " << i
          << ": omega_y sign flipped from bounce 0. "
          << "first=" << first.signedOmegaY
          << " current=" << snap.signedOmegaY;
      }
    }
  }
};

// ============================================================================
// Parameterized fixtures
// ============================================================================

class TiltedDropSingleAxisTest : public TiltedDropTest,
                                  public ::testing::WithParamInterface<TiltConfig>
{};

class TiltedDropCombinedAxisTest : public TiltedDropTest,
                                    public ::testing::WithParamInterface<TiltConfig>
{};

// ============================================================================
// Single-axis tests (X and Y tilts)
// ============================================================================

TEST_P(TiltedDropSingleAxisTest, BounceIsolation)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  auto config = GetParam();
  auto scene = setupScene(config);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kMinBounces))
    << config.label << ": expected at least " << kMinBounces << " bounces";

  printBounces(result.bounces, config.label);

  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    assertSingleAxisBounce(result.bounces[i], static_cast<int>(i),
                           config.axis, scene.initialEnergy, config.label);
  }

  const double firstRatio = crossAxisRatio(result.bounces.front(), config.axis);
  const double lastRatio = crossAxisRatio(result.bounces.back(), config.axis);
  EXPECT_LE(lastRatio, firstRatio + kCrossAxisAccumulationMargin)
    << config.label << ": cross-axis ratio grew between bounces";
}

TEST_P(TiltedDropSingleAxisTest, Frictionless_BounceIsolation)
{
  auto config = GetParam();
  auto scene = setupScene(config, /*restitution=*/0.5, /*friction=*/0.0);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kMinBounces))
    << config.label << " (frictionless): expected at least " << kMinBounces
    << " bounces";

  printBounces(result.bounces, config.label + " [frictionless]");

  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    assertFrictionlessSingleAxisBounce(
      result.bounces[i], static_cast<int>(i),
      config.axis, scene.initialEnergy, config.label + " [frictionless]");
  }
}

TEST_P(TiltedDropSingleAxisTest, Elastic_BounceIsolation)
{
  auto config = GetParam();
  auto scene = setupScene(config, /*restitution=*/1.0, /*friction=*/0.5);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kMinBounces))
    << config.label << " (elastic): expected at least " << kMinBounces
    << " bounces";

  printBounces(result.bounces, config.label + " [elastic]");

  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    assertSingleAxisBounce(
      result.bounces[i], static_cast<int>(i),
      config.axis, scene.initialEnergy, config.label + " [elastic]");

    // Tight energy conservation for elastic regime
    EXPECT_LE(result.bounces[i].energy,
              scene.initialEnergy * kElasticEnergyTolerance)
      << config.label << " [elastic] bounce " << i
      << ": energy not conserved within elastic tolerance. E="
      << result.bounces[i].energy << " initial=" << scene.initialEnergy;
  }
}

TEST_P(TiltedDropSingleAxisTest, Inelastic_ContactResponse)
{
  auto config = GetParam();
  auto scene = setupScene(config, /*restitution=*/0.0, /*friction=*/0.5);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kInelasticMinBounces))
    << config.label << " (inelastic): expected at least " << kInelasticMinBounces
    << " contact event";

  printBounces(result.bounces, config.label + " [inelastic]");

  // Energy should drop after contact (inelastic)
  const auto& firstBounce = result.bounces.front();
  EXPECT_LT(firstBounce.energy, scene.initialEnergy)
    << config.label << " [inelastic]: energy should decrease after inelastic contact";

  // No energy growth across any contact
  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    EXPECT_LE(result.bounces[i].energy,
              scene.initialEnergy * kEnergyGrowthTolerance)
      << config.label << " [inelastic] bounce " << i
      << ": energy growth beyond tolerance";
  }
}

INSTANTIATE_TEST_SUITE_P(
  TiltedDrop, TiltedDropSingleAxisTest,
  ::testing::Values(
    TiltConfig{TiltAxis::X, 5.0, "X_5deg"},
    TiltConfig{TiltAxis::X, 15.0, "X_15deg"},
    TiltConfig{TiltAxis::X, 30.0, "X_30deg"},
    TiltConfig{TiltAxis::Y, 5.0, "Y_5deg"},
    TiltConfig{TiltAxis::Y, 15.0, "Y_15deg"},
    TiltConfig{TiltAxis::Y, 30.0, "Y_30deg"}
  ),
  [](const ::testing::TestParamInfo<TiltConfig>& info) {
    return info.param.label;
  }
);

// ============================================================================
// Combined X+Y axis tests
// ============================================================================

TEST_P(TiltedDropCombinedAxisTest, BounceIsolation)
{
  // Ticket: 0084a_tilted_drop_rotation_tests
  auto config = GetParam();
  auto scene = setupScene(config);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kMinBounces))
    << config.label << ": expected at least " << kMinBounces << " bounces";

  printBounces(result.bounces, config.label);

  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    assertCombinedAxisBounce(result.bounces[i], static_cast<int>(i),
                             scene.initialEnergy, config.label);
  }

  assertOmegaSignConsistency(result.bounces, config.label);
}

TEST_P(TiltedDropCombinedAxisTest, Frictionless_BounceIsolation)
{
  auto config = GetParam();
  auto scene = setupScene(config, /*restitution=*/0.5, /*friction=*/0.0);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kMinBounces))
    << config.label << " (frictionless): expected at least " << kMinBounces
    << " bounces";

  printBounces(result.bounces, config.label + " [frictionless]");

  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    assertFrictionlessCombinedAxisBounce(
      result.bounces[i], static_cast<int>(i),
      scene.initialEnergy, config.label + " [frictionless]");
  }
}

TEST_P(TiltedDropCombinedAxisTest, Elastic_BounceIsolation)
{
  auto config = GetParam();
  auto scene = setupScene(config, /*restitution=*/1.0, /*friction=*/0.5);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kMinBounces))
    << config.label << " (elastic): expected at least " << kMinBounces
    << " bounces";

  printBounces(result.bounces, config.label + " [elastic]");

  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    assertCombinedAxisBounce(
      result.bounces[i], static_cast<int>(i),
      scene.initialEnergy, config.label + " [elastic]");

    // Tight energy conservation for elastic regime
    EXPECT_LE(result.bounces[i].energy,
              scene.initialEnergy * kElasticEnergyTolerance)
      << config.label << " [elastic] bounce " << i
      << ": energy not conserved within elastic tolerance. E="
      << result.bounces[i].energy << " initial=" << scene.initialEnergy;
  }

  assertOmegaSignConsistency(result.bounces, config.label + " [elastic]");
}

TEST_P(TiltedDropCombinedAxisTest, Inelastic_ContactResponse)
{
  auto config = GetParam();
  auto scene = setupScene(config, /*restitution=*/0.0, /*friction=*/0.5);
  auto result = detectBounces(scene.cubeId, scene.initialPosition);

  ASSERT_FALSE(result.nanDetected) << "NaN detected in position";
  ASSERT_GE(result.bounces.size(), static_cast<size_t>(kInelasticMinBounces))
    << config.label << " (inelastic): expected at least " << kInelasticMinBounces
    << " contact event";

  printBounces(result.bounces, config.label + " [inelastic]");

  // Energy should drop after contact (inelastic)
  const auto& firstBounce = result.bounces.front();
  EXPECT_LT(firstBounce.energy, scene.initialEnergy)
    << config.label << " [inelastic]: energy should decrease after inelastic contact";

  // Z-axis suppression on first contact
  if (firstBounce.omegaNorm > 1e-6)
  {
    EXPECT_LE(firstBounce.omegaZ, firstBounce.omegaNorm * kXYZAxisRatio)
      << config.label << " [inelastic]: spurious Z rotation. omegaZ="
      << firstBounce.omegaZ << " norm=" << firstBounce.omegaNorm;
  }

  // No energy growth across any contact
  for (size_t i = 0; i < result.bounces.size(); ++i)
  {
    EXPECT_LE(result.bounces[i].energy,
              scene.initialEnergy * kEnergyGrowthTolerance)
      << config.label << " [inelastic] bounce " << i
      << ": energy growth beyond tolerance";
  }
}

INSTANTIATE_TEST_SUITE_P(
  TiltedDrop, TiltedDropCombinedAxisTest,
  ::testing::Values(
    TiltConfig{TiltAxis::XY, 5.0, "XY_5deg"},
    TiltConfig{TiltAxis::XY, 15.0, "XY_15deg"},
    TiltConfig{TiltAxis::XY, 30.0, "XY_30deg"}
  ),
  [](const ::testing::TestParamInfo<TiltConfig>& info) {
    return info.param.label;
  }
);
