// Ticket: 0055a_tilted_cube_trajectory_test_suite
// Test: Tilted cube trajectory direction tests (diagnostic suite)
//
// DIAGNOSTIC TEST SUITE: These tests expose a friction direction bug where
// tilted cubes produce zero or incorrect lateral displacement upon contact
// with the floor. Tests assert that displacement is non-zero and in the
// physically correct direction for each tilt orientation.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/TangentBasis.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

/// Create a cube point cloud with the given size (centered at origin)
std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate{-half, -half, -half}, Coordinate{half, -half, -half},
          Coordinate{half, half, -half},   Coordinate{-half, half, -half},
          Coordinate{-half, -half, half},  Coordinate{half, -half, half},
          Coordinate{half, half, half},    Coordinate{-half, half, half}};
}

/// Result structure for tilted cube simulation
struct TiltResult
{
  Coordinate finalPosition;
  QuaternionD finalOrientation;
  double lateralDisplacementX;
  double lateralDisplacementY;
  double finalZ;
  bool nanDetected;

  // Angular velocity captured at first significant contact (frame where Z
  // velocity reverses sign, indicating floor impact)
  Eigen::Vector3d angularVelocityAtImpact{0.0, 0.0, 0.0};
  bool impactCaptured{false};

  // Peak yaw (Z-axis angular velocity) observed during simulation
  double peakAbsYawRate{0.0};
};

/// Run a tilted cube simulation with specified tilt angles.
/// Returns the final position and orientation after the simulation.
TiltResult runTiltedCubeSimulation(double tiltX,
                                   double tiltY,
                                   double frictionCoeff = 0.5,
                                   double restitution = 0.3,
                                   int frames = 200)
{
  // 16ms per frame (~60 FPS)
  constexpr int kFrameDtMs = 16;

  WorldModel world;

  // Floor: large cube centered at z=-50 (top face at z=0)
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  const auto& floorAsset =
    world.spawnEnvironmentObject(1, floorHull, floorFrame);
  // No non-const accessor for environment objects; const_cast required
  const_cast<AssetEnvironment&>(floorAsset).setFrictionCoefficient(
    frictionCoeff);

  // Tilted cube: 1m unit cube, 10 kg
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  // Build tilt quaternion: rotate about X first, then Y
  Eigen::Quaterniond tiltQuat =
    Eigen::Quaterniond{Eigen::AngleAxisd{tiltY, Eigen::Vector3d::UnitY()}} *
    Eigen::Quaterniond{Eigen::AngleAxisd{tiltX, Eigen::Vector3d::UnitX()}};

  // Position cube so lowest corner is at z=0.01 (just above floor)
  double minZ = std::numeric_limits<double>::max();
  for (const auto& point : cubePoints)
  {
    Eigen::Vector3d rotated =
      tiltQuat * Eigen::Vector3d{point.x(), point.y(), point.z()};
    minZ = std::min(minZ, rotated.z());
  }
  double centerHeightZ = 0.01 - minZ;

  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerHeightZ}, tiltQuat};
  world.spawnObject(2, cubeHull, 10.0, cubeFrame);

  // Instance ID = 1 (first inertial object spawned)
  uint32_t const cubeInstanceId = 1;
  world.getObject(cubeInstanceId).setCoefficientOfRestitution(restitution);
  world.getObject(cubeInstanceId).setFrictionCoefficient(frictionCoeff);

  // Run simulation with incrementing timestamps.
  // Track angular velocity at impact and peak yaw rate.
  bool nanDetected = false;
  bool impactCaptured = false;
  Eigen::Vector3d angularVelocityAtImpact{0.0, 0.0, 0.0};
  double peakAbsYawRate = 0.0;
  double prevVz = 0.0;

  for (int i = 1; i <= frames; ++i)
  {
    world.update(std::chrono::milliseconds{i * kFrameDtMs});

    auto const& state = world.getObject(cubeInstanceId).getInertialState();
    auto const& pos = state.position;

    if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()))
    {
      nanDetected = true;
      break;
    }

    double vz = state.velocity.z();

    // Detect first impact: Z velocity transitions from negative to
    // non-negative (cube was falling, floor pushed it back up)
    if (!impactCaptured && i > 1 && prevVz < -0.01 && vz >= 0.0)
    {
      auto omega = state.getAngularVelocity();
      angularVelocityAtImpact =
        Eigen::Vector3d{omega.x(), omega.y(), omega.z()};
      impactCaptured = true;
    }
    prevVz = vz;

    // Track peak absolute yaw rate (Z-component of angular velocity)
    auto omega = state.getAngularVelocity();
    double absYaw = std::abs(omega.z());
    peakAbsYawRate = std::max(peakAbsYawRate, absYaw);
  }

  // Extract final state
  TiltResult result;
  result.nanDetected = nanDetected;
  result.impactCaptured = impactCaptured;
  result.angularVelocityAtImpact = angularVelocityAtImpact;
  result.peakAbsYawRate = peakAbsYawRate;

  auto const& finalState = world.getObject(cubeInstanceId).getInertialState();
  result.finalPosition = finalState.position;
  result.finalOrientation = finalState.orientation;
  result.lateralDisplacementX = finalState.position.x();
  result.lateralDisplacementY = finalState.position.y();
  result.finalZ = finalState.position.z();

  return result;
}

/// Result structure for thrown/sliding cube simulations.
/// Captures per-frame snapshots of velocity and angular velocity to detect
/// rotation reversal relative to sliding direction.
struct SlidingResult
{
  bool nanDetected{false};

  // Final state
  Coordinate finalPosition;
  double finalVx{0.0};

  // Rotation-vs-sliding consistency metric:
  // Count frames where the cube is sliding (|vx| > threshold) AND the
  // angular velocity about Y has the WRONG sign relative to sliding.
  //
  // Physics: if sliding in +X, friction acts in -X at the contact point.
  // Torque = r × F where r ≈ (0, 0, -h/2) (contact below CoM):
  //   (0, 0, -h/2) × (-F, 0, 0) = (0, +Fh/2, 0)
  // So sliding in +X → positive ωy (rolling forward).
  // If friction direction is wrong, ωy will be negative (rolling backward).
  int framesSliding{0};
  int framesReversedRotation{0};  // sliding in +X but ωy < 0 (or vice versa)

  // Peak reversed rotation rate (worst violation)
  double peakReversedOmegaY{0.0};

  // Spurious cross-axis translation tracking
  double peakAbsY{0.0};    // peak |posY| during simulation
  double peakAbsVy{0.0};   // peak |vy| during simulation

  // Time series for diagnostic output
  struct FrameSnapshot
  {
    int frame;
    double vx;
    double vy;
    double omegaY;
    double posX;
    double posY;
    double posZ;
  };
  std::vector<FrameSnapshot> snapshots;
};

/// Run a sliding cube simulation: cube on the floor with initial velocity.
/// The cube has a compound tilt (tiltX, tiltY) and is launched with
/// initialVx along the X axis. We track whether friction-induced rotation
/// is consistent with the sliding direction.
SlidingResult runSlidingCubeSimulation(double tiltX,
                                       double tiltY,
                                       double initialVx,
                                       double frictionCoeff = 0.5,
                                       double restitution = 0.5,
                                       int frames = 300,
                                       int frameDtMs = 10)
{
  constexpr double kSlidingThreshold = 0.1;  // m/s minimum to count as sliding

  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  const auto& floorAsset =
    world.spawnEnvironmentObject(1, floorHull, floorFrame);
  const_cast<AssetEnvironment&>(floorAsset).setFrictionCoefficient(
    frictionCoeff);

  // Tilted cube
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  Eigen::Quaterniond tiltQuat =
    Eigen::Quaterniond{Eigen::AngleAxisd{tiltY, Eigen::Vector3d::UnitY()}} *
    Eigen::Quaterniond{Eigen::AngleAxisd{tiltX, Eigen::Vector3d::UnitX()}};

  // Position cube so lowest point is at z=0.01
  double minZ = std::numeric_limits<double>::max();
  for (const auto& point : cubePoints)
  {
    Eigen::Vector3d rotated =
      tiltQuat * Eigen::Vector3d{point.x(), point.y(), point.z()};
    minZ = std::min(minZ, rotated.z());
  }
  double centerHeightZ = 0.01 - minZ;

  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerHeightZ}, tiltQuat};
  world.spawnObject(2, cubeHull, 10.0, cubeFrame);

  uint32_t const cubeInstanceId = 1;
  world.getObject(cubeInstanceId).setCoefficientOfRestitution(restitution);
  world.getObject(cubeInstanceId).setFrictionCoefficient(frictionCoeff);

  // Set initial velocity
  world.getObject(cubeInstanceId).getInertialState().velocity =
    Vector3D{initialVx, 0.0, 0.0};

  SlidingResult result;

  for (int i = 1; i <= frames; ++i)
  {
    world.update(std::chrono::milliseconds{i * frameDtMs});

    auto const& state = world.getObject(cubeInstanceId).getInertialState();
    auto const& pos = state.position;

    if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()))
    {
      result.nanDetected = true;
      break;
    }

    double vx = state.velocity.x();
    double vy = state.velocity.y();
    auto omega = state.getAngularVelocity();
    double omegaY = omega.y();

    // Track peak cross-axis (Y) displacement and velocity
    result.peakAbsY = std::max(result.peakAbsY, std::abs(pos.y()));
    result.peakAbsVy = std::max(result.peakAbsVy, std::abs(vy));

    // Record snapshot every 10 frames for diagnostics
    if (i % 10 == 0)
    {
      result.snapshots.push_back(
        {i, vx, vy, omegaY, pos.x(), pos.y(), pos.z()});
    }

    // Check rotation-vs-sliding consistency when sliding significantly
    if (std::abs(vx) > kSlidingThreshold)
    {
      result.framesSliding++;

      // Physics: sliding in +X → ωy should be positive (rolling forward)
      //          sliding in -X → ωy should be negative
      // "Reversed" means the signs disagree: vx * ωy < 0
      bool reversed = (vx * omegaY < 0.0);
      if (reversed)
      {
        result.framesReversedRotation++;
        double absOmegaY = std::abs(omegaY);
        result.peakReversedOmegaY =
          std::max(result.peakReversedOmegaY, absOmegaY);
      }
    }
  }

  auto const& finalState = world.getObject(cubeInstanceId).getInertialState();
  result.finalPosition = finalState.position;
  result.finalVx = finalState.velocity.x();

  return result;
}

}  // namespace

// ============================================================================
// Helper: compute lateral displacement magnitude
// ============================================================================

double lateralMagnitude(TiltResult const& r)
{
  return std::sqrt(r.lateralDisplacementX * r.lateralDisplacementX +
                   r.lateralDisplacementY * r.lateralDisplacementY);
}

// ============================================================================
// Individual Tilt Orientation Tests (T1-T8)
//
// Each test drops a cube tilted by θ = 0.1 rad (~5.7°) and asserts:
//   1. No NaN / divergence
//   2. Non-zero lateral displacement (friction should cause lateral motion)
//   3. Displacement direction consistent with tilt geometry
//
// Right-handed coordinate system (Z-up). The lower edge/corner contacts the
// floor first; friction anchors it while the CoM swings AWAY from the contact:
//   +X rot: y=-0.5 edge drops → CoM displaces toward +Y
//   -X rot: y=+0.5 edge drops → CoM displaces toward -Y
//   +Y rot: x=+0.5 edge drops → CoM displaces toward -X
//   -Y rot: x=-0.5 edge drops → CoM displaces toward +X
// ============================================================================

constexpr double kTheta = 0.1;         // radians (~5.7 degrees)
constexpr double kMinDisplacement = 1e-4;  // minimum expected lateral motion [m]

TEST(TiltedCubeTrajectory, T1_PurePositiveXTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(kTheta, 0.0);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: +X tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // +X rotation drops y=-0.5 edge → CoM swings toward +Y
  EXPECT_GT(result.lateralDisplacementY, 0.0)
    << "DIAGNOSTIC: +X tilt should displace toward +Y. "
    << "Actual Y=" << result.lateralDisplacementY;
}

TEST(TiltedCubeTrajectory, T2_PureNegativeXTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(-kTheta, 0.0);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: -X tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // -X rotation drops y=+0.5 edge → CoM swings toward -Y
  EXPECT_LT(result.lateralDisplacementY, 0.0)
    << "DIAGNOSTIC: -X tilt should displace toward -Y. "
    << "Actual Y=" << result.lateralDisplacementY;
}

TEST(TiltedCubeTrajectory, T3_PurePositiveYTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(0.0, kTheta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: +Y tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // +Y rotation drops x=+0.5 edge → CoM swings toward -X
  EXPECT_LT(result.lateralDisplacementX, 0.0)
    << "DIAGNOSTIC: +Y tilt should displace toward -X. "
    << "Actual X=" << result.lateralDisplacementX;
}

TEST(TiltedCubeTrajectory, T4_PureNegativeYTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(0.0, -kTheta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: -Y tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // -Y rotation drops x=-0.5 edge → CoM swings toward +X
  EXPECT_GT(result.lateralDisplacementX, 0.0)
    << "DIAGNOSTIC: -Y tilt should displace toward +X. "
    << "Actual X=" << result.lateralDisplacementX;
}

// ============================================================================
// Compound Tilt Tests (T5-T8)
//
// DIAGNOSTIC: These tests expose incorrect coupled behavior when a cube is
// tilted about both X and Y simultaneously. The user confirms that single-axis
// tilts (T1-T4) behave correctly, but compound tilts produce "visibly
// incorrect rotation."
//
// For compound tilts, the displacement magnitude should be comparable to the
// vector sum of the single-axis effects (superposition). A cube tilted at
// (+θ,+θ) should move approximately sqrt(2) times as far as a single-axis
// tilt at θ, since both axes contribute. We require at least 50% of the
// superposition prediction.
//
// Additionally, tilts about X and Y should NOT produce yaw (Z-axis rotation).
// A physically correct bounce produces rotation about axes in the XY plane
// only. Any significant yaw rate indicates incorrect friction coupling.
// ============================================================================

// Maximum allowed yaw rate for a tilt that is purely about X and Y.
// Physically, yaw should be exactly zero. We allow a small tolerance for
// numerical noise. If the friction direction is wrong, yaw will be large.
constexpr double kMaxSpuriousYawRate = 0.05;  // rad/s

TEST(TiltedCubeTrajectory, T5_CompoundXPlusYTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(kTheta, kTheta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: +X+Y tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // (+X,+Y) tilt drops (x=+0.5, y=-0.5) corner → CoM swings toward -X, +Y
  EXPECT_LT(result.lateralDisplacementX, 0.0)
    << "DIAGNOSTIC: +X+Y tilt should displace toward -X. "
    << "Actual X=" << result.lateralDisplacementX;
  EXPECT_GT(result.lateralDisplacementY, 0.0)
    << "DIAGNOSTIC: +X+Y tilt should displace toward +Y. "
    << "Actual Y=" << result.lateralDisplacementY;

  // No spurious yaw: tilt about X and Y should not produce Z-axis rotation
  EXPECT_LT(result.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: +X+Y compound tilt produced spurious yaw rotation. "
    << "Peak |yaw rate|=" << result.peakAbsYawRate << " rad/s. "
    << "Expected near zero for tilt purely about X and Y axes.";
}

TEST(TiltedCubeTrajectory, T6_CompoundXMinusYTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(kTheta, -kTheta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: +X-Y tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // (+X,-Y) tilt drops (x=-0.5, y=-0.5) corner → CoM swings toward +X, +Y
  EXPECT_GT(result.lateralDisplacementX, 0.0)
    << "DIAGNOSTIC: +X-Y tilt should displace toward +X. "
    << "Actual X=" << result.lateralDisplacementX;
  EXPECT_GT(result.lateralDisplacementY, 0.0)
    << "DIAGNOSTIC: +X-Y tilt should displace toward +Y. "
    << "Actual Y=" << result.lateralDisplacementY;

  // No spurious yaw
  EXPECT_LT(result.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: +X-Y compound tilt produced spurious yaw rotation. "
    << "Peak |yaw rate|=" << result.peakAbsYawRate << " rad/s.";
}

TEST(TiltedCubeTrajectory, T7_CompoundNegXPlusYTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(-kTheta, kTheta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: -X+Y tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // (-X,+Y) tilt drops (x=+0.5, y=+0.5) corner → CoM swings toward -X, -Y
  EXPECT_LT(result.lateralDisplacementX, 0.0)
    << "DIAGNOSTIC: -X+Y tilt should displace toward -X. "
    << "Actual X=" << result.lateralDisplacementX;
  EXPECT_LT(result.lateralDisplacementY, 0.0)
    << "DIAGNOSTIC: -X+Y tilt should displace toward -Y. "
    << "Actual Y=" << result.lateralDisplacementY;

  // No spurious yaw
  EXPECT_LT(result.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: -X+Y compound tilt produced spurious yaw rotation. "
    << "Peak |yaw rate|=" << result.peakAbsYawRate << " rad/s.";
}

TEST(TiltedCubeTrajectory, T8_CompoundNegXMinusYTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult result = runTiltedCubeSimulation(-kTheta, -kTheta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";

  double mag = lateralMagnitude(result);
  EXPECT_GT(mag, kMinDisplacement)
    << "DIAGNOSTIC: -X-Y tilt produced no lateral displacement. "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // (-X,-Y) tilt drops (x=-0.5, y=+0.5) corner → CoM swings toward +X, -Y
  EXPECT_GT(result.lateralDisplacementX, 0.0)
    << "DIAGNOSTIC: -X-Y tilt should displace toward +X. "
    << "Actual X=" << result.lateralDisplacementX;
  EXPECT_LT(result.lateralDisplacementY, 0.0)
    << "DIAGNOSTIC: -X-Y tilt should displace toward -Y. "
    << "Actual Y=" << result.lateralDisplacementY;

  // No spurious yaw
  EXPECT_LT(result.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: -X-Y compound tilt produced spurious yaw rotation. "
    << "Peak |yaw rate|=" << result.peakAbsYawRate << " rad/s.";
}

// ============================================================================
// Symmetry Tests
//
// Negating the tilt angle should mirror the trajectory. These tests verify:
//   1. Both configurations produce non-zero displacement
//   2. Displacement in the tilt-affected axis has opposite sign
//   3. Magnitudes are comparable (within 2x)
// ============================================================================

TEST(TiltedCubeTrajectory, Symmetry_XTilt_MirrorsYDisplacement)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult resultPos = runTiltedCubeSimulation(kTheta, 0.0);
  TiltResult resultNeg = runTiltedCubeSimulation(-kTheta, 0.0);

  ASSERT_FALSE(resultPos.nanDetected) << "NaN in +X tilt";
  ASSERT_FALSE(resultNeg.nanDetected) << "NaN in -X tilt";

  double magPos = lateralMagnitude(resultPos);
  double magNeg = lateralMagnitude(resultNeg);

  // Both must produce displacement
  EXPECT_GT(magPos, kMinDisplacement)
    << "+X tilt produced no displacement";
  EXPECT_GT(magNeg, kMinDisplacement)
    << "-X tilt produced no displacement";

  // Y displacement should flip sign (X rotation primarily affects Y)
  if (magPos > kMinDisplacement && magNeg > kMinDisplacement)
  {
    EXPECT_LT(resultPos.lateralDisplacementY * resultNeg.lateralDisplacementY,
              0.0)
      << "DIAGNOSTIC: X tilt symmetry broken — Y displacement should flip sign. "
      << "+X→Y=" << resultPos.lateralDisplacementY
      << ", -X→Y=" << resultNeg.lateralDisplacementY;

    // Magnitudes within 2x
    double ratio = magPos / magNeg;
    EXPECT_GT(ratio, 0.5) << "Magnitude ratio out of range: " << ratio;
    EXPECT_LT(ratio, 2.0) << "Magnitude ratio out of range: " << ratio;
  }
}

TEST(TiltedCubeTrajectory, Symmetry_YTilt_MirrorsXDisplacement)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  TiltResult resultPos = runTiltedCubeSimulation(0.0, kTheta);
  TiltResult resultNeg = runTiltedCubeSimulation(0.0, -kTheta);

  ASSERT_FALSE(resultPos.nanDetected) << "NaN in +Y tilt";
  ASSERT_FALSE(resultNeg.nanDetected) << "NaN in -Y tilt";

  double magPos = lateralMagnitude(resultPos);
  double magNeg = lateralMagnitude(resultNeg);

  EXPECT_GT(magPos, kMinDisplacement)
    << "+Y tilt produced no displacement";
  EXPECT_GT(magNeg, kMinDisplacement)
    << "-Y tilt produced no displacement";

  // X displacement should flip sign (Y rotation primarily affects X)
  if (magPos > kMinDisplacement && magNeg > kMinDisplacement)
  {
    EXPECT_LT(resultPos.lateralDisplacementX * resultNeg.lateralDisplacementX,
              0.0)
      << "DIAGNOSTIC: Y tilt symmetry broken — X displacement should flip sign. "
      << "+Y→X=" << resultPos.lateralDisplacementX
      << ", -Y→X=" << resultNeg.lateralDisplacementX;

    double ratio = magPos / magNeg;
    EXPECT_GT(ratio, 0.5) << "Magnitude ratio out of range: " << ratio;
    EXPECT_LT(ratio, 2.0) << "Magnitude ratio out of range: " << ratio;
  }
}

TEST(TiltedCubeTrajectory, Symmetry_CompoundTilt_DiagonalMirror)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  // T5 (+X,+Y) vs T8 (-X,-Y): full negation should fully mirror
  TiltResult resultT5 = runTiltedCubeSimulation(kTheta, kTheta);
  TiltResult resultT8 = runTiltedCubeSimulation(-kTheta, -kTheta);

  ASSERT_FALSE(resultT5.nanDetected) << "NaN in T5 (+X+Y tilt)";
  ASSERT_FALSE(resultT8.nanDetected) << "NaN in T8 (-X-Y tilt)";

  double magT5 = lateralMagnitude(resultT5);
  double magT8 = lateralMagnitude(resultT8);

  EXPECT_GT(magT5, kMinDisplacement)
    << "T5 (+X+Y) produced no displacement";
  EXPECT_GT(magT8, kMinDisplacement)
    << "T8 (-X-Y) produced no displacement";

  // Both X and Y displacement should flip sign
  if (magT5 > kMinDisplacement && magT8 > kMinDisplacement)
  {
    EXPECT_LT(resultT5.lateralDisplacementX * resultT8.lateralDisplacementX,
              0.0)
      << "DIAGNOSTIC: Diagonal mirror X broken. "
      << "T5.X=" << resultT5.lateralDisplacementX
      << ", T8.X=" << resultT8.lateralDisplacementX;
    EXPECT_LT(resultT5.lateralDisplacementY * resultT8.lateralDisplacementY,
              0.0)
      << "DIAGNOSTIC: Diagonal mirror Y broken. "
      << "T5.Y=" << resultT5.lateralDisplacementY
      << ", T8.Y=" << resultT8.lateralDisplacementY;

    double ratio = magT5 / magT8;
    EXPECT_GT(ratio, 0.5) << "Magnitude ratio out of range: " << ratio;
    EXPECT_LT(ratio, 2.0) << "Magnitude ratio out of range: " << ratio;
  }
}

TEST(TiltedCubeTrajectory, Symmetry_CompoundTilt_AntiDiagonalMirror)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  // T6 (+X,-Y) vs T7 (-X,+Y): full negation should fully mirror
  TiltResult resultT6 = runTiltedCubeSimulation(kTheta, -kTheta);
  TiltResult resultT7 = runTiltedCubeSimulation(-kTheta, kTheta);

  ASSERT_FALSE(resultT6.nanDetected) << "NaN in T6 (+X-Y tilt)";
  ASSERT_FALSE(resultT7.nanDetected) << "NaN in T7 (-X+Y tilt)";

  double magT6 = lateralMagnitude(resultT6);
  double magT7 = lateralMagnitude(resultT7);

  EXPECT_GT(magT6, kMinDisplacement)
    << "T6 (+X-Y) produced no displacement";
  EXPECT_GT(magT7, kMinDisplacement)
    << "T7 (-X+Y) produced no displacement";

  if (magT6 > kMinDisplacement && magT7 > kMinDisplacement)
  {
    EXPECT_LT(resultT6.lateralDisplacementX * resultT7.lateralDisplacementX,
              0.0)
      << "DIAGNOSTIC: Anti-diagonal mirror X broken. "
      << "T6.X=" << resultT6.lateralDisplacementX
      << ", T7.X=" << resultT7.lateralDisplacementX;
    EXPECT_LT(resultT6.lateralDisplacementY * resultT7.lateralDisplacementY,
              0.0)
      << "DIAGNOSTIC: Anti-diagonal mirror Y broken. "
      << "T6.Y=" << resultT6.lateralDisplacementY
      << ", T7.Y=" << resultT7.lateralDisplacementY;

    double ratio = magT6 / magT7;
    EXPECT_GT(ratio, 0.5) << "Magnitude ratio out of range: " << ratio;
    EXPECT_LT(ratio, 2.0) << "Magnitude ratio out of range: " << ratio;
  }
}

// ============================================================================
// Compound Tilt Diagnostic Tests
//
// These tests specifically target the physically wrong behavior observed in
// compound tilts. They compare compound results against single-axis baselines
// to quantify how much the coupled response deviates from correct physics.
// ============================================================================

TEST(TiltedCubeTrajectory, Compound_SuperpositionMagnitude)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  //
  // A cube tilted at (+θ, +θ) should produce lateral displacement comparable
  // to the vector sum of the single-axis effects. For small angles with the
  // same θ, the expected compound magnitude is approximately:
  //   mag_compound ≈ sqrt(mag_x^2 + mag_y^2)
  //
  // We require the compound displacement to be at least 50% of this
  // superposition prediction. If friction direction is wrong for compound
  // tilts, the magnitude will be severely suppressed.

  TiltResult singleX = runTiltedCubeSimulation(kTheta, 0.0);
  TiltResult singleY = runTiltedCubeSimulation(0.0, kTheta);
  TiltResult compound = runTiltedCubeSimulation(kTheta, kTheta);

  ASSERT_FALSE(singleX.nanDetected);
  ASSERT_FALSE(singleY.nanDetected);
  ASSERT_FALSE(compound.nanDetected);

  double magX = lateralMagnitude(singleX);
  double magY = lateralMagnitude(singleY);
  double magCompound = lateralMagnitude(compound);
  double superpositionPrediction = std::sqrt(magX * magX + magY * magY);

  // Compound displacement must be at least 50% of superposition prediction
  double ratio = magCompound / superpositionPrediction;
  EXPECT_GT(ratio, 0.5)
    << "DIAGNOSTIC: Compound displacement severely suppressed. "
    << "Single-X mag=" << magX
    << ", Single-Y mag=" << magY
    << ", Superposition prediction=" << superpositionPrediction
    << ", Compound mag=" << magCompound
    << " (" << (ratio * 100.0) << "% of prediction). "
    << "Compound pos=(" << compound.lateralDisplacementX
    << ", " << compound.lateralDisplacementY << ")";
}

TEST(TiltedCubeTrajectory, Compound_NoSpuriousYaw)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  //
  // A cube tilted about X and Y should rotate about axes in the XY plane
  // only. Physically, the impact torque has no Z-axis component because:
  //   1. The contact normal is approximately (0,0,1)
  //   2. The friction force is in the XY plane
  //   3. The lever arm from CoM to contact is mostly in the Z direction
  //   4. Cross products produce torques about X and Y, not Z
  //
  // If the friction direction is incorrect, it can produce a lever arm
  // component that generates yaw torque.

  // Test all four compound configurations
  TiltResult t5 = runTiltedCubeSimulation(kTheta, kTheta);
  TiltResult t6 = runTiltedCubeSimulation(kTheta, -kTheta);
  TiltResult t7 = runTiltedCubeSimulation(-kTheta, kTheta);
  TiltResult t8 = runTiltedCubeSimulation(-kTheta, -kTheta);

  ASSERT_FALSE(t5.nanDetected);
  ASSERT_FALSE(t6.nanDetected);
  ASSERT_FALSE(t7.nanDetected);
  ASSERT_FALSE(t8.nanDetected);

  EXPECT_LT(t5.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: T5 (+X+Y) spurious yaw=" << t5.peakAbsYawRate
    << " rad/s. Impact ω=(" << t5.angularVelocityAtImpact.x()
    << ", " << t5.angularVelocityAtImpact.y()
    << ", " << t5.angularVelocityAtImpact.z() << ")";

  EXPECT_LT(t6.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: T6 (+X-Y) spurious yaw=" << t6.peakAbsYawRate
    << " rad/s. Impact ω=(" << t6.angularVelocityAtImpact.x()
    << ", " << t6.angularVelocityAtImpact.y()
    << ", " << t6.angularVelocityAtImpact.z() << ")";

  EXPECT_LT(t7.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: T7 (-X+Y) spurious yaw=" << t7.peakAbsYawRate
    << " rad/s. Impact ω=(" << t7.angularVelocityAtImpact.x()
    << ", " << t7.angularVelocityAtImpact.y()
    << ", " << t7.angularVelocityAtImpact.z() << ")";

  EXPECT_LT(t8.peakAbsYawRate, kMaxSpuriousYawRate)
    << "DIAGNOSTIC: T8 (-X-Y) spurious yaw=" << t8.peakAbsYawRate
    << " rad/s. Impact ω=(" << t8.angularVelocityAtImpact.x()
    << ", " << t8.angularVelocityAtImpact.y()
    << ", " << t8.angularVelocityAtImpact.z() << ")";

  // Also verify single-axis tilts do NOT have yaw (baseline check)
  TiltResult t1 = runTiltedCubeSimulation(kTheta, 0.0);
  TiltResult t3 = runTiltedCubeSimulation(0.0, kTheta);

  EXPECT_LT(t1.peakAbsYawRate, kMaxSpuriousYawRate)
    << "BASELINE: T1 (+X) should have no yaw either. "
    << "Peak |yaw|=" << t1.peakAbsYawRate;
  EXPECT_LT(t3.peakAbsYawRate, kMaxSpuriousYawRate)
    << "BASELINE: T3 (+Y) should have no yaw either. "
    << "Peak |yaw|=" << t3.peakAbsYawRate;
}

TEST(TiltedCubeTrajectory, Compound_DisplacementAngleMatchesTilt)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  //
  // For a symmetric compound tilt (+θ, +θ), the displacement direction
  // should be approximately 45° from both axes (i.e., |dX| ≈ |dY|).
  // More precisely, the displacement direction should reflect the tilt
  // geometry: equal tilt about both axes → equal displacement components.
  //
  // We test that the ratio |dX|/|dY| is between 0.3 and 3.0 (generous
  // bounds). If the friction direction is wrong, one component could be
  // severely suppressed or enhanced relative to the other.

  TiltResult t5 = runTiltedCubeSimulation(kTheta, kTheta);

  ASSERT_FALSE(t5.nanDetected);

  double absX = std::abs(t5.lateralDisplacementX);
  double absY = std::abs(t5.lateralDisplacementY);

  // Both components must be non-negligible
  EXPECT_GT(absX, kMinDisplacement)
    << "DIAGNOSTIC: +X+Y tilt has negligible X displacement. "
    << "X=" << t5.lateralDisplacementX;
  EXPECT_GT(absY, kMinDisplacement)
    << "DIAGNOSTIC: +X+Y tilt has negligible Y displacement. "
    << "Y=" << t5.lateralDisplacementY;

  if (absX > kMinDisplacement && absY > kMinDisplacement)
  {
    double componentRatio = absX / absY;
    EXPECT_GT(componentRatio, 0.3)
      << "DIAGNOSTIC: Displacement angle severely skewed for symmetric tilt. "
      << "|dX|=" << absX << ", |dY|=" << absY
      << ", ratio=" << componentRatio
      << ". Expected ~1.0 for equal-angle tilt.";
    EXPECT_LT(componentRatio, 3.0)
      << "DIAGNOSTIC: Displacement angle severely skewed for symmetric tilt. "
      << "|dX|=" << absX << ", |dY|=" << absY
      << ", ratio=" << componentRatio
      << ". Expected ~1.0 for equal-angle tilt.";
  }
}

// ============================================================================
// Friction Rotation Direction Tests
//
// DIAGNOSTIC: These tests capture the "reversed rotation" bug observed in the
// GUI. When a cube slides along the floor, friction should cause it to roll
// FORWARD (in the direction of motion). The physics:
//
//   Sliding in +X → friction acts in -X at the contact point
//   Torque = r × F where r ≈ (0, 0, -h/2) (contact below CoM)
//   τ = (0, 0, -h/2) × (-F, 0, 0) = (0, +Fh/2, 0)
//   → positive ωy = rolling forward (correct)
//
// If friction direction is wrong, ωy will be negative (rolling backward
// while sliding forward), which is the bug the user observes.
//
// The bug manifests specifically when the cube has a compound tilt (rotation
// about both X and Y). A pure single-axis tilt works correctly.
// ============================================================================

TEST(TiltedCubeTrajectory, Sliding_CompoundDrop_DirectionReversal)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  //
  // CORE BUG: The user observes: "When I drop a cube with rotations
  // around X and Y set to PI/6 each, the cube appears to initially
  // tumble in the correct direction... After a few bounces, this
  // behavior shifts and the cube slides, but rotates in the opposite
  // direction of the sliding."
  //
  // This test captures "direction reversal": the displacement direction
  // in the early simulation (first 100 frames) should be consistent
  // with the displacement direction in the late simulation (frames 200-500).
  // If friction is wrong, the late phase reverses direction.
  //
  // We use a longer simulation (500 frames = 8s) to allow multiple
  // bounces and observe the settling behavior.

  // Drop with compound tilt (PI/6, PI/6), no initial velocity
  // GUI defaults: friction=0.5, restitution=0.5
  TiltResult early = runTiltedCubeSimulation(
    M_PI / 6.0, M_PI / 6.0,  // compound tilt
    0.5, 0.5,                 // friction, restitution (GUI defaults)
    100);                     // first 100 frames only

  TiltResult late = runTiltedCubeSimulation(
    M_PI / 6.0, M_PI / 6.0,  // same compound tilt
    0.5, 0.5,                 // same parameters
    500);                     // full 500 frames

  ASSERT_FALSE(early.nanDetected);
  ASSERT_FALSE(late.nanDetected);

  double earlyMag = lateralMagnitude(early);
  double lateMag = lateralMagnitude(late);

  // Both should have some displacement
  ASSERT_GT(earlyMag, kMinDisplacement) << "No early displacement";
  ASSERT_GT(lateMag, kMinDisplacement) << "No late displacement";

  // The displacement DIRECTION should be consistent between early and
  // late phases. Compute the dot product of the 2D displacement vectors:
  // if positive, same direction; if negative, reversed.
  double dotProduct = early.lateralDisplacementX * late.lateralDisplacementX +
                      early.lateralDisplacementY * late.lateralDisplacementY;

  EXPECT_GT(dotProduct, 0.0)
    << "DIAGNOSTIC: Displacement direction REVERSED between early and late. "
    << "Early (100 frames): (" << early.lateralDisplacementX
    << ", " << early.lateralDisplacementY << ") mag=" << earlyMag
    << ". Late (500 frames): (" << late.lateralDisplacementX
    << ", " << late.lateralDisplacementY << ") mag=" << lateMag
    << ". The cube initially moved one way, then reversed.";
}

TEST(TiltedCubeTrajectory, Sliding_ThrownCube_SDLAppConfig)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  //
  // EXACT reproduction of the user's SDLApp.cpp configuration:
  //   AngularCoordinate{M_PI / 3, 0.01, 0.}  (pitch=PI/3, roll=0.01)
  //   velocity = (5, 0, 0)
  //   restitution = 0.5, friction = 0.5, timestep = 10ms
  //
  // The user reports "very peculiar, reversed rotations and motions"
  // and spurious Y-direction translations from the tiny 0.01 rad roll.
  //
  // Assertions:
  //   1. Final X position is positive (thrown +X, friction cannot reverse)
  //   2. |Y displacement| stays small — a 0.01 rad perturbation should not
  //      cause large cross-axis motion. We bound |Y| < 20% of |X|.
  //   3. Rotation direction matches sliding direction in settled phase.

  SlidingResult result = runSlidingCubeSimulation(
    0.01,        // tiltX (roll) = 0.01 — tiny perturbation
    M_PI / 3.0,  // tiltY (pitch) = PI/3 — large pitch
    5.0,         // initialVx = 5 m/s
    0.5, 0.5,   // friction, restitution (GUI defaults)
    2000,        // 2000 frames = 20 seconds at 10ms
    10);         // 10ms timestep (matches SDLApp frameDelta)

  ASSERT_FALSE(result.nanDetected);

  // 1. Final X must be positive: cube thrown +X cannot end up at -X.
  EXPECT_GT(result.finalPosition.x(), 0.0)
    << "DIAGNOSTIC: Cube thrown in +X ended up at -X! "
    << "Final pos=(" << result.finalPosition.x()
    << ", " << result.finalPosition.y()
    << ", " << result.finalPosition.z() << ")";

  // 2. Spurious Y translation: a 0.01 rad X-axis perturbation on a cube
  // thrown purely in +X should produce negligible Y motion. The initial
  // momentum is entirely along X; friction acts primarily along X to
  // decelerate the slide. The tiny roll shifts the contact point by
  // ~0.005m in Y, which should produce proportionally tiny Y impulse.
  //
  // We assert peak |Y| < 20% of final |X|. If friction/collision coupling
  // is wrong, the tiny perturbation gets amplified into large spurious Y.
  double absX = std::abs(result.finalPosition.x());
  double absY = std::abs(result.finalPosition.y());
  if (absX > 0.1)
  {
    double yToXRatio = absY / absX;
    EXPECT_LT(yToXRatio, 0.2)
      << "DIAGNOSTIC: Spurious Y displacement from 0.01 rad perturbation. "
      << "|Y|=" << absY << ", |X|=" << absX
      << ", |Y|/|X|=" << yToXRatio
      << ". Peak |Y|=" << result.peakAbsY
      << ", Peak |vy|=" << result.peakAbsVy;
  }

  // Also check peak Y displacement during the entire trajectory
  EXPECT_LT(result.peakAbsY, 2.0)
    << "DIAGNOSTIC: Peak Y displacement=" << result.peakAbsY
    << "m from a 0.01 rad perturbation. This is unreasonably large.";

  // 3. Settled-phase rotation direction check.
  // After frame 500 (5s), the cube should be past the initial tumbling.
  int settledReversed = 0;
  int settledSliding = 0;
  for (auto const& s : result.snapshots)
  {
    if (s.frame < 500)
      continue;
    if (std::abs(s.vx) > 0.05)
    {
      settledSliding++;
      if (s.vx * s.omegaY < 0.0)
        settledReversed++;
    }
  }
  if (settledSliding > 3)
  {
    double frac = static_cast<double>(settledReversed) /
                  static_cast<double>(settledSliding);
    EXPECT_LT(frac, 0.3)
      << "DIAGNOSTIC: Settled phase (>5s) shows reversed rotation. "
      << settledReversed << "/" << settledSliding
      << " sliding frames have vx*ωy<0 (backward rolling).";
  }

  // Print trajectory for investigation
  if (!result.snapshots.empty())
  {
    std::string diag = "Trajectory (every 10 frames):\n";
    for (auto const& s : result.snapshots)
    {
      diag += "  frame " + std::to_string(s.frame) +
              ": vx=" + std::to_string(s.vx) +
              " vy=" + std::to_string(s.vy) +
              " wy=" + std::to_string(s.omegaY) +
              " x=" + std::to_string(s.posX) +
              " y=" + std::to_string(s.posY) +
              " z=" + std::to_string(s.posZ) +
              (s.vx * s.omegaY < 0.0 ? " REV-ROT" : "") + "\n";
    }
    SCOPED_TRACE(diag);
  }
}

TEST(TiltedCubeTrajectory, Sliding_PurePitch_vs_CompoundTilt_SpuriousY)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  //
  // Compare pure pitch (tiltX=0) against compound tilt (tiltX=0.01).
  // Both cubes are thrown in +X at 5 m/s with the same PI/3 pitch.
  //
  // The pure pitch baseline should have ZERO Y displacement (by symmetry).
  // The compound tilt adds 0.01 rad roll — proportionally tiny, so its
  // Y displacement should be small (bounded by the perturbation magnitude).
  //
  // If the collision/friction coupling amplifies the perturbation, the
  // compound case will have disproportionately large Y displacement.

  SlidingResult purePitch = runSlidingCubeSimulation(
    0.0, M_PI / 3.0, 5.0, 0.5, 0.5, 2000, 10);
  SlidingResult compound = runSlidingCubeSimulation(
    0.01, M_PI / 3.0, 5.0, 0.5, 0.5, 2000, 10);

  ASSERT_FALSE(purePitch.nanDetected);
  ASSERT_FALSE(compound.nanDetected);

  // Pure pitch baseline: Y displacement should be near zero by symmetry.
  EXPECT_LT(std::abs(purePitch.finalPosition.y()), 0.1)
    << "BASELINE: Pure pitch (no roll) should have ~zero Y displacement. "
    << "Actual Y=" << purePitch.finalPosition.y();

  // Pure pitch: X should be positive.
  EXPECT_GT(purePitch.finalPosition.x(), 0.0)
    << "BASELINE: Pure pitch cube thrown +X ended at x="
    << purePitch.finalPosition.x();

  // Compound: X should also be positive and in the same ballpark.
  EXPECT_GT(compound.finalPosition.x(), 0.0)
    << "DIAGNOSTIC: Compound tilt cube thrown +X ended at x="
    << compound.finalPosition.x()
    << " (pure pitch: x=" << purePitch.finalPosition.x() << ")";

  // Compound: Y displacement should be proportional to the perturbation.
  // 0.01 rad is 1/105 of PI/3. Y displacement should be << X displacement.
  double compoundAbsY = std::abs(compound.finalPosition.y());
  double compoundAbsX = std::abs(compound.finalPosition.x());
  if (compoundAbsX > 0.1)
  {
    double yToXRatio = compoundAbsY / compoundAbsX;
    EXPECT_LT(yToXRatio, 0.2)
      << "DIAGNOSTIC: Compound Y/X ratio=" << yToXRatio
      << " — tiny 0.01 rad perturbation caused disproportionate Y motion. "
      << "Compound pos=(" << compound.finalPosition.x()
      << ", " << compound.finalPosition.y() << "). "
      << "Pure pitch pos=(" << purePitch.finalPosition.x()
      << ", " << purePitch.finalPosition.y() << ").";
  }

  // Compound X should be similar to pure pitch X (perturbation is tiny).
  if (purePitch.finalPosition.x() > 0.1 && compound.finalPosition.x() > 0.1)
  {
    double xRatio = compound.finalPosition.x() / purePitch.finalPosition.x();
    EXPECT_GT(xRatio, 0.3)
      << "DIAGNOSTIC: Compound X diverged from pure pitch. "
      << "Pure=" << purePitch.finalPosition.x()
      << ", Compound=" << compound.finalPosition.x()
      << ", Ratio=" << xRatio;
    EXPECT_LT(xRatio, 3.0)
      << "DIAGNOSTIC: Compound X diverged from pure pitch. "
      << "Pure=" << purePitch.finalPosition.x()
      << ", Compound=" << compound.finalPosition.x()
      << ", Ratio=" << xRatio;
  }
}

TEST(TiltedCubeTrajectory, Sliding_CompoundDrop_SettledPhaseRotation)
{
  // Ticket: 0055a_tilted_cube_trajectory_test_suite
  //
  // Drop a cube with compound tilt (PI/6, PI/6), no initial velocity.
  // After settling:
  //   1. Rotation direction matches sliding direction
  //   2. Y displacement is proportional to initial Y-component of tilt
  //      (comparable to X displacement for symmetric PI/6, PI/6 tilt)

  SlidingResult result = runSlidingCubeSimulation(
    M_PI / 6.0,  // roll = PI/6
    M_PI / 6.0,  // pitch = PI/6
    0.0,         // no initial velocity — pure drop
    0.5, 0.5,   // friction, restitution (GUI defaults)
    2000,        // 20 seconds
    10);         // 10ms timestep

  ASSERT_FALSE(result.nanDetected);

  // Settled-phase rotation direction
  int settledReversed = 0;
  int settledSliding = 0;
  for (auto const& s : result.snapshots)
  {
    if (s.frame < 500)
      continue;
    if (std::abs(s.vx) > 0.05)
    {
      settledSliding++;
      if (s.vx * s.omegaY < 0.0)
        settledReversed++;
    }
  }
  if (settledSliding > 3)
  {
    double frac = static_cast<double>(settledReversed) /
                  static_cast<double>(settledSliding);
    EXPECT_LT(frac, 0.3)
      << "DIAGNOSTIC: Settled phase shows reversed rotation. "
      << settledReversed << "/" << settledSliding
      << " sliding frames have vx*ωy<0 (backward rolling).";
  }

  // Print trajectory with Y data
  if (!result.snapshots.empty())
  {
    std::string diag = "Drop trajectory (every 10 frames):\n";
    for (auto const& s : result.snapshots)
    {
      diag += "  frame " + std::to_string(s.frame) +
              ": vx=" + std::to_string(s.vx) +
              " vy=" + std::to_string(s.vy) +
              " wy=" + std::to_string(s.omegaY) +
              " x=" + std::to_string(s.posX) +
              " y=" + std::to_string(s.posY) +
              " z=" + std::to_string(s.posZ) +
              (s.vx * s.omegaY < 0.0 ? " REV-ROT" : "") + "\n";
    }
    SCOPED_TRACE(diag);
  }
}

// ============================================================================
// Phase 2a Diagnostic: EPA Contact Normal Instrumentation
//
// Ticket: 0055b_friction_direction_root_cause
//
// This test instruments the collision pipeline to capture EPA contact normals
// and tangent basis vectors frame-by-frame during the failing scenario
// (compound tilt with initial velocity). By comparing the pure pitch baseline
// (which works correctly) against the compound tilt (which fails), we can
// isolate whether the EPA normal deviates from (0,0,1) and whether the
// tangent basis rotates out of the floor plane.
// ============================================================================

namespace
{

/// Per-frame collision diagnostic data
struct CollisionDiagnostic
{
  int frame;
  bool hasCollision;

  // EPA contact normal (world space)
  double normalX{0.0};
  double normalY{0.0};
  double normalZ{0.0};

  // Lateral magnitude of normal (should be ~0 for floor contact)
  double normalLateral{0.0};

  // Tangent basis vectors
  double t1x{0.0};
  double t1y{0.0};
  double t1z{0.0};
  double t2x{0.0};
  double t2y{0.0};
  double t2z{0.0};

  // Tangent Z-components (should be ~0 for floor-aligned tangents)
  double t1zAbs{0.0};
  double t2zAbs{0.0};

  // Contact point info
  double penetrationDepth{0.0};
  size_t contactCount{0};

  // First contact point (world space)
  double cpAx{0.0};
  double cpAy{0.0};
  double cpAz{0.0};
};

/// Run an instrumented sliding simulation that captures EPA normals each frame.
/// Returns a vector of per-frame collision diagnostics.
std::vector<CollisionDiagnostic> runInstrumentedSimulation(
  double tiltX,
  double tiltY,
  double initialVx,
  double frictionCoeff = 0.5,
  double restitution = 0.5,
  int frames = 200,
  int frameDtMs = 10)
{
  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  const auto& floorAsset =
    world.spawnEnvironmentObject(1, floorHull, floorFrame);
  const_cast<AssetEnvironment&>(floorAsset).setFrictionCoefficient(
    frictionCoeff);

  // Tilted cube
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  Eigen::Quaterniond tiltQuat =
    Eigen::Quaterniond{Eigen::AngleAxisd{tiltY, Eigen::Vector3d::UnitY()}} *
    Eigen::Quaterniond{Eigen::AngleAxisd{tiltX, Eigen::Vector3d::UnitX()}};

  double minZ = std::numeric_limits<double>::max();
  for (const auto& point : cubePoints)
  {
    Eigen::Vector3d rotated =
      tiltQuat * Eigen::Vector3d{point.x(), point.y(), point.z()};
    minZ = std::min(minZ, rotated.z());
  }
  double centerHeightZ = 0.01 - minZ;

  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerHeightZ}, tiltQuat};
  world.spawnObject(2, cubeHull, 10.0, cubeFrame);

  uint32_t const cubeInstanceId = 1;
  world.getObject(cubeInstanceId).setCoefficientOfRestitution(restitution);
  world.getObject(cubeInstanceId).setFrictionCoefficient(frictionCoeff);

  // Set initial velocity
  world.getObject(cubeInstanceId).getInertialState().velocity =
    Vector3D{initialVx, 0.0, 0.0};

  // Collision handler for manual probing
  CollisionHandler collisionHandler{1e-6};

  std::vector<CollisionDiagnostic> diagnostics;

  for (int i = 1; i <= frames; ++i)
  {
    world.update(std::chrono::milliseconds{i * frameDtMs});

    // Probe collision between cube and floor AFTER physics update
    // to see what the collision system would compute for this frame's state
    const auto& cube = world.getObject(cubeInstanceId);
    const auto& envObjects = world.getEnvironmentalObjects();
    if (envObjects.empty())
      continue;
    const auto& floor = envObjects[0];

    auto result = collisionHandler.checkCollision(cube, floor);

    CollisionDiagnostic diag;
    diag.frame = i;
    diag.hasCollision = result.has_value();

    if (result)
    {
      const auto& n = result->normal;
      diag.normalX = n.x();
      diag.normalY = n.y();
      diag.normalZ = n.z();
      diag.normalLateral = std::sqrt(n.x() * n.x() + n.y() * n.y());
      diag.penetrationDepth = result->penetrationDepth;
      diag.contactCount = result->contactCount;

      if (result->contactCount > 0)
      {
        diag.cpAx = result->contacts[0].pointA.x();
        diag.cpAy = result->contacts[0].pointA.y();
        diag.cpAz = result->contacts[0].pointA.z();
      }

      // Compute tangent basis from this normal
      try
      {
        auto tangentFrame = tangent_basis::computeTangentBasis(n);
        diag.t1x = tangentFrame.t1.x();
        diag.t1y = tangentFrame.t1.y();
        diag.t1z = tangentFrame.t1.z();
        diag.t2x = tangentFrame.t2.x();
        diag.t2y = tangentFrame.t2.y();
        diag.t2z = tangentFrame.t2.z();
        diag.t1zAbs = std::abs(tangentFrame.t1.z());
        diag.t2zAbs = std::abs(tangentFrame.t2.z());
      }
      catch (...)
      {
        // Normal not unit length — diagnostic will show zero tangents
      }
    }

    diagnostics.push_back(diag);
  }

  return diagnostics;
}

}  // namespace

TEST(TiltedCubeTrajectory, Diag_EPANormal_PurePitch_vs_Compound)
{
  // Ticket: 0055b_friction_direction_root_cause
  //
  // Phase 2a: EPA Contact Normal Accuracy
  //
  // Run instrumented simulations for:
  //   A) Pure pitch (0, PI/3) — known GOOD baseline
  //   B) Compound tilt (0.01, PI/3) — known FAILING case
  //
  // For each, capture EPA contact normals at every frame and check:
  //   1. Normal is close to (0, 0, 1) — floor contact normal
  //   2. Lateral normal component is small
  //   3. Tangent vectors lie in the XY plane (small Z component)
  //
  // If the normal differs significantly between A and B, the root cause
  // is in the EPA/SAT collision detection. If normals are identical but
  // behavior differs, the root cause is downstream (constraint solver,
  // force application, etc.)

  auto diagPure = runInstrumentedSimulation(
    0.0, M_PI / 3.0, 5.0, 0.5, 0.5, 200, 10);
  auto diagCompound = runInstrumentedSimulation(
    0.01, M_PI / 3.0, 5.0, 0.5, 0.5, 200, 10);

  // Collect contact frames for analysis
  int pureContactFrames = 0;
  int compoundContactFrames = 0;
  double pureMaxLateral = 0.0;
  double compoundMaxLateral = 0.0;
  double pureMaxT1z = 0.0;
  double compoundMaxT1z = 0.0;
  double pureMaxT2z = 0.0;
  double compoundMaxT2z = 0.0;

  // Track worst normals
  CollisionDiagnostic pureWorstNormal{};
  CollisionDiagnostic compoundWorstNormal{};

  for (const auto& d : diagPure)
  {
    if (!d.hasCollision)
      continue;
    pureContactFrames++;
    if (d.normalLateral > pureMaxLateral)
    {
      pureMaxLateral = d.normalLateral;
      pureWorstNormal = d;
    }
    pureMaxT1z = std::max(pureMaxT1z, d.t1zAbs);
    pureMaxT2z = std::max(pureMaxT2z, d.t2zAbs);
  }

  for (const auto& d : diagCompound)
  {
    if (!d.hasCollision)
      continue;
    compoundContactFrames++;
    if (d.normalLateral > compoundMaxLateral)
    {
      compoundMaxLateral = d.normalLateral;
      compoundWorstNormal = d;
    }
    compoundMaxT1z = std::max(compoundMaxT1z, d.t1zAbs);
    compoundMaxT2z = std::max(compoundMaxT2z, d.t2zAbs);
  }

  // Print diagnostic summary (always printed, not just on failure)
  std::cerr << "\n=== EPA Normal Diagnostic Summary ===\n";
  std::cerr << "Pure pitch (0, PI/3): "
            << pureContactFrames << " contact frames\n";
  std::cerr << "  Max lateral normal: " << pureMaxLateral << "\n";
  std::cerr << "  Max |t1.z|: " << pureMaxT1z << "\n";
  std::cerr << "  Max |t2.z|: " << pureMaxT2z << "\n";
  std::cerr << "  Worst normal: ("
            << pureWorstNormal.normalX << ", "
            << pureWorstNormal.normalY << ", "
            << pureWorstNormal.normalZ << ") at frame "
            << pureWorstNormal.frame << "\n";
  std::cerr << "Compound (0.01, PI/3): "
            << compoundContactFrames << " contact frames\n";
  std::cerr << "  Max lateral normal: " << compoundMaxLateral << "\n";
  std::cerr << "  Max |t1.z|: " << compoundMaxT1z << "\n";
  std::cerr << "  Max |t2.z|: " << compoundMaxT2z << "\n";
  std::cerr << "  Worst normal: ("
            << compoundWorstNormal.normalX << ", "
            << compoundWorstNormal.normalY << ", "
            << compoundWorstNormal.normalZ << ") at frame "
            << compoundWorstNormal.frame << "\n";

  // Print first 20 contact frames from each for detailed comparison
  std::cerr << "Pure pitch contact frames (first 20):\n";
  int count = 0;
  for (const auto& d : diagPure)
  {
    if (!d.hasCollision)
      continue;
    if (count++ >= 20)
      break;
    std::cerr << "  f" << d.frame
              << " n=(" << d.normalX << "," << d.normalY << "," << d.normalZ
              << ") lat=" << d.normalLateral
              << " depth=" << d.penetrationDepth
              << " pts=" << d.contactCount
              << " cpA=(" << d.cpAx << "," << d.cpAy << "," << d.cpAz
              << ")\n";
  }

  std::cerr << "Compound contact frames (first 20):\n";
  count = 0;
  for (const auto& d : diagCompound)
  {
    if (!d.hasCollision)
      continue;
    if (count++ >= 20)
      break;
    std::cerr << "  f" << d.frame
              << " n=(" << d.normalX << "," << d.normalY << "," << d.normalZ
              << ") lat=" << d.normalLateral
              << " depth=" << d.penetrationDepth
              << " pts=" << d.contactCount
              << " t1=(" << d.t1x << "," << d.t1y << "," << d.t1z
              << ") t2=(" << d.t2x << "," << d.t2y << "," << d.t2z
              << ") cpA=(" << d.cpAx << "," << d.cpAy << "," << d.cpAz
              << ")\n";
  }

  // ---- Assertions ----

  // Both scenarios must have contacts
  ASSERT_GT(pureContactFrames, 0)
    << "Pure pitch had no collisions in 200 frames";
  ASSERT_GT(compoundContactFrames, 0)
    << "Compound had no collisions in 200 frames";

  // For a cube on a flat floor, the contact normal should be approximately
  // (0, 0, 1). Any lateral component means the EPA picked a wrong face.
  // Tolerance: 0.1 allows normals up to ~5.7° from vertical.
  EXPECT_LT(pureMaxLateral, 0.1)
    << "DIAGNOSTIC: Pure pitch EPA normal has large lateral component. "
    << "Max lateral=" << pureMaxLateral
    << " at frame " << pureWorstNormal.frame
    << " normal=(" << pureWorstNormal.normalX
    << ", " << pureWorstNormal.normalY
    << ", " << pureWorstNormal.normalZ << ")";

  EXPECT_LT(compoundMaxLateral, 0.1)
    << "DIAGNOSTIC: Compound EPA normal has large lateral component. "
    << "Max lateral=" << compoundMaxLateral
    << " at frame " << compoundWorstNormal.frame
    << " normal=(" << compoundWorstNormal.normalX
    << ", " << compoundWorstNormal.normalY
    << ", " << compoundWorstNormal.normalZ << ")";

  // Tangent basis Z components should be small for floor-aligned tangents.
  // If the normal is (0,0,1), tangents should have z=0.
  EXPECT_LT(pureMaxT1z, 0.1)
    << "DIAGNOSTIC: Pure pitch t1 has large Z component="
    << pureMaxT1z;
  EXPECT_LT(compoundMaxT1z, 0.1)
    << "DIAGNOSTIC: Compound t1 has large Z component="
    << compoundMaxT1z;
  EXPECT_LT(pureMaxT2z, 0.1)
    << "DIAGNOSTIC: Pure pitch t2 has large Z component="
    << pureMaxT2z;
  EXPECT_LT(compoundMaxT2z, 0.1)
    << "DIAGNOSTIC: Compound t2 has large Z component="
    << compoundMaxT2z;

  // Key comparison: if compound lateral is much larger than pure pitch,
  // the EPA is producing different normals for the tilted case.
  if (pureMaxLateral > 1e-6)
  {
    double lateralRatio = compoundMaxLateral / pureMaxLateral;
    EXPECT_LT(lateralRatio, 3.0)
      << "DIAGNOSTIC: Compound normal lateral is " << lateralRatio
      << "x worse than pure pitch. EPA behaves differently for compound tilt.";
  }
}

TEST(TiltedCubeTrajectory, Diag_ContactCount_And_LeverArm)
{
  // Ticket: 0055b_friction_direction_root_cause
  //
  // Phase 2b: Contact Count and Lever Arm Analysis
  //
  // H1 (EPA normal perturbation) is RULED OUT — normals are perfect.
  // New hypothesis (H2): Single-contact-point corner contact creates
  // asymmetric yaw torque via off-center lever arm, which amplifies
  // the tiny 0.01 rad perturbation into large spurious motion.
  //
  // This test compares:
  //   A) Pure pitch (0, PI/3) — 2 contact points (edge)
  //   B) Compound (0.01, PI/3) — 1 contact point (corner)
  //
  // For each, track contact count, contact Y-offset from CoM, yaw rate,
  // and lateral velocity to see if single-point contact creates yaw coupling.

  // Run longer (500 frames = 5 seconds) to see full development
  auto diagPure = runInstrumentedSimulation(
    0.0, M_PI / 3.0, 5.0, 0.5, 0.5, 500, 10);
  auto diagCompound = runInstrumentedSimulation(
    0.01, M_PI / 3.0, 5.0, 0.5, 0.5, 500, 10);

  // Count contact point distribution
  int pure1pt = 0;
  int pure2pt = 0;
  int pure3pt = 0;
  int pure4pt = 0;
  int compound1pt = 0;
  int compound2pt = 0;
  int compound3pt = 0;
  int compound4pt = 0;

  for (const auto& d : diagPure)
  {
    if (!d.hasCollision)
      continue;
    if (d.contactCount == 1)
      pure1pt++;
    else if (d.contactCount == 2)
      pure2pt++;
    else if (d.contactCount == 3)
      pure3pt++;
    else if (d.contactCount == 4)
      pure4pt++;
  }

  for (const auto& d : diagCompound)
  {
    if (!d.hasCollision)
      continue;
    if (d.contactCount == 1)
      compound1pt++;
    else if (d.contactCount == 2)
      compound2pt++;
    else if (d.contactCount == 3)
      compound3pt++;
    else if (d.contactCount == 4)
      compound4pt++;
  }

  std::cerr << "\n=== Contact Count Distribution ===\n";
  std::cerr << "Pure pitch:  1pt=" << pure1pt << " 2pt=" << pure2pt
            << " 3pt=" << pure3pt << " 4pt=" << pure4pt << "\n";
  std::cerr << "Compound:    1pt=" << compound1pt << " 2pt=" << compound2pt
            << " 3pt=" << compound3pt << " 4pt=" << compound4pt << "\n";

  // Print contact point Y-coordinates for compound (shows lever arm)
  std::cerr << "\nCompound contact point Y-coordinates (first 30 contacts):\n";
  int count = 0;
  for (const auto& d : diagCompound)
  {
    if (!d.hasCollision)
      continue;
    if (count++ >= 30)
      break;
    std::cerr << "  f" << d.frame
              << " pts=" << d.contactCount
              << " cpA_Y=" << d.cpAy
              << " depth=" << d.penetrationDepth << "\n";
  }

  // KEY ASSERTION: If compound has significantly more 1-point contacts
  // than pure pitch, this supports the lever arm hypothesis.
  int pureTotal = pure1pt + pure2pt + pure3pt + pure4pt;
  int compoundTotal = compound1pt + compound2pt + compound3pt + compound4pt;

  double pure1ptFrac = pureTotal > 0
                         ? static_cast<double>(pure1pt) / pureTotal
                         : 0.0;
  double compound1ptFrac = compoundTotal > 0
                             ? static_cast<double>(compound1pt) / compoundTotal
                             : 0.0;

  std::cerr << "\n1-point contact fraction:\n"
            << "  Pure pitch: " << (pure1ptFrac * 100.0) << "%\n"
            << "  Compound:   " << (compound1ptFrac * 100.0) << "%\n";

  // If compound has more than 2x the 1-point contact fraction, flag it.
  // This would mean the compound tilt produces systematically worse
  // contact manifolds.
  if (pure1ptFrac > 0.01)
  {
    EXPECT_LT(compound1ptFrac / pure1ptFrac, 5.0)
      << "DIAGNOSTIC: Compound has " << (compound1ptFrac / pure1ptFrac)
      << "x more 1-point contacts than pure pitch. "
      << "This asymmetric contact manifold may cause yaw coupling.";
  }
}

TEST(TiltedCubeTrajectory, Diag_EnergyBalance_CompoundVsPurePitch)
{
  // Ticket: 0055b_friction_direction_root_cause
  //
  // Phase 2c: Energy Balance Analysis
  //
  // Track total kinetic energy (translational + rotational) each frame for
  // both pure pitch and compound tilt. If the compound case shows growing
  // KE after settling, something is injecting energy.
  //
  // Also run with friction=0 to isolate friction's contribution.

  constexpr int kFrames = 1000;
  constexpr int kFrameDtMs = 10;

  auto runEnergyTrack = [](double tiltX, double tiltY, double initialVx,
                           double friction, double restitution)
  {
    WorldModel world;

    auto floorPoints = createCubePoints(100.0);
    ConvexHull floorHull{floorPoints};
    ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
    const auto& floorAsset =
      world.spawnEnvironmentObject(1, floorHull, floorFrame);
    const_cast<AssetEnvironment&>(floorAsset).setFrictionCoefficient(friction);

    auto cubePoints = createCubePoints(1.0);
    ConvexHull cubeHull{cubePoints};

    Eigen::Quaterniond tiltQuat =
      Eigen::Quaterniond{Eigen::AngleAxisd{tiltY, Eigen::Vector3d::UnitY()}} *
      Eigen::Quaterniond{
        Eigen::AngleAxisd{tiltX, Eigen::Vector3d::UnitX()}};

    double minZ = std::numeric_limits<double>::max();
    for (const auto& point : cubePoints)
    {
      Eigen::Vector3d rotated =
        tiltQuat * Eigen::Vector3d{point.x(), point.y(), point.z()};
      minZ = std::min(minZ, rotated.z());
    }
    double centerHeightZ = 0.01 - minZ;

    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerHeightZ}, tiltQuat};
    world.spawnObject(2, cubeHull, 10.0, cubeFrame);

    uint32_t const cubeId = 1;
    world.getObject(cubeId).setCoefficientOfRestitution(restitution);
    world.getObject(cubeId).setFrictionCoefficient(friction);
    world.getObject(cubeId).getInertialState().velocity =
      Vector3D{initialVx, 0.0, 0.0};

    struct FrameEnergy
    {
      int frame;
      double keTranslational;
      double keRotational;
      double keTotal;
      double posX;
      double posY;
      double posZ;
      double vx;
      double vy;
    };
    std::vector<FrameEnergy> energyLog;

    for (int i = 1; i <= kFrames; ++i)
    {
      world.update(std::chrono::milliseconds{i * kFrameDtMs});

      auto const& state = world.getObject(cubeId).getInertialState();
      double mass = world.getObject(cubeId).getMass();
      auto I = world.getObject(cubeId).getInertiaTensor();
      auto omega = state.getAngularVelocity();
      Eigen::Vector3d w{omega.x(), omega.y(), omega.z()};

      double keT = 0.5 * mass * state.velocity.squaredNorm();
      double keR = 0.5 * w.dot(I * w);
      double keTotal = keT + keR;

      if (i % 10 == 0)
      {
        energyLog.push_back(
          {i, keT, keR, keTotal, state.position.x(), state.position.y(),
           state.position.z(), state.velocity.x(), state.velocity.y()});
      }
    }

    return energyLog;
  };

  auto pureFriction = runEnergyTrack(0.0, M_PI / 3.0, 5.0, 0.5, 0.5);
  auto compoundFriction = runEnergyTrack(0.01, M_PI / 3.0, 5.0, 0.5, 0.5);
  auto compoundNoFriction =
    runEnergyTrack(0.01, M_PI / 3.0, 5.0, 0.0, 0.5);

  // Print energy evolution for each case
  std::cerr << "\n=== Energy Balance (every 10 frames) ===\n";
  std::cerr << "Pure pitch (friction=0.5):\n";
  for (const auto& e : pureFriction)
  {
    if (e.frame % 50 == 0)
    {
      std::cerr << "  f" << e.frame
                << " KE_T=" << e.keTranslational
                << " KE_R=" << e.keRotational
                << " KE=" << e.keTotal
                << " x=" << e.posX << " y=" << e.posY
                << " vx=" << e.vx << " vy=" << e.vy << "\n";
    }
  }

  std::cerr << "\nCompound (friction=0.5):\n";
  for (const auto& e : compoundFriction)
  {
    if (e.frame % 50 == 0)
    {
      std::cerr << "  f" << e.frame
                << " KE_T=" << e.keTranslational
                << " KE_R=" << e.keRotational
                << " KE=" << e.keTotal
                << " x=" << e.posX << " y=" << e.posY
                << " vx=" << e.vx << " vy=" << e.vy << "\n";
    }
  }

  std::cerr << "\nCompound (friction=0.0):\n";
  for (const auto& e : compoundNoFriction)
  {
    if (e.frame % 50 == 0)
    {
      std::cerr << "  f" << e.frame
                << " KE_T=" << e.keTranslational
                << " KE_R=" << e.keRotational
                << " KE=" << e.keTotal
                << " x=" << e.posX << " y=" << e.posY
                << " vx=" << e.vx << " vy=" << e.vy << "\n";
    }
  }

  // Initial KE = 0.5 * 10 * 5^2 = 125 J
  // With friction, KE should DECREASE over time (dissipation)
  // If KE exceeds initial value, energy is being injected

  double initialKE = 0.5 * 10.0 * 5.0 * 5.0;  // 125 J

  // Check if compound case's KE ever exceeds initial + 10%
  double compoundPeakKE = 0.0;
  int compoundPeakFrame = 0;
  for (const auto& e : compoundFriction)
  {
    if (e.keTotal > compoundPeakKE)
    {
      compoundPeakKE = e.keTotal;
      compoundPeakFrame = e.frame;
    }
  }

  double purePeakKE = 0.0;
  for (const auto& e : pureFriction)
  {
    purePeakKE = std::max(purePeakKE, e.keTotal);
  }

  std::cerr << "\nPeak KE: pure=" << purePeakKE
            << " compound=" << compoundPeakKE
            << " (at frame " << compoundPeakFrame << ")"
            << " initial=" << initialKE << "\n";

  EXPECT_LT(compoundPeakKE, initialKE * 2.0)
    << "DIAGNOSTIC: Compound case peak KE=" << compoundPeakKE
    << " exceeds 2x initial KE=" << initialKE
    << ". Energy is being injected at frame " << compoundPeakFrame;

  // Check if compound no-friction case has the same issue
  double noFrictionPeakKE = 0.0;
  for (const auto& e : compoundNoFriction)
  {
    noFrictionPeakKE = std::max(noFrictionPeakKE, e.keTotal);
  }

  std::cerr << "No-friction peak KE: " << noFrictionPeakKE << "\n";

  // If the no-friction case also shows energy growth, the issue is NOT
  // friction-specific — it's in the normal constraint or position corrector.
  // If only the friction case diverges, friction is the root cause.
  if (noFrictionPeakKE < initialKE * 1.5 && compoundPeakKE > initialKE * 2.0)
  {
    std::cerr << "DIAGNOSIS: Energy injection requires friction. "
              << "Root cause is in friction constraint, not normal/position.\n";
  }
  else if (noFrictionPeakKE > initialKE * 2.0)
  {
    std::cerr << "DIAGNOSIS: Energy injection occurs WITHOUT friction. "
              << "Root cause is in normal constraint or position corrector.\n";
  }
}

TEST(TiltedCubeTrajectory, Diag_EPANormal_NormalTimeSeries)
{
  // Ticket: 0055b_friction_direction_root_cause
  //
  // Capture the EPA normal time series for the compound tilt scenario to
  // see if the normal is stable or oscillating. An oscillating normal would
  // indicate EPA is selecting different Minkowski faces frame-to-frame,
  // which would cause the tangent basis to flip and friction to push in
  // inconsistent directions.

  auto diag = runInstrumentedSimulation(
    0.01, M_PI / 3.0, 5.0, 0.5, 0.5, 500, 10);

  // Check for normal direction flips (sign changes in nx or ny)
  int normalFlips = 0;
  double prevNx = 0.0;
  double prevNy = 0.0;
  bool prevHadCollision = false;

  std::string timeSeries = "Normal time series (compound, contact frames):\n";
  int contactCount = 0;

  for (const auto& d : diag)
  {
    if (!d.hasCollision)
    {
      prevHadCollision = false;
      continue;
    }

    contactCount++;

    // Log every 5th contact frame to keep output manageable
    if (contactCount % 5 == 1 || contactCount <= 10)
    {
      timeSeries += "  f" + std::to_string(d.frame) +
                    " n=(" + std::to_string(d.normalX) +
                    "," + std::to_string(d.normalY) +
                    "," + std::to_string(d.normalZ) +
                    ") lat=" + std::to_string(d.normalLateral) +
                    " depth=" + std::to_string(d.penetrationDepth) + "\n";
    }

    // Detect lateral normal flips (sign changes)
    if (prevHadCollision)
    {
      // Check if nx changed sign significantly
      if (std::abs(d.normalX) > 0.01 && std::abs(prevNx) > 0.01 &&
          d.normalX * prevNx < 0.0)
      {
        normalFlips++;
      }
      // Check if ny changed sign significantly
      if (std::abs(d.normalY) > 0.01 && std::abs(prevNy) > 0.01 &&
          d.normalY * prevNy < 0.0)
      {
        normalFlips++;
      }
    }

    prevNx = d.normalX;
    prevNy = d.normalY;
    prevHadCollision = true;
  }

  std::cerr << timeSeries;

  // If there are many normal flips, EPA is unstable for this configuration
  EXPECT_LT(normalFlips, 5)
    << "DIAGNOSTIC: EPA normal flipped " << normalFlips
    << " times across " << contactCount
    << " contact frames. This instability would cause friction to push "
    << "in inconsistent directions.";
}

TEST(TiltedCubeTrajectory, Diag_YawCoupling_SinglePointFriction)
{
  // Ticket: 0055b_friction_direction_root_cause
  //
  // HYPOTHESIS: Single-point contact with friction creates uncompensated
  // yaw torque (rotation about Z-axis). With multi-point contacts, yaw
  // torques from friction at opposite corners cancel. With one point, the
  // yaw torque is unopposed and drives the instability.
  //
  // This test tracks omega_z (yaw rate) over time for:
  //   1. Pure pitch (multi-point contacts) - should have ~0 yaw
  //   2. Compound tilt with friction - expect yaw growth
  //   3. Compound tilt without friction - should have ~0 yaw

  auto runYawTrack = [](double tiltX, double tiltY, double initialVx,
                        double friction, double restitution) {
    WorldModel world;

    auto floorPoints = createCubePoints(100.0);
    ConvexHull floorHull{floorPoints};
    ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
    const auto& floorAsset =
      world.spawnEnvironmentObject(1, floorHull, floorFrame);
    const_cast<AssetEnvironment&>(floorAsset).setFrictionCoefficient(friction);

    auto cubePoints = createCubePoints(1.0);
    ConvexHull cubeHull{cubePoints};

    Eigen::Quaterniond tiltQuat =
      Eigen::Quaterniond{Eigen::AngleAxisd{tiltY, Eigen::Vector3d::UnitY()}} *
      Eigen::Quaterniond{Eigen::AngleAxisd{tiltX, Eigen::Vector3d::UnitX()}};

    double minZ = std::numeric_limits<double>::max();
    for (const auto& point : cubePoints)
    {
      Eigen::Vector3d rotated =
        tiltQuat * Eigen::Vector3d{point.x(), point.y(), point.z()};
      minZ = std::min(minZ, rotated.z());
    }
    double centerHeightZ = 0.01 - minZ;

    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerHeightZ}, tiltQuat};
    world.spawnObject(2, cubeHull, 10.0, cubeFrame);

    uint32_t const cubeId = 1;
    world.getObject(cubeId).setCoefficientOfRestitution(restitution);
    world.getObject(cubeId).setFrictionCoefficient(friction);
    world.getObject(cubeId).getInertialState().velocity =
      Vector3D{initialVx, 0.0, 0.0};

    constexpr int kFrames = 500;
    constexpr int kFrameDtMs = 10;

    struct YawData
    {
      int frame;
      double omegaX;
      double omegaY;
      double omegaZ;
      double yawAngle;  // cumulative from omega_z integration
    };
    std::vector<YawData> yawLog;

    double cumulativeYaw = 0.0;
    for (int i = 1; i <= kFrames; ++i)
    {
      world.update(std::chrono::milliseconds{i * kFrameDtMs});

      auto const& state = world.getObject(cubeId).getInertialState();
      auto omega = state.getAngularVelocity();
      cumulativeYaw += omega.z() * (kFrameDtMs / 1000.0);

      if (i % 10 == 0)
      {
        yawLog.push_back(
          {i, omega.x(), omega.y(), omega.z(), cumulativeYaw});
      }
    }
    return yawLog;
  };

  auto purePitch = runYawTrack(0.0, M_PI / 3.0, 5.0, 0.5, 0.5);
  auto compoundFriction = runYawTrack(0.01, M_PI / 3.0, 5.0, 0.5, 0.5);
  auto compoundNoFriction = runYawTrack(0.01, M_PI / 3.0, 5.0, 0.0, 0.5);

  std::cerr << "\n=== Yaw Coupling Analysis ===\n";

  auto printYaw = [](const char* label, const auto& log) {
    std::cerr << label << ":\n";
    double peakOmegaZ = 0.0;
    for (const auto& y : log)
    {
      peakOmegaZ = std::max(peakOmegaZ, std::abs(y.omegaZ));
      if (y.frame <= 200 || y.frame % 100 == 0)
      {
        std::cerr << "  f" << y.frame
                  << " wz=" << y.omegaZ
                  << " yaw=" << y.yawAngle
                  << " wx=" << y.omegaX
                  << " wy=" << y.omegaY << "\n";
      }
    }
    std::cerr << "  Peak |omega_z|: " << peakOmegaZ << " rad/s\n\n";
  };

  printYaw("Pure pitch (friction=0.5)", purePitch);
  printYaw("Compound (friction=0.5)", compoundFriction);
  printYaw("Compound (friction=0.0)", compoundNoFriction);

  // Pure pitch should have negligible yaw
  double purePeakYaw = 0.0;
  for (const auto& y : purePitch)
  {
    purePeakYaw = std::max(purePeakYaw, std::abs(y.omegaZ));
  }

  // Compound with friction should show significant yaw
  double compoundPeakYaw = 0.0;
  for (const auto& y : compoundFriction)
  {
    compoundPeakYaw = std::max(compoundPeakYaw, std::abs(y.omegaZ));
  }

  // Compound without friction should have negligible yaw
  double noFrictionPeakYaw = 0.0;
  for (const auto& y : compoundNoFriction)
  {
    noFrictionPeakYaw = std::max(noFrictionPeakYaw, std::abs(y.omegaZ));
  }

  std::cerr << "Summary: pure yaw=" << purePeakYaw
            << "  compound+friction yaw=" << compoundPeakYaw
            << "  compound-friction yaw=" << noFrictionPeakYaw << "\n";

  if (compoundPeakYaw > 10.0 * purePeakYaw &&
      compoundPeakYaw > 10.0 * noFrictionPeakYaw)
  {
    std::cerr << "CONFIRMED: Single-point friction creates uncompensated yaw "
              << "torque. This is the energy injection mechanism.\n";
  }
}

// ============================================================================
// Diagnostic: Total Mechanical Energy (KE + PE) frame-by-frame
// ============================================================================
//
// Ticket: 0055c_friction_direction_fix
//
// Measures total mechanical energy E = KE_trans + KE_rot + mgh every frame
// for both friction and frictionless cases. Any frame where total E increases
// means a non-conservative force injected energy.
//
// Uses the EXACT scenario from the failing Sliding_ThrownCube_SDLAppConfig
// test: PI/3 pitch, 0.01 roll, 5 m/s initial Vx, friction=0.5, e=0.5,
// dt=10ms. This is the scenario where the user observed "very peculiar,
// reversed rotations and motions" and spurious Y translations.

TEST(TiltedCubeTrajectory, Diag_MechanicalEnergy_FrictionInjection)
{
  constexpr int kFrameDtMs = 10;
  constexpr int kFrames = 500;
  constexpr double kMass = 10.0;
  constexpr double kGravZ = -9.81;
  constexpr double kTiltX = 0.01;       // roll — tiny perturbation
  constexpr double kTiltY = M_PI / 3.0; // pitch — large tilt
  constexpr double kInitialVx = 5.0;    // thrown in +X direction

  // Run two simulations: with and without friction
  for (double friction : {0.5, 0.0})
  {
    WorldModel world;

    auto floorPoints = createCubePoints(100.0);
    ConvexHull floorHull{floorPoints};
    ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
    const auto& floorAsset =
      world.spawnEnvironmentObject(1, floorHull, floorFrame);
    const_cast<AssetEnvironment&>(floorAsset).setFrictionCoefficient(friction);

    auto cubePoints = createCubePoints(1.0);
    ConvexHull cubeHull{cubePoints};

    Eigen::Quaterniond tiltQuat =
      Eigen::Quaterniond{Eigen::AngleAxisd{kTiltY, Eigen::Vector3d::UnitY()}} *
      Eigen::Quaterniond{Eigen::AngleAxisd{kTiltX, Eigen::Vector3d::UnitX()}};

    double minZ = std::numeric_limits<double>::max();
    for (const auto& pt : cubePoints)
    {
      Eigen::Vector3d rotated =
        tiltQuat * Eigen::Vector3d{pt.x(), pt.y(), pt.z()};
      minZ = std::min(minZ, rotated.z());
    }
    double centerZ = 0.01 - minZ;

    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerZ}, tiltQuat};
    world.spawnObject(2, cubeHull, kMass, cubeFrame);
    uint32_t const cubeId = 1;
    world.getObject(cubeId).setCoefficientOfRestitution(0.5);
    world.getObject(cubeId).setFrictionCoefficient(friction);

    // Set initial horizontal velocity — this is the key difference
    world.getObject(cubeId).getInertialState().velocity =
      Vector3D{kInitialVx, 0.0, 0.0};

    // Get the actual inertia tensor for accurate rotational KE
    const Eigen::Matrix3d& inertiaTensor =
      world.getObject(cubeId).getInertiaTensor();

    std::string label = (friction > 0.0) ? "friction=0.5" : "friction=0.0";
    std::cerr << "\n=== Mechanical Energy (" << label
              << ", vx0=" << kInitialVx << ") ===\n";

    double prevE = std::numeric_limits<double>::quiet_NaN();
    double maxEnergyGain = 0.0;
    int maxGainFrame = -1;
    int injectionCount = 0;
    double cumulativeInjection = 0.0;

    for (int i = 1; i <= kFrames; ++i)
    {
      world.update(std::chrono::milliseconds{i * kFrameDtMs});

      auto const& state = world.getObject(cubeId).getInertialState();
      auto const& pos = state.position;
      auto const& vel = state.velocity;
      auto omega = state.getAngularVelocity();

      // Translational KE = ½mv²
      double keT = 0.5 * kMass *
        (vel.x() * vel.x() + vel.y() * vel.y() + vel.z() * vel.z());

      // Rotational KE = ½ω^T·I·ω (using actual inertia tensor)
      Eigen::Vector3d omegaVec{omega.x(), omega.y(), omega.z()};
      double keR = 0.5 * omegaVec.transpose() * inertiaTensor * omegaVec;

      // Gravitational PE = mgh
      double pe = kMass * (-kGravZ) * pos.z();

      double E = keT + keR + pe;

      double deltaE = 0.0;
      if (!std::isnan(prevE))
      {
        deltaE = E - prevE;
        if (deltaE > 1e-6)
        {
          injectionCount++;
          cumulativeInjection += deltaE;
          if (deltaE > maxEnergyGain)
          {
            maxEnergyGain = deltaE;
            maxGainFrame = i;
          }
        }
      }

      // Print every 10th frame, plus any injection frame
      if (i % 10 == 0 || i <= 20 || deltaE > 1e-4)
      {
        std::cerr << "  f" << i
                  << " E=" << E
                  << " keT=" << keT
                  << " keR=" << keR
                  << " pe=" << pe
                  << " dE=" << deltaE
                  << " vy=" << vel.y()
                  << " wz=" << omega.z()
                  << (deltaE > 1e-6 ? " <<< INJECTION" : "")
                  << "\n";
      }

      prevE = E;
    }

    std::cerr << "\n  Summary (" << label << "): "
              << "injectionFrames=" << injectionCount
              << " maxGain=" << maxEnergyGain
              << " at frame " << maxGainFrame
              << " cumulativeInjection=" << cumulativeInjection << "\n";
  }
}
