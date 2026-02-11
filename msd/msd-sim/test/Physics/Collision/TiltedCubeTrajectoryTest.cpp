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
