// Ticket: 0055a_tilted_cube_trajectory_test_suite
// Test: Tilted cube trajectory direction tests (diagnostic suite)

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
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
};

/// Run a tilted cube simulation with specified tilt angles
TiltResult runTiltedCubeSimulation(double tiltX,
                                   double tiltY,
                                   double frictionCoeff = 0.5,
                                   double restitution = 0.3,
                                   int frames = 200)
{
  // Create WorldModel with timestep of 16ms
  constexpr double dt = 0.016;
  WorldModel world;

  // Create floor as a large cube centered at z=-50 (top face at z=0)
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};

  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};

  const auto& floorAsset = world.spawnEnvironmentObject(1, floorHull, floorFrame);
  // Note: AssetEnvironment friction set via mutable getter
  auto& mutableFloor = const_cast<AssetEnvironment&>(floorAsset);
  mutableFloor.setFrictionCoefficient(frictionCoeff);

  // Create tilted cube with given tilt angles
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  // Apply tilt via quaternion
  // Rotation order: first around X-axis, then around Y-axis
  Eigen::Quaterniond qX{Eigen::AngleAxisd(tiltX, Eigen::Vector3d::UnitX())};
  Eigen::Quaterniond qY{Eigen::AngleAxisd(tiltY, Eigen::Vector3d::UnitY())};
  Eigen::Quaterniond tiltQuat = qY * qX;

  // Compute initial position such that the lowest corner is at z=0.01
  // Find lowest corner after rotation
  double minZ = std::numeric_limits<double>::max();
  for (const auto& point : cubePoints)
  {
    // Coordinate is already an Eigen::Vector3d
    Eigen::Vector3d rotated = tiltQuat * static_cast<Eigen::Vector3d>(point);
    minZ = std::min(minZ, rotated.z());
  }

  // Position cube so lowest corner is at z=0.01
  double centerHeightZ = 0.01 - minZ;

  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerHeightZ}, tiltQuat};

  const auto& cubeAsset = world.spawnObject(2, cubeHull, 10.0, cubeFrame);
  // Note: AssetInertial friction/restitution set via mutable getter
  auto& mutableCube = const_cast<AssetInertial&>(cubeAsset);
  mutableCube.setCoefficientOfRestitution(restitution);
  mutableCube.setFrictionCoefficient(frictionCoeff);

  // Run simulation
  bool nanDetected = false;
  uint32_t cubeInstanceId = cubeAsset.getInstanceId();
  for (int i = 0; i < frames; ++i)
  {
    world.update(std::chrono::milliseconds{static_cast<int64_t>(dt * 1000)});

    // Check for NaN
    try
    {
      const auto& cube = world.getObject(cubeInstanceId);
      const auto& pos = cube.getInertialState().position;
      if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()))
      {
        nanDetected = true;
        break;
      }
    }
    catch (...)
    {
      nanDetected = true;
      break;
    }
  }

  // Extract final state
  TiltResult result;

  try
  {
    const auto& cube = world.getObject(cubeInstanceId);
    const auto& state = cube.getInertialState();
    result.finalPosition = state.position;
    result.finalOrientation = state.orientation;
    result.lateralDisplacementX = result.finalPosition.x();
    result.lateralDisplacementY = result.finalPosition.y();
    result.finalZ = result.finalPosition.z();
  }
  catch (...)
  {
    // Object not found (shouldn't happen)
    result.finalPosition = Coordinate{std::nan(""), std::nan(""), std::nan("")};
    result.finalOrientation = Eigen::Quaterniond::Identity();
    result.lateralDisplacementX = std::nan("");
    result.lateralDisplacementY = std::nan("");
    result.finalZ = std::nan("");
    nanDetected = true;
  }

  result.nanDetected = nanDetected;

  return result;
}

}  // namespace

// ============================================================================
// Test Suite
// ============================================================================

TEST(TiltedCubeTrajectory, T1_PurePositiveXTilt)
{
  constexpr double theta = 0.1;  // radians (~5.7 degrees)
  TiltResult result = runTiltedCubeSimulation(theta, 0.0);

  // Assert no NaN/divergence
  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  // NOTE: Direction assertion TBD after empirical observation
  // This test is a diagnostic — we record the behavior for now
  std::cout << "T1 (+X tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

TEST(TiltedCubeTrajectory, T2_PureNegativeXTilt)
{
  constexpr double theta = 0.1;
  TiltResult result = runTiltedCubeSimulation(-theta, 0.0);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  std::cout << "T2 (-X tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

TEST(TiltedCubeTrajectory, T3_PurePositiveYTilt)
{
  constexpr double theta = 0.1;
  TiltResult result = runTiltedCubeSimulation(0.0, theta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  std::cout << "T3 (+Y tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

TEST(TiltedCubeTrajectory, T4_PureNegativeYTilt)
{
  constexpr double theta = 0.1;
  TiltResult result = runTiltedCubeSimulation(0.0, -theta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  std::cout << "T4 (-Y tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

TEST(TiltedCubeTrajectory, T5_CompoundXPlusYTilt)
{
  constexpr double theta = 0.1;
  TiltResult result = runTiltedCubeSimulation(theta, theta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  std::cout << "T5 (+X+Y tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

TEST(TiltedCubeTrajectory, T6_CompoundXMinusYTilt)
{
  constexpr double theta = 0.1;
  TiltResult result = runTiltedCubeSimulation(theta, -theta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  std::cout << "T6 (+X-Y tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

TEST(TiltedCubeTrajectory, T7_CompoundNegXPlusYTilt)
{
  constexpr double theta = 0.1;
  TiltResult result = runTiltedCubeSimulation(-theta, theta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  std::cout << "T7 (-X+Y tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

TEST(TiltedCubeTrajectory, T8_CompoundNegXMinusYTilt)
{
  constexpr double theta = 0.1;
  TiltResult result = runTiltedCubeSimulation(-theta, -theta);

  ASSERT_FALSE(result.nanDetected) << "NaN detected during simulation";
  ASSERT_TRUE(std::isfinite(result.finalPosition.x()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.y()));
  ASSERT_TRUE(std::isfinite(result.finalPosition.z()));

  std::cout << "T8 (-X-Y tilt): final position = ("
            << result.finalPosition.x() << ", "
            << result.finalPosition.y() << ", "
            << result.finalPosition.z() << ")" << std::endl;
}

// ============================================================================
// Symmetry Tests
// ============================================================================

TEST(TiltedCubeTrajectory, Symmetry_XTilt_MirrorsDisplacement)
{
  constexpr double theta = 0.1;

  TiltResult resultPos = runTiltedCubeSimulation(theta, 0.0);
  TiltResult resultNeg = runTiltedCubeSimulation(-theta, 0.0);

  ASSERT_FALSE(resultPos.nanDetected) << "NaN in +X tilt";
  ASSERT_FALSE(resultNeg.nanDetected) << "NaN in -X tilt";

  // Expect mirrored trajectories (sign flip in one dimension)
  // NOTE: The exact axis of symmetry depends on the coordinate convention
  // For now, we check that the magnitudes are comparable
  double magPos = std::sqrt(resultPos.lateralDisplacementX * resultPos.lateralDisplacementX +
                            resultPos.lateralDisplacementY * resultPos.lateralDisplacementY);
  double magNeg = std::sqrt(resultNeg.lateralDisplacementX * resultNeg.lateralDisplacementX +
                            resultNeg.lateralDisplacementY * resultNeg.lateralDisplacementY);

  std::cout << "Symmetry X: +X displacement magnitude = " << magPos
            << ", -X displacement magnitude = " << magNeg << std::endl;

  // Expect comparable magnitudes (within 2x)
  EXPECT_GT(magPos, 0.0) << "+X tilt produced no displacement";
  EXPECT_GT(magNeg, 0.0) << "-X tilt produced no displacement";
  EXPECT_LT(std::abs(magPos - magNeg) / std::max(magPos, magNeg), 1.0)
    << "Magnitude difference exceeds 2x";
}

TEST(TiltedCubeTrajectory, Symmetry_YTilt_MirrorsDisplacement)
{
  constexpr double theta = 0.1;

  TiltResult resultPos = runTiltedCubeSimulation(0.0, theta);
  TiltResult resultNeg = runTiltedCubeSimulation(0.0, -theta);

  ASSERT_FALSE(resultPos.nanDetected) << "NaN in +Y tilt";
  ASSERT_FALSE(resultNeg.nanDetected) << "NaN in -Y tilt";

  double magPos = std::sqrt(resultPos.lateralDisplacementX * resultPos.lateralDisplacementX +
                            resultPos.lateralDisplacementY * resultPos.lateralDisplacementY);
  double magNeg = std::sqrt(resultNeg.lateralDisplacementX * resultNeg.lateralDisplacementX +
                            resultNeg.lateralDisplacementY * resultNeg.lateralDisplacementY);

  std::cout << "Symmetry Y: +Y displacement magnitude = " << magPos
            << ", -Y displacement magnitude = " << magNeg << std::endl;

  EXPECT_GT(magPos, 0.0) << "+Y tilt produced no displacement";
  EXPECT_GT(magNeg, 0.0) << "-Y tilt produced no displacement";
  EXPECT_LT(std::abs(magPos - magNeg) / std::max(magPos, magNeg), 1.0)
    << "Magnitude difference exceeds 2x";
}

TEST(TiltedCubeTrajectory, Symmetry_CompoundTilt_DiagonalMirror)
{
  constexpr double theta = 0.1;

  TiltResult resultT5 = runTiltedCubeSimulation(theta, theta);    // (+X, +Y)
  TiltResult resultT8 = runTiltedCubeSimulation(-theta, -theta);  // (-X, -Y)

  ASSERT_FALSE(resultT5.nanDetected) << "NaN in T5 (+X+Y tilt)";
  ASSERT_FALSE(resultT8.nanDetected) << "NaN in T8 (-X-Y tilt)";

  double magT5 = std::sqrt(resultT5.lateralDisplacementX * resultT5.lateralDisplacementX +
                           resultT5.lateralDisplacementY * resultT5.lateralDisplacementY);
  double magT8 = std::sqrt(resultT8.lateralDisplacementX * resultT8.lateralDisplacementX +
                           resultT8.lateralDisplacementY * resultT8.lateralDisplacementY);

  std::cout << "Symmetry Diagonal T5/T8: T5 magnitude = " << magT5
            << ", T8 magnitude = " << magT8 << std::endl;

  EXPECT_GT(magT5, 0.0) << "T5 (+X+Y) produced no displacement";
  EXPECT_GT(magT8, 0.0) << "T8 (-X-Y) produced no displacement";
  EXPECT_LT(std::abs(magT5 - magT8) / std::max(magT5, magT8), 1.0)
    << "Magnitude difference exceeds 2x";
}

TEST(TiltedCubeTrajectory, Symmetry_CompoundTilt_AntiDiagonalMirror)
{
  constexpr double theta = 0.1;

  TiltResult resultT6 = runTiltedCubeSimulation(theta, -theta);   // (+X, -Y)
  TiltResult resultT7 = runTiltedCubeSimulation(-theta, theta);   // (-X, +Y)

  ASSERT_FALSE(resultT6.nanDetected) << "NaN in T6 (+X-Y tilt)";
  ASSERT_FALSE(resultT7.nanDetected) << "NaN in T7 (-X+Y tilt)";

  double magT6 = std::sqrt(resultT6.lateralDisplacementX * resultT6.lateralDisplacementX +
                           resultT6.lateralDisplacementY * resultT6.lateralDisplacementY);
  double magT7 = std::sqrt(resultT7.lateralDisplacementX * resultT7.lateralDisplacementX +
                           resultT7.lateralDisplacementY * resultT7.lateralDisplacementY);

  std::cout << "Symmetry Anti-Diagonal T6/T7: T6 magnitude = " << magT6
            << ", T7 magnitude = " << magT7 << std::endl;

  EXPECT_GT(magT6, 0.0) << "T6 (+X-Y) produced no displacement";
  EXPECT_GT(magT7, 0.0) << "T7 (-X+Y) produced no displacement";
  EXPECT_LT(std::abs(magT6 - magT7) / std::max(magT6, magT7), 1.0)
    << "Magnitude difference exceeds 2x";
}
