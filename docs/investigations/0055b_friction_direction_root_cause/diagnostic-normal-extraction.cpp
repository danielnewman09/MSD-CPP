// Diagnostic program to extract EPA contact normals during the failing test
// Ticket: 0055b_friction_direction_root_cause
//
// This is a standalone diagnostic tool, not part of the test suite.
// Compile with: cmake --build --preset conan-debug
//
// Usage: Run the Sliding_ThrownCube_SDLAppConfig test scenario and print EPA normals
// at each frame of contact to understand if lateral normal components are causing
// tangent basis misalignment.

#include <cmath>
#include <iostream>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/TangentBasis.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

/// Create a cube point cloud with the given size (centered at origin)
std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate{-half, -half, -half}, Coordinate{half, -half, -half},
          Coordinate{half, half, -half},   Coordinate{-half, half, -half},
          Coordinate{-half, -half, half},  Coordinate{half, -half, half},
          Coordinate{half, half, half},    Coordinate{-half, half, half}};
}

int main()
{
  // Reproduce Sliding_ThrownCube_SDLAppConfig scenario
  constexpr double kTiltX = M_PI / 3.0;  // 60 degrees about X
  constexpr double kTiltY = 0.01;        // 0.01 radians about Y
  constexpr double kInitialVx = 5.0;     // m/s
  constexpr double kFriction = 0.5;
  constexpr double kRestitution = 0.5;
  constexpr int kFrameDtMs = 10;  // 10ms timestep
  constexpr int kMaxFrames = 50;   // Only first 50 frames to capture initial contact

  WorldModel world;

  // Floor: large cube centered at z=-50 (top face at z=0)
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  const auto& floorAsset =
    world.spawnEnvironmentObject(1, floorHull, floorFrame);
  const_cast<AssetEnvironment&>(floorAsset).setFrictionCoefficient(kFriction);

  // Tilted cube: 1m unit cube, 10 kg
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  // Build tilt quaternion: rotate about X first, then Y
  Eigen::Quaterniond tiltQuat =
    Eigen::Quaterniond{Eigen::AngleAxisd{kTiltY, Eigen::Vector3d::UnitY()}} *
    Eigen::Quaterniond{Eigen::AngleAxisd{kTiltX, Eigen::Vector3d::UnitX()}};

  // Position cube so lowest point is at z=0.01
  double minZ = std::numeric_limits<double>::max();
  for (const auto& point : cubePoints)
  {
    Eigen::Vector3d rotated =
      tiltQuat * Eigen::Vector3d{point.x(), point.y(), point.z()};
    minZ = std::min(minZ, rotated.z());
  }
  double centerHeightZ = 0.01 - minZ;

  // Spawn cube with initial velocity
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, centerHeightZ}, tiltQuat};
  world.spawnObject(2, cubeHull, 10.0, cubeFrame);

  uint32_t const cubeInstanceId = 1;
  world.getObject(cubeInstanceId).setCoefficientOfRestitution(kRestitution);
  world.getObject(cubeInstanceId).setFrictionCoefficient(kFriction);

  // Set initial velocity
  auto& cubeObj = world.getObject(cubeInstanceId);
  InertialState initialState = cubeObj.getInertialState();
  initialState.velocity = Coordinate{kInitialVx, 0.0, 0.0};
  cubeObj.setInertialState(initialState);

  // Create collision handler for manual EPA normal extraction
  CollisionHandler collisionHandler{1e-6};

  std::cout << "=== EPA Normal Diagnostic ===" << std::endl;
  std::cout << "Config: tiltX=" << kTiltX << " rad, tiltY=" << kTiltY
            << " rad, vx=" << kInitialVx << " m/s" << std::endl;
  std::cout << std::endl;

  bool firstContactDetected = false;

  // Run simulation
  for (int frame = 1; frame <= kMaxFrames; ++frame)
  {
    world.update(std::chrono::milliseconds{frame * kFrameDtMs});

    auto const& cubeState = cubeObj.getInertialState();

    // Manually check collision with EPA
    auto const& floorPhysical = floorAsset.getPhysical();
    auto const& cubePhysical = cubeObj.getPhysical();

    auto collisionResult =
      collisionHandler.checkCollision(cubePhysical, floorPhysical);

    if (collisionResult && !firstContactDetected)
    {
      firstContactDetected = true;

      std::cout << "=== FIRST CONTACT (Frame " << frame << ") ===" << std::endl;
      std::cout << "Position: (" << cubeState.position.x() << ", "
                << cubeState.position.y() << ", " << cubeState.position.z()
                << ")" << std::endl;
      std::cout << "Velocity: (" << cubeState.velocity.x() << ", "
                << cubeState.velocity.y() << ", " << cubeState.velocity.z()
                << ")" << std::endl;
      std::cout << std::endl;

      // Extract EPA normal
      const Coordinate& normal = collisionResult->contactNormal;
      std::cout << "EPA Normal: (" << normal.x() << ", " << normal.y() << ", "
                << normal.z() << ")" << std::endl;
      std::cout << "Normal magnitude: " << normal.norm() << std::endl;

      // Expected normal for floor contact: (0, 0, 1) pointing up
      const double lateralX = std::abs(normal.x());
      const double lateralY = std::abs(normal.y());
      const double verticalZ = normal.z();

      std::cout << "Lateral components: |nx|=" << lateralX
                << ", |ny|=" << lateralY << std::endl;
      std::cout << "Vertical component: nz=" << verticalZ << std::endl;
      std::cout << std::endl;

      // Compute tangent basis from EPA normal
      TangentFrame tangentFrame = tangent_basis::computeTangentBasis(normal);

      std::cout << "Tangent t1: (" << tangentFrame.t1.x() << ", "
                << tangentFrame.t1.y() << ", " << tangentFrame.t1.z() << ")"
                << std::endl;
      std::cout << "Tangent t2: (" << tangentFrame.t2.x() << ", "
                << tangentFrame.t2.y() << ", " << tangentFrame.t2.z() << ")"
                << std::endl;

      // Check if tangents are in floor plane (Z component should be ~0)
      const double t1_z = std::abs(tangentFrame.t1.z());
      const double t2_z = std::abs(tangentFrame.t2.z());

      std::cout << "Tangent Z components: |t1_z|=" << t1_z
                << ", |t2_z|=" << t2_z << std::endl;

      if (t1_z > 0.01 || t2_z > 0.01)
      {
        std::cout << "WARNING: Tangent vectors have significant Z component!"
                  << std::endl;
        std::cout << "         Friction will act partially in normal direction."
                  << std::endl;
      }

      std::cout << std::endl;

      // Compute angle between EPA normal and expected normal (0, 0, 1)
      Coordinate expectedNormal{0.0, 0.0, 1.0};
      double dotProduct = normal.dot(expectedNormal);
      double angleDeg = std::acos(dotProduct) * 180.0 / M_PI;

      std::cout << "Angle between EPA normal and vertical: " << angleDeg
                << " degrees" << std::endl;
      std::cout << std::endl;

      std::cout << "Penetration depth: " << collisionResult->penetrationDepth
                << " m" << std::endl;
      std::cout << "Contact count: " << collisionResult->contactCount
                << std::endl;
      std::cout << std::endl;
    }

    // Print every 10 frames if still in contact
    if (collisionResult && frame % 10 == 0)
    {
      std::cout << "Frame " << frame << ": Still in contact, depth="
                << collisionResult->penetrationDepth << " m" << std::endl;
    }

    // Stop after first contact is resolved
    if (firstContactDetected && !collisionResult)
    {
      std::cout << "Contact resolved at frame " << frame << std::endl;
      break;
    }
  }

  if (!firstContactDetected)
  {
    std::cout << "WARNING: No contact detected in first " << kMaxFrames
              << " frames!" << std::endl;
  }

  return 0;
}
