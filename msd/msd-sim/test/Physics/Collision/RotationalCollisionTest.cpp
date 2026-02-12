// Ticket: 0039c_rotational_coupling_test_suite
// Test: Scenario B -- Rotation initiation from off-center impacts
//
// DIAGNOSTIC TEST SUITE: Some tests are EXPECTED to fail because they
// investigate a known energy injection bug in rotational collisions.
// Failures are valid diagnostic findings, not test implementation errors.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate{-half, -half, -half},
          Coordinate{half, -half, -half},
          Coordinate{half, half, -half},
          Coordinate{-half, half, -half},
          Coordinate{-half, -half, half},
          Coordinate{half, -half, half},
          Coordinate{half, half, half},
          Coordinate{-half, half, half}};
}

/// Create an icosphere point cloud with the given radius.
std::vector<Coordinate> createSpherePoints(double radius)
{
  double const phi = (1.0 + std::sqrt(5.0)) / 2.0;

  std::vector<Eigen::Vector3d> vertices = {
    {-1,  phi, 0}, { 1,  phi, 0}, {-1, -phi, 0}, { 1, -phi, 0},
    { 0, -1,  phi}, { 0,  1,  phi}, { 0, -1, -phi}, { 0,  1, -phi},
    { phi, 0, -1}, { phi, 0,  1}, {-phi, 0, -1}, {-phi, 0,  1}
  };

  for (auto& v : vertices)
  {
    v.normalize();
  }

  std::vector<std::array<size_t, 3>> faces = {
    {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
    {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
    {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
    {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
  };

  auto getMidpoint = [&](size_t i1, size_t i2) -> size_t
  {
    Eigen::Vector3d mid = (vertices[i1] + vertices[i2]) * 0.5;
    mid.normalize();
    vertices.push_back(mid);
    return vertices.size() - 1;
  };

  for (int sub = 0; sub < 2; ++sub)
  {
    std::vector<std::array<size_t, 3>> newFaces;
    for (const auto& face : faces)
    {
      size_t a = getMidpoint(face[0], face[1]);
      size_t b = getMidpoint(face[1], face[2]);
      size_t c = getMidpoint(face[2], face[0]);

      newFaces.push_back({face[0], a, c});
      newFaces.push_back({face[1], b, a});
      newFaces.push_back({face[2], c, b});
      newFaces.push_back({a, b, c});
    }
    faces = newFaces;
  }

  std::vector<Coordinate> points;
  points.reserve(vertices.size());
  for (const auto& v : vertices)
  {
    points.emplace_back(v.x() * radius, v.y() * radius, v.z() * radius);
  }

  return points;
}

/// Create an L-shaped hull for asymmetric COM tests (substitute for B5).
/// The L-shape has vertices forming two attached cuboids:
///   main body: [-0.5, 0.5] x [-0.5, 0.5] x [-0.5, 0.5]
///   extension: [0.5, 1.5] x [-0.25, 0.25] x [-0.25, 0.25]
/// The COM of this hull will NOT be at the geometric center.
std::vector<Coordinate> createLShapePoints()
{
  // Main body (1x1x1 cube)
  std::vector<Coordinate> points = {
    Coordinate{-0.5, -0.5, -0.5},
    Coordinate{0.5, -0.5, -0.5},
    Coordinate{0.5, 0.5, -0.5},
    Coordinate{-0.5, 0.5, -0.5},
    Coordinate{-0.5, -0.5, 0.5},
    Coordinate{0.5, -0.5, 0.5},
    Coordinate{0.5, 0.5, 0.5},
    Coordinate{-0.5, 0.5, 0.5},
  };

  // Extension arm (1x0.5x0.5 cuboid extending in +x)
  points.push_back(Coordinate{1.5, -0.25, -0.25});
  points.push_back(Coordinate{1.5, 0.25, -0.25});
  points.push_back(Coordinate{1.5, -0.25, 0.25});
  points.push_back(Coordinate{1.5, 0.25, 0.25});

  return points;
}

/// Create a rod (elongated box): length x width x width
std::vector<Coordinate> createRodPoints(double length, double width)
{
  double halfL = length / 2.0;
  double halfW = width / 2.0;
  return {Coordinate{-halfL, -halfW, -halfW},
          Coordinate{halfL, -halfW, -halfW},
          Coordinate{halfL, halfW, -halfW},
          Coordinate{-halfL, halfW, -halfW},
          Coordinate{-halfL, -halfW, halfW},
          Coordinate{halfL, -halfW, halfW},
          Coordinate{halfL, halfW, halfW},
          Coordinate{-halfL, halfW, halfW}};
}

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

}  // anonymous namespace

// ============================================================================
// B1: Cube corner impact on floor at 45 degrees
// Validates: Lever arm coupling -- rotation should initiate from off-center impact
// ============================================================================

TEST(RotationalCollisionTest, B1_CubeCornerImpact_RotationInitiated)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  WorldModel world;

  // Floor: large cube centered at z=-50 (surface at z=0)
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube: 1m x 1m x 1m, rotated 45 degrees about x-axis AND 45 degrees about
  // y-axis so a corner points downward.
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  Eigen::Quaterniond q =
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitY()};

  // Position the cube so the lowest corner is at approximately z=2 above floor
  // For a unit cube rotated 45/45 degrees, the half-diagonal is sqrt(3)/2 ~ 0.866
  double const halfDiag = std::sqrt(3.0) / 2.0;
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 2.0 + halfDiag}, q};
  world.spawnObject(1, cubeHull, 1.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(0.7);
  world.getObject(cubeId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergy = computeSystemEnergy(world);

  // Simulate enough frames for impact and bouncing
  // Free fall from ~2.87m takes about sqrt(2*2.87/9.81) ~ 0.76s ~ 48 frames
  bool rotationDetected = false;
  bool nanDetected = false;
  double maxOmega = 0.0;

  for (int i = 1; i <= 200; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& state = world.getObject(cubeId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();
    double omegaMag = omega.norm();

    // Check for NaN
    if (std::isnan(omegaMag) || std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxOmega = std::max(maxOmega, omegaMag);

    if (omegaMag > 0.1)
    {
      rotationDetected = true;
    }
  }

  // DIAGNOSTIC: No NaN or explosion
  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in cube corner impact simulation";

  // DIAGNOSTIC: Post-impact angular velocity should be non-zero
  EXPECT_TRUE(rotationDetected)
    << "DIAGNOSTIC: Cube should begin rotating after corner impact. "
    << "Max omega=" << maxOmega << " rad/s";

  EXPECT_GT(maxOmega, 0.1)
    << "DIAGNOSTIC: Post-impact |omega| should exceed 0.1 rad/s. "
    << "Got maxOmega=" << maxOmega;

  // DIAGNOSTIC: Energy should decrease with each bounce (e=0.7 < 1)
  double const finalEnergy = computeSystemEnergy(world);
  if (!nanDetected)
  {
    EXPECT_LE(finalEnergy, initialEnergy * 1.05)
      << "DIAGNOSTIC: Energy should not grow significantly. "
      << "Initial=" << initialEnergy << " Final=" << finalEnergy
      << " Ratio=" << (finalEnergy / initialEnergy);
  }
}

// ============================================================================
// B2: Cube edge impact (one edge parallel to floor)
// Validates: Edge contact handling -- predictable rotation axis
// ============================================================================

TEST(RotationalCollisionTest, B2_CubeEdgeImpact_PredictableRotationAxis)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Cube: rotated 45 degrees about x-axis only (edge down, edge parallel to
  // y-axis in world frame). This means the cube has an edge pointing down
  // along the y-axis.
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};

  Eigen::Quaterniond q{
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitY()}};

  // For a cube rotated 45 degrees about y, the bottom edge is at -sqrt(2)/2
  // below center. Place center so bottom edge is at z=1 above floor.
  double const halfDiag2D = std::sqrt(2.0) / 2.0;
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 1.0 + halfDiag2D}, q};
  world.spawnObject(1, cubeHull, 1.0, cubeFrame);

  uint32_t cubeId = 1;
  world.getObject(cubeId).setCoefficientOfRestitution(0.7);
  world.getObject(cubeId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double const initialEnergy = computeSystemEnergy(world);

  // Simulate for impact
  bool nanDetected = false;
  double maxOmegaY = 0.0;
  double maxOmegaTotal = 0.0;

  for (int i = 1; i <= 200; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& state = world.getObject(cubeId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();

    if (std::isnan(omega.norm()) || std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    // For rotation about y-axis (the edge-parallel axis), we expect
    // the rotation to be primarily about y. The edge is parallel to y-axis
    // so the torque should cause rotation about y.
    maxOmegaY = std::max(maxOmegaY, std::abs(omega.y()));
    maxOmegaTotal = std::max(maxOmegaTotal, omega.norm());
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in cube edge impact simulation";

  // DIAGNOSTIC: Rotation should be initiated
  EXPECT_GT(maxOmegaTotal, 0.1)
    << "DIAGNOSTIC: Edge impact should initiate rotation. "
    << "Max omega=" << maxOmegaTotal << " rad/s";

  // DIAGNOSTIC: Rotation axis should be primarily about the y-axis
  // (perpendicular to the plane of rotation which is the xz-plane)
  if (maxOmegaTotal > 0.1)
  {
    double const yFraction = maxOmegaY / maxOmegaTotal;
    EXPECT_GT(yFraction, 0.3)
      << "DIAGNOSTIC: Rotation should have significant y-component. "
      << "y-fraction=" << yFraction
      << " (omega_y=" << maxOmegaY
      << ", omega_total=" << maxOmegaTotal << ")";
  }

  // DIAGNOSTIC: Energy should not grow
  double const finalEnergy = computeSystemEnergy(world);
  if (!nanDetected)
  {
    EXPECT_LE(finalEnergy, initialEnergy * 1.05)
      << "DIAGNOSTIC: Energy should not grow. "
      << "Initial=" << initialEnergy << " Final=" << finalEnergy;
  }
}

// ============================================================================
// B3: Sphere with symmetric contact drops vertically (negative test)
// Validates: Lever arm = 0 for symmetric contact -- no rotation
//
// FIX: This test now PASSES after ticket 0047a_revert_gravity_preapply.
// With gravity pre-apply (0047), restitution-gravity coupling introduced
// spurious torque via the e*J*g*dt term in the constraint RHS. The extra
// term caused off-center impulse application at the EPA contact point,
// producing rotation where there should be none.
//
// After revert (0047a): Pure normal impulse at contact point, no coupling
// term, no spurious torque → sphere does not rotate ✅
// ============================================================================

TEST(RotationalCollisionTest, B3_SphereDrop_NoRotation)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Sphere dropped vertically
  auto spherePoints = createSpherePoints(0.5);
  ConvexHull sphereHull{spherePoints};
  ReferenceFrame sphereFrame{Coordinate{0.0, 0.0, 2.0}};
  world.spawnObject(1, sphereHull, 1.0, sphereFrame);

  uint32_t sphereId = 1;
  world.getObject(sphereId).setCoefficientOfRestitution(0.7);
  world.getObject(sphereId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double maxOmega = 0.0;
  double maxLateralDrift = 0.0;

  for (int i = 1; i <= 200; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& state = world.getObject(sphereId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();
    maxOmega = std::max(maxOmega, omega.norm());

    // Check for lateral drift (should stay near x=0, y=0)
    double lateralDist = std::sqrt(state.position.x() * state.position.x() +
                                   state.position.y() * state.position.y());
    maxLateralDrift = std::max(maxLateralDrift, lateralDist);
  }

  // DIAGNOSTIC: Symmetric contact should produce no rotation
  // Note: icosphere is not a perfect sphere, so very small rotation is
  // acceptable from polyhedral contact geometry artifacts
  EXPECT_LT(maxOmega, 0.5)
    << "DIAGNOSTIC: Sphere should have minimal rotation. "
    << "Got maxOmega=" << maxOmega << " rad/s";

  // DIAGNOSTIC: Pure vertical bouncing, no lateral drift
  EXPECT_LT(maxLateralDrift, 0.5)
    << "DIAGNOSTIC: Sphere should not drift laterally. "
    << "Got maxLateralDrift=" << maxLateralDrift << " m";
}

// ============================================================================
// B4: Rod (elongated box) falls flat (negative test)
// Validates: No rotation for symmetric flat contact at COM level
// ============================================================================

TEST(RotationalCollisionTest, B4_RodFallsFlat_NoRotation)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Rod: 2m x 0.2m x 0.2m, long axis horizontal (parallel to x-axis)
  auto rodPoints = createRodPoints(2.0, 0.2);
  ConvexHull rodHull{rodPoints};

  // Position with bottom face at z=1 (center at z=1.1 since half-height=0.1)
  ReferenceFrame rodFrame{Coordinate{0.0, 0.0, 1.1}};
  world.spawnObject(1, rodHull, 1.0, rodFrame);

  uint32_t rodId = 1;
  world.getObject(rodId).setCoefficientOfRestitution(0.3);
  world.getObject(rodId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double maxOmega = 0.0;

  for (int i = 1; i <= 300; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& state = world.getObject(rodId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();

    if (std::isnan(omega.norm()))
    {
      break;
    }

    maxOmega = std::max(maxOmega, omega.norm());
  }

  // DIAGNOSTIC: Rod falling flat should not rotate
  // Multiple contact points along the bottom face should produce zero net torque
  EXPECT_LT(maxOmega, 0.5)
    << "DIAGNOSTIC: Rod should settle flat without significant rotation. "
    << "Got maxOmega=" << maxOmega << " rad/s";
}

// ============================================================================
// B5: Asymmetric hull (L-shape) dropped flat
// Validates: r = P_contact - P_COM (not geometric center)
//
// NOTE: ConvexHull computes COM from geometry, so we cannot directly offset
// the COM. Instead, we use an L-shaped hull whose COM is naturally offset from
// the geometric center. If the physics engine correctly uses P_COM (not
// geometric center), the L-shape should rotate upon flat-face impact.
// ============================================================================

TEST(RotationalCollisionTest, B5_LShapeDrop_RotationFromAsymmetricCOM)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  WorldModel world;

  // Floor
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // L-shape hull: COM is offset in +x direction from geometric center
  auto lPoints = createLShapePoints();
  ConvexHull lHull{lPoints};

  // Drop from height with flat bottom
  ReferenceFrame lFrame{Coordinate{0.0, 0.0, 2.0}};
  world.spawnObject(1, lHull, 1.0, lFrame);

  uint32_t lId = 1;
  world.getObject(lId).setCoefficientOfRestitution(0.5);
  world.getObject(lId).getInertialState().velocity = Velocity{0.0, 0.0, 0.0};

  double maxOmega = 0.0;
  bool nanDetected = false;

  for (int i = 1; i <= 300; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});

    auto const& state = world.getObject(lId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();

    if (std::isnan(omega.norm()) || std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxOmega = std::max(maxOmega, omega.norm());
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in L-shape drop simulation";

  // DIAGNOSTIC: The L-shape has an asymmetric COM. When it impacts the floor,
  // the contact points will NOT be centered under the COM, creating a net
  // torque. This tests that lever arms use P_contact - P_COM, not
  // P_contact - P_geometric_center.
  //
  // NOTE: Whether or not rotation occurs depends on whether the convex hull
  // of the L-shape points creates a shape with sufficiently offset COM.
  // The convex hull may "fill in" the L-shape, making it more symmetric.
  // If no rotation is observed, that may indicate the convex hull COM is
  // close enough to the geometric center of the convex hull that torques
  // cancel out. This is still a valid diagnostic result.
  EXPECT_GT(maxOmega, 0.01)
    << "DIAGNOSTIC: L-shape should exhibit some rotation due to asymmetric COM. "
    << "Got maxOmega=" << maxOmega << " rad/s. "
    << "Note: convex hull may fill the L-shape, reducing asymmetry.";
}
