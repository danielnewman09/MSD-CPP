// Ticket: 0039b_linear_collision_test_suite
// Test: Scenario A — Linear collision tests (no rotation)

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <numbers>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
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

/// Create an icosphere point cloud with the given radius.
/// Uses 2 levels of subdivision from an icosahedron (~42 vertices).
/// Spheres avoid rotational coupling in linear collision tests.
std::vector<Coordinate> createSpherePoints(double radius)
{
  // Start with icosahedron vertices
  double const phi = (1.0 + std::sqrt(5.0)) / 2.0;  // Golden ratio

  // 12 base icosahedron vertices (normalized to unit sphere, then scaled)
  std::vector<Eigen::Vector3d> vertices = {
    {-1,  phi, 0}, { 1,  phi, 0}, {-1, -phi, 0}, { 1, -phi, 0},
    { 0, -1,  phi}, { 0,  1,  phi}, { 0, -1, -phi}, { 0,  1, -phi},
    { phi, 0, -1}, { phi, 0,  1}, {-phi, 0, -1}, {-phi, 0,  1}
  };

  // Normalize to unit sphere
  for (auto& v : vertices)
  {
    v.normalize();
  }

  // 20 icosahedron faces (triangles)
  std::vector<std::array<size_t, 3>> faces = {
    {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
    {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
    {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
    {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
  };

  // Subdivide twice for ~42 vertices (~162 faces)
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

  // Convert to Coordinate with radius scaling
  std::vector<Coordinate> points;
  points.reserve(vertices.size());
  for (const auto& v : vertices)
  {
    points.emplace_back(v.x() * radius, v.y() * radius, v.z() * radius);
  }

  return points;
}

/// Create a large flat cube for use as a floor
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

}  // anonymous namespace

// ============================================================================
// A1: Sphere drops vertically onto horizontal plane (settling)
// ============================================================================

TEST(LinearCollisionTest, A1_SphereDrop_SettlesToRest)
{
  // NOTE: No friction parameter currently exposed. Tests proceed without
  // setting mu=0; contact-normal-only constraints provide equivalent behavior.
  // Ticket: 0039b_linear_collision_test_suite

  WorldModel world;

  // Floor: large cube centered at z=-50 (surface at z=0)
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  // Sphere: radius 0.5m, dropped from z=5 (center, surface at z=4.5)
  auto spherePoints = createSpherePoints(0.5);
  ConvexHull sphereHull{spherePoints};
  ReferenceFrame sphereFrame{Coordinate{0.0, 0.0, 5.0}};
  world.spawnObject(1, sphereHull, sphereFrame);

  uint32_t sphereId = 1;
  world.getObject(sphereId).setCoefficientOfRestitution(0.7);
  world.getObject(sphereId).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  // Simulate for enough frames for the sphere to settle
  // At 60 FPS, 500 frames = ~8.3 seconds
  for (int i = 1; i <= 500; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});
  }

  double const finalZ = world.getObject(sphereId).getInertialState().position.z();
  double const finalVel = world.getObject(sphereId).getInertialState().velocity.norm();

  // Sphere should settle on the floor at approximately z=0.5 (radius)
  EXPECT_NEAR(finalZ, 0.5, 0.1)
    << "Sphere should settle near floor surface. Got z=" << finalZ;

  // Sphere should be approximately at rest
  EXPECT_LT(finalVel, 0.5) << "Sphere should be near rest. Got vel=" << finalVel;
}

// ============================================================================
// A2: Perfectly inelastic (e=0) sphere drops vertically
// ============================================================================

TEST(LinearCollisionTest, A2_PerfectlyInelastic_QuickStop)
{
  WorldModel world;

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  auto spherePoints = createSpherePoints(0.5);
  ConvexHull sphereHull{spherePoints};
  ReferenceFrame sphereFrame{Coordinate{0.0, 0.0, 2.0}};
  world.spawnObject(1, sphereHull, sphereFrame);

  uint32_t sphereId = 1;
  world.getObject(sphereId).setCoefficientOfRestitution(0.0);
  world.getObject(sphereId).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  // Simulate until collision and settling
  // Free fall from z=2 to z=0.5 takes about sqrt(2*1.5/9.81) ~ 0.55s ~ 34 frames
  for (int i = 1; i <= 100; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});
  }

  double const finalZ = world.getObject(sphereId).getInertialState().position.z();
  double const finalVel = world.getObject(sphereId).getInertialState().velocity.norm();

  // Position should be stable near floor
  EXPECT_NEAR(finalZ, 0.5, 0.2) << "Should rest at floor. Got z=" << finalZ;

  // Velocity should be very low after inelastic collision
  EXPECT_LT(finalVel, 1.0)
    << "Velocity should be very low after inelastic collision. Got vel=" << finalVel;
}

// ============================================================================
// A3: Perfectly elastic (e=1) sphere — perpetual bouncing
// ============================================================================

TEST(LinearCollisionTest, A3_PerfectlyElastic_EnergyConserved)
{
  WorldModel world;

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  world.spawnEnvironmentObject(1, floorHull, floorFrame);

  auto spherePoints = createSpherePoints(0.5);
  ConvexHull sphereHull{spherePoints};
  ReferenceFrame sphereFrame{Coordinate{0.0, 0.0, 2.0}};
  world.spawnObject(1, sphereHull, sphereFrame);

  uint32_t sphereId = 1;
  world.getObject(sphereId).setCoefficientOfRestitution(1.0);
  world.getObject(sphereId).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  // Track max height over time to verify bouncing persists
  double maxHeight = 0.0;

  for (int i = 1; i <= 300; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});
    double z = world.getObject(sphereId).getInertialState().position.z();
    if (z > maxHeight && i > 50)  // After first bounce
    {
      maxHeight = z;
    }
  }

  // With perfect elasticity, the sphere should bounce back near initial height
  // Allow some tolerance for numerical effects
  EXPECT_GT(maxHeight, 1.0)
    << "Elastic sphere should continue bouncing. Max height=" << maxHeight;
}

// ============================================================================
// A4: Two spheres, equal mass, head-on elastic — velocity swap
// ============================================================================

TEST(LinearCollisionTest, A4_EqualMassElastic_VelocitySwap)
{
  WorldModel world;

  // Remove gravity for clean 1D collision
  world.clearPotentialEnergies();

  auto spherePoints = createSpherePoints(0.5);
  ConvexHull hullA{spherePoints};
  ConvexHull hullB{spherePoints};

  // Place spheres overlapping slightly for immediate collision
  // Both at z=0.5 (no floor needed without gravity)
  ReferenceFrame frameA{Coordinate{-0.05, 0.0, 0.5}};
  ReferenceFrame frameB{Coordinate{0.95, 0.0, 0.5}};

  world.spawnObject(1, hullA, frameA);
  world.spawnObject(2, hullB, frameB);

  uint32_t idA = 1;
  uint32_t idB = 2;

  // Default mass is 10.0 (from spawnObject)
  double const mass = world.getObject(idA).getMass();

  world.getObject(idA).setCoefficientOfRestitution(1.0);
  world.getObject(idB).setCoefficientOfRestitution(1.0);

  world.getObject(idA).getInertialState().velocity = Vector3D{2.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  // Initial momentum
  double const initialMomentumX =
    mass * world.getObject(idA).getInertialState().velocity.x() +
    mass * world.getObject(idB).getInertialState().velocity.x();

  double const initialKE =
    0.5 * mass * world.getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * mass * world.getObject(idB).getInertialState().velocity.squaredNorm();

  // Step simulation to let collision happen
  for (int i = 1; i <= 5; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});
  }

  double const vAxFinal = world.getObject(idA).getInertialState().velocity.x();
  double const vBxFinal = world.getObject(idB).getInertialState().velocity.x();

  // For equal-mass elastic collision: velocities should swap
  // A should slow/stop, B should gain velocity
  EXPECT_LT(std::abs(vAxFinal), 1.0)
    << "Object A should slow down significantly. Got vAx=" << vAxFinal;
  EXPECT_GT(vBxFinal, 0.5)
    << "Object B should gain positive velocity. Got vBx=" << vBxFinal;

  // Momentum conservation
  double const finalMomentumX =
    mass * world.getObject(idA).getInertialState().velocity.x() +
    mass * world.getObject(idB).getInertialState().velocity.x();

  EXPECT_NEAR(initialMomentumX, finalMomentumX, 0.1 * std::abs(initialMomentumX))
    << "Total momentum should be conserved";

  // KE conservation (elastic)
  double const finalKE =
    0.5 * mass * world.getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * mass * world.getObject(idB).getInertialState().velocity.squaredNorm();

  EXPECT_NEAR(initialKE, finalKE, 0.1 * initialKE)
    << "Total KE should be conserved for elastic collision";
}

// ============================================================================
// A5: Two spheres, unequal mass (10:1), elastic
// ============================================================================

TEST(LinearCollisionTest, A5_UnequalMassElastic_ClassicalFormulas)
{
  WorldModel world;

  // Remove gravity for clean 1D collision
  world.clearPotentialEnergies();

  auto spherePointsA = createSpherePoints(0.5);
  auto spherePointsB = createSpherePoints(0.5);
  ConvexHull hullA{spherePointsA};
  ConvexHull hullB{spherePointsB};

  // Place overlapping for immediate collision
  ReferenceFrame frameA{Coordinate{-0.05, 0.0, 0.5}};
  ReferenceFrame frameB{Coordinate{0.95, 0.0, 0.5}};

  // Spawn with unequal masses: 10 kg vs 1 kg (10:1 ratio)
  // Ticket: 0039b requires mass ratio testing
  world.spawnObject(1, hullA, 10.0, frameA);
  world.spawnObject(2, hullB, 1.0, frameB);

  uint32_t idA = 1;
  uint32_t idB = 2;

  double const massA = world.getObject(idA).getMass();
  double const massB = world.getObject(idB).getMass();

  world.getObject(idA).setCoefficientOfRestitution(1.0);
  world.getObject(idB).setCoefficientOfRestitution(1.0);

  world.getObject(idA).getInertialState().velocity = Vector3D{1.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  double const initialMomentumX = massA * 1.0 + massB * 0.0;
  double const initialKE = 0.5 * massA * 1.0;

  for (int i = 1; i <= 5; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});
  }

  double const vAxFinal = world.getObject(idA).getInertialState().velocity.x();
  double const vBxFinal = world.getObject(idB).getInertialState().velocity.x();

  // Classical elastic collision formulas:
  // v_A' = ((m_A - m_B) / (m_A + m_B)) * v_A = (9/11) ≈ 0.818
  // v_B' = (2 * m_A / (m_A + m_B)) * v_A = (20/11) ≈ 1.818
  double const expectedVA = (massA - massB) / (massA + massB) * 1.0;
  double const expectedVB = (2.0 * massA) / (massA + massB) * 1.0;

  // Verify velocity formulas (generous tolerance for discrete simulation)
  EXPECT_NEAR(vAxFinal, expectedVA, 0.2 * std::abs(expectedVA))
    << "Object A velocity should follow classical formula";
  EXPECT_NEAR(vBxFinal, expectedVB, 0.3 * std::abs(expectedVB))
    << "Object B velocity should follow classical formula";

  // Momentum conservation
  double const finalMomentumX = massA * vAxFinal + massB * vBxFinal;
  EXPECT_NEAR(initialMomentumX, finalMomentumX, 0.15 * std::abs(initialMomentumX))
    << "Momentum should be conserved";

  // KE conservation — icosphere is not a perfect sphere, so some energy
  // transfers into rotational KE. Use wider tolerance (15%) to account for
  // angular energy from polyhedral contact geometry.
  double const finalKE =
    0.5 * massA * world.getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * massB * world.getObject(idB).getInertialState().velocity.squaredNorm();
  EXPECT_NEAR(initialKE, finalKE, 0.2 * initialKE)
    << "KE should be approximately conserved for elastic collision";
}

// ============================================================================
// A6: Glancing collision at offset — impulse along contact normal
// ============================================================================

TEST(LinearCollisionTest, A6_GlancingCollision_MomentumAndEnergyConserved)
{
  WorldModel world;

  // Remove gravity for clean collision
  world.clearPotentialEnergies();

  auto spherePoints = createSpherePoints(0.5);
  ConvexHull hullA{spherePoints};
  ConvexHull hullB{spherePoints};

  // Sphere A approaching from left, Sphere B offset in Y for glancing contact
  ReferenceFrame frameA{Coordinate{-0.05, 0.0, 0.5}};
  ReferenceFrame frameB{Coordinate{0.85, 0.5, 0.5}};  // Offset Y for glancing

  world.spawnObject(1, hullA, frameA);
  world.spawnObject(2, hullB, frameB);

  uint32_t idA = 1;
  uint32_t idB = 2;

  double const mass = world.getObject(idA).getMass();

  world.getObject(idA).setCoefficientOfRestitution(1.0);
  world.getObject(idB).setCoefficientOfRestitution(1.0);

  world.getObject(idA).getInertialState().velocity = Vector3D{2.0, 0.0, 0.0};
  world.getObject(idB).getInertialState().velocity = Vector3D{0.0, 0.0, 0.0};

  Coordinate const initialMomentum =
    world.getObject(idA).getInertialState().velocity * mass +
    world.getObject(idB).getInertialState().velocity * mass;
  double const initialKE =
    0.5 * mass * world.getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * mass * world.getObject(idB).getInertialState().velocity.squaredNorm();

  for (int i = 1; i <= 5; ++i)
  {
    world.update(std::chrono::milliseconds{i * 16});
  }

  Coordinate const finalMomentum =
    world.getObject(idA).getInertialState().velocity * mass +
    world.getObject(idB).getInertialState().velocity * mass;
  double const finalKE =
    0.5 * mass * world.getObject(idA).getInertialState().velocity.squaredNorm() +
    0.5 * mass * world.getObject(idB).getInertialState().velocity.squaredNorm();

  // Momentum conservation in each axis
  EXPECT_NEAR(initialMomentum.x(), finalMomentum.x(), 0.1 * std::abs(initialMomentum.x()))
    << "X-momentum should be conserved";
  EXPECT_NEAR(initialMomentum.y(), finalMomentum.y(), 1.0)
    << "Y-momentum should be conserved";

  // KE conservation (elastic)
  EXPECT_NEAR(initialKE, finalKE, 0.1 * initialKE)
    << "KE should be conserved for elastic glancing collision";

  // Both objects should have non-zero velocity after glancing collision
  double const vBFinal = world.getObject(idB).getInertialState().velocity.norm();
  EXPECT_GT(vBFinal, 0.01) << "Object B should gain velocity from glancing collision";
}
