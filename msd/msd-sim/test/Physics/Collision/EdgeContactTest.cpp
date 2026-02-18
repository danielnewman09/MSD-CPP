// Ticket: 0040c_edge_contact_manifold
// Ticket: 0062d_replay_stability_edge_contact_tests
// Test: Edge contact manifold generation
// Validates that edge-edge contacts produce 2 contact points with geometric
// extent, enabling torque generation from edge impacts.
//
// Multi-frame tests converted to ReplayEnabledTest fixture for automatic replay recording.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

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

std::vector<Coordinate> createTetrahedronPoints(double size)
{
  double const s = size;
  return {Coordinate{s, s, s},
          Coordinate{s, -s, -s},
          Coordinate{-s, s, -s},
          Coordinate{-s, -s, s}};
}

}  // namespace

// ============================================================================
// ConvexHull::findClosestEdge Unit Tests
// ============================================================================

TEST(ConvexHullEdge, FindClosestEdge_CubeVertex_ReturnsAdjacentEdge)
{
  // Ticket: 0040c_edge_contact_manifold
  // Query a point near a cube vertex — should return an edge adjacent to
  // that vertex.
  auto points = createCubePoints(2.0);
  ConvexHull hull{points};

  // Query near vertex (1, 1, 1) — slightly outside
  Coordinate const query{1.1, 1.1, 1.1};
  auto edge = hull.findClosestEdge(query);

  // The closest edge should have at least one endpoint at (1, 1, 1)
  Coordinate const vertex{1.0, 1.0, 1.0};
  bool const startAtVertex = (edge.start - vertex).norm() < 1e-6;
  bool const endAtVertex = (edge.end - vertex).norm() < 1e-6;
  EXPECT_TRUE(startAtVertex || endAtVertex)
    << "Closest edge should be adjacent to the queried vertex";
}

TEST(ConvexHullEdge, FindClosestEdge_CubeEdgeMidpoint_ReturnsThatEdge)
{
  // Ticket: 0040c_edge_contact_manifold
  // Query a point at the midpoint of an edge — should return that edge.
  auto points = createCubePoints(2.0);
  ConvexHull hull{points};

  // Midpoint of edge from (1, -1, 1) to (1, 1, 1) is (1, 0, 1)
  // Query slightly outside the hull
  Coordinate const query{1.1, 0.0, 1.0};
  auto edge = hull.findClosestEdge(query);

  // Both endpoints should be at x=1, z=1, with y = +/- 1
  Coordinate const v0{1.0, -1.0, 1.0};
  Coordinate const v1{1.0, 1.0, 1.0};

  bool const matchesForward =
    ((edge.start - v0).norm() < 1e-6 && (edge.end - v1).norm() < 1e-6);
  bool const matchesReverse =
    ((edge.start - v1).norm() < 1e-6 && (edge.end - v0).norm() < 1e-6);

  EXPECT_TRUE(matchesForward || matchesReverse)
    << "Closest edge should match the edge whose midpoint was queried";
}

TEST(ConvexHullEdge, FindClosestEdge_CubeFaceCenter_ReturnsNearestEdge)
{
  // Ticket: 0040c_edge_contact_manifold
  // Query a point at a face center — should return one of the 4 face edges.
  auto points = createCubePoints(2.0);
  ConvexHull hull{points};

  // Center of +z face is at (0, 0, 1), query slightly outside
  Coordinate const query{0.0, 0.0, 1.1};
  auto edge = hull.findClosestEdge(query);

  // Both edge endpoints should have z = 1.0
  EXPECT_NEAR(edge.start.z(), 1.0, 1e-6);
  EXPECT_NEAR(edge.end.z(), 1.0, 1e-6);
}

TEST(ConvexHullEdge, FindClosestEdge_TetrahedronVertex_ReturnsAdjacentEdge)
{
  // Ticket: 0040c_edge_contact_manifold
  // Works correctly for non-cube geometry (tetrahedron).
  auto points = createTetrahedronPoints(1.0);
  ConvexHull hull{points};

  // Query near first vertex (1, 1, 1)
  Coordinate const query{1.2, 1.2, 1.2};
  auto edge = hull.findClosestEdge(query);

  // The closest edge should have at least one endpoint near (1, 1, 1)
  Coordinate const vertex{1.0, 1.0, 1.0};
  bool const startNearVertex = (edge.start - vertex).norm() < 1e-6;
  bool const endNearVertex = (edge.end - vertex).norm() < 1e-6;
  EXPECT_TRUE(startNearVertex || endNearVertex)
    << "Closest edge should be adjacent to the queried vertex";
}

// ============================================================================
// Edge Contact Detection via CollisionHandler
// ============================================================================

TEST(EdgeContact, CubeEdgeOnFloor_ProducesMultipleContacts)
{
  // Ticket: 0040c_edge_contact_manifold
  // A cube rotated 45 degrees about one axis so its edge contacts a floor.
  // The edge-edge contact path should activate and generate 2 contact points.

  // Floor: large flat cube
  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  AssetEnvironment floor{1, 1, floorHull, floorFrame};

  // Cube: rotated 45 degrees about y-axis (edge pointing down)
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  Eigen::Quaterniond q{
    Eigen::AngleAxisd{std::numbers::pi / 4.0, Eigen::Vector3d::UnitY()}};
  // Position so that the bottom edge slightly penetrates the floor
  double const halfDiag2D = std::sqrt(2.0) / 2.0;
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, halfDiag2D - 0.01}, q};
  AssetInertial cube{1, 1, cubeHull, 1.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value())
    << "Edge-on cube should collide with floor";

  // With edge contact manifold, we should get 2 contact points
  EXPECT_GE(result->contactCount, 2u)
    << "Edge contact should produce at least 2 contact points";
}

TEST(EdgeContact, ContactPoints_HaveGeometricExtent)
{
  // Ticket: 0040c_edge_contact_manifold
  // The 2 contact points from edge contact should be separated in space.

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  AssetEnvironment floor{1, 1, floorHull, floorFrame};

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  Eigen::Quaterniond q{
    Eigen::AngleAxisd{std::numbers::pi / 4.0, Eigen::Vector3d::UnitY()}};
  double const halfDiag2D = std::sqrt(2.0) / 2.0;
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, halfDiag2D - 0.01}, q};
  AssetInertial cube{1, 1, cubeHull, 1.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value());

  if (result->contactCount >= 2)
  {
    // Contact points should have non-zero separation
    Coordinate const cp0_mid =
      (result->contacts[0].pointA + result->contacts[0].pointB) * 0.5;
    Coordinate const cp1_mid =
      (result->contacts[1].pointA + result->contacts[1].pointB) * 0.5;
    double const separation = (cp1_mid - cp0_mid).norm();

    EXPECT_GT(separation, 0.01)
      << "Edge contact points should have geometric extent (separation > "
         "0.01m)";
  }
}

TEST(EdgeContact, LeverArm_CrossNormal_NonZero)
{
  // Ticket: 0040c_edge_contact_manifold
  // For edge contacts, at least one contact point should produce
  // a non-zero r x n cross product (enabling torque generation).

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  AssetEnvironment floor{1, 1, floorHull, floorFrame};

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  Eigen::Quaterniond q{
    Eigen::AngleAxisd{std::numbers::pi / 4.0, Eigen::Vector3d::UnitY()}};
  double const halfDiag2D = std::sqrt(2.0) / 2.0;
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, halfDiag2D - 0.01}, q};
  AssetInertial cube{1, 1, cubeHull, 1.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value());

  if (result->contactCount >= 2)
  {
    // Compute lever arm from cube COM to contact point
    Coordinate const com = cube.getInertialState().position;
    Coordinate const& normal = result->normal;

    bool hasNonZeroCross = false;
    for (size_t i = 0; i < result->contactCount; ++i)
    {
      Coordinate const r = result->contacts[i].pointA - com;
      Coordinate const cross = r.cross(normal);
      if (cross.norm() > 1e-6)
      {
        hasNonZeroCross = true;
        break;
      }
    }

    EXPECT_TRUE(hasNonZeroCross)
      << "At least one contact point should have non-zero r x n "
         "(lever arm cross normal)";
  }
}

TEST(EdgeContact, ContactPoints_HavePositiveDepth)
{
  // Ticket: 0040c_edge_contact_manifold
  // All edge contact points should have positive penetration depth.

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  AssetEnvironment floor{1, 1, floorHull, floorFrame};

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  Eigen::Quaterniond q{
    Eigen::AngleAxisd{std::numbers::pi / 4.0, Eigen::Vector3d::UnitY()}};
  double const halfDiag2D = std::sqrt(2.0) / 2.0;
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, halfDiag2D - 0.01}, q};
  AssetInertial cube{1, 1, cubeHull, 1.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value());

  for (size_t i = 0; i < result->contactCount; ++i)
  {
    EXPECT_GT(result->contacts[i].depth, 0.0)
      << "Contact point " << i << " should have positive depth";
  }
}

// ============================================================================
// Integration Test: Edge Impact Initiates Rotation
// ============================================================================

TEST_F(ReplayEnabledTest, EdgeContact_CubeEdgeImpact_InitiatesRotation)
{
  // Ticket: 0040c_edge_contact_manifold
  // Ticket: 0062d_replay_stability_edge_contact_tests
  //
  // PHYSICS: A cube with an edge pointing down is dropped onto a floor.
  // To break the symmetry (a perfectly aligned edge produces zero net
  // torque because both contact points are equidistant from the COM),
  // we tilt the cube 5° about the X-axis so one end of the edge hits
  // first. This asymmetric contact should initiate rotation about X.

  // Floor: large cube centered at z=-50 (surface at z=0)
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: rotated 45° about Y (edge down) + 5° about X (asymmetric tilt)
  Eigen::Quaterniond q =
    Eigen::AngleAxisd{5.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{std::numbers::pi / 4.0, Eigen::Vector3d::UnitY()};

  double const halfDiag2D = std::sqrt(2.0) / 2.0;

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, 1.0 + halfDiag2D},
                                   1.0,  // mass (kg)
                                   0.7,  // restitution
                                   0.5); // friction
  uint32_t cubeId = cube.getInstanceId();

  // Manually set orientation (fixture limitation workaround)
  world().getObject(cubeId).getInertialState().orientation = q;

  // Simulate until collision and check for rotation
  bool collisionOccurred = false;

  for (int i = 0; i < 200; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();
    if (state.position.z() < halfDiag2D + 0.5)
    {
      collisionOccurred = true;
    }

    if (collisionOccurred)
    {
      double const qDotMag = state.quaternionRate.norm();
      if (qDotMag > 1e-4)
      {
        SUCCEED();
        return;
      }
    }
  }

  auto const& finalState = world().getObject(cubeId).getInertialState();
  double const finalQDotMag = finalState.quaternionRate.norm();

  EXPECT_GT(finalQDotMag, 1e-4)
    << "Asymmetric edge impact should initiate rotation. "
    << "quaternionRate.norm()=" << finalQDotMag;
}

// ============================================================================
// Regression: Face-face contacts unaffected
// ============================================================================

TEST(EdgeContact, FaceFaceContact_StillProducesMultipleContacts)
{
  // Ticket: 0040c_edge_contact_manifold
  // Face-face contacts (non-rotated cubes) should still produce multiple
  // contact points via the normal clipping path (not the edge path).

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  AssetEnvironment floor{1, 1, floorHull, floorFrame};

  // Non-rotated cube slightly penetrating the floor
  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.49}};
  AssetInertial cube{1, 1, cubeHull, 1.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value())
    << "Face-on cube should collide with floor";

  // Face-face contacts should produce >= 3 contact points (4 typically)
  EXPECT_GE(result->contactCount, 3u)
    << "Face-face contact should still produce >= 3 contact points";
}

// ============================================================================
// Edge case: Very small penetration
// ============================================================================

TEST(EdgeContact, SmallPenetration_StillDetected)
{
  // Ticket: 0040c_edge_contact_manifold
  // Edge contact with very small penetration should still be detected.

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  AssetEnvironment floor{1, 1, floorHull, floorFrame};

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  Eigen::Quaterniond q{
    Eigen::AngleAxisd{std::numbers::pi / 4.0, Eigen::Vector3d::UnitY()}};
  double const halfDiag2D = std::sqrt(2.0) / 2.0;
  // Very small penetration (0.001m)
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, halfDiag2D - 0.001}, q};
  AssetInertial cube{1, 1, cubeHull, 1.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value())
    << "Small edge penetration should still be detected";

  EXPECT_GT(result->penetrationDepth, 0.0)
    << "Penetration depth should be positive";
}
