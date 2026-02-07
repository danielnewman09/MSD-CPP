// Ticket: 0040a_per_contact_penetration_depth
// Design: docs/designs/0040a-per-contact-penetration-depth/design.md
// Test: Per-contact penetration depth computation

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
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

}  // anonymous namespace

// ============================================================================
// SingleContact_DepthMatchesEPA
// A single contact point's depth should approximately equal
// CollisionResult::penetrationDepth
// ============================================================================

TEST(PerContactDepth, SingleContact_DepthMatchesEPA)
{
  // Ticket: 0040a_per_contact_penetration_depth
  // Two cubes colliding edge-on should produce a single (or few) contact(s).
  // For single-contact case, depth should match penetrationDepth.

  auto cubeAPoints = createCubePoints(1.0);
  ConvexHull cubeAHull{cubeAPoints};
  // Cube A centered at origin
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  AssetInertial assetA{1, 1, cubeAHull, 10.0, frameA};

  auto cubeBPoints = createCubePoints(1.0);
  ConvexHull cubeBHull{cubeBPoints};
  // Cube B overlapping slightly along x-axis (0.1m overlap)
  ReferenceFrame frameB{Coordinate{0.9, 0.5, 0.5}};
  // Rotate 45 degrees about Z to get an edge-on collision
  Eigen::Quaterniond rotation{
    Eigen::AngleAxisd{std::numbers::pi / 4.0, Eigen::Vector3d::UnitZ()}};
  frameB.setQuaternion(rotation);
  AssetInertial assetB{1, 2, cubeBHull, 10.0, frameB};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value()) << "Expected collision between overlapping cubes";
  ASSERT_GE(result->contactCount, 1u) << "Expected at least 1 contact point";

  // For any result, all depths should be non-negative
  for (size_t i = 0; i < result->contactCount; ++i)
  {
    EXPECT_GE(result->contacts[i].depth, 0.0)
      << "Contact " << i << " has negative depth: " << result->contacts[i].depth;
  }

  // Max per-contact depth should not exceed EPA penetrationDepth
  double maxContactDepth = 0.0;
  for (size_t i = 0; i < result->contactCount; ++i)
  {
    maxContactDepth = std::max(maxContactDepth, result->contacts[i].depth);
  }

  // Max contact depth should be close to (but not exceed) penetrationDepth
  EXPECT_LE(maxContactDepth, result->penetrationDepth + 1e-4)
    << "Max per-contact depth should not significantly exceed EPA penetrationDepth";
}

// ============================================================================
// FourContact_FlatLanding_EqualDepths
// Cube landing flat on plane: all 4 contacts should have approximately
// equal depth
// ============================================================================

TEST(PerContactDepth, FourContact_FlatLanding_EqualDepths)
{
  // Ticket: 0040a_per_contact_penetration_depth
  // A cube sitting flat on a large floor cube should produce 4 contacts
  // with approximately equal penetration depths.

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};  // Surface at z=0
  AssetEnvironment floor{1, 100, floorHull, floorFrame};

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  // Place cube so bottom face is 0.05m below floor surface (slight overlap)
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.45}};
  AssetInertial cube{1, 1, cubeHull, 10.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value()) << "Expected collision between cube and floor";

  // For flat landing, we expect 4 contact points
  EXPECT_EQ(result->contactCount, 4u)
    << "Expected 4 contact points for flat cube on floor";

  if (result->contactCount == 4)
  {
    // All depths should be approximately equal for a flat-landing cube
    double minDepth = result->contacts[0].depth;
    double maxDepth = result->contacts[0].depth;
    for (size_t i = 1; i < result->contactCount; ++i)
    {
      minDepth = std::min(minDepth, result->contacts[i].depth);
      maxDepth = std::max(maxDepth, result->contacts[i].depth);
    }

    double const spread = maxDepth - minDepth;
    EXPECT_LT(spread, 0.01)
      << "Flat-landing depths should be nearly equal. "
      << "Min=" << minDepth << " Max=" << maxDepth << " Spread=" << spread;

    // All depths should be approximately 0.05m (the overlap)
    for (size_t i = 0; i < result->contactCount; ++i)
    {
      EXPECT_NEAR(result->contacts[i].depth, 0.05, 0.02)
        << "Contact " << i << " depth should be near 0.05m overlap";
    }
  }
}

// ============================================================================
// TiltedCube_AsymmetricDepths
// Cube tilted 15 degrees: deeper-side contacts should have larger depth
// ============================================================================

TEST(PerContactDepth, TiltedCube_AsymmetricDepths)
{
  // Ticket: 0040a_per_contact_penetration_depth
  // Two axis-aligned cubes overlapping with one shifted slightly in both
  // x and z, producing a face-face contact where the clipping polygon
  // has varying projection distances. The offset creates asymmetric
  // penetration depths across the contact manifold.

  auto cubeAPoints = createCubePoints(2.0);
  ConvexHull cubeAHull{cubeAPoints};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  AssetInertial assetA{1, 1, cubeAHull, 10.0, frameA};

  auto cubeBPoints = createCubePoints(2.0);
  ConvexHull cubeBHull{cubeBPoints};
  // Cube B offset so the overlapping face region has varying depth.
  // A slight tilt (3 degrees about Y) produces asymmetric contact depths
  // while keeping the contact as face-face (not edge).
  double const tiltAngle = 3.0 * std::numbers::pi / 180.0;
  ReferenceFrame frameB{Coordinate{1.8, 0.0, 0.0}};
  Eigen::Quaterniond tilt{Eigen::AngleAxisd{tiltAngle, Eigen::Vector3d::UnitY()}};
  frameB.setQuaternion(tilt);
  AssetInertial assetB{1, 2, cubeBHull, 10.0, frameB};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value()) << "Expected collision between overlapping cubes";
  ASSERT_GE(result->contactCount, 1u) << "Expected at least 1 contact";

  // All depths should be non-negative
  for (size_t i = 0; i < result->contactCount; ++i)
  {
    EXPECT_GE(result->contacts[i].depth, 0.0)
      << "Contact " << i << " should have non-negative depth";
  }

  // If we get multiple contacts, verify depths vary
  if (result->contactCount >= 2)
  {
    std::vector<double> depths;
    for (size_t i = 0; i < result->contactCount; ++i)
    {
      depths.push_back(result->contacts[i].depth);
    }
    std::sort(depths.begin(), depths.end());

    double const deepest = depths.back();
    double const shallowest = depths.front();
    EXPECT_GT(deepest, shallowest + 1e-6)
      << "Tilted cube should have asymmetric depths. "
      << "Deepest=" << deepest << " Shallowest=" << shallowest;
  }
}

// ============================================================================
// GrazingContact_NearZeroDepth
// Barely touching objects should have near-zero depth
// ============================================================================

TEST(PerContactDepth, GrazingContact_NearZeroDepth)
{
  // Ticket: 0040a_per_contact_penetration_depth
  // Two cubes barely overlapping: depths should be very small.

  auto cubeAPoints = createCubePoints(1.0);
  ConvexHull cubeAHull{cubeAPoints};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  AssetInertial assetA{1, 1, cubeAHull, 10.0, frameA};

  auto cubeBPoints = createCubePoints(1.0);
  ConvexHull cubeBHull{cubeBPoints};
  // Barely overlapping: 0.01m penetration along x-axis
  ReferenceFrame frameB{Coordinate{0.99, 0.0, 0.0}};
  AssetInertial assetB{1, 2, cubeBHull, 10.0, frameB};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(assetA, assetB);

  ASSERT_TRUE(result.has_value()) << "Expected collision for overlapping cubes";
  ASSERT_GE(result->contactCount, 1u);

  // All depths should be near zero (around 0.01m)
  for (size_t i = 0; i < result->contactCount; ++i)
  {
    EXPECT_LT(result->contacts[i].depth, 0.05)
      << "Contact " << i << " depth should be small for grazing contact";
    EXPECT_GE(result->contacts[i].depth, 0.0)
      << "Contact " << i << " depth should be non-negative";
  }
}

// ============================================================================
// MaxDepth_MatchesPenetrationDepth
// max(contactPoint.depth) should not exceed CollisionResult::penetrationDepth
// ============================================================================

TEST(PerContactDepth, MaxDepth_MatchesPenetrationDepth)
{
  // Ticket: 0040a_per_contact_penetration_depth
  // For any collision, the maximum per-contact depth should not significantly
  // exceed the EPA-computed penetrationDepth (which is the polytope distance).

  auto floorPoints = createCubePoints(100.0);
  ConvexHull floorHull{floorPoints};
  ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
  AssetEnvironment floor{1, 100, floorHull, floorFrame};

  auto cubePoints = createCubePoints(1.0);
  ConvexHull cubeHull{cubePoints};
  // Moderate overlap: 0.1m penetration
  ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.4}};
  AssetInertial cube{1, 1, cubeHull, 10.0, cubeFrame};

  CollisionHandler handler{1e-6};
  auto result = handler.checkCollision(cube, floor);

  ASSERT_TRUE(result.has_value());
  ASSERT_GE(result->contactCount, 1u);

  double maxContactDepth = 0.0;
  for (size_t i = 0; i < result->contactCount; ++i)
  {
    maxContactDepth = std::max(maxContactDepth, result->contacts[i].depth);
  }

  // Max contact depth should be bounded by EPA penetrationDepth (with tolerance)
  EXPECT_LE(maxContactDepth, result->penetrationDepth + 1e-3)
    << "Max per-contact depth (" << maxContactDepth
    << ") should not exceed EPA penetrationDepth ("
    << result->penetrationDepth << ")";

  // Max contact depth should also be reasonably close to penetrationDepth
  // (it represents the same geometric quantity for the deepest contact)
  EXPECT_GT(maxContactDepth, 0.0)
    << "Max per-contact depth should be positive for overlapping objects";
}

// ============================================================================
// DepthNonNegative
// All contactPoint.depth values must be >= 0 for any collision
// ============================================================================

TEST(PerContactDepth, DepthNonNegative)
{
  // Ticket: 0040a_per_contact_penetration_depth
  // Test with various collision configurations to ensure depth is never negative.

  CollisionHandler handler{1e-6};

  // Configuration 1: Flat cube on floor
  {
    auto floorPoints = createCubePoints(100.0);
    ConvexHull floorHull{floorPoints};
    ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
    AssetEnvironment floor{1, 100, floorHull, floorFrame};

    auto cubePoints = createCubePoints(1.0);
    ConvexHull cubeHull{cubePoints};
    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.45}};
    AssetInertial cube{1, 1, cubeHull, 10.0, cubeFrame};

    auto result = handler.checkCollision(cube, floor);
    if (result.has_value())
    {
      for (size_t i = 0; i < result->contactCount; ++i)
      {
        EXPECT_GE(result->contacts[i].depth, 0.0)
          << "Config1 contact " << i << " has negative depth";
      }
    }
  }

  // Configuration 2: Tilted cube on floor
  {
    auto floorPoints = createCubePoints(100.0);
    ConvexHull floorHull{floorPoints};
    ReferenceFrame floorFrame{Coordinate{0.0, 0.0, -50.0}};
    AssetEnvironment floor{1, 101, floorHull, floorFrame};

    auto cubePoints = createCubePoints(1.0);
    ConvexHull cubeHull{cubePoints};
    ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, 0.5}};
    Eigen::Quaterniond tilt{
      Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitY()}};
    cubeFrame.setQuaternion(tilt);
    AssetInertial cube{1, 2, cubeHull, 10.0, cubeFrame};

    auto result = handler.checkCollision(cube, floor);
    if (result.has_value())
    {
      for (size_t i = 0; i < result->contactCount; ++i)
      {
        EXPECT_GE(result->contacts[i].depth, 0.0)
          << "Config2 contact " << i << " has negative depth";
      }
    }
  }

  // Configuration 3: Two cubes colliding along x-axis
  {
    auto cubeAPoints = createCubePoints(1.0);
    ConvexHull cubeAHull{cubeAPoints};
    ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
    AssetInertial assetA{1, 3, cubeAHull, 10.0, frameA};

    auto cubeBPoints = createCubePoints(1.0);
    ConvexHull cubeBHull{cubeBPoints};
    ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};
    AssetInertial assetB{1, 4, cubeBHull, 10.0, frameB};

    auto result = handler.checkCollision(assetA, assetB);
    if (result.has_value())
    {
      for (size_t i = 0; i < result->contactCount; ++i)
      {
        EXPECT_GE(result->contacts[i].depth, 0.0)
          << "Config3 contact " << i << " has negative depth";
      }
    }
  }
}
