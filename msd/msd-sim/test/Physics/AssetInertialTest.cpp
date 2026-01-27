// Ticket: 0027_collision_response_system
// Design: docs/designs/0027_collision_response_system/design.md

#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

// Create a simple cube as a point cloud
std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate(-half, -half, -half), Coordinate(half, -half, -half),
          Coordinate(half, half, -half),   Coordinate(-half, half, -half),
          Coordinate(-half, -half, half),  Coordinate(half, -half, half),
          Coordinate(half, half, half),    Coordinate(-half, half, half)};
}

}  // anonymous namespace

// ============================================================================
// Coefficient of Restitution Tests
// ============================================================================

TEST(AssetInertialTest, getCoefficientOfRestitution_Default)
{
  // Default construction should give e = 0.5
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  EXPECT_DOUBLE_EQ(0.5, asset.getCoefficientOfRestitution());
}

TEST(AssetInertialTest, setCoefficientOfRestitution_Valid)
{
  // Setting valid values [0, 1] should succeed
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Test boundary values
  asset.setCoefficientOfRestitution(0.0);
  EXPECT_DOUBLE_EQ(0.0, asset.getCoefficientOfRestitution());

  asset.setCoefficientOfRestitution(1.0);
  EXPECT_DOUBLE_EQ(1.0, asset.getCoefficientOfRestitution());

  // Test intermediate value
  asset.setCoefficientOfRestitution(0.75);
  EXPECT_DOUBLE_EQ(0.75, asset.getCoefficientOfRestitution());
}

TEST(AssetInertialTest, setCoefficientOfRestitution_InvalidLow)
{
  // Setting e < 0 should throw
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  EXPECT_THROW(asset.setCoefficientOfRestitution(-0.1), std::invalid_argument);
  EXPECT_THROW(asset.setCoefficientOfRestitution(-1.0), std::invalid_argument);

  // Original value should be unchanged after failed set
  EXPECT_DOUBLE_EQ(0.5, asset.getCoefficientOfRestitution());
}

TEST(AssetInertialTest, setCoefficientOfRestitution_InvalidHigh)
{
  // Setting e > 1 should throw
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  EXPECT_THROW(asset.setCoefficientOfRestitution(1.1), std::invalid_argument);
  EXPECT_THROW(asset.setCoefficientOfRestitution(2.0), std::invalid_argument);

  // Original value should be unchanged after failed set
  EXPECT_DOUBLE_EQ(0.5, asset.getCoefficientOfRestitution());
}

TEST(AssetInertialTest, Constructor_WithRestitution)
{
  // Extended constructor with custom restitution
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame, 0.8};

  EXPECT_DOUBLE_EQ(0.8, asset.getCoefficientOfRestitution());
}

TEST(AssetInertialTest, Constructor_WithRestitution_InvalidValue)
{
  // Constructor should validate restitution
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  // e < 0
  EXPECT_THROW((AssetInertial{0, 0, hull, 10.0, frame, -0.1}),
               std::invalid_argument);

  // e > 1
  EXPECT_THROW((AssetInertial{0, 0, hull, 10.0, frame, 1.5}),
               std::invalid_argument);
}

TEST(AssetInertialTest, Constructor_RestitutionBoundary)
{
  // Boundary values should be accepted
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial assetInelastic{0, 0, hull, 10.0, frame, 0.0};
  EXPECT_DOUBLE_EQ(0.0, assetInelastic.getCoefficientOfRestitution());

  AssetInertial assetElastic{0, 1, hull, 10.0, frame, 1.0};
  EXPECT_DOUBLE_EQ(1.0, assetElastic.getCoefficientOfRestitution());
}
