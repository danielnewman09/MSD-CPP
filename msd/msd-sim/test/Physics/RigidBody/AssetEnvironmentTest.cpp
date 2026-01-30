// Ticket: 0032a_two_body_constraint_infrastructure
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"

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
  return {Coordinate(-half, -half, -half),
          Coordinate(half, -half, -half),
          Coordinate(half, half, -half),
          Coordinate(-half, half, -half),
          Coordinate(-half, -half, half),
          Coordinate(half, -half, half),
          Coordinate(half, half, half),
          Coordinate(-half, half, half)};
}

}  // anonymous namespace

// ============================================================================
// AssetEnvironment Tests
// ============================================================================

TEST(AssetEnvironmentTest, GetInverseMass_ReturnsZero_0032a)
{
  // Test: getInverseMass() returns 0.0 for infinite mass
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  AssetEnvironment environment{1, 1, hull, frame};

  EXPECT_DOUBLE_EQ(0.0, environment.getInverseMass());
}

TEST(AssetEnvironmentTest, GetInverseInertiaTensor_ReturnsZeroMatrix_0032a)
{
  // Test: getInverseInertiaTensor() returns zero matrix for infinite inertia
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  AssetEnvironment environment{1, 1, hull, frame};

  const Eigen::Matrix3d& inverseInertia = environment.getInverseInertiaTensor();

  EXPECT_DOUBLE_EQ(0.0, inverseInertia(0, 0));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(0, 1));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(0, 2));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(1, 0));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(1, 1));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(1, 2));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(2, 0));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(2, 1));
  EXPECT_DOUBLE_EQ(0.0, inverseInertia(2, 2));
}

TEST(AssetEnvironmentTest, GetInertialState_ReturnsZeroVelocity_0032a)
{
  // Test: getInertialState() returns static state with zero velocity
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{1, 2, 3}};

  AssetEnvironment environment{1, 1, hull, frame};

  const InertialState& state = environment.getInertialState();

  // Velocity should be zero
  EXPECT_DOUBLE_EQ(0.0, state.velocity.x());
  EXPECT_DOUBLE_EQ(0.0, state.velocity.y());
  EXPECT_DOUBLE_EQ(0.0, state.velocity.z());

  // Angular velocity should be zero
  Coordinate angularVel = state.getAngularVelocity();
  EXPECT_DOUBLE_EQ(0.0, angularVel.x());
  EXPECT_DOUBLE_EQ(0.0, angularVel.y());
  EXPECT_DOUBLE_EQ(0.0, angularVel.z());

  // Position should match frame
  EXPECT_DOUBLE_EQ(1.0, state.position.x());
  EXPECT_DOUBLE_EQ(2.0, state.position.y());
  EXPECT_DOUBLE_EQ(3.0, state.position.z());
}

TEST(AssetEnvironmentTest, GetCoefficientOfRestitution_DefaultValue_0032a)
{
  // Test: getCoefficientOfRestitution() returns default value (0.5)
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  AssetEnvironment environment{1, 1, hull, frame};

  EXPECT_DOUBLE_EQ(0.5, environment.getCoefficientOfRestitution());
}

TEST(AssetEnvironmentTest, SetCoefficientOfRestitution_UpdatesValue_0032a)
{
  // Test: setCoefficientOfRestitution() updates the value
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  AssetEnvironment environment{1, 1, hull, frame};

  environment.setCoefficientOfRestitution(0.8);
  EXPECT_DOUBLE_EQ(0.8, environment.getCoefficientOfRestitution());
}

TEST(AssetEnvironmentTest, SetCoefficientOfRestitution_ValidatesRange_0032a)
{
  // Test: setCoefficientOfRestitution() validates range [0, 1]
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  AssetEnvironment environment{1, 1, hull, frame};

  // Valid values should not throw
  EXPECT_NO_THROW(environment.setCoefficientOfRestitution(0.0));
  EXPECT_NO_THROW(environment.setCoefficientOfRestitution(1.0));
  EXPECT_NO_THROW(environment.setCoefficientOfRestitution(0.5));

  // Invalid values should throw
  EXPECT_THROW(environment.setCoefficientOfRestitution(-0.1), std::invalid_argument);
  EXPECT_THROW(environment.setCoefficientOfRestitution(1.1), std::invalid_argument);
}

TEST(AssetEnvironmentTest, Constructor_WithRestitution_SetsValue_0032a)
{
  // Test: Constructor with restitution parameter sets the value
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  AssetEnvironment environment{1, 1, hull, frame, 0.7};

  EXPECT_DOUBLE_EQ(0.7, environment.getCoefficientOfRestitution());
}
