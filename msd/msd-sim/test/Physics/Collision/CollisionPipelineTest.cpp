// Ticket: 0036_collision_pipeline_extraction
// Ticket: 0062e_replay_diagnostic_parameter_tests
// Design: docs/designs/0036_collision_pipeline_extraction/design.md
//
// NOTE: All tests in this file are single-step collision pipeline tests
// (no multi-frame simulation), so they remain as TEST() rather than TEST_F.

#include <gtest/gtest.h>

#include <span>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ============================================================================
// Helpers
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
// CollisionPipeline Unit Tests (AC5: exercise pipeline in isolation)
// ============================================================================

TEST(CollisionPipelineTest, execute_EmptyScene_NoError)
{
  CollisionPipeline pipeline{};

  std::vector<AssetInertial> inertials;
  std::vector<AssetEnvironment> environments;

  EXPECT_NO_THROW(
    pipeline.execute(std::span{inertials},
                     std::span<const AssetEnvironment>{environments},
                     0.016));
}

TEST(CollisionPipelineTest, execute_ZeroDt_EarlyReturn)
{
  CollisionPipeline pipeline{};

  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  std::vector<AssetInertial> inertials;
  inertials.emplace_back(0, 1, hull, 10.0, frame);

  std::vector<AssetEnvironment> environments;

  EXPECT_NO_THROW(
    pipeline.execute(std::span{inertials},
                     std::span<const AssetEnvironment>{environments},
                     0.0));
}

TEST(CollisionPipelineTest, execute_SeparatedObjects_NoForceApplied)
{
  CollisionPipeline pipeline{};

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{10.0, 0.0, 0.0}};  // Far apart

  std::vector<AssetInertial> inertials;
  inertials.emplace_back(0, 1, hullA, 10.0, frameA);
  inertials.emplace_back(1, 2, hullB, 10.0, frameB);

  std::vector<AssetEnvironment> environments;

  // Store initial velocities
  inertials[0].getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};
  inertials[1].getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};

  pipeline.execute(std::span{inertials},
                   std::span<const AssetEnvironment>{environments},
                   0.016);

  // No forces should have been applied (no collision)
  EXPECT_DOUBLE_EQ(0.0, inertials[0].getAccumulatedForce().norm());
  EXPECT_DOUBLE_EQ(0.0, inertials[1].getAccumulatedForce().norm());
}

TEST(CollisionPipelineTest, execute_OverlappingObjects_ForcesApplied)
{
  CollisionPipeline pipeline{};

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};  // Overlapping

  std::vector<AssetInertial> inertials;
  inertials.emplace_back(0, 1, hullA, 10.0, frameA);
  inertials.emplace_back(1, 2, hullB, 10.0, frameB);

  // Set approaching velocities
  inertials[0].setCoefficientOfRestitution(1.0);
  inertials[1].setCoefficientOfRestitution(1.0);
  inertials[0].getInertialState().velocity = Coordinate{1.0, 0.0, 0.0};
  inertials[1].getInertialState().velocity = Coordinate{-1.0, 0.0, 0.0};

  std::vector<AssetEnvironment> environments;

  pipeline.execute(std::span{inertials},
                   std::span<const AssetEnvironment>{environments},
                   0.016);

  // At least one object should have accumulated force (collision response)
  double totalForce = inertials[0].getAccumulatedForce().norm() +
                      inertials[1].getAccumulatedForce().norm();
  EXPECT_GT(totalForce, 0.0);
}

TEST(CollisionPipelineTest, execute_InertialVsEnvironment_OnlyInertialGetsForce)
{
  CollisionPipeline pipeline{};

  auto points = createCubePoints(1.0);
  ConvexHull hullDynamic{points};
  ConvexHull hullStatic{points};

  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};  // Overlapping

  std::vector<AssetInertial> inertials;
  inertials.emplace_back(0, 1, hullDynamic, 10.0, frameDynamic);
  inertials[0].setCoefficientOfRestitution(1.0);
  inertials[0].getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};

  std::vector<AssetEnvironment> environments;
  environments.emplace_back(1, 2, hullStatic, frameStatic);

  pipeline.execute(std::span{inertials},
                   std::span<const AssetEnvironment>{environments},
                   0.016);

  // Inertial object should have accumulated force from collision
  EXPECT_GT(inertials[0].getAccumulatedForce().norm(), 0.0);
}

TEST(CollisionPipelineTest, execute_MultipleCalls_NoMemoryIssues)
{
  CollisionPipeline pipeline{};

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};

  std::vector<AssetInertial> inertials;
  inertials.emplace_back(0, 1, hullA, 10.0, frameA);
  inertials.emplace_back(1, 2, hullB, 10.0, frameB);

  inertials[0].setCoefficientOfRestitution(0.5);
  inertials[1].setCoefficientOfRestitution(0.5);

  std::vector<AssetEnvironment> environments;

  // Execute pipeline multiple times (simulating multiple frames)
  for (int i = 0; i < 5; ++i)
  {
    inertials[0].clearForces();
    inertials[1].clearForces();

    EXPECT_NO_THROW(
      pipeline.execute(std::span{inertials},
                       std::span<const AssetEnvironment>{environments},
                       0.016));
  }
}

TEST(CollisionPipelineTest, execute_MomentumConservation_InertialVsInertial)
{
  CollisionPipeline pipeline{};

  auto points = createCubePoints(1.0);
  ConvexHull hullA{points};
  ConvexHull hullB{points};

  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  double mass = 10.0;

  std::vector<AssetInertial> inertials;
  inertials.emplace_back(0, 1, hullA, mass, frameA);
  inertials.emplace_back(1, 2, hullB, mass, frameB);

  inertials[0].setCoefficientOfRestitution(1.0);
  inertials[1].setCoefficientOfRestitution(1.0);
  inertials[0].getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};
  inertials[1].getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};

  std::vector<AssetEnvironment> environments;

  // Total force should sum to approximately zero (Newton's third law)
  pipeline.execute(std::span{inertials},
                   std::span<const AssetEnvironment>{environments},
                   0.016);

  // Forces should be equal and opposite (momentum conservation)
  Coordinate totalForce =
    inertials[0].getAccumulatedForce() + inertials[1].getAccumulatedForce();

  EXPECT_NEAR(0.0, totalForce.x(), 1e-6);
  EXPECT_NEAR(0.0, totalForce.y(), 1e-6);
  EXPECT_NEAR(0.0, totalForce.z(), 1e-6);
}
