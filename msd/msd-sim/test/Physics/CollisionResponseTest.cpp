// Ticket: 0027_collision_response_system
// Refactored: Lagrangian mechanics with frictionless constraints
// Design: docs/designs/0027_collision_response_system/design.md

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "msd-sim/src/Environment/AngularCoordinate.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/CollisionResponse.hpp"
#include "msd-sim/src/Physics/CollisionResult.hpp"
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
// combineRestitution() Tests
// ============================================================================

TEST(CollisionResponseTest, combineRestitution_ZeroZero)
{
  // Fully inelastic: e(0, 0) → 0
  double result = CollisionResponse::combineRestitution(0.0, 0.0);
  EXPECT_DOUBLE_EQ(0.0, result);
}

TEST(CollisionResponseTest, combineRestitution_OneOne)
{
  // Fully elastic: e(1, 1) → 1
  double result = CollisionResponse::combineRestitution(1.0, 1.0);
  EXPECT_DOUBLE_EQ(1.0, result);
}

TEST(CollisionResponseTest, combineRestitution_ZeroOne)
{
  // Asymmetric inelastic: e(0, 1) → 0
  // Geometric mean: sqrt(0 * 1) = 0
  double result = CollisionResponse::combineRestitution(0.0, 1.0);
  EXPECT_DOUBLE_EQ(0.0, result);
}

TEST(CollisionResponseTest, combineRestitution_Symmetric)
{
  // Symmetry: e(A,B) = e(B,A)
  double resultAB = CollisionResponse::combineRestitution(0.3, 0.7);
  double resultBA = CollisionResponse::combineRestitution(0.7, 0.3);
  EXPECT_DOUBLE_EQ(resultAB, resultBA);

  // Also verify geometric mean formula
  double expected = std::sqrt(0.3 * 0.7);
  EXPECT_DOUBLE_EQ(expected, resultAB);
}

TEST(CollisionResponseTest, combineRestitution_HalfHalf)
{
  // Common case: e(0.5, 0.5) → 0.5
  double result = CollisionResponse::combineRestitution(0.5, 0.5);
  EXPECT_DOUBLE_EQ(0.5, result);
}

// ============================================================================
// computeLagrangeMultiplier() Tests (was computeImpulseMagnitude)
// ============================================================================

TEST(CollisionResponseTest, computeLagrangeMultiplier_SeparatingObjects)
{
  // Objects moving apart: constraint satisfied → λ = 0
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{1.0, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  // Set velocities: moving apart (A moving left, B moving right)
  assetA.getInertialState().velocity = Coordinate{-1.0, 0.0, 0.0};
  assetB.getInertialState().velocity = Coordinate{1.0, 0.0, 0.0};

  // Synthetic collision result (normal points from A to B along +x)
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},   // normal (A→B)
                         0.1,                         // penetration depth
                         Coordinate{0.5, 0.0, 0.0},   // contact point A
                         Coordinate{0.5, 0.0, 0.0}};  // contact point B

  double lambda =
    CollisionResponse::computeLagrangeMultiplier(assetA, assetB, result, 1.0);

  // vRel · n < 0 → constraint satisfied → λ = 0
  EXPECT_EQ(lambda, 0.0);
}

TEST(CollisionResponseTest, computeLagrangeMultiplier_HeadOnElastic)
{
  // Equal mass head-on collision with e=1.0
  // Lagrange multiplier = (1+e) * vRelNormal / effectiveMass
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  // Head-on collision: A moving right, B moving left
  assetA.getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};
  assetB.getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};

  // Collision result (normal points from A to B along x-axis)
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},    // normal (A→B)
                         0.1,                          // penetration depth
                         Coordinate{0.45, 0.0, 0.0},   // contact point A
                         Coordinate{0.45, 0.0, 0.0}};  // contact point B

  double lambda = CollisionResponse::computeLagrangeMultiplier(
    assetA, assetB, result, 1.0);  // Fully elastic

  // λ = (1+e) * vRelNormal / (1/mA + 1/mB + angular)
  // vRel = (2,0,0) - (-2,0,0) = (4,0,0)
  // vRelNormal = 4
  // effectiveMass ≈ 1/10 + 1/10 = 0.2 (angular terms small for center contact)
  // λ = 2 * 4 / 0.2 = 40
  EXPECT_NEAR(lambda, 40.0, 1e-10);
}

TEST(CollisionResponseTest, computeLagrangeMultiplier_StaticObjectsZeroLambda)
{
  // Both objects at rest → constraint satisfied → λ = 0
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  // Both at rest
  assetA.getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};
  assetB.getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};

  CollisionResult result{Coordinate{1.0, 0.0, 0.0},    // normal
                         0.1,                          // penetration
                         Coordinate{0.45, 0.0, 0.0},   // contact A
                         Coordinate{0.45, 0.0, 0.0}};  // contact B

  double lambda =
    CollisionResponse::computeLagrangeMultiplier(assetA, assetB, result, 1.0);

  // vRelNormal = 0 → not violating constraint → λ = 0
  EXPECT_DOUBLE_EQ(0.0, lambda);
}

// ============================================================================
// applyPositionStabilization() Tests (was applyPositionCorrection)
// ============================================================================

TEST(CollisionResponseTest, applyPositionStabilization_NoPenetration)
{
  // Penetration below slop threshold → no correction
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{1.0, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  Coordinate originalPosA = assetA.getInertialState().position;
  Coordinate originalPosB = assetB.getInertialState().position;

  // Penetration below slop (0.01m)
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},
                         0.005,  // < kSlop
                         Coordinate{0.5, 0.0, 0.0},
                         Coordinate{0.5, 0.0, 0.0}};

  CollisionResponse::applyPositionStabilization(assetA, assetB, result);

  // Positions should not change
  EXPECT_DOUBLE_EQ(originalPosA.x(), assetA.getInertialState().position.x());
  EXPECT_DOUBLE_EQ(originalPosB.x(), assetB.getInertialState().position.x());
}

TEST(CollisionResponseTest, applyPositionStabilization_DeepPenetration)
{
  // Significant penetration → Baumgarte stabilization applied
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.8, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  // Sync initial positions
  assetA.getInertialState().position = frameA.getOrigin();
  assetB.getInertialState().position = frameB.getOrigin();

  // Deep penetration (0.2m > slop)
  double penetration = 0.2;
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},
                         penetration,
                         Coordinate{0.4, 0.0, 0.0},
                         Coordinate{0.4, 0.0, 0.0}};

  CollisionResponse::applyPositionStabilization(assetA, assetB, result);

  // Expected correction: max(0.2 - 0.01, 0) * 0.8 = 0.19 * 0.8 = 0.152
  double expectedCorrection = (penetration - CollisionResponse::kSlop) *
                              CollisionResponse::kCorrectionFactor;

  // Equal mass → each moves half the correction
  Coordinate expectedPosA =
    Coordinate{0.0, 0.0, 0.0} -
    Coordinate{1.0, 0.0, 0.0} * (expectedCorrection * 0.5);
  Coordinate expectedPosB =
    Coordinate{0.8, 0.0, 0.0} +
    Coordinate{1.0, 0.0, 0.0} * (expectedCorrection * 0.5);

  EXPECT_NEAR(expectedPosA.x(), assetA.getInertialState().position.x(), 1e-10);
  EXPECT_NEAR(expectedPosB.x(), assetB.getInertialState().position.x(), 1e-10);
}

TEST(CollisionResponseTest, applyPositionStabilization_MassWeighting)
{
  // Heavier object should move less (mass-weighted correction)
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  // Different masses: A=30kg (heavy), B=10kg (light)
  AssetInertial assetA{0, 0, hull, 30.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  assetA.getInertialState().position = frameA.getOrigin();
  assetB.getInertialState().position = frameB.getOrigin();

  Coordinate originalPosA = assetA.getInertialState().position;
  Coordinate originalPosB = assetB.getInertialState().position;

  double penetration = 0.1;
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},
                         penetration,
                         Coordinate{0.45, 0.0, 0.0},
                         Coordinate{0.45, 0.0, 0.0}};

  CollisionResponse::applyPositionStabilization(assetA, assetB, result);

  // Mass weighting:
  // invMassSum = 1/30 + 1/10 = 4/30
  // weightA = (1/30) / (4/30) = 0.25
  // weightB = (1/10) / (4/30) = 0.75

  double correction = (penetration - CollisionResponse::kSlop) *
                      CollisionResponse::kCorrectionFactor;

  double deltaA = (assetA.getInertialState().position - originalPosA).norm();
  double deltaB = (assetB.getInertialState().position - originalPosB).norm();

  // Heavier A should move less than lighter B
  EXPECT_LT(deltaA, deltaB);

  // Verify exact weighting
  double expectedDeltaA = correction * 0.25;
  double expectedDeltaB = correction * 0.75;

  EXPECT_NEAR(expectedDeltaA, deltaA, 1e-10);
  EXPECT_NEAR(expectedDeltaB, deltaB, 1e-10);
}

// ============================================================================
// Dynamic-Static Collision Tests (Inertial vs Environment)
// ============================================================================

TEST(CollisionResponseStaticTest, computeLagrangeMultiplierStatic_SeparatingObject)
{
  // Dynamic object moving away from static → constraint satisfied → λ = 0
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{1.0, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  // Dynamic moving away from static (in -x direction)
  dynamic.getInertialState().velocity = Coordinate{-1.0, 0.0, 0.0};

  // Collision result: normal points from dynamic toward static (+x)
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},  // normal
                         0.1,                         // penetration
                         Coordinate{0.5, 0.0, 0.0},   // contact point A
                         Coordinate{0.5, 0.0, 0.0}};  // contact point B

  double lambda = CollisionResponse::computeLagrangeMultiplierStatic(
      dynamic, staticObj, result, 1.0);

  // vRel · n < 0 → separating → λ = 0
  EXPECT_EQ(lambda, 0.0);
}

TEST(CollisionResponseStaticTest, computeLagrangeMultiplierStatic_ApproachingObject)
{
  // Dynamic object approaching static → λ > 0
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.9, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  // Dynamic moving toward static (in +x direction)
  dynamic.getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};

  // Collision result: normal points from dynamic toward static (+x)
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},   // normal
                         0.1,                          // penetration
                         Coordinate{0.45, 0.0, 0.0},   // contact point A
                         Coordinate{0.45, 0.0, 0.0}};  // contact point B

  double lambda = CollisionResponse::computeLagrangeMultiplierStatic(
      dynamic, staticObj, result, 1.0);  // Fully elastic

  // Should have positive λ
  EXPECT_GT(lambda, 0.0);

  // λ = (1 + e) * vRelNormal / (1/m + angular_term)
  // With contact at center, angular_term ≈ 0
  // λ = 2.0 * 2.0 / 0.1 = 40.0
  EXPECT_NEAR(lambda, 40.0, 0.1);
}

TEST(CollisionResponseStaticTest, computeLagrangeMultiplierStatic_InelasticCollision)
{
  // Inelastic collision: λ_inelastic = λ_elastic / 2
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.9, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};

  CollisionResult result{Coordinate{1.0, 0.0, 0.0},
                         0.1,
                         Coordinate{0.45, 0.0, 0.0},
                         Coordinate{0.45, 0.0, 0.0}};

  // Fully inelastic (e=0)
  double lambdaInelastic = CollisionResponse::computeLagrangeMultiplierStatic(
      dynamic, staticObj, result, 0.0);

  // Fully elastic (e=1)
  double lambdaElastic = CollisionResponse::computeLagrangeMultiplierStatic(
      dynamic, staticObj, result, 1.0);

  // λ_elastic = (1+1) * v / m_eff = 2 * λ_inelastic
  EXPECT_NEAR(lambdaElastic, 2.0 * lambdaInelastic, 0.01);
}

TEST(CollisionResponseStaticTest, applyPositionStabilizationStatic_NoPenetration)
{
  // Penetration below slop threshold → no correction
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{1.0, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().position = frameDynamic.getOrigin();
  Coordinate originalPos = dynamic.getInertialState().position;

  // Penetration below slop (0.01m)
  CollisionResult result{Coordinate{1.0, 0.0, 0.0}, 0.005,  // < kSlop
                         Coordinate{0.5, 0.0, 0.0}, Coordinate{0.5, 0.0, 0.0}};

  CollisionResponse::applyPositionStabilizationStatic(dynamic, staticObj, result);

  // Position should not change
  EXPECT_DOUBLE_EQ(originalPos.x(), dynamic.getInertialState().position.x());
}

TEST(CollisionResponseStaticTest, applyPositionStabilizationStatic_FullCorrectionToDynamic)
{
  // All correction applied to dynamic object (static has infinite mass)
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().position = frameDynamic.getOrigin();

  // Deep penetration
  double penetration = 0.2;
  CollisionResult result{Coordinate{1.0, 0.0, 0.0}, penetration,
                         Coordinate{0.4, 0.0, 0.0}, Coordinate{0.4, 0.0, 0.0}};

  CollisionResponse::applyPositionStabilizationStatic(dynamic, staticObj, result);

  // Expected correction: (0.2 - 0.01) * 0.8 = 0.152
  double expectedCorrection = (penetration - CollisionResponse::kSlop) *
                              CollisionResponse::kCorrectionFactor;

  // Dynamic should move full correction in -x direction
  Coordinate expectedPos = Coordinate{0.0, 0.0, 0.0} -
                           Coordinate{1.0, 0.0, 0.0} * expectedCorrection;

  EXPECT_NEAR(expectedPos.x(), dynamic.getInertialState().position.x(), 1e-10);
}

TEST(CollisionResponseStaticTest, kEnvironmentRestitution_DefaultValue)
{
  // Verify default environment restitution constant
  EXPECT_DOUBLE_EQ(0.5, CollisionResponse::kEnvironmentRestitution);
}

// ========== Lagrangian Constraint Response Tests (Frictionless) ==========

TEST(CollisionResponseStaticConstraintTest, applyConstraintResponseStatic_SingleContact)
{
  // Single contact: frictionless constraint response
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic, 1.0};  // elastic
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().position = frameDynamic.getOrigin();
  dynamic.getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};  // Moving toward static
  Coordinate originalVel = dynamic.getInertialState().velocity;

  // Single contact manifold
  CollisionResult result;
  result.normal = Coordinate{1.0, 0.0, 0.0};  // Dynamic toward static
  result.penetrationDepth = 0.1;
  result.contactCount = 1;
  result.contacts[0] = ContactPoint{Coordinate{0.5, 0.0, 0.0}, Coordinate{0.5, 0.0, 0.0}};

  double restitution = 1.0;  // Elastic
  CollisionResponse::applyConstraintResponseStatic(dynamic, staticObj, result, restitution);

  // Velocity should change (constraint force applied)
  EXPECT_NE(originalVel.x(), dynamic.getInertialState().velocity.x());

  // For elastic collision, normal velocity should reverse sign
  EXPECT_LT(dynamic.getInertialState().velocity.x(), 0.0);
}

TEST(CollisionResponseStaticConstraintTest, applyConstraintResponseStatic_MultipleContacts)
{
  // Multiple contacts: constraint force distributed across manifold
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic, 1.0};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().position = frameDynamic.getOrigin();
  double initialVx = 2.0;
  dynamic.getInertialState().velocity = Coordinate{initialVx, 0.0, 0.0};
  AngularRate originalAngularVel = dynamic.getInertialState().getAngularVelocity();

  // 3-contact manifold (face-face collision)
  CollisionResult result;
  result.normal = Coordinate{1.0, 0.0, 0.0};
  result.penetrationDepth = 0.1;
  result.contactCount = 3;
  // Symmetric contacts to minimize angular effects
  result.contacts[0] = ContactPoint{Coordinate{0.5, 0.0, 0.5}, Coordinate{0.5, 0.0, 0.5}};
  result.contacts[1] = ContactPoint{Coordinate{0.5, 0.0, 0.0}, Coordinate{0.5, 0.0, 0.0}};
  result.contacts[2] = ContactPoint{Coordinate{0.5, 0.0, -0.5}, Coordinate{0.5, 0.0, -0.5}};

  double restitution = 1.0;
  CollisionResponse::applyConstraintResponseStatic(dynamic, staticObj, result, restitution);

  // Linear velocity should decrease (constraint force in -x direction)
  EXPECT_LT(dynamic.getInertialState().velocity.x(), initialVx);

  // With symmetric contacts, net angular impulse about Y axis should be ~zero
  double angularChangeY = std::abs(dynamic.getInertialState().getAngularVelocity().y() -
                                   originalAngularVel.y());
  EXPECT_LT(angularChangeY, 1e-10);
}

TEST(CollisionResponseStaticConstraintTest, applyConstraintResponseStatic_SeparatingNoForce)
{
  // Objects separating: constraint satisfied → no force applied
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic, 1.0};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().position = frameDynamic.getOrigin();
  dynamic.getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};  // Moving away
  Coordinate originalVel = dynamic.getInertialState().velocity;

  CollisionResult result;
  result.normal = Coordinate{1.0, 0.0, 0.0};  // Points from dynamic toward static
  result.penetrationDepth = 0.1;
  result.contactCount = 1;
  result.contacts[0] = ContactPoint{Coordinate{0.5, 0.0, 0.0}, Coordinate{0.5, 0.0, 0.0}};

  double restitution = 1.0;
  CollisionResponse::applyConstraintResponseStatic(dynamic, staticObj, result, restitution);

  // Velocity should not change (constraint already satisfied)
  EXPECT_DOUBLE_EQ(originalVel.x(), dynamic.getInertialState().velocity.x());
}

// ============================================================================
// applyConstraintResponse() Tests (Dynamic-Dynamic)
// ============================================================================

TEST(CollisionResponseConstraintTest, applyConstraintResponse_HeadOnFrictionless)
{
  // Head-on collision: frictionless constraint response
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  // Head-on collision: A moving right, B moving left
  assetA.getInertialState().velocity = Coordinate{2.0, 0.0, 0.0};
  assetB.getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};

  Coordinate originalVelA = assetA.getInertialState().velocity;
  Coordinate originalVelB = assetB.getInertialState().velocity;

  CollisionResult result;
  result.normal = Coordinate{1.0, 0.0, 0.0};  // A→B
  result.penetrationDepth = 0.1;
  result.contactCount = 1;
  result.contacts[0] = ContactPoint{Coordinate{0.45, 0.0, 0.0}, Coordinate{0.45, 0.0, 0.0}};

  CollisionResponse::applyConstraintResponse(assetA, assetB, result, 1.0);  // elastic

  // Velocities should have exchanged (equal mass elastic collision)
  // A was going right (+2), now should go left (~-2)
  // B was going left (-2), now should go right (~+2)
  EXPECT_NEAR(assetA.getInertialState().velocity.x(), originalVelB.x(), 0.1);
  EXPECT_NEAR(assetB.getInertialState().velocity.x(), originalVelA.x(), 0.1);
}

TEST(CollisionResponseConstraintTest, applyConstraintResponse_FrictionlessSliding)
{
  // Tangential velocity preserved (frictionless)
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{0.9, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  // A moving diagonally: normal (+x) and tangential (+y) components
  assetA.getInertialState().velocity = Coordinate{2.0, 3.0, 0.0};
  assetB.getInertialState().velocity = Coordinate{0.0, 0.0, 0.0};

  double originalTangentVelA = assetA.getInertialState().velocity.y();

  CollisionResult result;
  result.normal = Coordinate{1.0, 0.0, 0.0};  // A→B along x
  result.penetrationDepth = 0.1;
  result.contactCount = 1;
  result.contacts[0] = ContactPoint{Coordinate{0.45, 0.0, 0.0}, Coordinate{0.45, 0.0, 0.0}};

  CollisionResponse::applyConstraintResponse(assetA, assetB, result, 1.0);

  // Normal velocity should change
  EXPECT_NE(2.0, assetA.getInertialState().velocity.x());

  // Tangential velocity should be preserved (frictionless!)
  // Without friction, only the normal component is affected
  double totalTangentMomentum = assetA.getMass() * assetA.getInertialState().velocity.y() +
                                assetB.getMass() * assetB.getInertialState().velocity.y();
  double originalTangentMomentum = assetA.getMass() * originalTangentVelA;

  EXPECT_NEAR(originalTangentMomentum, totalTangentMomentum, 1e-10);
}
