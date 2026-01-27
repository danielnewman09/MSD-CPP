// Ticket: 0027_collision_response_system
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
// computeImpulseMagnitude() Tests
// ============================================================================

TEST(CollisionResponseTest, computeImpulseMagnitude_SeparatingObjects)
{
  // Objects moving apart should have zero impulse
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameA{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameB{Coordinate{1.0, 0.0, 0.0}};

  AssetInertial assetA{0, 0, hull, 10.0, frameA};
  AssetInertial assetB{0, 1, hull, 10.0, frameB};

  // Set velocities: moving apart (A moving left, B moving right)
  // A is at origin, B is to the right at x=1
  assetA.getInertialState().velocity = Coordinate{-1.0, 0.0, 0.0};
  assetB.getInertialState().velocity = Coordinate{1.0, 0.0, 0.0};

  // Synthetic collision result (normal points from A to B along +x)
  CollisionResult result{Coordinate{1.0, 0.0, 0.0},   // normal (A→B)
                         0.1,                         // penetration depth
                         Coordinate{0.5, 0.0, 0.0},   // contact point A
                         Coordinate{0.5, 0.0, 0.0}};  // contact point B

  double impulse =
    CollisionResponse::computeImpulseMagnitude(assetA, assetB, result, 1.0);

  // vRel = vA - vB = (-1, 0, 0) - (1, 0, 0) = (-2, 0, 0)
  // vRelNormal = vRel · n = -2 < 0 → objects separating → impulse = 0
  EXPECT_EQ(impulse, 0.0);
}

TEST(CollisionResponseTest, computeImpulseMagnitude_HeadOnElastic)
{
  // Equal mass head-on collision with e=1.0 should produce specific impulse
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

  double impulse = CollisionResponse::computeImpulseMagnitude(
    assetA, assetB, result, 1.0);  // Fully elastic

  EXPECT_NEAR(impulse, 40.0, 1e-10);
}

TEST(CollisionResponseTest, computeImpulseMagnitude_StaticObjectsNoImpulse)
{
  // Both objects at rest → no impulse needed
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

  double impulse =
    CollisionResponse::computeImpulseMagnitude(assetA, assetB, result, 1.0);

  // At rest: vRel = 0, vRelNormal = 0
  // This triggers the separating check (vRelNormal > 0? Actually = 0)
  // Formula: j = -(1 + e) * 0 / denom = 0
  EXPECT_DOUBLE_EQ(0.0, impulse);
}

// ============================================================================
// applyPositionCorrection() Tests
// ============================================================================

TEST(CollisionResponseTest, applyPositionCorrection_NoPenetration)
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

  CollisionResponse::applyPositionCorrection(assetA, assetB, result);

  // Positions should not change
  EXPECT_DOUBLE_EQ(originalPosA.x(), assetA.getInertialState().position.x());
  EXPECT_DOUBLE_EQ(originalPosB.x(), assetB.getInertialState().position.x());
}

TEST(CollisionResponseTest, applyPositionCorrection_DeepPenetration)
{
  // Significant penetration → objects separated
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

  CollisionResponse::applyPositionCorrection(assetA, assetB, result);

  // Expected correction: max(0.2 - 0.01, 0) * 0.8 = 0.19 * 0.8 = 0.152
  double expectedCorrection = (penetration - CollisionResponse::kSlop) *
                              CollisionResponse::kCorrectionFactor;

  // Equal mass → each moves half the correction
  // A moves in -normal direction, B moves in +normal direction
  Coordinate expectedPosA =
    Coordinate{0.0, 0.0, 0.0} -
    Coordinate{1.0, 0.0, 0.0} * (expectedCorrection * 0.5);
  Coordinate expectedPosB =
    Coordinate{0.8, 0.0, 0.0} +
    Coordinate{1.0, 0.0, 0.0} * (expectedCorrection * 0.5);

  EXPECT_NEAR(expectedPosA.x(), assetA.getInertialState().position.x(), 1e-10);
  EXPECT_NEAR(expectedPosB.x(), assetB.getInertialState().position.x(), 1e-10);
}

TEST(CollisionResponseTest, applyPositionCorrection_MassWeighting)
{
  // Heavier object should move less than lighter object
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

  CollisionResponse::applyPositionCorrection(assetA, assetB, result);

  // Mass weighting:
  // invMassSum = 1/30 + 1/10 = 1/30 + 3/30 = 4/30
  // weightA = (1/30) / (4/30) = 1/4 = 0.25
  // weightB = (1/10) / (4/30) = 3/4 = 0.75

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

TEST(CollisionResponseStaticTest, computeImpulseMagnitudeStatic_SeparatingObject)
{
  // Dynamic object moving away from static should have zero impulse
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

  double impulse = CollisionResponse::computeImpulseMagnitudeStatic(
      dynamic, staticObj, result, 1.0);

  // vRel · n = -1.0 < 0 → separating → no impulse
  EXPECT_EQ(impulse, 0.0);
}

TEST(CollisionResponseStaticTest, computeImpulseMagnitudeStatic_ApproachingObject)
{
  // Dynamic object approaching static should receive impulse
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

  double impulse = CollisionResponse::computeImpulseMagnitudeStatic(
      dynamic, staticObj, result, 1.0);  // Fully elastic

  // Should have positive impulse
  EXPECT_GT(impulse, 0.0);

  // For elastic collision with static wall:
  // j = (1 + e) * vRelNormal / (1/m + angular_term)
  // With contact at center, angular_term ≈ 0
  // j = 2.0 * 2.0 / 0.1 = 40.0
  EXPECT_NEAR(impulse, 40.0, 0.1);
}

TEST(CollisionResponseStaticTest, computeImpulseMagnitudeStatic_InelasticCollision)
{
  // Inelastic collision with static wall
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
  double impulseInelastic = CollisionResponse::computeImpulseMagnitudeStatic(
      dynamic, staticObj, result, 0.0);

  // Fully elastic (e=1)
  double impulseElastic = CollisionResponse::computeImpulseMagnitudeStatic(
      dynamic, staticObj, result, 1.0);

  // Elastic impulse should be twice inelastic
  // Inelastic: j = 1 * vRelNormal / denom
  // Elastic: j = 2 * vRelNormal / denom
  EXPECT_NEAR(impulseElastic, 2.0 * impulseInelastic, 0.01);
}

TEST(CollisionResponseStaticTest, applyPositionCorrectionStatic_NoPenetration)
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

  CollisionResponse::applyPositionCorrectionStatic(dynamic, staticObj, result);

  // Position should not change
  EXPECT_DOUBLE_EQ(originalPos.x(), dynamic.getInertialState().position.x());
}

TEST(CollisionResponseStaticTest, applyPositionCorrectionStatic_FullCorrectionToDynamic)
{
  // All correction should be applied to dynamic object (static is immovable)
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

  CollisionResponse::applyPositionCorrectionStatic(dynamic, staticObj, result);

  // Expected correction: (0.2 - 0.01) * 0.8 = 0.152
  double expectedCorrection = (penetration - CollisionResponse::kSlop) *
                              CollisionResponse::kCorrectionFactor;

  // Dynamic should move full correction in -x direction (away from static)
  Coordinate expectedPos = Coordinate{0.0, 0.0, 0.0} -
                           Coordinate{1.0, 0.0, 0.0} * expectedCorrection;

  EXPECT_NEAR(expectedPos.x(), dynamic.getInertialState().position.x(), 1e-10);
}

TEST(CollisionResponseStaticTest, kEnvironmentRestitution_DefaultValue)
{
  // Verify default environment restitution constant
  EXPECT_DOUBLE_EQ(0.5, CollisionResponse::kEnvironmentRestitution);
}

// ========== Manifold-Aware Static Collision Tests (Ticket: 0029) ==========

TEST(CollisionResponseStaticManifoldTest, applyImpulseManifoldStatic_SingleContact)
{
  // Single contact should behave same as non-manifold version
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
  CollisionResponse::applyImpulseManifoldStatic(dynamic, staticObj, result, restitution);

  // Velocity should change (impulse applied)
  EXPECT_NE(originalVel.x(), dynamic.getInertialState().velocity.x());

  // For elastic collision, velocity should reverse sign (approximately)
  EXPECT_LT(dynamic.getInertialState().velocity.x(), 0.0);
}

TEST(CollisionResponseStaticManifoldTest, applyImpulseManifoldStatic_MultipleContacts)
{
  // Multiple contacts should distribute impulse and apply angular effects
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic, 1.0};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().position = frameDynamic.getOrigin();
  double initialVx = 2.0;
  dynamic.getInertialState().velocity = Coordinate{initialVx, 0.0, 0.0};
  AngularRate originalAngularVel = dynamic.getInertialState().angularVelocity;

  // 3-contact manifold (face-face collision)
  // Use contacts at center position to minimize angular effects on impulse magnitude
  CollisionResult result;
  result.normal = Coordinate{1.0, 0.0, 0.0};
  result.penetrationDepth = 0.1;
  result.contactCount = 3;
  // Contacts at different vertical positions to generate opposing torques
  result.contacts[0] = ContactPoint{Coordinate{0.5, 0.0, 0.5}, Coordinate{0.5, 0.0, 0.5}};
  result.contacts[1] = ContactPoint{Coordinate{0.5, 0.0, 0.0}, Coordinate{0.5, 0.0, 0.0}};
  result.contacts[2] = ContactPoint{Coordinate{0.5, 0.0, -0.5}, Coordinate{0.5, 0.0, -0.5}};

  double restitution = 1.0;
  CollisionResponse::applyImpulseManifoldStatic(dynamic, staticObj, result, restitution);

  // Linear velocity should decrease (impulse applied in -x direction)
  EXPECT_LT(dynamic.getInertialState().velocity.x(), initialVx);

  // With symmetric contact points around vertical axis, net angular impulse
  // about Y axis should be near zero (opposing torques from top/bottom contacts cancel)
  // The Y component represents rotation about the axis perpendicular to collision normal
  double angularChangeY = std::abs(dynamic.getInertialState().angularVelocity.y() -
                                   originalAngularVel.y());
  EXPECT_LT(angularChangeY, 1e-10);
}

TEST(CollisionResponseStaticManifoldTest, applyImpulseManifoldStatic_SeparatingNoImpulse)
{
  // Objects separating should not receive impulse
  auto points = createCubePoints(1.0);
  ConvexHull hull{points};
  ReferenceFrame frameDynamic{Coordinate{0.0, 0.0, 0.0}};
  ReferenceFrame frameStatic{Coordinate{0.8, 0.0, 0.0}};

  AssetInertial dynamic{0, 0, hull, 10.0, frameDynamic, 1.0};
  AssetEnvironment staticObj{0, 1, hull, frameStatic};

  dynamic.getInertialState().position = frameDynamic.getOrigin();
  dynamic.getInertialState().velocity = Coordinate{-2.0, 0.0, 0.0};  // Moving away from static
  Coordinate originalVel = dynamic.getInertialState().velocity;

  CollisionResult result;
  result.normal = Coordinate{1.0, 0.0, 0.0};  // Points from dynamic toward static
  result.penetrationDepth = 0.1;
  result.contactCount = 1;
  result.contacts[0] = ContactPoint{Coordinate{0.5, 0.0, 0.0}, Coordinate{0.5, 0.0, 0.0}};

  double restitution = 1.0;
  CollisionResponse::applyImpulseManifoldStatic(dynamic, staticObj, result, restitution);

  // Velocity should not change (objects separating)
  EXPECT_DOUBLE_EQ(originalVel.x(), dynamic.getInertialState().velocity.x());
}
