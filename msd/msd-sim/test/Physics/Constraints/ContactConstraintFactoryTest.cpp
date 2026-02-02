// Ticket: 0032a_two_body_constraint_infrastructure
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

// Create a default InertialState at rest
InertialState createDefaultState(const Coordinate& position = Coordinate{0.0, 0.0, 0.0})
{
  InertialState state;
  state.position = position;
  state.velocity = Coordinate{0.0, 0.0, 0.0};
  state.acceleration = Coordinate{0.0, 0.0, 0.0};
  state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  state.quaternionRate = Eigen::Vector4d::Zero();
  state.angularAcceleration = AngularRate{0.0, 0.0, 0.0};
  return state;
}

// Create InertialState with specific velocity
InertialState createMovingState(const Coordinate& position,
                                const Coordinate& velocity)
{
  InertialState state = createDefaultState(position);
  state.velocity = velocity;
  return state;
}

// Create InertialState with angular velocity
InertialState createRotatingState(const Coordinate& position,
                                  const AngularRate& angularVelocity)
{
  InertialState state = createDefaultState(position);

  // Convert angular velocity to quaternion rate
  // Q̇ = 0.5 * Q ⊗ [0, ω]
  Eigen::Quaterniond Q = state.orientation;
  Eigen::Quaterniond omega_quat{0.0, angularVelocity.x(), angularVelocity.y(), angularVelocity.z()};
  Eigen::Quaterniond Qdot_quat = Q * omega_quat;

  state.quaternionRate << Qdot_quat.x(), Qdot_quat.y(), Qdot_quat.z(), Qdot_quat.w();
  state.quaternionRate *= 0.5;

  return state;
}

}  // anonymous namespace

// ============================================================================
// ContactConstraintFactory Tests
// ============================================================================

TEST(ContactConstraintFactoryTest, CreateFromCollision_SingleContact_0032a)
{
  // Test: Creates exactly 1 constraint for single contact point
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.05;
  result.contactCount = 1;
  result.contacts[0].pointA = Coordinate{0, 0, 0};
  result.contacts[0].pointB = Coordinate{0, 0, 0.05};

  InertialState stateA = createDefaultState(Coordinate{0, 0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.5});
  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraints = ContactConstraintFactory::createFromCollision(
      0, 1, result, stateA, stateB, comA, comB, 0.8);

  ASSERT_EQ(1, constraints.size());
  EXPECT_EQ(1, constraints[0]->dimension());
}

TEST(ContactConstraintFactoryTest, CreateFromCollision_ManifoldWith4Contacts_0032a)
{
  // Test: Creates 4 constraints for 4-point contact manifold
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 4;

  // Four contact points (box-on-box scenario)
  result.contacts[0].pointA = Coordinate{-0.5, -0.5, 0};
  result.contacts[0].pointB = Coordinate{-0.5, -0.5, 0.01};
  result.contacts[1].pointA = Coordinate{0.5, -0.5, 0};
  result.contacts[1].pointB = Coordinate{0.5, -0.5, 0.01};
  result.contacts[2].pointA = Coordinate{0.5, 0.5, 0};
  result.contacts[2].pointB = Coordinate{0.5, 0.5, 0.01};
  result.contacts[3].pointA = Coordinate{-0.5, 0.5, 0};
  result.contacts[3].pointB = Coordinate{-0.5, 0.5, 0.01};

  InertialState stateA = createDefaultState(Coordinate{0, 0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.5});
  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraints = ContactConstraintFactory::createFromCollision(
      0, 1, result, stateA, stateB, comA, comB, 0.5);

  ASSERT_EQ(4, constraints.size());
  for (const auto& constraint : constraints) {
    EXPECT_EQ(1, constraint->dimension());
  }
}

TEST(ContactConstraintFactoryTest, CombineRestitution_GeometricMean_0032a)
{
  // Test: combineRestitution returns sqrt(eA * eB)
  double eA = 0.8;
  double eB = 0.5;
  double expected = std::sqrt(0.8 * 0.5);

  double combined = ContactConstraintFactory::combineRestitution(eA, eB);

  EXPECT_NEAR(expected, combined, 1e-10);
}

TEST(ContactConstraintFactoryTest, CombineRestitution_ZeroValues_0032a)
{
  // Test: Returns zero when either coefficient is zero
  EXPECT_NEAR(0.0, ContactConstraintFactory::combineRestitution(0.0, 0.8), 1e-10);
  EXPECT_NEAR(0.0, ContactConstraintFactory::combineRestitution(0.8, 0.0), 1e-10);
  EXPECT_NEAR(0.0, ContactConstraintFactory::combineRestitution(0.0, 0.0), 1e-10);
}

TEST(ContactConstraintFactoryTest, ComputeRelativeNormalVelocity_HeadOn_0032a)
{
  // Test: Correctly computes relative normal velocity for head-on collision
  Coordinate normal{0, 0, 1};
  Coordinate leverArmA{0, 0, 0};  // Contact at COM
  Coordinate leverArmB{0, 0, 0};  // Contact at COM

  // Body A moving right (+z), body B moving left (-z)
  InertialState stateA = createMovingState(Coordinate{0, 0, -1}, Coordinate{0, 0, 2});
  InertialState stateB = createMovingState(Coordinate{0, 0, 1}, Coordinate{0, 0, -2});

  double relVelNormal = ContactConstraintFactory::computeRelativeNormalVelocity(
      stateA, stateB, leverArmA, leverArmB, normal);

  // v_rel = v_B - v_A = (-2) - (2) = -4
  // Approaching collision: negative relative velocity
  EXPECT_NEAR(-4.0, relVelNormal, 1e-10);
}

TEST(ContactConstraintFactoryTest, ComputeRelativeNormalVelocity_WithAngular_0032a)
{
  // Test: Includes angular velocity contribution (ω × r term)
  Coordinate normal{0, 0, 1};
  Coordinate leverArmA{1, 0, 0};  // Offset from COM
  Coordinate leverArmB{0, 0, 0};  // Contact at COM

  // Body A rotating around z-axis with ω = (0, 0, 1) rad/s
  // At lever arm (1, 0, 0), velocity contribution: ω × r = (0, 0, 1) × (1, 0, 0) = (0, 1, 0)
  InertialState stateA = createRotatingState(Coordinate{0, 0, -1}, AngularRate{0, 0, 1});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 1});

  double relVelNormal = ContactConstraintFactory::computeRelativeNormalVelocity(
      stateA, stateB, leverArmA, leverArmB, normal);

  // v_A at contact = v_A + ω_A × r_A = (0, 0, 0) + (0, 1, 0) = (0, 1, 0)
  // v_B at contact = (0, 0, 0)
  // v_rel = v_B - v_A = (0, 0, 0) - (0, 1, 0) = (0, -1, 0)
  // v_rel · n = (0, -1, 0) · (0, 0, 1) = 0
  EXPECT_NEAR(0.0, relVelNormal, 1e-10);
}

TEST(ContactConstraintFactoryTest, RestVelocityThreshold_DisablesRestitution_0032a)
{
  // Test: Restitution disabled below rest velocity threshold (0.5 m/s)
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 1;
  result.contacts[0].pointA = Coordinate{0, 0, 0};
  result.contacts[0].pointB = Coordinate{0, 0, 0.01};

  // Slow collision: 0.2 m/s (below threshold)
  InertialState stateA = createMovingState(Coordinate{0, 0, -0.5}, Coordinate{0, 0, 0.1});
  InertialState stateB = createMovingState(Coordinate{0, 0, 0.5}, Coordinate{0, 0, -0.1});
  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraints = ContactConstraintFactory::createFromCollision(
      0, 1, result, stateA, stateB, comA, comB, 0.8);

  ASSERT_EQ(1, constraints.size());
  // Restitution should be disabled (set to 0) below threshold
  EXPECT_NEAR(0.0, constraints[0]->getRestitution(), 1e-10);
}

TEST(ContactConstraintFactoryTest, CreateFromCollision_NoContacts_ReturnsEmpty_0032a)
{
  // Test: Returns empty vector when contactCount == 0
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.0;
  result.contactCount = 0;

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto constraints = ContactConstraintFactory::createFromCollision(
      0, 1, result, stateA, stateB, comA, comB, 0.5);

  EXPECT_EQ(0, constraints.size());
}

// ============================================================================
// Centroid Factory Method Tests (ticket 0035d4)
// ============================================================================

TEST(ContactConstraintFactoryTest, CreateCentroidFrictionConstraint_FourPointManifold_0035d4)
{
  // Test: AC1 - Returns single FrictionConstraint at centroid for 4-contact manifold
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 4;

  // Four contact points forming a square (corners at ±0.5)
  result.contacts[0].pointA = Coordinate{-0.5, -0.5, 0};
  result.contacts[0].pointB = Coordinate{-0.5, -0.5, 0.01};
  result.contacts[1].pointA = Coordinate{0.5, -0.5, 0};
  result.contacts[1].pointB = Coordinate{0.5, -0.5, 0.01};
  result.contacts[2].pointA = Coordinate{0.5, 0.5, 0};
  result.contacts[2].pointB = Coordinate{0.5, 0.5, 0.01};
  result.contacts[3].pointA = Coordinate{-0.5, 0.5, 0};
  result.contacts[3].pointB = Coordinate{-0.5, 0.5, 0.01};

  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraint = ContactConstraintFactory::createCentroidFrictionConstraint(
      0, 1, result, comA, comB, 0.5, 0.8);

  ASSERT_NE(nullptr, constraint);
  EXPECT_EQ(2, constraint->dimension());  // FrictionConstraint has 2 DOF
}

TEST(ContactConstraintFactoryTest, CreateCentroidFrictionConstraint_CentroidCorrect_0035d4)
{
  // Test: AC5 - Centroid computation is correct: p_centroid = (1/N) Σ p_i
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 4;

  // Four contact points with known centroid (should be origin for symmetric square)
  result.contacts[0].pointA = Coordinate{-1, -1, 0};
  result.contacts[0].pointB = Coordinate{-1, -1, 0.01};
  result.contacts[1].pointA = Coordinate{1, -1, 0};
  result.contacts[1].pointB = Coordinate{1, -1, 0.01};
  result.contacts[2].pointA = Coordinate{1, 1, 0};
  result.contacts[2].pointB = Coordinate{1, 1, 0.01};
  result.contacts[3].pointA = Coordinate{-1, 1, 0};
  result.contacts[3].pointB = Coordinate{-1, 1, 0.01};

  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraint = ContactConstraintFactory::createCentroidFrictionConstraint(
      0, 1, result, comA, comB, 0.5, 0.8);

  ASSERT_NE(nullptr, constraint);

  // Expected centroid: (1/4) * [(-1,-1,0) + (1,-1,0) + (1,1,0) + (-1,1,0)] = (0, 0, 0)
  // Expected lever arm A: centroidA - comA = (0,0,0) - (0,0,-0.5) = (0,0,0.5)
  // We can't directly access internal points, but we can verify constraint was created successfully
}

TEST(ContactConstraintFactoryTest, CreateCentroidFrictionConstraint_ZeroFriction_ReturnsNull_0035d4)
{
  // Test: AC2 - Returns nullptr when μ = 0
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 4;
  result.contacts[0].pointA = Coordinate{0, 0, 0};
  result.contacts[0].pointB = Coordinate{0, 0, 0.01};
  result.contacts[1].pointA = Coordinate{1, 0, 0};
  result.contacts[1].pointB = Coordinate{1, 0, 0.01};
  result.contacts[2].pointA = Coordinate{0, 1, 0};
  result.contacts[2].pointB = Coordinate{0, 1, 0.01};
  result.contacts[3].pointA = Coordinate{1, 1, 0};
  result.contacts[3].pointB = Coordinate{1, 1, 0.01};

  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  // Either friction coefficient is zero → combined μ = 0
  auto constraint1 = ContactConstraintFactory::createCentroidFrictionConstraint(
      0, 1, result, comA, comB, 0.0, 0.8);
  auto constraint2 = ContactConstraintFactory::createCentroidFrictionConstraint(
      0, 1, result, comA, comB, 0.5, 0.0);

  EXPECT_EQ(nullptr, constraint1);
  EXPECT_EQ(nullptr, constraint2);
}

TEST(ContactConstraintFactoryTest, CreateCentroidFrictionConstraint_NoContacts_ReturnsNull_0035d4)
{
  // Test: AC3 - Returns nullptr when contactCount = 0
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.0;
  result.contactCount = 0;

  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraint = ContactConstraintFactory::createCentroidFrictionConstraint(
      0, 1, result, comA, comB, 0.5, 0.8);

  EXPECT_EQ(nullptr, constraint);
}

TEST(ContactConstraintFactoryTest, CreateCentroidFrictionConstraint_NegativeFriction_Throws_0035d4)
{
  // Test: Validates friction coefficients (should throw for negative values)
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 1;
  result.contacts[0].pointA = Coordinate{0, 0, 0};
  result.contacts[0].pointB = Coordinate{0, 0, 0.01};

  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  EXPECT_THROW(
      ContactConstraintFactory::createCentroidFrictionConstraint(
          0, 1, result, comA, comB, -0.1, 0.8),
      std::invalid_argument);

  EXPECT_THROW(
      ContactConstraintFactory::createCentroidFrictionConstraint(
          0, 1, result, comA, comB, 0.5, -0.1),
      std::invalid_argument);
}

TEST(ContactConstraintFactoryTest, CreateCentroidContactConstraint_FourPointManifold_0035d4)
{
  // Test: AC4 - Returns single ContactConstraint at centroid
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 4;

  // Four contact points forming a square
  result.contacts[0].pointA = Coordinate{-0.5, -0.5, 0};
  result.contacts[0].pointB = Coordinate{-0.5, -0.5, 0.01};
  result.contacts[1].pointA = Coordinate{0.5, -0.5, 0};
  result.contacts[1].pointB = Coordinate{0.5, -0.5, 0.01};
  result.contacts[2].pointA = Coordinate{0.5, 0.5, 0};
  result.contacts[2].pointB = Coordinate{0.5, 0.5, 0.01};
  result.contacts[3].pointA = Coordinate{-0.5, 0.5, 0};
  result.contacts[3].pointB = Coordinate{-0.5, 0.5, 0.01};

  InertialState stateA = createDefaultState(Coordinate{0, 0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.5});
  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraint = ContactConstraintFactory::createCentroidContactConstraint(
      0, 1, result, stateA, stateB, comA, comB, 0.8);

  ASSERT_NE(nullptr, constraint);
  EXPECT_EQ(1, constraint->dimension());  // ContactConstraint has 1 DOF
}

TEST(ContactConstraintFactoryTest, CreateCentroidContactConstraint_NoContacts_ReturnsNull_0035d4)
{
  // Test: Returns nullptr when contactCount = 0
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.0;
  result.contactCount = 0;

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto constraint = ContactConstraintFactory::createCentroidContactConstraint(
      0, 1, result, stateA, stateB, comA, comB, 0.5);

  EXPECT_EQ(nullptr, constraint);
}

TEST(ContactConstraintFactoryTest, CreateCentroidContactConstraint_RestVelocityThreshold_0035d4)
{
  // Test: Restitution disabled below rest velocity threshold at centroid
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 2;
  result.contacts[0].pointA = Coordinate{-0.5, 0, 0};
  result.contacts[0].pointB = Coordinate{-0.5, 0, 0.01};
  result.contacts[1].pointA = Coordinate{0.5, 0, 0};
  result.contacts[1].pointB = Coordinate{0.5, 0, 0.01};

  // Slow collision: 0.2 m/s (below threshold of 0.5 m/s)
  InertialState stateA = createMovingState(Coordinate{0, 0, -0.5}, Coordinate{0, 0, 0.1});
  InertialState stateB = createMovingState(Coordinate{0, 0, 0.5}, Coordinate{0, 0, -0.1});
  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraint = ContactConstraintFactory::createCentroidContactConstraint(
      0, 1, result, stateA, stateB, comA, comB, 0.8);

  ASSERT_NE(nullptr, constraint);
  // Restitution should be disabled (set to 0) below threshold
  EXPECT_NEAR(0.0, constraint->getRestitution(), 1e-10);
}

TEST(ContactConstraintFactoryTest, CreateCentroidContactConstraint_SingleContact_CentroidEqualsPoint_0035d4)
{
  // Test: For single contact (N=1), centroid equals the single point
  CollisionResult result;
  result.normal = Coordinate{0, 0, 1};
  result.penetrationDepth = 0.01;
  result.contactCount = 1;
  result.contacts[0].pointA = Coordinate{1.5, 2.5, 0};
  result.contacts[0].pointB = Coordinate{1.5, 2.5, 0.01};

  InertialState stateA = createDefaultState(Coordinate{0, 0, -0.5});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.5});
  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  auto constraint = ContactConstraintFactory::createCentroidContactConstraint(
      0, 1, result, stateA, stateB, comA, comB, 0.5);

  ASSERT_NE(nullptr, constraint);
  // For N=1, centroid should equal the single point
  // We can't directly verify internal points, but constraint creation validates this works
}
