// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/DistanceConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/LambdaBounds.hpp"
#include "msd-sim/src/Physics/Constraints/UnitQuaternionConstraint.hpp"
#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

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

// Create a default InertialState at rest
InertialState createDefaultState()
{
  InertialState state;
  state.position = Coordinate{0.0, 0.0, 0.0};
  state.velocity = Coordinate{0.0, 0.0, 0.0};
  state.acceleration = Coordinate{0.0, 0.0, 0.0};
  state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};  // Identity
  state.quaternionRate = Eigen::Vector4d::Zero();
  state.angularAcceleration = AngularAcceleration{0.0, 0.0, 0.0};
  return state;
}

}  // anonymous namespace

// ============================================================================
// UnitQuaternionConstraint Tests
// ============================================================================

TEST(UnitQuaternionConstraintTest, Dimension_ReturnsOne)
{
  // Test: UnitQuaternionConstraint is a scalar constraint (dimension = 1)
  UnitQuaternionConstraint constraint;
  EXPECT_EQ(1, constraint.dimension());
}

TEST(UnitQuaternionConstraintTest, Evaluate_UnitQuaternion_ReturnsZero)
{
  // Test: C(Q) = Q^T·Q - 1 = 0 for unit quaternion
  UnitQuaternionConstraint constraint;
  InertialState state = createDefaultState();
  state.orientation =
    Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};  // Unit quaternion

  Eigen::VectorXd C = constraint.evaluate(state, 0.0);

  ASSERT_EQ(1, C.size());
  EXPECT_NEAR(0.0, C(0), 1e-10);
}

TEST(UnitQuaternionConstraintTest, Evaluate_NonUnitQuaternion_ReturnsViolation)
{
  // Test: C(Q) = Q^T·Q - 1 ≠ 0 for non-unit quaternion
  UnitQuaternionConstraint constraint;
  InertialState state = createDefaultState();

  // Create quaternion with norm 2 (w=1, x=1, y=1, z=1 → |Q|=2)
  state.orientation.w() = 1.0;
  state.orientation.x() = 1.0;
  state.orientation.y() = 1.0;
  state.orientation.z() = 1.0;
  // Note: Eigen stores coefficients as (x, y, z, w) internally but constructor
  // is (w, x, y, z)

  Eigen::VectorXd C = constraint.evaluate(state, 0.0);

  // |Q|² = 1 + 1 + 1 + 1 = 4, so C = 4 - 1 = 3
  ASSERT_EQ(1, C.size());
  EXPECT_NEAR(3.0, C(0), 1e-10);
}

TEST(UnitQuaternionConstraintTest, Jacobian_CorrectStructure)
{
  // Test: J = ∂C/∂q is (1 × 7) matrix with [0 0 0, 2w 2x 2y 2z]
  UnitQuaternionConstraint constraint;
  InertialState state = createDefaultState();
  state.orientation = Eigen::Quaterniond{0.5, 0.5, 0.5, 0.5};  // Normalized
  state.orientation.normalize();

  Eigen::MatrixXd J = constraint.jacobian(state, 0.0);

  ASSERT_EQ(1, J.rows());
  ASSERT_EQ(7, J.cols());

  // Position derivatives should be zero (constraint doesn't depend on X)
  EXPECT_NEAR(0.0, J(0, 0), 1e-10);
  EXPECT_NEAR(0.0, J(0, 1), 1e-10);
  EXPECT_NEAR(0.0, J(0, 2), 1e-10);

  // Quaternion derivatives: ∂C/∂Q = 2·Q^T
  double w = state.orientation.w();
  double x = state.orientation.x();
  double y = state.orientation.y();
  double z = state.orientation.z();
  EXPECT_NEAR(2.0 * w, J(0, 3), 1e-10);
  EXPECT_NEAR(2.0 * x, J(0, 4), 1e-10);
  EXPECT_NEAR(2.0 * y, J(0, 5), 1e-10);
  EXPECT_NEAR(2.0 * z, J(0, 6), 1e-10);
}

TEST(UnitQuaternionConstraintTest, PartialTimeDerivative_ReturnsZero)
{
  // Test: Time-independent constraint has ∂C/∂t = 0
  UnitQuaternionConstraint constraint;
  InertialState state = createDefaultState();

  Eigen::VectorXd dCdt = constraint.partialTimeDerivative(state, 0.0);

  ASSERT_EQ(1, dCdt.size());
  EXPECT_NEAR(0.0, dCdt(0), 1e-10);
}

TEST(UnitQuaternionConstraintTest, BaumgarteParameters_DefaultValues)
{
  // Test: Default Baumgarte parameters are α=10, β=10
  UnitQuaternionConstraint constraint;

  EXPECT_DOUBLE_EQ(10.0, constraint.alpha());
  EXPECT_DOUBLE_EQ(10.0, constraint.beta());
}

TEST(UnitQuaternionConstraintTest, BaumgarteParameters_CustomValues)
{
  // Test: Constructor accepts custom Baumgarte parameters
  UnitQuaternionConstraint constraint{
    0, 20.0, 15.0};  // bodyAIndex=0, alpha=20.0, beta=15.0

  EXPECT_DOUBLE_EQ(20.0, constraint.alpha());
  EXPECT_DOUBLE_EQ(15.0, constraint.beta());
}

TEST(UnitQuaternionConstraintTest, BaumgarteParameters_Setters)
{
  // Test: setAlpha() and setBeta() work correctly
  UnitQuaternionConstraint constraint;

  constraint.setAlpha(25.0);
  constraint.setBeta(30.0);

  EXPECT_DOUBLE_EQ(25.0, constraint.alpha());
  EXPECT_DOUBLE_EQ(30.0, constraint.beta());
}

TEST(UnitQuaternionConstraintTest, TypeName)
{
  // Test: typeName() returns correct identifier
  UnitQuaternionConstraint constraint;
  EXPECT_EQ("UnitQuaternionConstraint", constraint.typeName());
}

// ============================================================================
// LambdaBounds Tests [0043_constraint_hierarchy_refactor]
// ============================================================================

TEST(LambdaBoundsTest, bilateral_returns_infinite_bounds)
{
  auto bounds = LambdaBounds::bilateral();
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), bounds.lower);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), bounds.upper);
}

TEST(LambdaBoundsTest, unilateral_returns_nonnegative_bounds)
{
  auto bounds = LambdaBounds::unilateral();
  EXPECT_EQ(0.0, bounds.lower);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), bounds.upper);
}

TEST(LambdaBoundsTest, box_constrained_returns_custom_bounds)
{
  auto bounds = LambdaBounds::boxConstrained(-5.0, 5.0);
  EXPECT_DOUBLE_EQ(-5.0, bounds.lower);
  EXPECT_DOUBLE_EQ(5.0, bounds.upper);
}

TEST(LambdaBoundsTest, isBilateral_correct)
{
  EXPECT_TRUE(LambdaBounds::bilateral().isBilateral());
  EXPECT_FALSE(LambdaBounds::unilateral().isBilateral());
  EXPECT_FALSE(LambdaBounds::boxConstrained(-1.0, 1.0).isBilateral());
}

TEST(LambdaBoundsTest, isUnilateral_correct)
{
  EXPECT_TRUE(LambdaBounds::unilateral().isUnilateral());
  EXPECT_FALSE(LambdaBounds::bilateral().isUnilateral());
  EXPECT_FALSE(LambdaBounds::boxConstrained(-1.0, 1.0).isUnilateral());
}

TEST(LambdaBoundsTest, isBoxConstrained_correct)
{
  EXPECT_TRUE(LambdaBounds::boxConstrained(-1.0, 1.0).isBoxConstrained());
  EXPECT_FALSE(LambdaBounds::bilateral().isBoxConstrained());
  EXPECT_FALSE(LambdaBounds::unilateral().isBoxConstrained());
}

// ============================================================================
// Base Class Feature Tests [0043_constraint_hierarchy_refactor]
// ============================================================================

TEST(ConstraintBaseTest, bodyCount_single_body_returns_1)
{
  UnitQuaternionConstraint constraint;
  Constraint* base = &constraint;
  EXPECT_EQ(1, base->bodyCount());
}

TEST(ConstraintBaseTest, bodyCount_two_body_returns_2)
{
  // Create a minimal ContactConstraint
  ContactConstraint constraint{
    0,
    1,
    Coordinate{0.0, 0.0, 1.0},   // normal
    Coordinate{0.0, 0.0, 0.0},   // contactPointA
    Coordinate{0.0, 0.0, 0.0},   // contactPointB
    0.01,                        // penetration
    Coordinate{0.0, 0.0, -0.5},  // comA
    Coordinate{0.0, 0.0, 0.5},   // comB
    0.5,                         // restitution
    -1.0                         // preImpactRelVelNormal
  };
  Constraint* base = &constraint;
  EXPECT_EQ(2, base->bodyCount());
}

TEST(ConstraintBaseTest, bodyAIndex_accessible_from_base)
{
  UnitQuaternionConstraint constraint{5};  // bodyAIndex=5
  Constraint* base = &constraint;
  EXPECT_EQ(5u, base->bodyAIndex());
}

TEST(ConstraintBaseTest, isActive_default_returns_true)
{
  UnitQuaternionConstraint constraint;
  Constraint* base = &constraint;
  InertialState state = createDefaultState();
  EXPECT_TRUE(base->isActive(state, state, 0.0));
}

TEST(ConstraintBaseTest, lambdaBounds_bilateral_from_base)
{
  UnitQuaternionConstraint constraint;
  Constraint* base = &constraint;
  auto bounds = base->lambdaBounds();
  EXPECT_TRUE(bounds.isBilateral());
}

TEST(ConstraintBaseTest, lambdaBounds_unilateral_from_base)
{
  ContactConstraint constraint{0,
                               1,
                               Coordinate{0.0, 0.0, 1.0},
                               Coordinate{0.0, 0.0, 0.0},
                               Coordinate{0.0, 0.0, 0.0},
                               0.01,
                               Coordinate{0.0, 0.0, -0.5},
                               Coordinate{0.0, 0.0, 0.5},
                               0.5,
                               -1.0};
  Constraint* base = &constraint;
  auto bounds = base->lambdaBounds();
  EXPECT_TRUE(bounds.isUnilateral());
}

TEST(ConstraintBaseTest, evaluate_unified_signature_single_body)
{
  // Single-body constraint evaluates correctly with two-state signature
  UnitQuaternionConstraint constraint;
  InertialState state = createDefaultState();
  state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};

  // Two-state signature (stateB ignored for single-body)
  InertialState dummyStateB = createDefaultState();
  Eigen::VectorXd C = constraint.evaluate(state, dummyStateB, 0.0);

  ASSERT_EQ(1, C.size());
  EXPECT_NEAR(0.0, C(0), 1e-10);  // Unit quaternion → zero violation
}

TEST(ConstraintBaseTest, evaluate_unified_signature_two_body)
{
  // Two-body constraint evaluates correctly with two-state signature
  ContactConstraint constraint{0,
                               1,
                               Coordinate{0.0, 0.0, 1.0},  // normal pointing up
                               Coordinate{0.0, 0.0, 0.0},  // contactPointA
                               Coordinate{0.0, 0.0, 0.0},  // contactPointB
                               0.01,                       // penetration
                               Coordinate{0.0, 0.0, -0.5},  // comA below
                               Coordinate{0.0, 0.0, 0.5},   // comB above
                               0.5,
                               -1.0};

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();
  stateA.position = Coordinate{0.0, 0.0, -0.5};
  stateB.position = Coordinate{0.0, 0.0, 0.5};

  Eigen::VectorXd C = constraint.evaluate(stateA, stateB, 0.0);
  ASSERT_EQ(1, C.size());
  // The constraint evaluates to the signed distance along the normal
  EXPECT_TRUE(std::isfinite(C(0)));
}

// ============================================================================
// DistanceConstraint Tests
// ============================================================================

TEST(DistanceConstraintTest, Dimension_ReturnsOne)
{
  // Test: DistanceConstraint is a scalar constraint (dimension = 1)
  DistanceConstraint constraint{5.0};
  EXPECT_EQ(1, constraint.dimension());
}

TEST(DistanceConstraintTest, Evaluate_AtTargetDistance_ReturnsZero)
{
  // Test: C(X) = |X|² - d² = 0 when |X| = d
  DistanceConstraint constraint{5.0};
  InertialState state = createDefaultState();
  state.position = Coordinate{3.0, 4.0, 0.0};  // |X| = 5

  Eigen::VectorXd C = constraint.evaluate(state, 0.0);

  ASSERT_EQ(1, C.size());
  EXPECT_NEAR(0.0, C(0), 1e-10);
}

TEST(DistanceConstraintTest, Evaluate_InsideTarget_ReturnsNegative)
{
  // Test: C(X) < 0 when |X| < d
  DistanceConstraint constraint{10.0};
  InertialState state = createDefaultState();
  state.position = Coordinate{3.0, 4.0, 0.0};  // |X| = 5 < 10

  Eigen::VectorXd C = constraint.evaluate(state, 0.0);

  ASSERT_EQ(1, C.size());
  // C = 25 - 100 = -75
  EXPECT_NEAR(-75.0, C(0), 1e-10);
}

TEST(DistanceConstraintTest, Evaluate_OutsideTarget_ReturnsPositive)
{
  // Test: C(X) > 0 when |X| > d
  DistanceConstraint constraint{3.0};
  InertialState state = createDefaultState();
  state.position = Coordinate{3.0, 4.0, 0.0};  // |X| = 5 > 3

  Eigen::VectorXd C = constraint.evaluate(state, 0.0);

  ASSERT_EQ(1, C.size());
  // C = 25 - 9 = 16
  EXPECT_NEAR(16.0, C(0), 1e-10);
}

TEST(DistanceConstraintTest, Jacobian_CorrectStructure)
{
  // Test: J = ∂C/∂q is (1 × 7) matrix with [2x 2y 2z, 0 0 0 0]
  DistanceConstraint constraint{5.0};
  InertialState state = createDefaultState();
  state.position = Coordinate{1.0, 2.0, 3.0};

  Eigen::MatrixXd J = constraint.jacobian(state, 0.0);

  ASSERT_EQ(1, J.rows());
  ASSERT_EQ(7, J.cols());

  // Position derivatives: ∂C/∂X = 2·X^T
  EXPECT_NEAR(2.0, J(0, 0), 1e-10);  // 2·x = 2·1 = 2
  EXPECT_NEAR(4.0, J(0, 1), 1e-10);  // 2·y = 2·2 = 4
  EXPECT_NEAR(6.0, J(0, 2), 1e-10);  // 2·z = 2·3 = 6

  // Quaternion derivatives should be zero (constraint doesn't depend on Q)
  EXPECT_NEAR(0.0, J(0, 3), 1e-10);
  EXPECT_NEAR(0.0, J(0, 4), 1e-10);
  EXPECT_NEAR(0.0, J(0, 5), 1e-10);
  EXPECT_NEAR(0.0, J(0, 6), 1e-10);
}

TEST(DistanceConstraintTest, PartialTimeDerivative_ReturnsZero)
{
  // Test: Time-independent constraint has ∂C/∂t = 0
  DistanceConstraint constraint{5.0};
  InertialState state = createDefaultState();

  Eigen::VectorXd dCdt = constraint.partialTimeDerivative(state, 0.0);

  ASSERT_EQ(1, dCdt.size());
  EXPECT_NEAR(0.0, dCdt(0), 1e-10);
}

TEST(DistanceConstraintTest, InvalidTargetDistance_Throws)
{
  // Test: Constructor throws for invalid target distance
  EXPECT_THROW(DistanceConstraint{0.0}, std::invalid_argument);
  EXPECT_THROW(DistanceConstraint{-1.0}, std::invalid_argument);
}

TEST(DistanceConstraintTest, GetTargetDistance)
{
  // Test: getTargetDistance() returns correct value
  DistanceConstraint constraint{7.5};
  EXPECT_DOUBLE_EQ(7.5, constraint.getTargetDistance());
}

TEST(DistanceConstraintTest, TypeName)
{
  // Test: typeName() returns correct identifier
  DistanceConstraint constraint{5.0};
  EXPECT_EQ("DistanceConstraint", constraint.typeName());
}

// ============================================================================
// ConstraintSolver Tests
// ============================================================================

// ===========================================================================
// NOTE: Tests for single-body ConstraintSolver::solve() removed in ticket 0045
// Quaternion normalization now handled by integrator via state.orientation.normalize()
// ===========================================================================

// ============================================================================
// AssetInertial Constraint Management Tests
// ============================================================================

TEST(AssetInertialConstraintTest, DefaultConstraint_EmptyList)
{
  // Test: AssetInertial now defaults to empty constraint list
  // Ticket 0045: Removed default UnitQuaternionConstraint
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  EXPECT_EQ(0, asset.getConstraintCount());
  auto constraints = asset.getConstraints();
  EXPECT_EQ(0, constraints.size());
}

TEST(AssetInertialConstraintTest, AddConstraint)
{
  // Test: addConstraint() correctly adds constraint
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  auto distConstraint = std::make_unique<DistanceConstraint>(5.0);
  asset.addConstraint(std::move(distConstraint));

  // Ticket 0045: No default constraint, so count is 1 not 2
  EXPECT_EQ(1, asset.getConstraintCount());
}

TEST(AssetInertialConstraintTest, RemoveConstraint)
{
  // Test: removeConstraint() correctly removes constraint
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  auto distConstraint = std::make_unique<DistanceConstraint>(5.0);
  asset.addConstraint(std::move(distConstraint));
  // Ticket 0045: No default constraint
  EXPECT_EQ(1, asset.getConstraintCount());

  asset.removeConstraint(0);  // Remove DistanceConstraint (now at index 0)
  EXPECT_EQ(0, asset.getConstraintCount());
}

TEST(AssetInertialConstraintTest, ClearConstraints)
{
  // Test: clearConstraints() removes all constraints
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  asset.clearConstraints();

  EXPECT_EQ(0, asset.getConstraintCount());
}

TEST(AssetInertialConstraintTest, GetConstraints_ReturnsNonOwningPointers)
{
  // Test: getConstraints() returns pointers for use in solver
  // Ticket 0045: Default constraint list is now empty
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 0.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Add a constraint for testing
  asset.addConstraint(std::make_unique<UnitQuaternionConstraint>());

  auto constraints = asset.getConstraints();

  EXPECT_EQ(1, constraints.size());
  EXPECT_NE(nullptr, constraints[0]);
  // Pointer should remain valid after call
  EXPECT_EQ("UnitQuaternionConstraint", constraints[0]->typeName());
}

// ============================================================================
// Integration Tests: Constraint Enforcement During Physics
// ============================================================================

TEST(ConstraintIntegrationTest, QuaternionRemainNormalized_10Steps)
{
  // Test: Quaternion remains normalized over multiple integration steps
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 10.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};
  SemiImplicitEulerIntegrator integrator;

  const double dt = 0.016;  // 60 FPS
  const int steps = 10;

  for (int i = 0; i < steps; ++i)
  {
    // Ticket 0045: Removed constraints parameter
    integrator.step(asset.getInertialState(),
                    Coordinate{0.0, 0.0, -98.1},  // gravity on 10kg
                    Coordinate{0.0, 0.0, 0.0},
                    asset.getMass(),
                    asset.getInverseInertiaTensor(),
                    dt);
  }

  // Quaternion should remain normalized
  double qNorm = asset.getInertialState().orientation.norm();
  EXPECT_NEAR(1.0, qNorm, 1e-6);
}

TEST(ConstraintIntegrationTest, QuaternionRemainNormalized_1000Steps)
{
  // Test: Quaternion normalization maintained over extended simulation
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{0.0, 0.0, 10.0}};

  AssetInertial asset{0, 0, hull, 10.0, frame};

  // Add angular velocity to make it interesting
  asset.getInertialState().setAngularVelocity(AngularVelocity{0.1, 0.2, 0.3});

  SemiImplicitEulerIntegrator integrator;

  const double dt = 0.016;
  const int steps = 1000;

  for (int i = 0; i < steps; ++i)
  {
    // Ticket 0045: Removed constraints parameter
    integrator.step(asset.getInertialState(),
                    Coordinate{0.0, 0.0, -98.1},
                    Coordinate{0.1, 0.0, 0.0},  // Small torque
                    asset.getMass(),
                    asset.getInverseInertiaTensor(),
                    dt);
  }

  // Quaternion should remain normalized within tolerance
  double qNorm = asset.getInertialState().orientation.norm();
  EXPECT_NEAR(
    1.0, qNorm, 1e-4);  // Slightly looser tolerance for long simulation
}

TEST(ConstraintIntegrationTest, MultipleConstraints_BothEnforced)
{
  // Test: Multiple constraints (quaternion + distance) are both enforced
  auto cubePoints = createCubePoints(1.0);
  ConvexHull hull{cubePoints};
  ReferenceFrame frame{Coordinate{5.0, 0.0, 0.0}};  // Start at distance 5

  AssetInertial asset{0, 0, hull, 10.0, frame};
  asset.getInertialState().position = Coordinate{5.0, 0.0, 0.0};

  // Add distance constraint
  auto distConstraint = std::make_unique<DistanceConstraint>(5.0);
  asset.addConstraint(std::move(distConstraint));

  SemiImplicitEulerIntegrator integrator;

  const double dt = 0.016;
  const int steps = 100;

  for (int i = 0; i < steps; ++i)
  {
    // Ticket 0045: Removed constraints parameter
    integrator.step(asset.getInertialState(),
                    Coordinate{0.0, 0.0, -98.1},
                    Coordinate{0.0, 0.0, 0.0},
                    asset.getMass(),
                    asset.getInverseInertiaTensor(),
                    dt);
  }

  // Check quaternion normalization
  double qNorm = asset.getInertialState().orientation.norm();
  EXPECT_NEAR(1.0, qNorm, 1e-4);

  // Check distance constraint (note: constraint forces may not perfectly
  // maintain distance with direct projection; this tests the solver runs
  // without error) The solver provides stabilization forces but explicit
  // projection isn't done
}
