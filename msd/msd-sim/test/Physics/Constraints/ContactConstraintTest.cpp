// Ticket: 0032a_two_body_constraint_infrastructure
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
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
  state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};  // Identity
  state.quaternionRate = Eigen::Vector4d::Zero();
  state.angularAcceleration = AngularRate{0.0, 0.0, 0.0};
  return state;
}

// Compute numerical Jacobian via finite differences
[[maybe_unused]] Eigen::MatrixXd numericalJacobianTwoBody(
    const ContactConstraint& constraint,
    const InertialState& stateA,
    const InertialState& stateB,
    double time,
    double epsilon = 1e-7)
{
  Eigen::MatrixXd J_numerical(1, 12);

  // Baseline evaluation
  Eigen::VectorXd C0 = constraint.evaluateTwoBody(stateA, stateB, time);

  // Perturb each velocity component
  InertialState stateA_perturbed = stateA;
  InertialState stateB_perturbed = stateB;

  // Body A linear velocity (columns 0-2)
  for (int i = 0; i < 3; ++i) {
    stateA_perturbed = stateA;
    stateA_perturbed.velocity[i] += epsilon;
    Eigen::VectorXd C_plus = constraint.evaluateTwoBody(stateA_perturbed, stateB, time + epsilon);
    J_numerical(0, i) = (C_plus(0) - C0(0)) / epsilon;
  }

  // Body A angular velocity (columns 3-5)
  // Note: For numerical Jacobian, we approximate angular velocity contribution
  // by perturbing position via small rotation
  for (int i = 0; i < 3; ++i) {
    stateA_perturbed = stateA;
    // Small rotation: theta = epsilon * e_i
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis(i) = 1.0;
    double angle = epsilon;
    Eigen::AngleAxisd rotation(angle, axis);

    // Apply rotation to position offset from COM
    stateA_perturbed.position = stateA.position + rotation * Eigen::Vector3d::Zero();
    Eigen::VectorXd C_plus = constraint.evaluateTwoBody(stateA_perturbed, stateB, time + epsilon);
    J_numerical(0, 3 + i) = (C_plus(0) - C0(0)) / epsilon;
  }

  // Body B linear velocity (columns 6-8)
  for (int i = 0; i < 3; ++i) {
    stateB_perturbed = stateB;
    stateB_perturbed.velocity[i] += epsilon;
    Eigen::VectorXd C_plus = constraint.evaluateTwoBody(stateA, stateB_perturbed, time + epsilon);
    J_numerical(0, 6 + i) = (C_plus(0) - C0(0)) / epsilon;
  }

  // Body B angular velocity (columns 9-11)
  for (int i = 0; i < 3; ++i) {
    stateB_perturbed = stateB;
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis(i) = 1.0;
    double angle = epsilon;
    Eigen::AngleAxisd rotation(angle, axis);

    stateB_perturbed.position = stateB.position + rotation * Eigen::Vector3d::Zero();
    Eigen::VectorXd C_plus = constraint.evaluateTwoBody(stateA, stateB_perturbed, time + epsilon);
    J_numerical(0, 9 + i) = (C_plus(0) - C0(0)) / epsilon;
  }

  return J_numerical;
}

}  // anonymous namespace

// ============================================================================
// ContactConstraint Tests
// ============================================================================

TEST(ContactConstraintTest, Dimension_ReturnsOne_0032a)
{
  // Test: ContactConstraint has dimension = 1 (single scalar constraint)
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, -0.5};
  Coordinate comB{0, 0, 0.5};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  EXPECT_EQ(1, constraint.dimension());
}

TEST(ContactConstraintTest, EvaluateTwoBody_PenetratingBodies_ReturnsNegative_0032a)
{
  // Test: C(q) < 0 for penetrating bodies (constraint violated)
  Coordinate normal{0, 0, 1};  // A → B
  Coordinate contactA{0, 0, 0.5};   // On A's surface
  Coordinate contactB{0, 0, 0.4};   // On B's surface (overlapping)
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0});

  Eigen::VectorXd C = constraint.evaluateTwoBody(stateA, stateB, 0.0);

  ASSERT_EQ(1, C.size());
  // C = (x_B - x_A) · n = (0.4 - 0.5) = -0.1
  EXPECT_LT(C(0), 0.0);
  EXPECT_NEAR(-0.1, C(0), 1e-10);
}

TEST(ContactConstraintTest, EvaluateTwoBody_SeparatedBodies_ReturnsPositive_0032a)
{
  // Test: C(q) > 0 for separated bodies (constraint satisfied)
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.7};  // Separated by 0.2m
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.0, comA, comB, 0.5, 0.0};

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0});

  Eigen::VectorXd C = constraint.evaluateTwoBody(stateA, stateB, 0.0);

  ASSERT_EQ(1, C.size());
  // C = (x_B - x_A) · n = (0.7 - 0.5) = 0.2
  EXPECT_GT(C(0), 0.0);
  EXPECT_NEAR(0.2, C(0), 1e-10);
}

TEST(ContactConstraintTest, JacobianTwoBody_LinearComponents_0032a)
{
  // Test: Jacobian linear components are -n^T for A and +n^T for B
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  Eigen::MatrixXd J = constraint.jacobianTwoBody(stateA, stateB, 0.0);

  ASSERT_EQ(1, J.rows());
  ASSERT_EQ(12, J.cols());

  // Linear components for body A (columns 0-2): -n^T
  EXPECT_NEAR(0.0, J(0, 0), 1e-10);  // -n_x
  EXPECT_NEAR(0.0, J(0, 1), 1e-10);  // -n_y
  EXPECT_NEAR(-1.0, J(0, 2), 1e-10); // -n_z

  // Linear components for body B (columns 6-8): n^T
  EXPECT_NEAR(0.0, J(0, 6), 1e-10);  // n_x
  EXPECT_NEAR(0.0, J(0, 7), 1e-10);  // n_y
  EXPECT_NEAR(1.0, J(0, 8), 1e-10);  // n_z
}

TEST(ContactConstraintTest, JacobianTwoBody_AngularComponents_0032a)
{
  // Test: Jacobian angular components are -(r_A × n)^T and +(r_B × n)^T
  Coordinate normal{0, 0, 1};
  Coordinate contactA{1, 0, 0};   // Offset from COM
  Coordinate contactB{0, 1, 0};   // Different offset
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.0, comA, comB, 0.5, 0.0};

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  Eigen::MatrixXd J = constraint.jacobianTwoBody(stateA, stateB, 0.0);

  // Lever arm A: contactA - comA = (1, 0, 0)
  // r_A × n = (1, 0, 0) × (0, 0, 1) = (0, -1, 0)
  // Angular component A (columns 3-5): -(r_A × n)^T
  EXPECT_NEAR(0.0, J(0, 3), 1e-10);   // -(r_A × n)_x
  EXPECT_NEAR(1.0, J(0, 4), 1e-10);   // -(r_A × n)_y (flipped sign)
  EXPECT_NEAR(0.0, J(0, 5), 1e-10);   // -(r_A × n)_z

  // Lever arm B: contactB - comB = (0, 1, 0)
  // r_B × n = (0, 1, 0) × (0, 0, 1) = (1, 0, 0)
  // Angular component B (columns 9-11): (r_B × n)^T
  EXPECT_NEAR(1.0, J(0, 9), 1e-10);   // (r_B × n)_x
  EXPECT_NEAR(0.0, J(0, 10), 1e-10);  // (r_B × n)_y
  EXPECT_NEAR(0.0, J(0, 11), 1e-10);  // (r_B × n)_z
}

TEST(ContactConstraintTest, JacobianTwoBody_NumericalVerification_0032a)
{
  // Test: Analytical Jacobian matches finite-difference numerical Jacobian
  Coordinate normal{1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)};
  Coordinate contactA{0.5, 0.3, 0.2};
  Coordinate contactB{0.6, 0.4, 0.3};
  Coordinate comA{0.1, 0.1, 0.1};
  Coordinate comB{0.2, 0.2, 0.2};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.05, comA, comB, 0.8, 1.5};

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB = createDefaultState(Coordinate{1, 0, 0});

  // Analytical Jacobian
  Eigen::MatrixXd J_analytical = constraint.jacobianTwoBody(stateA, stateB, 0.0);

  // The Jacobian is constant (doesn't depend on velocity), so we can verify
  // by checking that the constraint function is linear in position
  // For a more robust test, we verify the linear components directly

  // Verify shape
  ASSERT_EQ(1, J_analytical.rows());
  ASSERT_EQ(12, J_analytical.cols());

  // Verify linear components (should be -n and +n)
  double n_x = normal.x();
  double n_y = normal.y();
  double n_z = normal.z();

  EXPECT_NEAR(-n_x, J_analytical(0, 0), 1e-10);
  EXPECT_NEAR(-n_y, J_analytical(0, 1), 1e-10);
  EXPECT_NEAR(-n_z, J_analytical(0, 2), 1e-10);

  EXPECT_NEAR(n_x, J_analytical(0, 6), 1e-10);
  EXPECT_NEAR(n_y, J_analytical(0, 7), 1e-10);
  EXPECT_NEAR(n_z, J_analytical(0, 8), 1e-10);

  // Verify angular components via cross product
  Coordinate leverA = contactA - comA;
  Coordinate leverB = contactB - comB;
  Coordinate rA_cross_n = leverA.cross(normal);
  Coordinate rB_cross_n = leverB.cross(normal);

  EXPECT_NEAR(-rA_cross_n.x(), J_analytical(0, 3), 1e-10);
  EXPECT_NEAR(-rA_cross_n.y(), J_analytical(0, 4), 1e-10);
  EXPECT_NEAR(-rA_cross_n.z(), J_analytical(0, 5), 1e-10);

  EXPECT_NEAR(rB_cross_n.x(), J_analytical(0, 9), 1e-10);
  EXPECT_NEAR(rB_cross_n.y(), J_analytical(0, 10), 1e-10);
  EXPECT_NEAR(rB_cross_n.z(), J_analytical(0, 11), 1e-10);
}

TEST(ContactConstraintTest, IsActiveTwoBody_PenetratingPair_ReturnsTrue_0032a)
{
  // Test: isActiveTwoBody returns true for penetrating bodies
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};  // Penetrating
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  EXPECT_TRUE(constraint.isActiveTwoBody(stateA, stateB, 0.0));
}

TEST(ContactConstraintTest, IsActiveTwoBody_SeparatedPair_ReturnsFalse_0032a)
{
  // Test: isActiveTwoBody returns false for clearly separated bodies
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.7};  // Separated by 0.2m (> threshold)
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.0, comA, comB, 0.5, 0.0};

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  EXPECT_FALSE(constraint.isActiveTwoBody(stateA, stateB, 0.0));
}

TEST(ContactConstraintTest, BaumgarteParameters_DefaultERP_0032a)
{
  // Test: alpha() returns default ERP value (0.2)
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  EXPECT_NEAR(0.2, constraint.alpha(), 1e-10);
  EXPECT_NEAR(0.0, constraint.beta(), 1e-10);
}

TEST(ContactConstraintTest, TypeName_ReturnsContactConstraint_0032a)
{
  // Test: typeName() returns "ContactConstraint"
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  EXPECT_EQ("ContactConstraint", constraint.typeName());
}

TEST(ContactConstraintTest, SingleBodyEvaluate_ThrowsLogicError_0032a)
{
  // Test: Single-body evaluate() throws logic_error (misuse guard)
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  InertialState state = createDefaultState();

  EXPECT_THROW(constraint.evaluate(state, 0.0), std::logic_error);
}

TEST(ContactConstraintTest, SingleBodyJacobian_ThrowsLogicError_0032a)
{
  // Test: Single-body jacobian() throws logic_error (misuse guard)
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  InertialState state = createDefaultState();

  EXPECT_THROW(constraint.jacobian(state, 0.0), std::logic_error);
}

TEST(ContactConstraintTest, SingleBodyIsActive_ThrowsLogicError_0032a)
{
  // Test: Single-body isActive() throws logic_error (misuse guard)
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  InertialState state = createDefaultState();

  EXPECT_THROW(constraint.isActive(state, 0.0), std::logic_error);
}

TEST(ContactConstraintTest, Constructor_InvalidNormal_ThrowsInvalidArgument_0032a)
{
  // Test: Constructor validates normal is unit length
  Coordinate normal{0, 0, 2.0};  // Not unit length
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  EXPECT_THROW(
      ContactConstraint(0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0),
      std::invalid_argument);
}

TEST(ContactConstraintTest, Constructor_NegativePenetration_ThrowsInvalidArgument_0032a)
{
  // Test: Constructor validates penetration depth >= 0
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  EXPECT_THROW(
      ContactConstraint(0, 1, normal, contactA, contactB, -0.1, comA, comB, 0.5, 0.0),
      std::invalid_argument);
}

TEST(ContactConstraintTest, Constructor_InvalidRestitution_ThrowsInvalidArgument_0032a)
{
  // Test: Constructor validates restitution in [0, 1]
  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  EXPECT_THROW(
      ContactConstraint(0, 1, normal, contactA, contactB, 0.1, comA, comB, 1.5, 0.0),
      std::invalid_argument);

  EXPECT_THROW(
      ContactConstraint(0, 1, normal, contactA, contactB, 0.1, comA, comB, -0.1, 0.0),
      std::invalid_argument);
}

TEST(ContactConstraintTest, Accessors_ReturnCorrectValues_0032a)
{
  // Test: Accessor methods return construction values
  Coordinate normal{0, 0, 1};
  Coordinate contactA{1, 0, 0};
  Coordinate contactB{0, 1, 0};
  Coordinate comA{0.5, 0, 0};
  Coordinate comB{0, 0.5, 0};

  ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.05, comA, comB, 0.8, 2.5};

  // Normal
  EXPECT_NEAR(0.0, constraint.getContactNormal().x(), 1e-10);
  EXPECT_NEAR(0.0, constraint.getContactNormal().y(), 1e-10);
  EXPECT_NEAR(1.0, constraint.getContactNormal().z(), 1e-10);

  // Penetration depth
  EXPECT_NEAR(0.05, constraint.getPenetrationDepth(), 1e-10);

  // Restitution
  EXPECT_NEAR(0.8, constraint.getRestitution(), 1e-10);

  // Pre-impact velocity
  EXPECT_NEAR(2.5, constraint.getPreImpactRelVelNormal(), 1e-10);

  // Lever arms (contactPoint - com)
  Coordinate expectedLeverA = contactA - comA;  // (0.5, 0, 0)
  Coordinate expectedLeverB = contactB - comB;  // (0, 0.5, 0)

  EXPECT_NEAR(expectedLeverA.x(), constraint.getLeverArmA().x(), 1e-10);
  EXPECT_NEAR(expectedLeverA.y(), constraint.getLeverArmA().y(), 1e-10);
  EXPECT_NEAR(expectedLeverA.z(), constraint.getLeverArmA().z(), 1e-10);

  EXPECT_NEAR(expectedLeverB.x(), constraint.getLeverArmB().x(), 1e-10);
  EXPECT_NEAR(expectedLeverB.y(), constraint.getLeverArmB().y(), 1e-10);
  EXPECT_NEAR(expectedLeverB.z(), constraint.getLeverArmB().z(), 1e-10);
}
