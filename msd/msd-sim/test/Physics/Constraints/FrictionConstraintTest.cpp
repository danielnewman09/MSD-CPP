// Ticket: 0035a_tangent_basis_and_friction_constraint
// Design: docs/designs/0035a_tangent_basis_and_friction_constraint/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include <Eigen/Dense>
#include <cmath>

using namespace msd_sim;

namespace {
  InertialState createTestState(const Coordinate& pos, const Coordinate& vel, const Coordinate& omega) {
    InertialState state{};
    state.position = pos;
    state.velocity = vel;
    state.setAngularVelocity(omega);
    state.orientation = Eigen::Quaterniond::Identity();
    return state;
  }

  Eigen::MatrixXd computeNumericalJacobian(
      const FrictionConstraint& constraint,
      const InertialState& stateA,
      const InertialState& stateB,
      double time)
  {
    constexpr double h = 1e-7;
    const int dim = constraint.dimension();
    Eigen::MatrixXd J_numerical = Eigen::MatrixXd::Zero(dim, 12);
    Eigen::VectorXd C_base = constraint.evaluateTwoBody(stateA, stateB, time);

    for (int col = 0; col < 12; ++col) {
      InertialState perturbedA = stateA;
      InertialState perturbedB = stateB;

      if (col < 3) {
        perturbedA.velocity[col] += h;
      } else if (col < 6) {
        Coordinate omegaA = perturbedA.getAngularVelocity();
        omegaA[col - 3] += h;
        perturbedA.setAngularVelocity(omegaA);
      } else if (col < 9) {
        perturbedB.velocity[col - 6] += h;
      } else {
        Coordinate omegaB = perturbedB.getAngularVelocity();
        omegaB[col - 9] += h;
        perturbedB.setAngularVelocity(omegaB);
      }

      Eigen::VectorXd C_perturbed = constraint.evaluateTwoBody(perturbedA, perturbedB, time);
      J_numerical.col(col) = (C_perturbed - C_base) / h;
    }

    return J_numerical;
  }
}

TEST(FrictionConstraint, DimensionAlwaysReturnsTwo)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  EXPECT_EQ(constraint.dimension(), 2);
}

TEST(FrictionConstraint, JacobianDimensionsAreTwoByTwelve)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  InertialState stateA = createTestState(comA, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});
  InertialState stateB = createTestState(comB, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});

  Eigen::MatrixXd J = constraint.jacobianTwoBody(stateA, stateB, 0.0);
  EXPECT_EQ(J.rows(), 2);
  EXPECT_EQ(J.cols(), 12);
}

TEST(FrictionConstraint, JacobianRow1MatchesFiniteDifference)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.5, 0.0};
  Coordinate contactPointB{1.0, 0.5, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  InertialState stateA = createTestState(comA, Coordinate{1.0, 2.0, 0.0}, Coordinate{0.1, 0.0, 0.0});
  InertialState stateB = createTestState(comB, Coordinate{0.5, 1.0, 0.0}, Coordinate{0.0, 0.1, 0.0});

  Eigen::MatrixXd J_analytical = constraint.jacobianTwoBody(stateA, stateB, 0.0);
  Eigen::MatrixXd J_numerical = computeNumericalJacobian(constraint, stateA, stateB, 0.0);

  constexpr double kTolerance = 1e-5;
  for (int col = 0; col < 12; ++col) {
    EXPECT_NEAR(J_analytical(0, col), J_numerical(0, col), kTolerance);
  }
}

TEST(FrictionConstraint, JacobianRow2MatchesFiniteDifference)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.5, 0.0};
  Coordinate contactPointB{1.0, 0.5, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  InertialState stateA = createTestState(comA, Coordinate{1.0, 2.0, 0.0}, Coordinate{0.1, 0.0, 0.0});
  InertialState stateB = createTestState(comB, Coordinate{0.5, 1.0, 0.0}, Coordinate{0.0, 0.1, 0.0});

  Eigen::MatrixXd J_analytical = constraint.jacobianTwoBody(stateA, stateB, 0.0);
  Eigen::MatrixXd J_numerical = computeNumericalJacobian(constraint, stateA, stateB, 0.0);

  constexpr double kTolerance = 1e-5;
  for (int col = 0; col < 12; ++col) {
    EXPECT_NEAR(J_analytical(1, col), J_numerical(1, col), kTolerance);
  }
}

TEST(FrictionConstraint, JacobianStructureMatches)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  InertialState stateA = createTestState(comA, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});
  InertialState stateB = createTestState(comB, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});

  Eigen::MatrixXd J = constraint.jacobianTwoBody(stateA, stateB, 0.0);

  Coordinate t1 = constraint.getTangent1();
  Coordinate t2 = constraint.getTangent2();
  Coordinate rA = contactPointA - comA;
  Coordinate rB = contactPointB - comB;

  // Row 0 (t1 direction): Block 1 (vA): t1^T
  EXPECT_NEAR(J(0, 0), t1.x(), 1e-10);
  EXPECT_NEAR(J(0, 1), t1.y(), 1e-10);
  EXPECT_NEAR(J(0, 2), t1.z(), 1e-10);

  // Block 2 (ωA): (rA × t1)^T
  Coordinate rA_cross_t1 = rA.cross(t1);
  EXPECT_NEAR(J(0, 3), rA_cross_t1.x(), 1e-10);
  EXPECT_NEAR(J(0, 4), rA_cross_t1.y(), 1e-10);
  EXPECT_NEAR(J(0, 5), rA_cross_t1.z(), 1e-10);

  // Block 3 (vB): -t1^T
  EXPECT_NEAR(J(0, 6), -t1.x(), 1e-10);
  EXPECT_NEAR(J(0, 7), -t1.y(), 1e-10);
  EXPECT_NEAR(J(0, 8), -t1.z(), 1e-10);

  // Block 4 (ωB): -(rB × t1)^T
  Coordinate rB_cross_t1 = rB.cross(t1);
  EXPECT_NEAR(J(0, 9), -rB_cross_t1.x(), 1e-10);
  EXPECT_NEAR(J(0, 10), -rB_cross_t1.y(), 1e-10);
  EXPECT_NEAR(J(0, 11), -rB_cross_t1.z(), 1e-10);
}

TEST(FrictionConstraint, TangentVectorsAreOrthonormalAndPerpendicularToNormal)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  Coordinate t1 = constraint.getTangent1();
  Coordinate t2 = constraint.getTangent2();

  constexpr double kTolerance = 1e-6;
  EXPECT_NEAR(t1.norm(), 1.0, kTolerance);
  EXPECT_NEAR(t2.norm(), 1.0, kTolerance);
  EXPECT_NEAR(t1.dot(t2), 0.0, kTolerance);
  EXPECT_NEAR(t1.dot(normal), 0.0, kTolerance);
  EXPECT_NEAR(t2.dot(normal), 0.0, kTolerance);
}

TEST(FrictionConstraint, FrictionBoundsWhenNormalForceIsZero)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  constraint.setNormalLambda(0.0);

  auto [lowerBound, upperBound] = constraint.getFrictionBounds();
  EXPECT_NEAR(lowerBound, 0.0, 1e-15);
  EXPECT_NEAR(upperBound, 0.0, 1e-15);
}

TEST(FrictionConstraint, FrictionBoundsForPositiveNormalForce)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  constraint.setNormalLambda(100.0);

  auto [lowerBound, upperBound] = constraint.getFrictionBounds();

  constexpr double expectedBound = 0.5 / 1.41421356237309504880 * 100.0;
  constexpr double kTolerance = 1e-10;

  EXPECT_NEAR(lowerBound, -expectedBound, kTolerance);
  EXPECT_NEAR(upperBound, expectedBound, kTolerance);
}

TEST(FrictionConstraint, FrictionBoundsUpdateWithSetNormalLambda)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.4;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};

  constraint.setNormalLambda(50.0);
  auto [lower1, upper1] = constraint.getFrictionBounds();
  double expected1 = 0.4 / std::sqrt(2.0) * 50.0;
  EXPECT_NEAR(lower1, -expected1, 1e-10);
  EXPECT_NEAR(upper1, expected1, 1e-10);

  constraint.setNormalLambda(200.0);
  auto [lower2, upper2] = constraint.getFrictionBounds();
  double expected2 = 0.4 / std::sqrt(2.0) * 200.0;
  EXPECT_NEAR(lower2, -expected2, 1e-10);
  EXPECT_NEAR(upper2, expected2, 1e-10);

  EXPECT_GT(std::abs(upper2 - upper1), 1e-6);
}

TEST(FrictionConstraint, ActiveWhenFrictionCoefficientAndNormalForcePositive)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  InertialState stateA = createTestState(comA, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});
  InertialState stateB = createTestState(comB, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});

  constraint.setNormalLambda(100.0);
  EXPECT_TRUE(constraint.isActiveTwoBody(stateA, stateB, 0.0));
}

TEST(FrictionConstraint, InactiveWhenFrictionCoefficientIsZero)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.0;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  InertialState stateA = createTestState(comA, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});
  InertialState stateB = createTestState(comB, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});

  constraint.setNormalLambda(100.0);
  EXPECT_FALSE(constraint.isActiveTwoBody(stateA, stateB, 0.0));
}

TEST(FrictionConstraint, InactiveWhenNormalForceIsZero)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  InertialState stateA = createTestState(comA, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});
  InertialState stateB = createTestState(comB, Coordinate{0, 0, 0}, Coordinate{0, 0, 0});

  constraint.setNormalLambda(0.0);
  EXPECT_FALSE(constraint.isActiveTwoBody(stateA, stateB, 0.0));
}

TEST(FrictionConstraint, ConstructionWithNonUnitNormalThrows)
{
  Coordinate nonUnitNormal{2.0, 0.0, 0.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  EXPECT_THROW(
    FrictionConstraint(0, 1, nonUnitNormal, contactPointA, contactPointB, comA, comB, mu),
    std::invalid_argument
  );
}

TEST(FrictionConstraint, ConstructionWithNegativeFrictionCoefficientThrows)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = -0.5;

  EXPECT_THROW(
    FrictionConstraint(0, 1, normal, contactPointA, contactPointB, comA, comB, mu),
    std::invalid_argument
  );
}

TEST(FrictionConstraint, TypeNameReturnsFrictionConstraint)
{
  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactPointA{1.0, 0.0, 0.0};
  Coordinate contactPointB{1.0, 0.0, 0.1};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{1.0, 0.0, 0.0};
  double mu = 0.5;

  FrictionConstraint constraint{0, 1, normal, contactPointA, contactPointB, comA, comB, mu};
  EXPECT_EQ(constraint.typeName(), "FrictionConstraint");
}
