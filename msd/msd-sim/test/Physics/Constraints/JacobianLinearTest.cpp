// Ticket: 0039b_linear_collision_test_suite
// R3: Component Isolation Unit Tests — Jacobian, Effective Mass, RHS Assembly

#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions
// ============================================================================

namespace
{

InertialState createDefaultState(
  const Coordinate& position = Coordinate{0.0, 0.0, 0.0},
  const Coordinate& velocity = Coordinate{0.0, 0.0, 0.0})
{
  InertialState state;
  state.position = position;
  state.velocity = velocity;
  state.acceleration = Coordinate{0.0, 0.0, 0.0};
  state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  state.quaternionRate = Eigen::Vector4d::Zero();
  state.angularAcceleration = AngularRate{0.0, 0.0, 0.0};
  return state;
}

}  // anonymous namespace

// ============================================================================
// R3.1: Jacobian Linear Components
// ============================================================================

TEST(ContactConstraintJacobian, LinearComponentsSymmetric)
{
  // Verify J_linear = [-n^T, n^T] for two-body contact
  // Body A gets -n, Body B gets +n
  //
  // Use a general normal direction to ensure generality.
  Coordinate normal{
    1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  // Lever arms are zero so angular components are zero, isolating linear part
  ContactConstraint constraint{
    0, 1, normal, contactA, contactB, 0.0, comA, comB, 0.5, 0.0};

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  Eigen::MatrixXd J = constraint.jacobianTwoBody(stateA, stateB, 0.0);

  ASSERT_EQ(1, J.rows());
  ASSERT_EQ(12, J.cols());

  // Body A linear components (columns 0-2): -n^T
  EXPECT_NEAR(-normal.x(), J(0, 0), 1e-10);
  EXPECT_NEAR(-normal.y(), J(0, 1), 1e-10);
  EXPECT_NEAR(-normal.z(), J(0, 2), 1e-10);

  // Body B linear components (columns 6-8): +n^T
  EXPECT_NEAR(normal.x(), J(0, 6), 1e-10);
  EXPECT_NEAR(normal.y(), J(0, 7), 1e-10);
  EXPECT_NEAR(normal.z(), J(0, 8), 1e-10);

  // Symmetry check: Body A linear = -(Body B linear)
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(-J(0, i), J(0, 6 + i), 1e-10)
      << "Linear components not antisymmetric at index " << i;
  }

  // Angular components should be zero (lever arms are zero)
  for (int i = 3; i < 6; ++i)
  {
    EXPECT_NEAR(0.0, J(0, i), 1e-10)
      << "Body A angular component should be zero at index " << i;
  }
  for (int i = 9; i < 12; ++i)
  {
    EXPECT_NEAR(0.0, J(0, i), 1e-10)
      << "Body B angular component should be zero at index " << i;
  }
}

TEST(ContactConstraintJacobian, LinearComponentsUnitNormal)
{
  // Verify linear Jacobian components have unit magnitude.
  // The linear part for each body should be a unit vector (equal to n).

  // Test with several different normal directions
  std::vector<Coordinate> normals = {
    Coordinate{1.0, 0.0, 0.0},
    Coordinate{0.0, 1.0, 0.0},
    Coordinate{0.0, 0.0, 1.0},
    Coordinate{1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), 0.0},
    Coordinate{1.0 / std::sqrt(3.0),
               1.0 / std::sqrt(3.0),
               1.0 / std::sqrt(3.0)},
  };

  for (const auto& normal : normals)
  {
    Coordinate contactA{0.0, 0.0, 0.0};
    Coordinate contactB{0.0, 0.0, 0.0};
    Coordinate comA{0.0, 0.0, 0.0};
    Coordinate comB{0.0, 0.0, 0.0};

    ContactConstraint constraint{
      0, 1, normal, contactA, contactB, 0.0, comA, comB, 0.5, 0.0};

    InertialState stateA = createDefaultState();
    InertialState stateB = createDefaultState();

    Eigen::MatrixXd J = constraint.jacobianTwoBody(stateA, stateB, 0.0);

    // Body A linear part: columns 0-2
    Eigen::Vector3d linearA = J.block<1, 3>(0, 0).transpose();
    double magnitudeA = linearA.norm();
    EXPECT_NEAR(1.0, magnitudeA, 1e-10)
      << "Body A linear Jacobian magnitude should be 1 for normal ("
      << normal.x() << ", " << normal.y() << ", " << normal.z() << ")";

    // Body B linear part: columns 6-8
    Eigen::Vector3d linearB = J.block<1, 3>(0, 6).transpose();
    double magnitudeB = linearB.norm();
    EXPECT_NEAR(1.0, magnitudeB, 1e-10)
      << "Body B linear Jacobian magnitude should be 1 for normal ("
      << normal.x() << ", " << normal.y() << ", " << normal.z() << ")";
  }
}

// ============================================================================
// R3.2: Effective Mass Matrix (Linear Only)
// ============================================================================

TEST(EffectiveMass, SingleBody_InverseMassOnly)
{
  // For a single dynamic body (A) colliding with a static body (B, inverseMass=0),
  // with no rotation (lever arm = 0), the effective mass matrix should be:
  //   A = J * M_inv * J^T = (1/m_A) * n^T * n + 0 = 1/m_A
  //
  // Verify by checking the solver result: for a unit contact at rest with
  // known penetration, the computed lambda should match 1/m_A physics.

  double const massA = 5.0;
  double const inverseMassA = 1.0 / massA;
  double const inverseMassB = 0.0;  // Static body (infinite mass)

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  // Zero velocity, zero restitution, known penetration
  double const penetration = 0.1;
  double const restitution = 0.0;
  double const preVelNormal = 0.0;
  double const dt = 1.0 / 60.0;

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, penetration, comA, comB,
    restitution, preVelNormal);

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  // Build the effective mass manually to verify: A = J * M_inv * J^T
  // J = [-n^T, 0, n^T, 0] (1x12 with zero angular parts since lever=0)
  // M_inv for body A: (1/m_A)*I_3, angular: zero (not used, lever=0)
  // M_inv for body B: 0 (static)
  //
  // A = (-n)^T * (1/m_A) * (-n) + n^T * 0 * n
  //   = (1/m_A) * n^T*n = 1/m_A  (since n is unit)
  //
  // With regularization epsilon, A = 1/m_A + eps
  double const kRegEps = 1e-8;
  double const expectedA = inverseMassA + kRegEps;

  // The RHS for zero velocity and zero restitution:
  // b = -(1+0)*J*v + (ERP/dt)*penetration = 0 + (0.2/dt)*0.1
  double const erp = 0.2;
  double const expectedB = (erp / dt) * penetration;

  // lambda = b / A
  double const expectedLambda = expectedB / expectedA;

  ConstraintSolver solver;
  std::vector<TwoBodyConstraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                   stateB};
  std::vector<double> inverseMasses{inverseMassA, inverseMassB};

  // Use zero inverse inertia to isolate linear contribution
  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  auto result = solver.solveWithContacts(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0) << "Lambda should be positive (compressive)";

  // Verify lambda matches expected value from effective mass computation
  EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-6)
    << "Lambda should equal b/A where A = 1/m + eps";
}

TEST(EffectiveMass, TwoBody_EqualMass_HalfEffective)
{
  // For two equal-mass dynamic bodies colliding with zero lever arm:
  //   A = J * M_inv * J^T = (1/m)*n^T*n + (1/m)*n^T*n = 2/m
  // This means the effective mass is m/2.
  //
  // Compare with single-body case to verify the factor-of-2 relationship.

  double const mass = 10.0;
  double const inverseMass = 1.0 / mass;

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  double const penetration = 0.1;
  double const restitution = 0.0;
  double const preVelNormal = 0.0;
  double const dt = 1.0 / 60.0;

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, penetration, comA, comB,
    restitution, preVelNormal);

  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  // Expected effective mass matrix entry:
  // A = (1/m) + (1/m) + eps = 2/m + eps
  double const kRegEps = 1e-8;
  double const expectedA = 2.0 * inverseMass + kRegEps;

  // RHS: same as single-body case (b depends only on velocity and penetration)
  double const erp = 0.2;
  double const expectedB = (erp / dt) * penetration;

  double const expectedLambda = expectedB / expectedA;

  ConstraintSolver solver;
  std::vector<TwoBodyConstraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                   stateB};
  std::vector<double> inverseMasses{inverseMass, inverseMass};

  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  auto result = solver.solveWithContacts(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0);

  EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-6)
    << "Lambda should equal b/A where A = 2/m + eps (two-body effective mass)";

  // Also verify the factor-of-2 ratio against single-body case
  // Single-body: A_single = 1/m + eps, so lambda_single = b / (1/m + eps)
  // Two-body:    A_two    = 2/m + eps, so lambda_two    = b / (2/m + eps)
  // Ratio: lambda_two / lambda_single ≈ (1/m + eps) / (2/m + eps) ≈ 0.5
  double const aSingle = inverseMass + kRegEps;
  double const lambdaSingle = expectedB / aSingle;
  double const ratio = result.lambdas(0) / lambdaSingle;

  EXPECT_NEAR(0.5, ratio, 1e-4)
    << "Two-body lambda should be ~half of single-body lambda for equal masses";
}

// ============================================================================
// R3.3: RHS Assembly (Linear)
// ============================================================================

TEST(ContactRHS, RestitutionTerm_CorrectSign)
{
  // Verify b = -(1+e) * v_rel_normal + Baumgarte_term
  //
  // Setup: two bodies approaching along Z axis with known relative velocity.
  // Body A moving up, Body B stationary => relative velocity = v_B - v_A = -v_A
  // along normal direction.
  //
  // With normal = (0,0,1), J = [-n^T, 0, n^T, 0]
  // J * v = -v_A.z + v_B.z = relative normal velocity (B's approach minus A's)
  //
  // RHS formula: b = -(1+e) * J*v + (ERP/dt) * penetration

  double const mass = 10.0;
  double const inverseMass = 1.0 / mass;
  double const restitution = 0.7;
  double const dt = 1.0 / 60.0;

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  double const penetration = 0.05;
  double const preVelNormal = -3.0;  // Bodies approaching

  // Body A moving up at 3 m/s, Body B stationary
  InertialState stateA = createDefaultState(
    Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 3.0});
  InertialState stateB = createDefaultState(
    Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0});

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, penetration, comA, comB,
    restitution, preVelNormal);

  // Manually compute expected RHS:
  // J * v = [-n^T * v_A] + [n^T * v_B] = -1*3 + 0 = -3.0
  double const jv = -3.0;
  double const erp = 0.2;
  double const expectedB =
    -(1.0 + restitution) * jv + (erp / dt) * penetration;

  // expectedB = -(1.7) * (-3.0) + (0.2/dt)*0.05
  //           = 5.1 + (12.0)*0.05
  //           = 5.1 + 0.6
  //           = 5.7

  // Verify through solver: lambda = b / A
  double const kRegEps = 1e-8;
  double const effectiveA = 2.0 * inverseMass + kRegEps;  // Two equal bodies
  double const expectedLambda = expectedB / effectiveA;

  ConstraintSolver solver;
  std::vector<TwoBodyConstraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                   stateB};
  std::vector<double> inverseMasses{inverseMass, inverseMass};

  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  auto result = solver.solveWithContacts(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());

  // The restitution term should make bodies bounce apart, so lambda > 0
  EXPECT_GT(result.lambdas(0), 0.0)
    << "Lambda should be positive for approaching bodies with restitution";

  // Verify the computed lambda matches our expected RHS calculation
  EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-4)
    << "Lambda should match b/A where b includes -(1+e)*Jv restitution term";

  // Verify the sign: b should be positive (pushing bodies apart)
  EXPECT_GT(expectedB, 0.0)
    << "RHS b should be positive for approaching bodies (negative Jv)";
}

TEST(ContactRHS, ZeroVelocity_OnlyBaumgarte)
{
  // When v_rel = 0, only the Baumgarte stabilization term contributes to RHS.
  // b = -(1+e) * 0 + (ERP/dt) * penetration = (ERP/dt) * penetration
  //
  // This test verifies that with zero velocity, the only contribution is the
  // Baumgarte position-correction term.

  double const mass = 10.0;
  double const inverseMass = 1.0 / mass;
  double const restitution = 0.8;  // Restitution is irrelevant at zero velocity
  double const dt = 1.0 / 60.0;

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  double const penetration = 0.1;

  // Both bodies at rest
  InertialState stateA = createDefaultState();
  InertialState stateB = createDefaultState();

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, penetration, comA, comB,
    restitution, 0.0);

  // Expected RHS: b = -(1+e)*0 + (ERP/dt)*penetration = (ERP/dt)*penetration
  double const erp = 0.2;
  double const expectedB = (erp / dt) * penetration;

  // Expected lambda: b / A
  double const kRegEps = 1e-8;
  double const effectiveA = 2.0 * inverseMass + kRegEps;
  double const expectedLambda = expectedB / effectiveA;

  ConstraintSolver solver;
  std::vector<TwoBodyConstraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                   stateB};
  std::vector<double> inverseMasses{inverseMass, inverseMass};

  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  auto result = solver.solveWithContacts(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0)
    << "Lambda should be positive from Baumgarte correction alone";

  EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-6)
    << "Lambda should match (ERP/dt)*penetration / A, with no velocity term";

  // Verify that the result doesn't depend on restitution by comparing with e=0
  auto contactNoRestitution = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, penetration, comA, comB,
    0.0, 0.0);

  std::vector<TwoBodyConstraint*> constraintsNoRest{
    contactNoRestitution.get()};

  auto resultNoRest = solver.solveWithContacts(
    constraintsNoRest, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(resultNoRest.converged);
  EXPECT_NEAR(result.lambdas(0), resultNoRest.lambdas(0), 1e-10)
    << "At zero velocity, restitution value should not affect lambda";
}
