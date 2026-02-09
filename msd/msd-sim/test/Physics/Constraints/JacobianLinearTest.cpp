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
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
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

  Eigen::MatrixXd J = constraint.jacobian(stateA, stateB, 0.0);

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
    Coordinate{
      1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)},
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

    Eigen::MatrixXd J = constraint.jacobian(stateA, stateB, 0.0);

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
  // For a single dynamic body (A) colliding with a static body (B,
  // inverseMass=0), with no rotation (lever arm = 0), the effective mass matrix
  // should be:
  //   A = J * M_inv * J^T = (1/m_A) * n^T * n + 0 = 1/m_A
  //
  // Ticket: 0040b — Split impulse: velocity RHS no longer contains Baumgarte.
  // Use an approaching contact (non-zero velocity) to verify effective mass
  // via the restitution impulse rather than the removed Baumgarte term.

  double const massA = 5.0;
  double const inverseMassA = 1.0 / massA;
  double const inverseMassB = 0.0;  // Static body (infinite mass)

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  // Body A approaching B at 2 m/s, e=0 (perfectly inelastic), no penetration
  double const penetration = 0.0;
  double const restitution = 0.0;
  double const preVelNormal = 0.0;
  double const dt = 1.0 / 60.0;

  auto contact = std::make_unique<ContactConstraint>(0,
                                                     1,
                                                     normal,
                                                     contactA,
                                                     contactB,
                                                     penetration,
                                                     comA,
                                                     comB,
                                                     restitution,
                                                     preVelNormal);

  // Body A moving upward into static body B
  InertialState stateA =
    createDefaultState(Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 2.0});
  InertialState stateB = createDefaultState();

  // A = J * M_inv * J^T = (1/m_A) + eps (single body, linear only)
  double const kRegEps = 1e-8;
  double const expectedA = inverseMassA + kRegEps;

  // RHS: b = -(1+0)*jv + 0 = -jv where jv = -v_A.z = -2
  // b = -(-2) = 2
  double const jv = -2.0;
  double const expectedB = -(1.0 + restitution) * jv;

  // lambda = b / A
  double const expectedLambda = expectedB / expectedA;

  ConstraintSolver solver;
  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{inverseMassA, inverseMassB};

  // Use zero inverse inertia to isolate linear contribution
  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0)
    << "Lambda should be positive (compressive)";

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
  // Ticket: 0040b — Split impulse: use approaching velocity instead of
  // Baumgarte to verify the factor-of-2 effective mass relationship.

  double const mass = 10.0;
  double const inverseMass = 1.0 / mass;

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  double const penetration = 0.0;
  double const restitution = 0.0;
  double const preVelNormal = 0.0;
  double const dt = 1.0 / 60.0;

  auto contact = std::make_unique<ContactConstraint>(0,
                                                     1,
                                                     normal,
                                                     contactA,
                                                     contactB,
                                                     penetration,
                                                     comA,
                                                     comB,
                                                     restitution,
                                                     preVelNormal);

  // Body A approaching B along Z
  InertialState stateA =
    createDefaultState(Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 2.0});
  InertialState stateB =
    createDefaultState(Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, -2.0});

  // Expected effective mass matrix entry:
  // A = (1/m) + (1/m) + eps = 2/m + eps
  double const kRegEps = 1e-8;
  double const expectedA = 2.0 * inverseMass + kRegEps;

  // RHS: b = -(1+0)*jv where jv = -v_A.z + v_B.z = -2 + (-2) = -4
  // b = -(-4) = 4
  double const jv = -4.0;
  double const expectedB = -(1.0 + restitution) * jv;
  double const expectedLambda = expectedB / expectedA;

  ConstraintSolver solver;
  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{inverseMass, inverseMass};

  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0);

  EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-6)
    << "Lambda should equal b/A where A = 2/m + eps (two-body effective mass)";

  // Verify the factor-of-2 ratio: same RHS but single-body A = 1/m + eps
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
  // Ticket: 0040b — Split impulse with approach-velocity-gated slop correction.
  //
  // RHS formula:
  //   For impacts (|jv| > 0.5): b = -(1+e) * jv  (pure restitution)
  //   For resting (|jv| <= 0.5): b = -(1+e) * jv + slopCorrection
  //
  // This test verifies the impact case: approach speed 3 m/s >> 0.5 threshold,
  // so slop correction is zero and RHS is purely restitution-driven.

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
  double const preVelNormal = -3.0;  // Bodies approaching at 3 m/s

  // Body A moving up at 3 m/s, Body B stationary
  InertialState stateA =
    createDefaultState(Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 3.0});
  InertialState stateB =
    createDefaultState(Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, 0.0});

  auto contact = std::make_unique<ContactConstraint>(0,
                                                     1,
                                                     normal,
                                                     contactA,
                                                     contactB,
                                                     penetration,
                                                     comA,
                                                     comB,
                                                     restitution,
                                                     preVelNormal);

  // Manually compute expected RHS:
  // J * v = [-n^T * v_A] + [n^T * v_B] = -1*3 + 0 = -3.0
  double const jv = -3.0;
  // |jv| = 3.0 > 0.5 (impact), so slopCorrection = 0
  double const expectedB = -(1.0 + restitution) * jv;
  // expectedB = -(1.7) * (-3.0) = 5.1

  // Verify through solver: lambda = b / A
  double const kRegEps = 1e-8;
  double const effectiveA = 2.0 * inverseMass + kRegEps;  // Two equal bodies
  double const expectedLambda = expectedB / effectiveA;

  ConstraintSolver solver;
  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{inverseMass, inverseMass};

  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, dt);

  ASSERT_TRUE(result.converged);
  ASSERT_EQ(1, result.lambdas.size());

  // The restitution term should make bodies bounce apart, so lambda > 0
  EXPECT_GT(result.lambdas(0), 0.0)
    << "Lambda should be positive for approaching bodies with restitution";

  // Verify the computed lambda matches our expected RHS calculation
  EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-4)
    << "Lambda should match b/A with pure restitution (no slop for impacts)";

  // Verify the sign: b should be positive (pushing bodies apart)
  EXPECT_GT(expectedB, 0.0)
    << "RHS b should be positive for approaching bodies (negative Jv)";
}

TEST(ContactRHS, SlopCorrectionGatedByApproachVelocity)
{
  // Ticket: 0040b — Split impulse with approach-velocity-gated slop correction.
  //
  // Slop correction formula:
  //   if pen > 0.005 and |jv| <= 0.5:
  //     slopCorrection = min(0.2 * (pen - 0.005) / dt, 1.0)
  //   else:
  //     slopCorrection = 0
  //   b = -(1+e) * jv + slopCorrection
  //
  // Key behaviors:
  // - Resting contacts (|jv| ≈ 0) get gentle position recovery via slop
  // correction
  // - Impacts (|jv| > 0.5) get pure restitution, no slop (restitution handles
  // bounce)
  // - Slop correction capped at 1.0 m/s to prevent energy injection

  double const mass = 10.0;
  double const inverseMass = 1.0 / mass;
  double const dt = 1.0 / 60.0;

  Coordinate normal{0.0, 0.0, 1.0};
  Coordinate contactA{0.0, 0.0, 0.0};
  Coordinate contactB{0.0, 0.0, 0.0};
  Coordinate comA{0.0, 0.0, 0.0};
  Coordinate comB{0.0, 0.0, 0.0};

  double const kRegEps = 1e-8;
  double const effectiveA = 2.0 * inverseMass + kRegEps;

  ConstraintSolver solver;
  Eigen::Matrix3d zeroInertia = Eigen::Matrix3d::Zero();
  std::vector<double> inverseMasses{inverseMass, inverseMass};
  std::vector<Eigen::Matrix3d> inverseInertias{zeroInertia, zeroInertia};

  // Case 1: Resting contact (zero velocity, pen > slop) — slop correction
  // active
  {
    double const penetration = 0.1;
    InertialState stateA = createDefaultState();
    InertialState stateB = createDefaultState();
    std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                    stateB};

    auto contact = std::make_unique<ContactConstraint>(
      0, 1, normal, contactA, contactB, penetration, comA, comB, 0.8, 0.0);

    // |jv| = 0 <= 0.5, pen 0.1 > 0.005
    // slopCorrection = min(0.2 * (0.1 - 0.005) / (1/60), 1.0)
    //                = min(0.2 * 5.7, 1.0) = min(1.14, 1.0) = 1.0
    double const expectedB = 1.0;  // -(1+0.8)*0 + 1.0 = 1.0
    double const expectedLambda = expectedB / effectiveA;

    std::vector<Constraint*> constraints{contact.get()};
    auto result = solver.solve(
      constraints, states, inverseMasses, inverseInertias, 2, dt);

    ASSERT_TRUE(result.converged);
    EXPECT_GT(result.lambdas(0), 0.0)
      << "Resting contact with penetration should get slop correction";
    EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-6)
      << "Lambda should match capped slop correction (1.0 m/s)";
  }

  // Case 2: Impact (approaching at 1 m/s) — no slop correction
  {
    double const penetration = 0.1;
    double const approachSpeed = 1.0;
    double const preImpactVel = -approachSpeed;

    InertialState stateA = createDefaultState(
      Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, approachSpeed});
    InertialState stateB = createDefaultState();
    std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                    stateB};

    double const e = 0.8;
    auto contact = std::make_unique<ContactConstraint>(0,
                                                       1,
                                                       normal,
                                                       contactA,
                                                       contactB,
                                                       penetration,
                                                       comA,
                                                       comB,
                                                       e,
                                                       preImpactVel);

    // |jv| = 1.0 > 0.5, so slopCorrection = 0
    double const jv = -approachSpeed;
    double const expectedB = -(1.0 + e) * jv;  // -(1.8)*(-1.0) = 1.8
    double const expectedLambda = expectedB / effectiveA;

    std::vector<Constraint*> constraints{contact.get()};
    auto result = solver.solve(
      constraints, states, inverseMasses, inverseInertias, 2, dt);

    ASSERT_TRUE(result.converged);
    EXPECT_GT(result.lambdas(0), 0.0);
    EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-6)
      << "Impact: lambda should use pure restitution (no slop correction)";
  }

  // Case 3: Fast impact (approaching at 5 m/s) — no slop correction
  {
    double const penetration = 0.01;
    double const approachSpeed = 5.0;
    double const preImpactVel = -approachSpeed;

    InertialState stateA = createDefaultState(
      Coordinate{0.0, 0.0, 0.0}, Coordinate{0.0, 0.0, approachSpeed});
    InertialState stateB = createDefaultState();
    std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                    stateB};

    double const e = 0.8;
    auto contact = std::make_unique<ContactConstraint>(0,
                                                       1,
                                                       normal,
                                                       contactA,
                                                       contactB,
                                                       penetration,
                                                       comA,
                                                       comB,
                                                       e,
                                                       preImpactVel);

    // |jv| = 5.0 > 0.5, so slopCorrection = 0
    double const jv = -approachSpeed;
    double const expectedB = -(1.0 + e) * jv;  // -(1.8)*(-5.0) = 9.0
    double const expectedLambda = expectedB / effectiveA;

    std::vector<Constraint*> constraints{contact.get()};
    auto result = solver.solve(
      constraints, states, inverseMasses, inverseInertias, 2, dt);

    ASSERT_TRUE(result.converged);
    EXPECT_GT(result.lambdas(0), 0.0);
    EXPECT_NEAR(expectedLambda, result.lambdas(0), 1e-6)
      << "Fast impact: lambda should use pure restitution (no slop correction)";
  }
}
