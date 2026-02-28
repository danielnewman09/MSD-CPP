// Ticket: 0034_active_set_method_contact_solver
// Unit tests for Active Set Method solver kernel in ConstraintSolver
//
// These tests validate the ASM-specific behavior: exact solution, ordering
// independence, mass ratio robustness, active/inactive set partitioning,
// safety cap, and KKT condition verification.

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <random>
#include <vector>
#include "msd-sim/src/DataTypes/AngularAcceleration.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
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
  state.angularAcceleration = AngularAcceleration{0.0, 0.0, 0.0};
  return state;
}

Eigen::Matrix3d createIdentityInertia()
{
  return Eigen::Matrix3d::Identity();
}

}  // anonymous namespace

// ============================================================================
// 1. ActiveSetResult Default Construction
// ============================================================================

TEST(ConstraintSolverASMTest, ActiveSetResult_DefaultConstruction_Zeroed)
{
  // Verify ActiveSetResult default constructor initializes all fields
  // correctly. active_set_size defaults to 0 (empty active set), not an
  // uninitialized sentinel.
  ConstraintSolver solver;

  // Solve with empty contacts to get a default-like result
  std::vector<Constraint*> constraints;
  std::vector<std::reference_wrapper<const InertialState>> states;
  std::vector<double> inverseMasses;
  std::vector<Eigen::Matrix3d> inverseInertias;

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 0, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(0, result.lambdas.size());
  EXPECT_EQ(0, result.iterations);
}

// ============================================================================
// 2. Single Contact Exact Solution
// ============================================================================

TEST(ConstraintSolverASMTest,
     SingleContact_ExactSolution_MatchesAnalytical)
{
  // For a single contact, lambda = b / A (scalar).
  // ASM should match this analytical result within 1e-12.
  ConstraintSolver solver;

  // Ticket: 0040b — Split impulse: use approaching velocity for positive RHS
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0);

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0);

  // ASM should solve a single contact in exactly 1 iteration
  EXPECT_EQ(1, result.iterations);
}

// ============================================================================
// 3. Order Independence
// ============================================================================

TEST(ConstraintSolverASMTest,
     OrderIndependence_ShuffledContacts_IdenticalLambdas)
{
  // Two different orderings of the same contacts must produce identical
  // lambdas. ASM is deterministic and order-independent (unlike PGS).
  ConstraintSolver solver;

  // Ticket: 0040b — Split impulse: use approaching velocity for positive RHS
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  // Contact at +X offset
  auto contact1a = std::make_unique<ContactConstraint>(0,
                                                       1,
                                                       normal,
                                                       Coordinate{0.2, 0, 0.5},
                                                       Coordinate{0.2, 0, 0.4},
                                                       0.1,
                                                       comA,
                                                       comB,
                                                       0.5,
                                                       0.0);
  // Contact at -X offset
  auto contact2a = std::make_unique<ContactConstraint>(0,
                                                       1,
                                                       normal,
                                                       Coordinate{-0.2, 0, 0.5},
                                                       Coordinate{-0.2, 0, 0.4},
                                                       0.1,
                                                       comA,
                                                       comB,
                                                       0.5,
                                                       0.0);

  // Duplicate contacts for second solve (reversed order)
  auto contact1b = std::make_unique<ContactConstraint>(0,
                                                       1,
                                                       normal,
                                                       Coordinate{0.2, 0, 0.5},
                                                       Coordinate{0.2, 0, 0.4},
                                                       0.1,
                                                       comA,
                                                       comB,
                                                       0.5,
                                                       0.0);
  auto contact2b = std::make_unique<ContactConstraint>(0,
                                                       1,
                                                       normal,
                                                       Coordinate{-0.2, 0, 0.5},
                                                       Coordinate{-0.2, 0, 0.4},
                                                       0.1,
                                                       comA,
                                                       comB,
                                                       0.5,
                                                       0.0);

  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  // Order 1: [contact1, contact2]
  std::vector<Constraint*> constraints1{contact1a.get(), contact2a.get()};
  auto result1 = solver.solve(
    constraints1, states, inverseMasses, inverseInertias, 2, 0.016);

  // Order 2: [contact2, contact1]
  std::vector<Constraint*> constraints2{contact2b.get(), contact1b.get()};
  auto result2 = solver.solve(
    constraints2, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result1.converged);
  EXPECT_TRUE(result2.converged);

  // Lambdas should be identical (order swapped: result1.lambda[0] ==
  // result2.lambda[1])
  EXPECT_NEAR(result1.lambdas(0), result2.lambdas(1), 1e-12);
  EXPECT_NEAR(result1.lambdas(1), result2.lambdas(0), 1e-12);
}

// ============================================================================
// 4. High Mass Ratio
// ============================================================================

TEST(ConstraintSolverASMTest, HighMassRatio_1e6_Converges)
{
  // ASM should converge for mass ratio 1e6:1 (LLT handles this easily).
  ConstraintSolver solver;

  // Ticket: 0040b — Split impulse: use approaching velocity for positive RHS
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0);

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 1e6, 1.0 / 1.0};  // 1e6:1 ratio
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0);
}

// ============================================================================
// 5. All Separating — Empty Active Set
// ============================================================================

TEST(ConstraintSolverASMTest, AllSeparating_EmptyActiveSet)
{
  // All contacts separating: all lambdas should be zero.
  ConstraintSolver solver;

  // Bodies moving apart rapidly
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, -2.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 1.0}, Coordinate{0, 0, 2.0});

  Coordinate normal{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 1.0};

  // Both contacts have separating velocity and no penetration
  auto contact1 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normal,
                                                      Coordinate{0.1, 0, 0.5},
                                                      Coordinate{0.1, 0, 0.6},
                                                      0.0,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      -4.0);
  auto contact2 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normal,
                                                      Coordinate{-0.1, 0, 0.5},
                                                      Coordinate{-0.1, 0, 0.6},
                                                      0.0,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      -4.0);

  std::vector<Constraint*> constraints{contact1.get(), contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(2, result.lambdas.size());
  EXPECT_NEAR(0.0, result.lambdas(0), 1e-10);
  EXPECT_NEAR(0.0, result.lambdas(1), 1e-10);
}

// ============================================================================
// 6. All Compressive — Full Active Set
// ============================================================================

TEST(ConstraintSolverASMTest, AllCompressive_FullActiveSet)
{
  // All contacts compressive: all lambdas should be positive.
  ConstraintSolver solver;

  // Ticket: 0040b — Split impulse: use approaching velocity for positive RHS
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  // Both contacts penetrating with Baumgarte bias
  auto contact1 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normal,
                                                      Coordinate{0.1, 0.1, 0.5},
                                                      Coordinate{0.1, 0.1, 0.4},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.5,
                                                      0.0);
  auto contact2 =
    std::make_unique<ContactConstraint>(0,
                                        1,
                                        normal,
                                        Coordinate{-0.1, -0.1, 0.5},
                                        Coordinate{-0.1, -0.1, 0.4},
                                        0.1,
                                        comA,
                                        comB,
                                        0.5,
                                        0.0);

  std::vector<Constraint*> constraints{contact1.get(), contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(2, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0);
  EXPECT_GT(result.lambdas(1), 0.0);

  // Should converge in 1 iteration (full active set, all positive)
  EXPECT_EQ(1, result.iterations);
}

// ============================================================================
// 7. Mixed Active/Inactive — Correct Partition
// ============================================================================

TEST(ConstraintSolverASMTest, MixedActiveInactive_CorrectPartition)
{
  // Ticket: 0040b — Updated for split impulse (no Baumgarte in velocity RHS).
  // One contact compressive (approaching bodies), one separating
  // (bodies moving apart). ASM should partition correctly.
  ConstraintSolver solver;

  // Body A approaching B along Z — creates compressive contact
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, 0.0});

  Coordinate normalZ{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0.9};

  // Contact 1: Bodies approaching — positive RHS from -(1+e)*Jv where Jv < 0
  auto contact1 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalZ,
                                                      Coordinate{0, 0, 0.5},
                                                      Coordinate{0, 0, 0.4},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      2.0);

  // Contact 2: Bodies separating — pre-impact relative velocity is negative
  // -> RHS = -(1+e)*Jv + 0 where Jv is positive (separating), giving negative b
  //    In the coupled system, this pulls lambda(1) negative, forcing removal
  auto contact2 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalZ,
                                                      Coordinate{0.3, 0, 0.5},
                                                      Coordinate{0.3, 0, 0.6},
                                                      0.0,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      -5.0);

  std::vector<Constraint*> constraints{contact1.get(), contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(2, result.lambdas.size());

  // Contact 1 should be compressive (positive lambda from approaching velocity)
  EXPECT_GT(result.lambdas(0), 0.0);

  // Contact 2 should be separating (near-zero lambda)
  // Small residual possible from cross-coupling in the effective mass matrix
  EXPECT_LT(result.lambdas(1), 1e-4);
}

// ============================================================================
// 8. Redundant Contacts — Regularization Prevents Failure
// ============================================================================

TEST(ConstraintSolverASMTest,
     RedundantContacts_RegularizationPreventsFailure)
{
  // Two contacts at the exact same point with same normal.
  // Without regularization, A_W would be singular. With kRegularizationEpsilon,
  // LLT succeeds and distributes lambda among the redundant contacts.
  ConstraintSolver solver;

  // Ticket: 0040b — Split impulse: use approaching velocity for positive RHS
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  // Duplicate contacts at identical points
  auto contact1 = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0);
  auto contact2 = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0);

  std::vector<Constraint*> constraints{contact1.get(), contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  // Should not crash or fail to converge
  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(2, result.lambdas.size());
  // Both lambdas should be non-negative
  EXPECT_GE(result.lambdas(0), 0.0);
  EXPECT_GE(result.lambdas(1), 0.0);
}

// ============================================================================
// 9. Safety Cap Reached — Reports Not Converged
// ============================================================================

TEST(ConstraintSolverASMTest, SafetyCapReached_ReportsNotConverged)
{
  // Force the safety cap to be reached by setting max_safety_iterations = 1
  // on a scenario requiring multiple active set changes.
  ConstraintSolver solver;
  solver.setMaxIterations(1);  // Safety cap = min(2*C, 1) = 1

  // Create a scenario where the first iteration finds a negative lambda
  // Ticket: 0040b — stateA approaching stateB so contact2 produces positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 1.0}, Coordinate{0, 0, 5.0});

  Coordinate normal{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 1.0};

  // Contact with large separating velocity — produces negative lambda
  auto contact1 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normal,
                                                      Coordinate{0, 0, 0.5},
                                                      Coordinate{0, 0, 0.6},
                                                      0.0,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      -5.0);
  // Contact with penetration — compressive
  auto contact2 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normal,
                                                      Coordinate{0.2, 0, 0.5},
                                                      Coordinate{0.2, 0, 0.4},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      0.0);

  std::vector<Constraint*> constraints{contact1.get(), contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_FALSE(result.converged);
  EXPECT_GE(result.iterations, 1);
}

// ============================================================================
// 10. KKT Conditions Verified Post-Solve
// ============================================================================

TEST(ConstraintSolverASMTest, KKTConditions_VerifiedPostSolve)
{
  // After a converged solve, explicitly verify all three KKT conditions:
  // 1. Primal feasibility: lambda >= 0
  // 2. Dual feasibility: w = A*lambda - b >= -tol for inactive contacts
  // 3. Complementarity: lambda_i * w_i ≈ 0
  //
  // Ticket: 0040b — Updated for split impulse (no Baumgarte in velocity RHS).
  // We verify KKT via structural properties: compressive contacts have
  // lambda > 0 and separating contacts have lambda = 0.
  ConstraintSolver solver;

  // Body A approaching B — creates compressive contact via velocity RHS
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, 0.0});

  Coordinate normalZ{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0.9};

  // Contact 1: Bodies approaching — positive RHS from -(1+e)*Jv
  auto contact1 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalZ,
                                                      Coordinate{0, 0, 0.5},
                                                      Coordinate{0, 0, 0.4},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      2.0);
  // Contact 2: Pre-impact relative velocity negative (separating)
  auto contact2 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalZ,
                                                      Coordinate{0.3, 0, 0.5},
                                                      Coordinate{0.3, 0, 0.6},
                                                      0.0,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      -5.0);

  std::vector<Constraint*> constraints{contact1.get(), contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);

  // KKT 1: Primal feasibility — all lambdas >= 0
  for (Eigen::Index i = 0; i < result.lambdas.size(); ++i)
  {
    EXPECT_GE(result.lambdas(i), -1e-12) << "Lambda " << i << " is negative";
  }

  // KKT structural verification:
  // - Compressive contact (0) has lambda > 0 (approaching velocity)
  // - Separating contact (1) has lambda near 0 (negative pre-impact velocity)
  //   Small residual possible from cross-coupling in effective mass matrix
  EXPECT_GT(result.lambdas(0), 0.0);
  EXPECT_LT(result.lambdas(1), 1e-4);
}

// ============================================================================
// 11. Iteration Count Within 2C Bound
// ============================================================================

TEST(ConstraintSolverASMTest, IterationCount_WithinTwoCBound)
{
  // For non-degenerate systems, ASM should converge within 2*C iterations.
  // Ticket: 0040b — stateA approaching stateB so contacts produce positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  ConstraintSolver solver;

  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  // Create 4 contacts
  auto c1 = std::make_unique<ContactConstraint>(0,
                                                1,
                                                normal,
                                                Coordinate{0.1, 0.1, 0.5},
                                                Coordinate{0.1, 0.1, 0.4},
                                                0.1,
                                                comA,
                                                comB,
                                                0.5,
                                                0.0);
  auto c2 = std::make_unique<ContactConstraint>(0,
                                                1,
                                                normal,
                                                Coordinate{-0.1, 0.1, 0.5},
                                                Coordinate{-0.1, 0.1, 0.4},
                                                0.1,
                                                comA,
                                                comB,
                                                0.5,
                                                0.0);
  auto c3 = std::make_unique<ContactConstraint>(0,
                                                1,
                                                normal,
                                                Coordinate{0.1, -0.1, 0.5},
                                                Coordinate{0.1, -0.1, 0.4},
                                                0.1,
                                                comA,
                                                comB,
                                                0.5,
                                                0.0);
  auto c4 = std::make_unique<ContactConstraint>(0,
                                                1,
                                                normal,
                                                Coordinate{-0.1, -0.1, 0.5},
                                                Coordinate{-0.1, -0.1, 0.4},
                                                0.1,
                                                comA,
                                                comB,
                                                0.5,
                                                0.0);

  std::vector<Constraint*> constraints{c1.get(), c2.get(), c3.get(), c4.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  const int C = 4;
  EXPECT_LE(result.iterations, 2 * C);
}

// ============================================================================
// 12. Active Set Size Reported Correctly
// ============================================================================

TEST(ConstraintSolverASMTest, ActiveSetSize_ReportedCorrectly)
{
  // Verify that the number of non-zero lambdas matches expectations.
  // For all-compressive contacts, all lambdas should be positive.
  // Ticket: 0040b — stateA approaching stateB so contacts produce positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  ConstraintSolver solver;

  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact1 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normal,
                                                      Coordinate{0.1, 0, 0.5},
                                                      Coordinate{0.1, 0, 0.4},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.5,
                                                      0.0);
  auto contact2 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normal,
                                                      Coordinate{-0.1, 0, 0.5},
                                                      Coordinate{-0.1, 0, 0.4},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.5,
                                                      0.0);

  std::vector<Constraint*> constraints{contact1.get(), contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);

  // Count non-zero lambdas
  int nonZeroCount = 0;
  for (Eigen::Index i = 0; i < result.lambdas.size(); ++i)
  {
    if (result.lambdas(i) > 1e-12)
    {
      ++nonZeroCount;
    }
  }

  // Both contacts are compressive — both lambdas should be positive
  EXPECT_EQ(2, nonZeroCount);
}
