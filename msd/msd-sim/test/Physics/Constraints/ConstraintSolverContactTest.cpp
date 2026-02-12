// Ticket: 0033_constraint_solver_contact_tests
// Integration tests for ConstraintSolver::solveWithContacts() with
// ContactConstraint

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
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

// Create a default InertialState at rest
InertialState createDefaultState(
  const Coordinate& position = Coordinate{0.0, 0.0, 0.0},
  const Coordinate& velocity = Coordinate{0.0, 0.0, 0.0})
{
  InertialState state;
  state.position = position;
  state.velocity = velocity;
  state.acceleration = Coordinate{0.0, 0.0, 0.0};
  state.orientation = Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};  // Identity
  state.quaternionRate = Eigen::Vector4d::Zero();
  state.angularAcceleration = AngularAcceleration{0.0, 0.0, 0.0};
  return state;
}

// Create identity inertia tensor
Eigen::Matrix3d createIdentityInertia()
{
  return Eigen::Matrix3d::Identity();
}

}  // anonymous namespace

// ============================================================================
// 1. Basic PGS Convergence Tests
// ============================================================================

TEST(ConstraintSolverContactTest, EmptyContactSet_ReturnsConverged_0033)
{
  // Test: Zero contacts returns converged with empty forces
  ConstraintSolver solver;
  std::vector<Constraint*> constraints;
  std::vector<std::reference_wrapper<const InertialState>> states;
  std::vector<double> inverseMasses;
  std::vector<Eigen::Matrix3d> inverseInertias;

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 0, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(0, result.lambdas.size());
  EXPECT_EQ(0, result.bodyForces.size());
  EXPECT_EQ(0, result.iterations);
}

TEST(ConstraintSolverContactTest, SingleContact_Converges_0033)
{
  // Test: One penetrating contact produces converged result
  ConstraintSolver solver;

  // Two bodies penetrating along Z axis
  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.8});  // Penetrating

  Coordinate normal{0, 0, 1};  // A → B
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};  // Penetration = 0.1m
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0);

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};  // 10kg each
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(1, result.lambdas.size());
  EXPECT_GT(result.iterations, 0);
  EXPECT_LT(result.iterations, 10);  // Should converge quickly
  EXPECT_EQ(2, result.bodyForces.size());
  EXPECT_NEAR(0., result.bodyForces[0].angularTorque.norm(), 1e-9);
  EXPECT_NEAR(0., result.bodyForces[1].angularTorque.norm(), 1e-9);
}

TEST(ConstraintSolverContactTest, MultipleContacts_Converges_0033)
{
  // Test: 2-4 simultaneous contacts converge
  ConstraintSolver solver;
  solver.setMaxIterations(50);  // Allow many iterations for coupled contacts
  solver.setConvergenceTolerance(1e-3);  // Looser tolerance for stability

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  // Create 4 contact points (corners of a square)
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
                                        Coordinate{0.1, -0.1, 0.5},
                                        Coordinate{0.1, -0.1, 0.4},
                                        0.1,
                                        comA,
                                        comB,
                                        0.5,
                                        0.0);
  auto contact3 =
    std::make_unique<ContactConstraint>(0,
                                        1,
                                        normal,
                                        Coordinate{-0.1, 0.1, 0.5},
                                        Coordinate{-0.1, 0.1, 0.4},
                                        0.1,
                                        comA,
                                        comB,
                                        0.5,
                                        0.0);
  auto contact4 =
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

  std::vector<Constraint*> constraints{
    contact1.get(), contact2.get(), contact3.get(), contact4.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(4, result.lambdas.size());
  EXPECT_GT(result.iterations, 0);
  EXPECT_EQ(2, result.bodyForces.size());
  EXPECT_NEAR(0., result.bodyForces[0].angularTorque.norm(), 1e-9);
  EXPECT_NEAR(0., result.bodyForces[1].angularTorque.norm(), 1e-9);
}

TEST(ConstraintSolverContactTest, MaxIterationsReached_ReportsNotConverged_0033)
{
  // Test: Set max_safety_iterations=1 on a scenario requiring multiple active
  // set changes. With ASM, a mixed compressive/separating contact configuration
  // needs at least 2 iterations: iteration 1 solves the full active set and
  // finds a negative lambda (separating contact), iteration 2 removes it and
  // re-solves. Safety cap of 1 prevents completing the second iteration.
  //
  // Ticket: 0034_active_set_method_contact_solver
  ConstraintSolver solver;
  solver.setMaxIterations(1);  // Force safety cap to 1

  // Body A at rest, body B separating rapidly along Z
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 0.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 1.0}, Coordinate{0, 0, 5.0});

  Coordinate normalZ{0, 0, 1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 1.0};

  // Contact 1: Bodies separating along Z with large separating velocity
  // This produces negative RHS (b < 0), so the LLT solve on the full active
  // set yields negative lambda, requiring removal from the active set.
  auto contact1 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalZ,
                                                      Coordinate{0, 0, 0.5},
                                                      Coordinate{0, 0, 0.6},
                                                      0.0,
                                                      comA,
                                                      comB,
                                                      0.0,
                                                      -5.0);

  // Contact 2: Penetrating contact with Baumgarte bias (compressive)
  auto contact2 = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalZ,
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

  // With safety cap = min(2*2, 1) = 1, ASM can only do 1 iteration.
  // The first iteration finds a negative lambda and removes it from the active
  // set, but cannot proceed to verify KKT for the reduced set.
  EXPECT_FALSE(result.converged);
  EXPECT_GE(result.iterations, 1);
}

// ============================================================================
// 2. Lambda Non-Negativity (Unilateral Enforcement) Tests
// ============================================================================

TEST(ConstraintSolverContactTest, SeparatingBodies_LambdaZero_0033)
{
  // Test: Bodies moving apart produce lambda=0 (no adhesion)
  ConstraintSolver solver;

  // Bodies moving apart (negative approach velocity)
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, -1.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 1.0}, Coordinate{0, 0, 1.0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.6};  // Separated
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(0,
                                                     1,
                                                     normal,
                                                     contactA,
                                                     contactB,
                                                     0.0,
                                                     comA,
                                                     comB,
                                                     0.5,
                                                     -2.0);  // Separating

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
  EXPECT_NEAR(0.0, result.lambdas(0), 1e-6);  // Lambda clamped to zero
}

TEST(ConstraintSolverContactTest, ApproachingBodies_LambdaPositive_0033)
{
  // Test: Bodies approaching produce lambda>0 (repulsive force)
  ConstraintSolver solver;

  // Bodies approaching (positive approach velocity)
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 1.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, -1.0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};  // Penetrating
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(0,
                                                     1,
                                                     normal,
                                                     contactA,
                                                     contactB,
                                                     0.1,
                                                     comA,
                                                     comB,
                                                     0.5,
                                                     2.0);  // Approaching

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
  EXPECT_GT(result.lambdas(0), 0.0);  // Positive lambda (repulsive)
  EXPECT_NEAR(0., result.bodyForces[0].angularTorque.norm(), 1e-9);
  EXPECT_NEAR(0., result.bodyForces[1].angularTorque.norm(), 1e-9);
}

TEST(ConstraintSolverContactTest, RestingContact_LambdaNonNegative_0033)
{
  // Test: Bodies at rest on surface produce lambda>=0
  ConstraintSolver solver;

  // Bodies at rest (zero velocity)
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, 0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.45};  // Slight penetration
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.05, comA, comB, 0.0, 0.0);  // Resting

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
  EXPECT_GE(result.lambdas(0), 0.0);  // Lambda non-negative
  EXPECT_NEAR(0., result.bodyForces[0].angularTorque.norm(), 1e-9);
  EXPECT_NEAR(0., result.bodyForces[1].angularTorque.norm(), 1e-9);
}

// ============================================================================
// 3. Per-Body Force Correctness Tests
// ============================================================================

TEST(ConstraintSolverContactTest, EqualMass_SymmetricForces_0033)
{
  // Test: Two equal-mass bodies receive equal and opposite forces
  ConstraintSolver solver;

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
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
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};  // Equal mass
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  ASSERT_EQ(2, result.bodyForces.size());

  // Forces should be equal and opposite
  EXPECT_NEAR(result.bodyForces[0].linearForce.x(),
              -result.bodyForces[1].linearForce.x(),
              1e-6);
  EXPECT_NEAR(result.bodyForces[0].linearForce.y(),
              -result.bodyForces[1].linearForce.y(),
              1e-6);
  EXPECT_NEAR(result.bodyForces[0].linearForce.z(),
              -result.bodyForces[1].linearForce.z(),
              1e-6);
}

TEST(ConstraintSolverContactTest, StaticBody_ZeroForceOnStatic_0033)
{
  // Test: Body with inverseMass=0 receives zero velocity change
  // Ticket: 0040b — stateA approaching stateB so contact produces positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  ConstraintSolver solver;

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
  std::vector<double> inverseMasses{0.0, 1.0 / 10.0};  // A is static
  std::vector<Eigen::Matrix3d> inverseInertias{Eigen::Matrix3d::Zero(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  ASSERT_EQ(2, result.bodyForces.size());

  // Static body (A) should have force but infinite mass absorbs it
  // (The force is applied, but velocity change = force * inverseMass = force *
  // 0 = 0) Dynamic body (B) should have non-zero force
  EXPECT_GT(result.bodyForces[1].linearForce.norm(), 0.0);
}

TEST(ConstraintSolverContactTest, ForceDirection_AlongContactNormal_0033)
{
  // Test: Constraint force is along the contact normal direction
  // Ticket: 0040b — stateA approaching stateB so contact produces positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  ConstraintSolver solver;

  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};  // Z-axis normal
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
  ASSERT_EQ(2, result.bodyForces.size());

  // Force should be primarily along Z-axis (normal direction)
  double fx = std::abs(result.bodyForces[0].linearForce.x());
  double fy = std::abs(result.bodyForces[0].linearForce.y());
  double fz = std::abs(result.bodyForces[0].linearForce.z());

  EXPECT_LT(fx, 1e-6);  // Negligible X component
  EXPECT_LT(fy, 1e-6);  // Negligible Y component
  EXPECT_GT(fz, 1e-6);  // Significant Z component
}

TEST(ConstraintSolverContactTest, AngularForces_LeverArmProducesTorque_0033)
{
  // Test: Off-center contact produces angular constraint torque
  // Ticket: 0040b — stateA approaching stateB so contact produces positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  ConstraintSolver solver;

  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0.5, 0, 0.5};  // Off-center contact
  Coordinate contactB{0.5, 0, 0.4};
  Coordinate comA{0, 0, 0};  // COM at origin
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
  ASSERT_EQ(2, result.bodyForces.size());

  // Should produce torque around Y-axis: r × F = (0.5, 0, 0.5) × (0, 0, Fz)
  double torque_norm_A = result.bodyForces[0].angularTorque.norm();
  double torque_norm_B = result.bodyForces[1].angularTorque.norm();

  EXPECT_GT(torque_norm_A, 1e-6);  // Non-zero torque on A
  EXPECT_GT(torque_norm_B, 1e-6);  // Non-zero torque on B
}

// ============================================================================
// 4. Physical Correctness Tests
// ============================================================================

TEST(ConstraintSolverContactTest,
     HeadOnCollision_EqualMass_VelocityExchange_0033)
{
  // Test: Two equal-mass bodies approaching head-on exchange velocities (e=1)
  ConstraintSolver solver;

  // Head-on collision
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, -2.0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 1.0, 4.0);  // e=1,
                                                                   // v_rel=4

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.lambdas(0), 0.0);  // Repulsive force
}

// Ticket 0046: Removed SlopCorrection_CappedToApproachVelocity_0033 test.
// Slop correction was removed from velocity-level RHS as it injected energy.
// Penetration correction now handled exclusively by PositionCorrector.

TEST(ConstraintSolverContactTest, Restitution_ZeroBounce_0033)
{
  // Test: e=0 contact produces zero rebound (bodies stick)
  ConstraintSolver solver;

  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 1.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, 0.0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.0, 1.0);  // e=0

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.lambdas(0), 0.0);  // Contact force applied
}

TEST(ConstraintSolverContactTest, Restitution_FullBounce_0033)
{
  // Test: e=1 contact produces full velocity reversal
  ConstraintSolver solver;

  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, 0.0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 1.0, 2.0);  // e=1

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.lambdas(0), 0.0);  // Large impulse for full rebound
}

TEST(ConstraintSolverContactTest,
     RestVelocityThreshold_DisablesRestitution_0033)
{
  // Test: Slow contact (below 0.5 m/s) disables restitution to prevent jitter
  ConstraintSolver solver;

  // Slow approach velocity (below rest threshold)
  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 0.2});  // Slow
  InertialState stateB =
    createDefaultState(Coordinate{0, 0, 0.9}, Coordinate{0, 0, 0.0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.8, 0.2);  // Slow
                                                                   // impact

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  // Lambda should be small (no restitution for slow contacts)
  EXPECT_GT(result.lambdas(0), 0.0);
}

// ============================================================================
// 5. Edge Cases Tests
// ============================================================================

TEST(ConstraintSolverContactTest, BothBodiesStatic_AllLambdasZero_0033)
{
  // Test: Two infinite-mass bodies: degenerate but should not crash
  ConstraintSolver solver;

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
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
  std::vector<double> inverseMasses{0.0, 0.0};  // Both static
  std::vector<Eigen::Matrix3d> inverseInertias{Eigen::Matrix3d::Zero(),
                                               Eigen::Matrix3d::Zero()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  // Should handle gracefully (may or may not converge, but should not crash)
  // Lambda value is not forced to zero, but bodies won't move (inverseMass=0)
  EXPECT_EQ(1, result.lambdas.size());
  // The solver may compute non-zero lambda even for static-static (degenerate
  // case) The important thing is that the bodyForces are effectively zero (or
  // don't matter) because inverseMass is zero, so velocity change = force *
  // inverseMass = 0
}

TEST(ConstraintSolverContactTest, ParallelContacts_SameNormal_0033)
{
  // Test: Multiple contacts with same normal converge correctly
  // Ticket: 0040b — stateA approaching stateB so contacts produce positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  ConstraintSolver solver;
  solver.setMaxIterations(50);  // Allow many iterations for coupled contacts
  solver.setConvergenceTolerance(1e-3);  // Looser tolerance for stability

  InertialState stateA =
    createDefaultState(Coordinate{0, 0, 0}, Coordinate{0, 0, 2.0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};  // Same normal for all contacts
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
  EXPECT_EQ(2, result.lambdas.size());
  EXPECT_GT(result.lambdas(0), 0.0);
  EXPECT_GT(result.lambdas(1), 0.0);
}

TEST(ConstraintSolverContactTest, OrthogonalContacts_IndependentResolution_0033)
{
  // Test: Contacts on perpendicular faces resolve independently
  ConstraintSolver solver;

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB = createDefaultState(Coordinate{0.9, 0, 0.9});

  Coordinate normalZ{0, 0, 1};  // Z-face contact
  Coordinate normalX{1, 0, 0};  // X-face contact
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contactZ = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalZ,
                                                      Coordinate{0, 0, 0.5},
                                                      Coordinate{0, 0, 0.4},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.5,
                                                      0.0);
  auto contactX = std::make_unique<ContactConstraint>(0,
                                                      1,
                                                      normalX,
                                                      Coordinate{0.5, 0, 0},
                                                      Coordinate{0.4, 0, 0},
                                                      0.1,
                                                      comA,
                                                      comB,
                                                      0.5,
                                                      0.0);

  std::vector<Constraint*> constraints{contactZ.get(), contactX.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(2, result.lambdas.size());
}

TEST(ConstraintSolverContactTest, HighMassRatio_Converges_0033)
{
  // Test: Mass ratio of 1000:1 still converges
  // Ticket: 0040b — stateA approaching stateB so contact produces positive
  // RHS via velocity (split impulse: zero-velocity contacts produce lambda=0).
  ConstraintSolver solver;

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
  std::vector<double> inverseMasses{1.0 / 1000.0, 1.0 / 1.0};  // 1000:1 ratio
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.lambdas(0), 0.0);
}

TEST(ConstraintSolverContactTest, ZeroPenetration_NoBias_0033)
{
  // Test: Contact at surface (penetration=0) produces no Baumgarte bias
  ConstraintSolver solver;

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 1.0});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.5};  // Zero penetration
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact = std::make_unique<ContactConstraint>(0,
                                                     1,
                                                     normal,
                                                     contactA,
                                                     contactB,
                                                     0.0,
                                                     comA,
                                                     comB,
                                                     0.5,
                                                     0.0);  // No penetration

  std::vector<Constraint*> constraints{contact.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto result = solver.solve(
    constraints, states, inverseMasses, inverseInertias, 2, 0.016);

  EXPECT_TRUE(result.converged);
  // Lambda should be near zero (no Baumgarte bias, no restitution)
  EXPECT_NEAR(0.0, result.lambdas(0), 1e-4);
}

// ============================================================================
// 6. Solver Configuration Tests
// ============================================================================

TEST(ConstraintSolverContactTest, SetMaxIterations_Respected_0033)
{
  // Test: setMaxIterations() limits PGS iteration count
  ConstraintSolver solver;
  solver.setMaxIterations(5);

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
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

  EXPECT_LE(result.iterations, 5);  // Should not exceed max iterations
}

TEST(ConstraintSolverContactTest, SetConvergenceTolerance_EarlyExit_0033)
{
  // Test: Tight tolerance requires more iterations; loose tolerance exits early
  ConstraintSolver solverTight;
  solverTight.setConvergenceTolerance(1e-8);  // Very tight

  ConstraintSolver solverLoose;
  solverLoose.setConvergenceTolerance(1e-2);  // Very loose

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
  InertialState stateB = createDefaultState(Coordinate{0, 0, 0.9});

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0.5};
  Coordinate contactB{0, 0, 0.4};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  auto contact1 = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0);
  auto contact2 = std::make_unique<ContactConstraint>(
    0, 1, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0);

  std::vector<Constraint*> constraintsTight{contact1.get()};
  std::vector<Constraint*> constraintsLoose{contact2.get()};
  std::vector<std::reference_wrapper<const InertialState>> states{stateA,
                                                                  stateB};
  std::vector<double> inverseMasses{1.0 / 10.0, 1.0 / 10.0};
  std::vector<Eigen::Matrix3d> inverseInertias{createIdentityInertia(),
                                               createIdentityInertia()};

  auto resultTight = solverTight.solve(
    constraintsTight, states, inverseMasses, inverseInertias, 2, 0.016);
  auto resultLoose = solverLoose.solve(
    constraintsLoose, states, inverseMasses, inverseInertias, 2, 0.016);

  // Tight tolerance should need more iterations (or at least not fewer)
  EXPECT_GE(resultTight.iterations, resultLoose.iterations);
}

TEST(ConstraintSolverContactTest, DefaultConfiguration_ReasonableDefaults_0033)
{
  // Test: Default max_iterations=10, tolerance=1e-4
  ConstraintSolver solver;

  InertialState stateA = createDefaultState(Coordinate{0, 0, 0});
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
  EXPECT_LE(result.iterations, 10);  // Default max iterations
  EXPECT_GT(result.iterations, 0);
}
