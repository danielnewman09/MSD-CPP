// Ticket: 0030_lagrangian_quaternion_physics
// Acceptance Criteria Tests for Quaternion Physics System

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/Constraints/QuaternionConstraint.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

using namespace msd_sim;

namespace
{

// Helper: Create unit cube for testing
ConvexHull createUnitCube()
{
  std::vector<Coordinate> points = {{-0.5, -0.5, -0.5},
                                    {0.5, -0.5, -0.5},
                                    {0.5, 0.5, -0.5},
                                    {-0.5, 0.5, -0.5},
                                    {-0.5, -0.5, 0.5},
                                    {0.5, -0.5, 0.5},
                                    {0.5, 0.5, 0.5},
                                    {-0.5, 0.5, 0.5}};
  return ConvexHull{points};
}

}  // namespace

// ============================================================================
// AC1: Q̇ ↔ ω conversion round-trips correctly (within 1e-10 tolerance)
// ============================================================================

TEST(QuaternionPhysicsAC1, OmegaToQdotRoundTrip_Identity)
{
  // Test with identity quaternion
  Eigen::Quaterniond Q = Eigen::Quaterniond::Identity();
  AngularRate omega{1.0, 2.0, 3.0};

  // Convert ω → Q̇ → ω
  Eigen::Vector4d Qdot = InertialState::omegaToQuaternionRate(omega, Q);
  AngularRate omega_recovered = InertialState::quaternionRateToOmega(Qdot, Q);

  EXPECT_NEAR(omega_recovered.x(), omega.x(), 1e-10);
  EXPECT_NEAR(omega_recovered.y(), omega.y(), 1e-10);
  EXPECT_NEAR(omega_recovered.z(), omega.z(), 1e-10);
}

TEST(QuaternionPhysicsAC1, OmegaToQdotRoundTrip_RotatedQuaternion)
{
  // Test with 45° rotation about Z-axis
  Eigen::Quaterniond Q{Eigen::AngleAxisd{M_PI / 4, msd_sim::Vector3D::UnitZ()}};
  AngularRate omega{0.5, -1.5, 2.5};

  // Convert ω → Q̇ → ω
  Eigen::Vector4d Qdot = InertialState::omegaToQuaternionRate(omega, Q);
  AngularRate omega_recovered = InertialState::quaternionRateToOmega(Qdot, Q);

  EXPECT_NEAR(omega_recovered.x(), omega.x(), 1e-10);
  EXPECT_NEAR(omega_recovered.y(), omega.y(), 1e-10);
  EXPECT_NEAR(omega_recovered.z(), omega.z(), 1e-10);
}

TEST(QuaternionPhysicsAC1, OmegaToQdotRoundTrip_ArbitraryQuaternion)
{
  // Test with arbitrary rotation
  Eigen::Quaterniond Q{
    Eigen::AngleAxisd{1.2, msd_sim::Vector3D{1, 2, 3}.normalized()}};
  AngularRate omega{-2.0, 3.0, -1.0};

  // Convert ω → Q̇ → ω
  Eigen::Vector4d Qdot = InertialState::omegaToQuaternionRate(omega, Q);
  AngularRate omega_recovered = InertialState::quaternionRateToOmega(Qdot, Q);

  EXPECT_NEAR(omega_recovered.x(), omega.x(), 1e-10);
  EXPECT_NEAR(omega_recovered.y(), omega.y(), 1e-10);
  EXPECT_NEAR(omega_recovered.z(), omega.z(), 1e-10);
}

TEST(QuaternionPhysicsAC1, QdotToOmegaRoundTrip)
{
  // Test reverse direction: Q̇ → ω → Q̇
  Eigen::Quaterniond Q{Eigen::AngleAxisd{0.5, msd_sim::Vector3D::UnitY()}};

  // Valid Q̇ must be perpendicular to Q: Q · Q̇ = 0
  // For rotation about Z with Q at 30° about Y
  AngularRate omega{0.0, 0.0, 5.0};
  Eigen::Vector4d Qdot = InertialState::omegaToQuaternionRate(omega, Q);

  // Now recover
  AngularRate omega_recovered = InertialState::quaternionRateToOmega(Qdot, Q);
  Eigen::Vector4d Qdot_recovered =
    InertialState::omegaToQuaternionRate(omega_recovered, Q);

  EXPECT_NEAR(Qdot_recovered(0), Qdot(0), 1e-10);
  EXPECT_NEAR(Qdot_recovered(1), Qdot(1), 1e-10);
  EXPECT_NEAR(Qdot_recovered(2), Qdot(2), 1e-10);
  EXPECT_NEAR(Qdot_recovered(3), Qdot(3), 1e-10);
}

TEST(QuaternionPhysicsAC1, InertialStateGetSetAngularVelocity)
{
  InertialState state;
  state.orientation =
    Eigen::Quaterniond{Eigen::AngleAxisd{0.3, msd_sim::Vector3D::UnitX()}};

  AngularRate omega{1.5, -0.5, 2.0};
  state.setAngularVelocity(omega);

  AngularRate recovered = state.getAngularVelocity();

  EXPECT_NEAR(recovered.x(), omega.x(), 1e-10);
  EXPECT_NEAR(recovered.y(), omega.y(), 1e-10);
  EXPECT_NEAR(recovered.z(), omega.z(), 1e-10);
}

// ============================================================================
// AC2: Quaternion constraint maintains |Q|=1 over 10000 integration steps
// ============================================================================

TEST(QuaternionPhysicsAC2, ConstraintMaintainsUnitQuaternion_10000Steps)
{
  WorldModel world;
  ConvexHull hull = createUnitCube();
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  const AssetInertial& asset = world.spawnObject(0, hull, frame);
  uint32_t instanceId = asset.getInstanceId();
  AssetInertial& mutableAsset = world.getObject(instanceId);

  // Apply constant torque to create rotation
  mutableAsset.applyTorque(Coordinate{0, 0, 5.0});

  // Integrate for 10000 steps
  auto simTime = std::chrono::milliseconds{0};
  double maxViolation = 0.0;

  for (int i = 0; i < 10000; ++i)
  {
    simTime += std::chrono::milliseconds{16};
    world.update(simTime);

    // Check constraint violation
    const Eigen::Quaterniond& Q = mutableAsset.getInertialState().orientation;
    double violation = std::abs(Q.squaredNorm() - 1.0);
    maxViolation = std::max(maxViolation, violation);

    // Re-apply torque for continuous rotation
    mutableAsset.applyTorque(Coordinate{0, 0, 5.0});
  }

  EXPECT_LT(maxViolation, 1e-10)
    << "Maximum constraint violation: " << maxViolation;
}

TEST(QuaternionPhysicsAC2, ConstraintEnforcementNormalizesQuaternion)
{
  QuaternionConstraint constraint{10.0, 10.0};

  // Create unnormalized quaternion
  Eigen::Quaterniond Q{1.1, 0.1, 0.1, 0.1};  // Not unit length
  Eigen::Vector4d Qdot{0.01, 0.02, 0.03, 0.04};

  constraint.enforceConstraint(Q, Qdot);

  // Should be normalized after enforcement
  EXPECT_NEAR(Q.norm(), 1.0, 1e-10);
}

TEST(QuaternionPhysicsAC2, ConstraintEnforcementProjectsQdot)
{
  QuaternionConstraint constraint{10.0, 10.0};

  Eigen::Quaterniond Q = Eigen::Quaterniond::Identity();
  Eigen::Vector4d Qdot{0.1, 0.2, 0.3, 0.4};  // Not perpendicular to Q

  constraint.enforceConstraint(Q, Qdot);

  // Q̇ should be perpendicular to Q after enforcement: Q · Q̇ = 0
  double dotProduct = Q.coeffs().dot(Qdot);
  EXPECT_NEAR(dotProduct, 0.0, 1e-10);
}

// ============================================================================
// AC3: Free-fall test matches analytical solution z = z₀ - ½gt²
// ============================================================================

TEST(QuaternionPhysicsAC3, FreeFallMatchesAnalyticalSolution)
{
  WorldModel world;
  ConvexHull hull = createUnitCube();

  double z0 = 100.0;  // Start high to avoid ground
  ReferenceFrame frame{Coordinate{0, 0, z0}};

  const AssetInertial& asset = world.spawnObject(0, hull, frame);
  uint32_t instanceId = asset.getInstanceId();
  AssetInertial& mutableAsset = world.getObject(instanceId);

  // Simulate for 1 second (60 steps at 16.67ms)
  double dt = 0.016;
  int steps = 60;
  auto simTime = std::chrono::milliseconds{0};

  for (int i = 0; i < steps; ++i)
  {
    simTime += std::chrono::milliseconds{16};
    world.update(simTime);
  }

  // Analytical solution: z = z₀ - ½gt²
  double t = steps * dt;
  double g = 9.81;
  double z_analytical = z0 - 0.5 * g * t * t;

  // Semi-implicit Euler has some deviation, but should be within 1e-6 relative
  // error Actually for larger simulations, use absolute tolerance
  double z_actual = mutableAsset.getInertialState().position.z();

  // Allow 1% tolerance for numerical integration
  EXPECT_NEAR(z_actual, z_analytical, std::abs(z_analytical) * 0.01 + 1e-6);
}

TEST(QuaternionPhysicsAC3, FreeFallVelocityMatchesAnalytical)
{
  WorldModel world;
  ConvexHull hull = createUnitCube();
  ReferenceFrame frame{Coordinate{0, 0, 100.0}};

  const AssetInertial& asset = world.spawnObject(0, hull, frame);
  uint32_t instanceId = asset.getInstanceId();
  AssetInertial& mutableAsset = world.getObject(instanceId);

  // Simulate for 1 second
  auto simTime = std::chrono::milliseconds{0};
  for (int i = 0; i < 60; ++i)
  {
    simTime += std::chrono::milliseconds{16};
    world.update(simTime);
  }

  // Analytical: v = -g*t
  double t = 60 * 0.016;
  double v_analytical = -9.81 * t;
  double v_actual = mutableAsset.getInertialState().velocity.z();

  EXPECT_NEAR(v_actual, v_analytical, 0.5);  // Within 0.5 m/s
}

// ============================================================================
// AC4: No gimbal lock at 90° pitch (quaternion stays valid, no NaN)
// ============================================================================

TEST(QuaternionPhysicsAC4, NoGimbalLockAt90Pitch)
{
  WorldModel world;
  ConvexHull hull = createUnitCube();
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  const AssetInertial& asset = world.spawnObject(0, hull, frame);
  uint32_t instanceId = asset.getInstanceId();
  AssetInertial& mutableAsset = world.getObject(instanceId);

  // Set orientation to 90° pitch (gimbal lock for Euler angles)
  mutableAsset.getInertialState().orientation =
    Eigen::Quaterniond{Eigen::AngleAxisd{M_PI / 2, msd_sim::Vector3D::UnitY()}};

  // Apply torque about all axes
  mutableAsset.applyTorque(Coordinate{5.0, 5.0, 5.0});

  // Integrate for several steps
  auto simTime = std::chrono::milliseconds{0};
  for (int i = 0; i < 100; ++i)
  {
    simTime += std::chrono::milliseconds{16};
    world.update(simTime);
    mutableAsset.applyTorque(Coordinate{5.0, 5.0, 5.0});

    // Check for NaN
    const InertialState& state = mutableAsset.getInertialState();
    EXPECT_TRUE(std::isfinite(state.orientation.w())) << "NaN at step " << i;
    EXPECT_TRUE(std::isfinite(state.orientation.x())) << "NaN at step " << i;
    EXPECT_TRUE(std::isfinite(state.orientation.y())) << "NaN at step " << i;
    EXPECT_TRUE(std::isfinite(state.orientation.z())) << "NaN at step " << i;

    // Quaternion should remain unit length
    EXPECT_NEAR(state.orientation.norm(), 1.0, 1e-6);
  }
}

TEST(QuaternionPhysicsAC4, RotationThroughGimbalLock)
{
  WorldModel world;
  ConvexHull hull = createUnitCube();
  ReferenceFrame frame{Coordinate{0, 0, 0}};

  const AssetInertial& asset = world.spawnObject(0, hull, frame);
  uint32_t instanceId = asset.getInstanceId();
  AssetInertial& mutableAsset = world.getObject(instanceId);

  // Apply pure pitch rotation to pass through gimbal lock
  auto simTime = std::chrono::milliseconds{0};
  for (int i = 0; i < 500; ++i)
  {
    mutableAsset.applyTorque(Coordinate{0, 10.0, 0});  // Pitch torque
    simTime += std::chrono::milliseconds{16};
    world.update(simTime);

    const InertialState& state = mutableAsset.getInertialState();

    // No NaN propagation
    EXPECT_FALSE(std::isnan(state.orientation.w()));
    EXPECT_FALSE(std::isnan(state.orientation.x()));
    EXPECT_FALSE(std::isnan(state.orientation.y()));
    EXPECT_FALSE(std::isnan(state.orientation.z()));
  }
}

// ============================================================================
// AC5: GravityPotential produces correct force F = m*g
// ============================================================================

TEST(QuaternionPhysicsAC5, GravityForceCorrectMagnitude)
{
  // Test GravityPotential directly
  GravityPotential gravity{Coordinate{0, 0, -9.81}};

  InertialState state;
  state.position = Coordinate{10, 20, 30};
  state.orientation = Eigen::Quaterniond::Identity();

  double mass = 5.0;
  Coordinate force = gravity.computeForce(state, mass);

  EXPECT_NEAR(force.x(), 0.0, 1e-10);
  EXPECT_NEAR(force.y(), 0.0, 1e-10);
  EXPECT_NEAR(force.z(), -9.81 * mass, 1e-10);
}

TEST(QuaternionPhysicsAC5, GravityTorqueIsZero)
{
  // Uniform gravity produces no torque
  GravityPotential gravity{Coordinate{0, 0, -9.81}};

  InertialState state;
  state.orientation =
    Eigen::Quaterniond{Eigen::AngleAxisd{0.5, msd_sim::Vector3D::UnitX()}};

  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity() * 10.0;
  Coordinate torque = gravity.computeTorque(state, inertia);

  EXPECT_NEAR(torque.x(), 0.0, 1e-10);
  EXPECT_NEAR(torque.y(), 0.0, 1e-10);
  EXPECT_NEAR(torque.z(), 0.0, 1e-10);
}

TEST(QuaternionPhysicsAC5, GravityEnergyCorrect)
{
  GravityPotential gravity{Coordinate{0, 0, -9.81}};

  InertialState state;
  state.position = Coordinate{0, 0, 10.0};

  double mass = 2.0;
  double energy = gravity.computeEnergy(state, mass);

  // V = m * g * z (with g pointing down, energy = -m * (-9.81) * 10 = m * 9.81
  // * 10) Actually V = -m * g · r where g = (0, 0, -9.81) and r = (0, 0, 10) V
  // = -m * (0*0 + 0*0 + (-9.81)*10) = -m * (-98.1) = 98.1 * m
  EXPECT_NEAR(energy, 9.81 * 10.0 * mass, 1e-10);
}

TEST(QuaternionPhysicsAC5, GravityForceIndependentOfOrientation)
{
  GravityPotential gravity{Coordinate{0, 0, -9.81}};

  InertialState state1, state2;
  state1.position = Coordinate{0, 0, 0};
  state2.position = Coordinate{0, 0, 0};
  state1.orientation = Eigen::Quaterniond::Identity();
  state2.orientation = Eigen::Quaterniond{
    Eigen::AngleAxisd{1.5, msd_sim::Vector3D{1, 1, 1}.normalized()}};

  double mass = 3.0;
  Coordinate force1 = gravity.computeForce(state1, mass);
  Coordinate force2 = gravity.computeForce(state2, mass);

  // Force should be the same regardless of orientation
  EXPECT_NEAR(force1.x(), force2.x(), 1e-10);
  EXPECT_NEAR(force1.y(), force2.y(), 1e-10);
  EXPECT_NEAR(force1.z(), force2.z(), 1e-10);
}
