// Ticket: 0039a_energy_tracking_diagnostic_infrastructure
// Test: EnergyTracker unit tests

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <vector>

#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{
namespace test
{

// ========== Helper: Create potential energy vector ==========

static std::vector<std::unique_ptr<PotentialEnergy>> makeGravityPotentials(
  const Coordinate& gravity = Coordinate{0.0, 0.0, -9.81})
{
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(std::make_unique<GravityPotential>(gravity));
  return potentials;
}

// Uniform cube inertia tensor (body frame)
// I = (1/6) * m * s^2 for a uniform cube with side length s
static Eigen::Matrix3d cubeTensor(double mass, double sideLength)
{
  double const val = (1.0 / 6.0) * mass * sideLength * sideLength;
  Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
  I(0, 0) = val;
  I(1, 1) = val;
  I(2, 2) = val;
  return I;
}

// ========== Linear KE Tests ==========

TEST(EnergyTracker, LinearKE_MovingBody_ComputesCorrectly)
{
  // A 2 kg body moving at 3 m/s along x-axis
  // KE = 0.5 * 2 * 9 = 9 J
  InertialState state{};
  state.velocity = Velocity{3.0, 0.0, 0.0};

  double const mass = 2.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials();

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  EXPECT_NEAR(energy.linearKE, 9.0, 1e-10);
}

TEST(EnergyTracker, LinearKE_StationaryBody_ReturnsZero)
{
  InertialState state{};
  state.velocity = Velocity{0.0, 0.0, 0.0};

  double const mass = 5.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials();

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  EXPECT_NEAR(energy.linearKE, 0.0, 1e-10);
}

TEST(EnergyTracker, LinearKE_DiagonalVelocity_ComputesCorrectly)
{
  // Body moving at (1, 2, 3) m/s, mass = 1 kg
  // KE = 0.5 * 1 * (1 + 4 + 9) = 7.0 J
  InertialState state{};
  state.velocity = Velocity{1.0, 2.0, 3.0};

  double const mass = 1.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials();

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  EXPECT_NEAR(energy.linearKE, 7.0, 1e-10);
}

// ========== Rotational KE Tests ==========

TEST(EnergyTracker, RotationalKE_SpinningCube_AxisAligned)
{
  // A 1 kg unit cube spinning at 2 rad/s about z-axis
  // I_zz = (1/6) * 1 * 1 = 1/6
  // KE_rot = 0.5 * omega^2 * I_zz = 0.5 * 4 * (1/6) = 1/3
  InertialState state{};
  state.orientation = QuaternionD{1.0, 0.0, 0.0, 0.0};  // Identity
  state.setAngularVelocity(AngularVelocity{0.0, 0.0, 2.0});

  double const mass = 1.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials();

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  EXPECT_NEAR(energy.rotationalKE, 1.0 / 3.0, 1e-10);
}

TEST(EnergyTracker, RotationalKE_SpinningCube_Tilted)
{
  // A 1 kg unit cube tilted 45 degrees about x-axis, spinning at 2 rad/s about z
  // For a uniform cube with equal principal moments, rotational KE should be
  // the same regardless of orientation (I_world = R * I_body * R^T = I_body
  // when I_body is isotropic)
  InertialState state{};
  Eigen::Quaterniond const tilt{
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitX()}};
  state.orientation = QuaternionD{tilt};
  state.setAngularVelocity(AngularVelocity{0.0, 0.0, 2.0});

  double const mass = 1.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials();

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  // For isotropic inertia tensor, rotation doesn't change KE
  EXPECT_NEAR(energy.rotationalKE, 1.0 / 3.0, 1e-10);
}

TEST(EnergyTracker, RotationalKE_ConsistentAcrossOrientations)
{
  // For a non-isotropic body, verify that KE_rot computed with world-frame
  // inertia remains constant when a body has the same angular velocity
  // but is rotated to a different orientation.
  //
  // Key insight: For a body spinning about its principal axis, the rotational
  // KE depends on which principal axis is aligned with the rotation axis.
  // We verify the computation is correct by checking an analytical case.

  // Non-uniform rectangular box: I_xx != I_yy != I_zz
  double const mass = 2.0;
  Eigen::Matrix3d bodyInertia = Eigen::Matrix3d::Zero();
  bodyInertia(0, 0) = 1.0;   // I_xx
  bodyInertia(1, 1) = 2.0;   // I_yy
  bodyInertia(2, 2) = 3.0;   // I_zz

  // Case 1: Identity orientation, spinning about z (I_zz = 3.0)
  InertialState state1{};
  state1.orientation = QuaternionD{1.0, 0.0, 0.0, 0.0};
  state1.setAngularVelocity(AngularVelocity{0.0, 0.0, 1.0});
  auto potentials = makeGravityPotentials();
  auto energy1 = EnergyTracker::computeBodyEnergy(state1, mass, bodyInertia, potentials);
  // KE = 0.5 * 1^2 * 3.0 = 1.5
  EXPECT_NEAR(energy1.rotationalKE, 1.5, 1e-10);

  // Case 2: 90-degree rotation about y-axis
  // This maps body z-axis to world x-axis, so body I_zz aligns with world x
  // Spinning about world z-axis now uses world I_zz which is body I_xx = 1.0
  InertialState state2{};
  Eigen::Quaterniond const rot90y{
    Eigen::AngleAxisd{M_PI / 2.0, Eigen::Vector3d::UnitY()}};
  state2.orientation = QuaternionD{rot90y};
  state2.setAngularVelocity(AngularVelocity{0.0, 0.0, 1.0});
  auto energy2 = EnergyTracker::computeBodyEnergy(state2, mass, bodyInertia, potentials);
  // I_world_zz after 90-degree Y rotation = body I_xx = 1.0
  // KE = 0.5 * 1^2 * 1.0 = 0.5
  EXPECT_NEAR(energy2.rotationalKE, 0.5, 1e-10);
}

// ========== Potential Energy Tests ==========

TEST(EnergyTracker, PotentialEnergy_HeightProportional)
{
  // Object at height 10m, mass 5 kg, gravity (0,0,-9.81)
  // PE = -m * g . r = -5 * (0,0,-9.81) . (0,0,10) = -5 * (-98.1) = 490.5
  InertialState state{};
  state.position = Coordinate{0.0, 0.0, 10.0};

  double const mass = 5.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials();

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  EXPECT_NEAR(energy.potentialE, 490.5, 1e-10);
}

TEST(EnergyTracker, PotentialEnergy_ArbitraryGravityDirection)
{
  // Gravity along x-axis: g = (-9.81, 0, 0)
  // Object at (5, 0, 0), mass 2 kg
  // PE = -2 * (-9.81, 0, 0) . (5, 0, 0) = -2 * (-49.05) = 98.1
  InertialState state{};
  state.position = Coordinate{5.0, 0.0, 0.0};

  double const mass = 2.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials(Coordinate{-9.81, 0.0, 0.0});

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  EXPECT_NEAR(energy.potentialE, 98.1, 1e-10);
}

// ========== Total Energy Tests ==========

TEST(EnergyTracker, TotalEnergy_SumsAllComponents)
{
  // Moving body with rotation at some height
  InertialState state{};
  state.position = Coordinate{0.0, 0.0, 10.0};
  state.velocity = Velocity{3.0, 0.0, 0.0};
  state.orientation = QuaternionD{1.0, 0.0, 0.0, 0.0};
  state.setAngularVelocity(AngularVelocity{0.0, 0.0, 2.0});

  double const mass = 1.0;
  Eigen::Matrix3d const inertia = cubeTensor(mass, 1.0);
  auto potentials = makeGravityPotentials();

  auto energy = EnergyTracker::computeBodyEnergy(state, mass, inertia, potentials);

  double const expectedLinearKE = 0.5 * 1.0 * 9.0;     // 4.5
  double const expectedRotKE = 0.5 * 4.0 * (1.0 / 6.0); // 1/3
  double const expectedPE = 98.1;                         // -1 * -9.81 * 10
  double const expectedTotal = expectedLinearKE + expectedRotKE + expectedPE;

  EXPECT_NEAR(energy.total(), expectedTotal, 1e-10);
}

// ========== Anomaly Detection Tests ==========

TEST(EnergyTracker, IsEnergyInjection_DetectsIncrease)
{
  // Energy increased from 100 to 101 (1% increase, well above 1e-6 relative)
  EXPECT_TRUE(EnergyTracker::isEnergyInjection(101.0, 100.0));
}

TEST(EnergyTracker, IsEnergyInjection_IgnoresNumericalNoise)
{
  // Energy increased by a tiny amount (1e-8 relative to ~100 J)
  // Threshold = max(1e-6 * 100, 1e-9) = 1e-4
  double const current = 100.0 + 1e-8;
  double const previous = 100.0;
  EXPECT_FALSE(EnergyTracker::isEnergyInjection(current, previous));
}

TEST(EnergyTracker, IsEnergyInjection_AllowsEnergyDecrease)
{
  // Energy decreased from 100 to 99 — dissipation is physical, not anomalous
  EXPECT_FALSE(EnergyTracker::isEnergyInjection(99.0, 100.0));
}

TEST(EnergyTracker, IsEnergyInjection_NearZeroEnergy_UsesAbsoluteTolerance)
{
  // Near-zero energy: relative tolerance is tiny, absolute tolerance dominates
  // Threshold = max(1e-6 * 1e-10, 1e-9) = 1e-9
  double const current = 1e-10 + 1e-8;
  double const previous = 1e-10;
  // delta = 1e-8, threshold = 1e-9 → injection detected
  EXPECT_TRUE(EnergyTracker::isEnergyInjection(current, previous));
}

TEST(EnergyTracker, IsEnergyInjection_ExactlySameEnergy_ReturnsFalse)
{
  EXPECT_FALSE(EnergyTracker::isEnergyInjection(100.0, 100.0));
}

// ========== System Energy Tests ==========

TEST(EnergyTracker, SystemEnergy_MultipleBodyAggregation)
{
  // This test just verifies the BodyEnergy total() works correctly
  // as we can't easily instantiate AssetInertial without a ConvexHull
  EnergyTracker::BodyEnergy body1{};
  body1.linearKE = 10.0;
  body1.rotationalKE = 5.0;
  body1.potentialE = 20.0;

  EnergyTracker::BodyEnergy body2{};
  body2.linearKE = 15.0;
  body2.rotationalKE = 8.0;
  body2.potentialE = 30.0;

  EXPECT_NEAR(body1.total(), 35.0, 1e-10);
  EXPECT_NEAR(body2.total(), 53.0, 1e-10);
}

// ========== Record Conversion Tests ==========

TEST(EnergyTracker, BodyEnergy_ToRecord_SetsAllFields)
{
  EnergyTracker::BodyEnergy energy{};
  energy.linearKE = 10.0;
  energy.rotationalKE = 5.0;
  energy.potentialE = 20.0;

  auto record = energy.toRecord(42, 7);

  EXPECT_EQ(record.frame.id, 42U);
  EXPECT_EQ(record.body_id, 7U);
  EXPECT_NEAR(record.linear_ke, 10.0, 1e-10);
  EXPECT_NEAR(record.rotational_ke, 5.0, 1e-10);
  EXPECT_NEAR(record.potential_e, 20.0, 1e-10);
  EXPECT_NEAR(record.total_e, 35.0, 1e-10);
}

TEST(EnergyTracker, SystemEnergy_ToRecord_SetsAllFields)
{
  EnergyTracker::SystemEnergy sysEnergy{};
  sysEnergy.totalLinearKE = 25.0;
  sysEnergy.totalRotationalKE = 13.0;
  sysEnergy.totalPotentialE = 50.0;

  double const previousEnergy = 80.0;
  auto record = sysEnergy.toRecord(10, previousEnergy, true);

  EXPECT_EQ(record.frame.id, 10U);
  EXPECT_NEAR(record.total_linear_ke, 25.0, 1e-10);
  EXPECT_NEAR(record.total_rotational_ke, 13.0, 1e-10);
  EXPECT_NEAR(record.total_potential_e, 50.0, 1e-10);
  EXPECT_NEAR(record.total_system_e, 88.0, 1e-10);
  EXPECT_NEAR(record.delta_e, 8.0, 1e-10);  // 88 - 80
  EXPECT_EQ(record.energy_injection, 1U);    // 8.0 >> tolerance
  EXPECT_EQ(record.collision_active, 1U);
}

TEST(EnergyTracker, SystemEnergy_ToRecord_NoInjection_WhenEnergyDecreases)
{
  EnergyTracker::SystemEnergy sysEnergy{};
  sysEnergy.totalLinearKE = 20.0;

  double const previousEnergy = 100.0;
  auto record = sysEnergy.toRecord(1, previousEnergy, false);

  EXPECT_NEAR(record.delta_e, -80.0, 1e-10);
  EXPECT_EQ(record.energy_injection, 0U);
  EXPECT_EQ(record.collision_active, 0U);
}

}  // namespace test
}  // namespace msd_sim
