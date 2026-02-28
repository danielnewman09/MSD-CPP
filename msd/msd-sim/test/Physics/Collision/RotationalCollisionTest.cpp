// Ticket: 0039c_rotational_coupling_test_suite
// Ticket: 0062c_replay_rotational_collision_tests
// Test: Scenario B -- Rotation initiation from off-center impacts
// Converted to ReplayEnabledTest fixture for automatic replay recording
//
// DIAGNOSTIC TEST SUITE: Some tests are EXPECTED to fail because they
// investigate a known energy injection bug in rotational collisions.
// Failures are valid diagnostic findings, not test implementation errors.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include <Eigen/Geometry>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/test/Replay/ReplayEnabledTest.hpp"

using namespace msd_sim;

// ============================================================================
// Helper Functions (local to tests, not duplicating fixture functionality)
// ============================================================================

namespace
{

/// Compute total system energy using EnergyTracker with gravity potential
double computeSystemEnergy(const WorldModel& world)
{
  std::vector<std::unique_ptr<PotentialEnergy>> potentials;
  potentials.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));

  auto sysEnergy = EnergyTracker::computeSystemEnergy(
    world.getInertialAssets(), potentials);
  return sysEnergy.total();
}

}  // anonymous namespace

// ============================================================================
// B1: Cube corner impact on floor at 45 degrees
// Validates: Lever arm coupling -- rotation should initiate from off-center impact
// ============================================================================

TEST_F(ReplayEnabledTest, RotationalCollisionTest_CubeCornerImpact_RotationInitiated)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests

  // Floor: large cube centered at z=-50 (surface at z=0)
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: 1m x 1m x 1m, rotated 45 degrees about x-axis AND 45 degrees about
  // y-axis so a corner points downward.
  Eigen::Quaterniond q =
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitY()};

  // Position the cube so the lowest corner is at approximately z=2 above floor
  // For a unit cube rotated 45/45 degrees, the half-diagonal is sqrt(3)/2 ~ 0.866
  double const halfDiag = std::sqrt(3.0) / 2.0;

  // NOTE: Fixture doesn't yet support orientation parameter, need to spawn
  // and then manually set orientation. For now, spawn at standard orientation
  // and document the limitation.
  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, 2.0 + halfDiag},
                                   1.0,  // mass (kg)
                                   0.7,  // restitution
                                   0.5); // friction
  uint32_t cubeId = cube.getInstanceId();

  // Manually set orientation (fixture limitation workaround)
  world().getObject(cubeId).getInertialState().orientation = q;

  double const initialEnergy = computeSystemEnergy(world());

  // Simulate enough frames for impact and bouncing
  // Free fall from ~2.87m takes about sqrt(2*2.87/9.81) ~ 0.76s ~ 48 frames
  bool rotationDetected = false;
  bool nanDetected = false;
  double maxOmega = 0.0;

  for (int i = 1; i <= 200; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();
    double omegaMag = omega.norm();

    // Check for NaN
    if (std::isnan(omegaMag) || std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxOmega = std::max(maxOmega, omegaMag);

    if (omegaMag > 0.1)
    {
      rotationDetected = true;
    }
  }

  // DIAGNOSTIC: No NaN or explosion
  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in cube corner impact simulation";

  // DIAGNOSTIC: Post-impact angular velocity should be non-zero
  EXPECT_TRUE(rotationDetected)
    << "DIAGNOSTIC: Cube should begin rotating after corner impact. "
    << "Max omega=" << maxOmega << " rad/s";

  EXPECT_GT(maxOmega, 0.1)
    << "DIAGNOSTIC: Post-impact |omega| should exceed 0.1 rad/s. "
    << "Got maxOmega=" << maxOmega;

  // DIAGNOSTIC: Energy should decrease with each bounce (e=0.7 < 1)
  double const finalEnergy = computeSystemEnergy(world());
  if (!nanDetected)
  {
    EXPECT_LE(finalEnergy, initialEnergy * 1.05)
      << "DIAGNOSTIC: Energy should not grow significantly. "
      << "Initial=" << initialEnergy << " Final=" << finalEnergy
      << " Ratio=" << (finalEnergy / initialEnergy);
  }
}

// ============================================================================
// B2: Cube edge impact (asymmetric tilt to break symmetry)
// Validates: Edge contact handling -- predictable rotation axis
//
// PHYSICS: A cube rotated exactly 45° about one axis has a symmetric edge
// contact — both contact points are equidistant from the COM, producing
// equal and opposite torques that cancel. To test rotation from edge
// impact, we add a 5° tilt about X to break this symmetry so one end
// of the edge hits first, generating net torque.
// ============================================================================

TEST_F(ReplayEnabledTest, RotationalCollisionTest_CubeEdgeImpact_PredictableRotationAxis)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests

  // Floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Cube: rotated 45° about Y (edge down) + 5° about X (asymmetric tilt)
  Eigen::Quaterniond q =
    Eigen::AngleAxisd{5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()} *
    Eigen::AngleAxisd{M_PI / 4.0, Eigen::Vector3d::UnitY()};

  double const halfDiag2D = std::sqrt(2.0) / 2.0;

  const auto& cube = spawnInertial("unit_cube",
                                   Coordinate{0.0, 0.0, 1.0 + halfDiag2D},
                                   1.0,  // mass (kg)
                                   0.7,  // restitution
                                   0.5); // friction
  uint32_t cubeId = cube.getInstanceId();

  // Manually set orientation
  world().getObject(cubeId).getInertialState().orientation = q;

  double const initialEnergy = computeSystemEnergy(world());

  // Simulate for impact
  bool nanDetected = false;
  double maxOmegaY = 0.0;
  double maxOmegaTotal = 0.0;

  for (int i = 1; i <= 200; ++i)
  {
    step(1);

    auto const& state = world().getObject(cubeId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();

    if (std::isnan(omega.norm()) || std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    // For rotation about y-axis (the edge-parallel axis), we expect
    // the rotation to be primarily about y. The edge is parallel to y-axis
    // so the torque should cause rotation about y.
    maxOmegaY = std::max(maxOmegaY, std::abs(omega.y()));
    maxOmegaTotal = std::max(maxOmegaTotal, omega.norm());
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in cube edge impact simulation";

  // DIAGNOSTIC: Rotation should be initiated
  EXPECT_GT(maxOmegaTotal, 0.1)
    << "DIAGNOSTIC: Edge impact should initiate rotation. "
    << "Max omega=" << maxOmegaTotal << " rad/s";

  // DIAGNOSTIC: Rotation axis should be primarily about the y-axis
  // (perpendicular to the plane of rotation which is the xz-plane)
  if (maxOmegaTotal > 0.1)
  {
    double const yFraction = maxOmegaY / maxOmegaTotal;
    EXPECT_GT(yFraction, 0.3)
      << "DIAGNOSTIC: Rotation should have significant y-component. "
      << "y-fraction=" << yFraction
      << " (omega_y=" << maxOmegaY
      << ", omega_total=" << maxOmegaTotal << ")";
  }

  // DIAGNOSTIC: Energy should not grow
  double const finalEnergy = computeSystemEnergy(world());
  if (!nanDetected)
  {
    EXPECT_LE(finalEnergy, initialEnergy * 1.05)
      << "DIAGNOSTIC: Energy should not grow. "
      << "Initial=" << initialEnergy << " Final=" << finalEnergy;
  }
}

// ============================================================================
// B3: Sphere with symmetric contact drops vertically (negative test)
// Validates: Lever arm = 0 for symmetric contact -- no rotation
//
// FIX: This test now PASSES after ticket 0047a_revert_gravity_preapply.
// With gravity pre-apply (0047), restitution-gravity coupling introduced
// spurious torque via the e*J*g*dt term in the constraint RHS. The extra
// term caused off-center impulse application at the EPA contact point,
// producing rotation where there should be none.
//
// After revert (0047a): Pure normal impulse at contact point, no coupling
// term, no spurious torque → sphere does not rotate ✅
// ============================================================================

TEST_F(ReplayEnabledTest, RotationalCollisionTest_SphereDrop_NoRotation)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests

  // Floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Sphere dropped vertically
  const auto& sphere = spawnInertial("small_sphere",
                                     Coordinate{0.0, 0.0, 2.0},
                                     1.0,  // mass (kg)
                                     0.7,  // restitution
                                     0.5); // friction
  uint32_t sphereId = sphere.getInstanceId();

  double maxOmega = 0.0;
  double maxLateralDrift = 0.0;

  for (int i = 1; i <= 200; ++i)
  {
    step(1);

    auto const& state = world().getObject(sphereId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();
    maxOmega = std::max(maxOmega, omega.norm());

    // Check for lateral drift (should stay near x=0, y=0)
    double lateralDist = std::sqrt(state.position.x() * state.position.x() +
                                   state.position.y() * state.position.y());
    maxLateralDrift = std::max(maxLateralDrift, lateralDist);
  }

  // DIAGNOSTIC: Symmetric contact should produce no rotation
  // Note: icosphere is not a perfect sphere, so very small rotation is
  // acceptable from polyhedral contact geometry artifacts
  EXPECT_LT(maxOmega, 0.5)
    << "DIAGNOSTIC: Sphere should have minimal rotation. "
    << "Got maxOmega=" << maxOmega << " rad/s";

  // DIAGNOSTIC: Pure vertical bouncing, no lateral drift
  EXPECT_LT(maxLateralDrift, 0.5)
    << "DIAGNOSTIC: Sphere should not drift laterally. "
    << "Got maxLateralDrift=" << maxLateralDrift << " m";
}

// ============================================================================
// B4: Rod (elongated box) falls flat (negative test)
// Validates: No rotation for symmetric flat contact at COM level
//
// NOTE: This test is currently DISABLED because the test asset database
// does not include a rod/elongated box primitive. When the asset generator
// is extended to support custom dimensions, re-enable this test using
// a "rod" asset (2m x 0.2m x 0.2m).
// ============================================================================

TEST_F(ReplayEnabledTest, DISABLED_RotationalCollisionTest_RodFallsFlat_NoRotation)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests
  // DISABLED: Requires "rod" asset not yet in test database

  // Floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // Rod: 2m x 0.2m x 0.2m, long axis horizontal (parallel to x-axis)
  // Position with bottom face at z=1 (center at z=1.1 since half-height=0.1)
  const auto& rod = spawnInertial("rod",  // Asset does not exist yet
                                  Coordinate{0.0, 0.0, 1.1},
                                  1.0,  // mass (kg)
                                  0.3,  // restitution
                                  0.5); // friction
  uint32_t rodId = rod.getInstanceId();

  double maxOmega = 0.0;

  for (int i = 1; i <= 300; ++i)
  {
    step(1);

    auto const& state = world().getObject(rodId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();

    if (std::isnan(omega.norm()))
    {
      break;
    }

    maxOmega = std::max(maxOmega, omega.norm());
  }

  // DIAGNOSTIC: Rod falling flat should not rotate
  // Multiple contact points along the bottom face should produce zero net torque
  EXPECT_LT(maxOmega, 0.5)
    << "DIAGNOSTIC: Rod should settle flat without significant rotation. "
    << "Got maxOmega=" << maxOmega << " rad/s";
}

// ============================================================================
// B5: Asymmetric hull (L-shape) dropped flat
// Validates: r = P_contact - P_COM (not geometric center)
//
// NOTE: This test is currently DISABLED because the test asset database
// does not include an L-shaped primitive. When the asset generator is
// extended to support custom geometry, re-enable this test using an
// "l_shape" asset.
//
// NOTE: ConvexHull computes COM from geometry, so we cannot directly offset
// the COM. Instead, we use an L-shaped hull whose COM is naturally offset from
// the geometric center. If the physics engine correctly uses P_COM (not
// geometric center), the L-shape should rotate upon flat-face impact.
// ============================================================================

TEST_F(ReplayEnabledTest, DISABLED_RotationalCollisionTest_LShapeDrop_RotationFromAsymmetricCOM)
{
  // Ticket: 0039c_rotational_coupling_test_suite
  // Ticket: 0062c_replay_rotational_collision_tests
  // DISABLED: Requires "l_shape" asset not yet in test database

  // Floor
  spawnEnvironment("floor_slab", Coordinate{0.0, 0.0, -50.0});

  // L-shape hull: COM is offset in +x direction from geometric center
  const auto& lShape = spawnInertial("l_shape",  // Asset does not exist yet
                                     Coordinate{0.0, 0.0, 2.0},
                                     1.0,  // mass (kg)
                                     0.5,  // restitution
                                     0.5); // friction
  uint32_t lId = lShape.getInstanceId();

  double maxOmega = 0.0;
  bool nanDetected = false;

  for (int i = 1; i <= 300; ++i)
  {
    step(1);

    auto const& state = world().getObject(lId).getInertialState();
    AngularVelocity omega = state.getAngularVelocity();

    if (std::isnan(omega.norm()) || std::isnan(state.position.z()))
    {
      nanDetected = true;
      break;
    }

    maxOmega = std::max(maxOmega, omega.norm());
  }

  EXPECT_FALSE(nanDetected)
    << "DIAGNOSTIC: NaN detected in L-shape drop simulation";

  // DIAGNOSTIC: The L-shape has an asymmetric COM. When it impacts the floor,
  // the contact points will NOT be centered under the COM, creating a net
  // torque. This tests that lever arms use P_contact - P_COM, not
  // P_contact - P_geometric_center.
  //
  // NOTE: Whether or not rotation occurs depends on whether the convex hull
  // of the L-shape points creates a shape with sufficiently offset COM.
  // The convex hull may "fill in" the L-shape, making it more symmetric.
  // If no rotation is observed, that may indicate the convex hull COM is
  // close enough to the geometric center of the convex hull that torques
  // cancel out. This is still a valid diagnostic result.
  EXPECT_GT(maxOmega, 0.01)
    << "DIAGNOSTIC: L-shape should exhibit some rotation due to asymmetric COM. "
    << "Got maxOmega=" << maxOmega << " rad/s. "
    << "Note: convex hull may fill the L-shape, reducing asymmetry.";
}
