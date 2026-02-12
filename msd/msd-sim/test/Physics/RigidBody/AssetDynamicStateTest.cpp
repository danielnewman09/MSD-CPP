// Tests for AssetDynamicState value struct
// Ticket: 0056h_asset_dynamic_state_struct

#include <gtest/gtest.h>

#include "msd-sim/src/Physics/RigidBody/AssetDynamicState.hpp"

// ============================================================================
// Default Construction
// ============================================================================

TEST(AssetDynamicStateTest, DefaultConstruction_ForcesTorquesZero)
{
  msd_sim::AssetDynamicState state;

  EXPECT_DOUBLE_EQ(state.accumulatedForce.x(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedForce.y(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedForce.z(), 0.0);

  EXPECT_DOUBLE_EQ(state.accumulatedTorque.x(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedTorque.y(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedTorque.z(), 0.0);
}

TEST(AssetDynamicStateTest, DefaultConstruction_IdentityOrientation)
{
  msd_sim::AssetDynamicState state;

  EXPECT_DOUBLE_EQ(state.inertialState.orientation.w(), 1.0);
  EXPECT_DOUBLE_EQ(state.inertialState.orientation.x(), 0.0);
  EXPECT_DOUBLE_EQ(state.inertialState.orientation.y(), 0.0);
  EXPECT_DOUBLE_EQ(state.inertialState.orientation.z(), 0.0);
}

// ============================================================================
// clearForces()
// ============================================================================

TEST(AssetDynamicStateTest, ClearForces_ResetsNonZeroAccumulators)
{
  msd_sim::AssetDynamicState state;
  state.accumulatedForce = msd_sim::ForceVector{100.0, 200.0, 300.0};
  state.accumulatedTorque = msd_sim::TorqueVector{40.0, 50.0, 60.0};

  state.clearForces();

  EXPECT_DOUBLE_EQ(state.accumulatedForce.x(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedForce.y(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedForce.z(), 0.0);

  EXPECT_DOUBLE_EQ(state.accumulatedTorque.x(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedTorque.y(), 0.0);
  EXPECT_DOUBLE_EQ(state.accumulatedTorque.z(), 0.0);
}

TEST(AssetDynamicStateTest, ClearForces_PreservesKinematicState)
{
  msd_sim::AssetDynamicState state;
  state.inertialState.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  state.inertialState.velocity = msd_sim::Vector3D{4.0, 5.0, 6.0};
  state.accumulatedForce = msd_sim::ForceVector{100.0, 200.0, 300.0};

  state.clearForces();

  // Kinematic state should be untouched
  EXPECT_DOUBLE_EQ(state.inertialState.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(state.inertialState.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(state.inertialState.position.z(), 3.0);
  EXPECT_DOUBLE_EQ(state.inertialState.velocity.x(), 4.0);
  EXPECT_DOUBLE_EQ(state.inertialState.velocity.y(), 5.0);
  EXPECT_DOUBLE_EQ(state.inertialState.velocity.z(), 6.0);
}

// ============================================================================
// toRecord() / fromRecord() Round-Trip
// ============================================================================

TEST(AssetDynamicStateTest, ToRecord_CapturesAllFields)
{
  msd_sim::AssetDynamicState state;
  state.inertialState.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  state.inertialState.velocity = msd_sim::Vector3D{4.0, 5.0, 6.0};
  state.inertialState.acceleration = msd_sim::Vector3D{7.0, 8.0, 9.0};
  state.inertialState.orientation = msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};
  state.inertialState.setAngularVelocity(
    msd_sim::AngularRate{0.1, 0.2, 0.3});
  state.inertialState.angularAcceleration =
    msd_sim::AngularRate{10.0, 11.0, 12.0};
  state.accumulatedForce = msd_sim::ForceVector{100.0, 200.0, 300.0};
  state.accumulatedTorque = msd_sim::TorqueVector{40.0, 50.0, 60.0};

  auto record = state.toRecord(42);

  EXPECT_EQ(record.body_id, 42u);
  EXPECT_DOUBLE_EQ(record.kinematicState.position.x, 1.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.position.y, 2.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.position.z, 3.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.velocity.x, 4.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.velocity.y, 5.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.velocity.z, 6.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.acceleration.x, 7.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.acceleration.y, 8.0);
  EXPECT_DOUBLE_EQ(record.kinematicState.acceleration.z, 9.0);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.x, 100.0);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.y, 200.0);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.z, 300.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.x, 40.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.y, 50.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.z, 60.0);
}

TEST(AssetDynamicStateTest, FromRecord_ReconstructsCorrectly)
{
  // Build a record by serializing a known state (avoids aggregate init issues
  // with BaseTransferObject inheritance)
  msd_sim::AssetDynamicState original;
  original.inertialState.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  original.inertialState.velocity = msd_sim::Vector3D{4.0, 5.0, 6.0};
  original.inertialState.acceleration = msd_sim::Vector3D{7.0, 8.0, 9.0};
  original.inertialState.orientation =
    msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};
  original.inertialState.angularAcceleration =
    msd_sim::AngularRate{10.0, 11.0, 12.0};
  original.accumulatedForce = msd_sim::ForceVector{100.0, 200.0, 300.0};
  original.accumulatedTorque = msd_sim::TorqueVector{40.0, 50.0, 60.0};

  auto record = original.toRecord(7);
  auto state = msd_sim::AssetDynamicState::fromRecord(record);

  EXPECT_DOUBLE_EQ(state.inertialState.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(state.inertialState.position.y(), 2.0);
  EXPECT_DOUBLE_EQ(state.inertialState.position.z(), 3.0);
  EXPECT_DOUBLE_EQ(state.inertialState.velocity.x(), 4.0);
  EXPECT_DOUBLE_EQ(state.inertialState.velocity.y(), 5.0);
  EXPECT_DOUBLE_EQ(state.inertialState.velocity.z(), 6.0);
  EXPECT_DOUBLE_EQ(state.inertialState.acceleration.x(), 7.0);
  EXPECT_DOUBLE_EQ(state.inertialState.acceleration.y(), 8.0);
  EXPECT_DOUBLE_EQ(state.inertialState.acceleration.z(), 9.0);
  EXPECT_DOUBLE_EQ(state.inertialState.angularAcceleration.x(), 10.0);
  EXPECT_DOUBLE_EQ(state.inertialState.angularAcceleration.y(), 11.0);
  EXPECT_DOUBLE_EQ(state.inertialState.angularAcceleration.z(), 12.0);
  EXPECT_DOUBLE_EQ(state.accumulatedForce.x(), 100.0);
  EXPECT_DOUBLE_EQ(state.accumulatedForce.y(), 200.0);
  EXPECT_DOUBLE_EQ(state.accumulatedForce.z(), 300.0);
  EXPECT_DOUBLE_EQ(state.accumulatedTorque.x(), 40.0);
  EXPECT_DOUBLE_EQ(state.accumulatedTorque.y(), 50.0);
  EXPECT_DOUBLE_EQ(state.accumulatedTorque.z(), 60.0);
}

TEST(AssetDynamicStateTest, RoundTrip_Symmetry)
{
  msd_sim::AssetDynamicState original;
  original.inertialState.position = msd_sim::Coordinate{1.1, 2.2, 3.3};
  original.inertialState.velocity = msd_sim::Vector3D{4.4, 5.5, 6.6};
  original.inertialState.acceleration = msd_sim::Vector3D{7.7, 8.8, 9.9};
  original.inertialState.orientation =
    msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};
  original.inertialState.setAngularVelocity(
    msd_sim::AngularRate{0.1, 0.2, 0.3});
  original.inertialState.angularAcceleration =
    msd_sim::AngularRate{10.0, 11.0, 12.0};
  original.accumulatedForce = msd_sim::ForceVector{100.0, 200.0, 300.0};
  original.accumulatedTorque = msd_sim::TorqueVector{40.0, 50.0, 60.0};

  auto record = original.toRecord(99);
  auto reconstructed = msd_sim::AssetDynamicState::fromRecord(record);

  // Position
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.position.x(),
                   original.inertialState.position.x());
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.position.y(),
                   original.inertialState.position.y());
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.position.z(),
                   original.inertialState.position.z());

  // Velocity
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.velocity.x(),
                   original.inertialState.velocity.x());
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.velocity.y(),
                   original.inertialState.velocity.y());
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.velocity.z(),
                   original.inertialState.velocity.z());

  // Orientation
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.orientation.w(),
                   original.inertialState.orientation.w());
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.orientation.x(),
                   original.inertialState.orientation.x());
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.orientation.y(),
                   original.inertialState.orientation.y());
  EXPECT_DOUBLE_EQ(reconstructed.inertialState.orientation.z(),
                   original.inertialState.orientation.z());

  // Force/Torque
  EXPECT_DOUBLE_EQ(reconstructed.accumulatedForce.x(),
                   original.accumulatedForce.x());
  EXPECT_DOUBLE_EQ(reconstructed.accumulatedForce.y(),
                   original.accumulatedForce.y());
  EXPECT_DOUBLE_EQ(reconstructed.accumulatedForce.z(),
                   original.accumulatedForce.z());
  EXPECT_DOUBLE_EQ(reconstructed.accumulatedTorque.x(),
                   original.accumulatedTorque.x());
  EXPECT_DOUBLE_EQ(reconstructed.accumulatedTorque.y(),
                   original.accumulatedTorque.y());
  EXPECT_DOUBLE_EQ(reconstructed.accumulatedTorque.z(),
                   original.accumulatedTorque.z());
}
