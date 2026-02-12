// Tests for AssetDynamicState value struct
// Ticket: 0056h_asset_dynamic_state_struct

#include <gtest/gtest.h>

#include "msd-sim/src/DataTypes/Acceleration.hpp"
#include "msd-sim/src/DataTypes/AngularAcceleration.hpp"
#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-sim/src/DataTypes/Velocity.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetDynamicState.hpp"

// ============================================================================
// Default Construction
// ============================================================================

TEST(AssetDynamicStateTest, DefaultConstruction_ForcesVectorEmpty)
{
  msd_sim::AssetDynamicState state;

  EXPECT_TRUE(state.externalForces.empty());

  // Accumulated result should be zero
  auto accumulated = msd_sim::ExternalForce::accumulate(state.externalForces);
  EXPECT_DOUBLE_EQ(accumulated.force.x(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.force.y(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.force.z(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.torque.x(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.torque.y(), 0.0);
  EXPECT_DOUBLE_EQ(accumulated.torque.z(), 0.0);
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

TEST(AssetDynamicStateTest, ClearForces_ClearsNonEmptyVector)
{
  msd_sim::AssetDynamicState state;
  state.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{100.0, 200.0, 300.0},
                           msd_sim::TorqueVector{40.0, 50.0, 60.0},
                           msd_sim::Coordinate{0.0, 0.0, 0.0}});

  state.clearForces();

  EXPECT_TRUE(state.externalForces.empty());
}

TEST(AssetDynamicStateTest, ClearForces_PreservesKinematicState)
{
  msd_sim::AssetDynamicState state;
  state.inertialState.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  state.inertialState.velocity = msd_sim::Velocity{4.0, 5.0, 6.0};
  state.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{100.0, 200.0, 300.0},
                           msd_sim::TorqueVector{0.0, 0.0, 0.0},
                           msd_sim::Coordinate{0.0, 0.0, 0.0}});

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
// ExternalForce::accumulate()
// ============================================================================

TEST(AssetDynamicStateTest, Accumulate_SumsMultipleForces)
{
  std::vector<msd_sim::ExternalForce> forces;
  forces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{10.0, 0.0, 0.0},
                           msd_sim::TorqueVector{1.0, 0.0, 0.0},
                           msd_sim::Coordinate{1.0, 0.0, 0.0}});
  forces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{0.0, 20.0, 0.0},
                           msd_sim::TorqueVector{0.0, 2.0, 0.0},
                           msd_sim::Coordinate{0.0, 1.0, 0.0}});

  auto result = msd_sim::ExternalForce::accumulate(forces);

  EXPECT_DOUBLE_EQ(result.force.x(), 10.0);
  EXPECT_DOUBLE_EQ(result.force.y(), 20.0);
  EXPECT_DOUBLE_EQ(result.force.z(), 0.0);
  EXPECT_DOUBLE_EQ(result.torque.x(), 1.0);
  EXPECT_DOUBLE_EQ(result.torque.y(), 2.0);
  EXPECT_DOUBLE_EQ(result.torque.z(), 0.0);
  // Mean application point
  EXPECT_DOUBLE_EQ(result.applicationPoint.x(), 0.5);
  EXPECT_DOUBLE_EQ(result.applicationPoint.y(), 0.5);
  EXPECT_DOUBLE_EQ(result.applicationPoint.z(), 0.0);
}

// ============================================================================
// toRecord() / fromRecord() Round-Trip
// ============================================================================

TEST(AssetDynamicStateTest, ToRecord_CapturesIndividualForces)
{
  msd_sim::AssetDynamicState state;
  state.inertialState.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  state.inertialState.velocity = msd_sim::Velocity{4.0, 5.0, 6.0};
  state.inertialState.acceleration = msd_sim::Acceleration{7.0, 8.0, 9.0};
  state.inertialState.orientation = msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};
  state.inertialState.setAngularVelocity(
    msd_sim::AngularVelocity{0.1, 0.2, 0.3});
  state.inertialState.angularAcceleration =
    msd_sim::AngularAcceleration{10.0, 11.0, 12.0};

  // Push individual forces
  state.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{60.0, 100.0, 150.0},
                           msd_sim::TorqueVector{20.0, 25.0, 30.0},
                           msd_sim::Coordinate{1.0, 0.0, 0.0}});
  state.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{40.0, 100.0, 150.0},
                           msd_sim::TorqueVector{20.0, 25.0, 30.0},
                           msd_sim::Coordinate{0.0, 1.0, 0.0}});

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

  // Two individual force entries preserved
  ASSERT_EQ(record.externalForces.data.size(), 2u);

  EXPECT_DOUBLE_EQ(record.externalForces.data[0].force.x, 60.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].force.y, 100.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].force.z, 150.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].torque.x, 20.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].torque.y, 25.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].torque.z, 30.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].applicationPoint.x, 1.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].applicationPoint.y, 0.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[0].applicationPoint.z, 0.0);

  EXPECT_DOUBLE_EQ(record.externalForces.data[1].force.x, 40.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].force.y, 100.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].force.z, 150.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].torque.x, 20.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].torque.y, 25.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].torque.z, 30.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].applicationPoint.x, 0.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].applicationPoint.y, 1.0);
  EXPECT_DOUBLE_EQ(record.externalForces.data[1].applicationPoint.z, 0.0);
}

TEST(AssetDynamicStateTest, FromRecord_ReconstructsMultipleEntries)
{
  // Build a record by serializing a known state with two forces
  msd_sim::AssetDynamicState original;
  original.inertialState.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  original.inertialState.velocity = msd_sim::Velocity{4.0, 5.0, 6.0};
  original.inertialState.acceleration = msd_sim::Acceleration{7.0, 8.0, 9.0};
  original.inertialState.orientation =
    msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};
  original.inertialState.angularAcceleration =
    msd_sim::AngularAcceleration{10.0, 11.0, 12.0};
  original.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{100.0, 200.0, 300.0},
                           msd_sim::TorqueVector{40.0, 50.0, 60.0},
                           msd_sim::Coordinate{5.0, 6.0, 7.0}});
  original.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{10.0, 20.0, 30.0},
                           msd_sim::TorqueVector{4.0, 5.0, 6.0},
                           msd_sim::Coordinate{1.0, 2.0, 3.0}});

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

  // fromRecord reconstructs both individual entries
  ASSERT_EQ(state.externalForces.size(), 2u);
  EXPECT_DOUBLE_EQ(state.externalForces[0].force.x(), 100.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].force.y(), 200.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].force.z(), 300.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].torque.x(), 40.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].torque.y(), 50.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].torque.z(), 60.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].applicationPoint.x(), 5.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].applicationPoint.y(), 6.0);
  EXPECT_DOUBLE_EQ(state.externalForces[0].applicationPoint.z(), 7.0);

  EXPECT_DOUBLE_EQ(state.externalForces[1].force.x(), 10.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].force.y(), 20.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].force.z(), 30.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].torque.x(), 4.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].torque.y(), 5.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].torque.z(), 6.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].applicationPoint.x(), 1.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].applicationPoint.y(), 2.0);
  EXPECT_DOUBLE_EQ(state.externalForces[1].applicationPoint.z(), 3.0);
}

TEST(AssetDynamicStateTest, RoundTrip_Symmetry)
{
  msd_sim::AssetDynamicState original;
  original.inertialState.position = msd_sim::Coordinate{1.1, 2.2, 3.3};
  original.inertialState.velocity = msd_sim::Velocity{4.4, 5.5, 6.6};
  original.inertialState.acceleration = msd_sim::Acceleration{7.7, 8.8, 9.9};
  original.inertialState.orientation =
    msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};
  original.inertialState.setAngularVelocity(
    msd_sim::AngularVelocity{0.1, 0.2, 0.3});
  original.inertialState.angularAcceleration =
    msd_sim::AngularAcceleration{10.0, 11.0, 12.0};
  original.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{100.0, 200.0, 300.0},
                           msd_sim::TorqueVector{40.0, 50.0, 60.0},
                           msd_sim::Coordinate{1.0, 2.0, 3.0}});
  original.externalForces.push_back(
    msd_sim::ExternalForce{msd_sim::ForceVector{7.0, 8.0, 9.0},
                           msd_sim::TorqueVector{1.0, 2.0, 3.0},
                           msd_sim::Coordinate{4.0, 5.0, 6.0}});

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

  // Individual force entries preserved
  ASSERT_EQ(reconstructed.externalForces.size(),
            original.externalForces.size());
  for (size_t i = 0; i < original.externalForces.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].force.x(),
                     original.externalForces[i].force.x());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].force.y(),
                     original.externalForces[i].force.y());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].force.z(),
                     original.externalForces[i].force.z());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].torque.x(),
                     original.externalForces[i].torque.x());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].torque.y(),
                     original.externalForces[i].torque.y());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].torque.z(),
                     original.externalForces[i].torque.z());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].applicationPoint.x(),
                     original.externalForces[i].applicationPoint.x());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].applicationPoint.y(),
                     original.externalForces[i].applicationPoint.y());
    EXPECT_DOUBLE_EQ(reconstructed.externalForces[i].applicationPoint.z(),
                     original.externalForces[i].applicationPoint.z());
  }
}
