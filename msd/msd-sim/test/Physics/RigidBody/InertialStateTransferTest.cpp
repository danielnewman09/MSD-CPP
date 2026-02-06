// Ticket: vector_datatype_refactor
// Transfer tests for InertialState

#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"
#include "msd-transfer/src/InertialStateRecord.hpp"

namespace fs = std::filesystem;

class InertialStateTransferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_inertial_state_transfer.db";
    if (fs::exists(testDbPath_))
    {
      fs::remove(testDbPath_);
    }
    auto& logger = cpp_sqlite::Logger::getInstance();
    db_ = std::make_unique<cpp_sqlite::Database>(
      testDbPath_, true, logger.getLogger());
  }

  void TearDown() override
  {
    db_.reset();
  }

  std::string testDbPath_;
  std::unique_ptr<cpp_sqlite::Database> db_;
};

// ============================================================================
// Round-Trip Tests
// ============================================================================

TEST_F(InertialStateTransferTest, RoundTrip_AllFields)
{
  // Create domain object with known values
  msd_sim::InertialState original;
  original.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  original.velocity = msd_sim::Vector3D{4.0, 5.0, 6.0};
  original.acceleration = msd_sim::Vector3D{7.0, 8.0, 9.0};
  original.orientation = msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};  // w,x,y,z
  original.quaternionRate = msd_sim::Vector4D{0.1, 0.2, 0.3, 0.4};
  original.angularAcceleration = msd_sim::AngularRate{10.0, 11.0, 12.0};

  // Serialize to record
  auto record = original.toRecord();

  // Verify nested record fields
  EXPECT_DOUBLE_EQ(record.position.x, 1.0);
  EXPECT_DOUBLE_EQ(record.position.y, 2.0);
  EXPECT_DOUBLE_EQ(record.position.z, 3.0);

  EXPECT_DOUBLE_EQ(record.velocity.x, 4.0);
  EXPECT_DOUBLE_EQ(record.velocity.y, 5.0);
  EXPECT_DOUBLE_EQ(record.velocity.z, 6.0);

  EXPECT_DOUBLE_EQ(record.acceleration.x, 7.0);
  EXPECT_DOUBLE_EQ(record.acceleration.y, 8.0);
  EXPECT_DOUBLE_EQ(record.acceleration.z, 9.0);

  EXPECT_DOUBLE_EQ(record.orientation.w, 0.5);
  EXPECT_DOUBLE_EQ(record.orientation.x, 0.5);
  EXPECT_DOUBLE_EQ(record.orientation.y, 0.5);
  EXPECT_DOUBLE_EQ(record.orientation.z, 0.5);

  EXPECT_DOUBLE_EQ(record.quaternionRate.x, 0.1);
  EXPECT_DOUBLE_EQ(record.quaternionRate.y, 0.2);
  EXPECT_DOUBLE_EQ(record.quaternionRate.z, 0.3);
  EXPECT_DOUBLE_EQ(record.quaternionRate.w, 0.4);

  EXPECT_DOUBLE_EQ(record.angularAcceleration.pitch, 10.0);
  EXPECT_DOUBLE_EQ(record.angularAcceleration.roll, 11.0);
  EXPECT_DOUBLE_EQ(record.angularAcceleration.yaw, 12.0);

  // Deserialize back to domain object
  auto restored = msd_sim::InertialState::fromRecord(record);

  // Compare all components
  EXPECT_DOUBLE_EQ(original.position.x(), restored.position.x());
  EXPECT_DOUBLE_EQ(original.position.y(), restored.position.y());
  EXPECT_DOUBLE_EQ(original.position.z(), restored.position.z());

  EXPECT_DOUBLE_EQ(original.velocity.x(), restored.velocity.x());
  EXPECT_DOUBLE_EQ(original.velocity.y(), restored.velocity.y());
  EXPECT_DOUBLE_EQ(original.velocity.z(), restored.velocity.z());

  EXPECT_DOUBLE_EQ(original.acceleration.x(), restored.acceleration.x());
  EXPECT_DOUBLE_EQ(original.acceleration.y(), restored.acceleration.y());
  EXPECT_DOUBLE_EQ(original.acceleration.z(), restored.acceleration.z());

  EXPECT_DOUBLE_EQ(original.orientation.w(), restored.orientation.w());
  EXPECT_DOUBLE_EQ(original.orientation.x(), restored.orientation.x());
  EXPECT_DOUBLE_EQ(original.orientation.y(), restored.orientation.y());
  EXPECT_DOUBLE_EQ(original.orientation.z(), restored.orientation.z());

  EXPECT_DOUBLE_EQ(original.quaternionRate.x(), restored.quaternionRate.x());
  EXPECT_DOUBLE_EQ(original.quaternionRate.y(), restored.quaternionRate.y());
  EXPECT_DOUBLE_EQ(original.quaternionRate.z(), restored.quaternionRate.z());
  EXPECT_DOUBLE_EQ(original.quaternionRate.w(), restored.quaternionRate.w());

  EXPECT_DOUBLE_EQ(original.angularAcceleration.pitch(),
                   restored.angularAcceleration.pitch());
  EXPECT_DOUBLE_EQ(original.angularAcceleration.roll(),
                   restored.angularAcceleration.roll());
  EXPECT_DOUBLE_EQ(original.angularAcceleration.yaw(),
                   restored.angularAcceleration.yaw());
}

TEST_F(InertialStateTransferTest, RoundTrip_DefaultValues)
{
  // Test with default-constructed InertialState
  msd_sim::InertialState original;

  auto record = original.toRecord();
  auto restored = msd_sim::InertialState::fromRecord(record);

  // Default orientation is identity quaternion
  EXPECT_DOUBLE_EQ(restored.orientation.w(), 1.0);
  EXPECT_DOUBLE_EQ(restored.orientation.x(), 0.0);
  EXPECT_DOUBLE_EQ(restored.orientation.y(), 0.0);
  EXPECT_DOUBLE_EQ(restored.orientation.z(), 0.0);

  // Default quaternion rate is zero
  EXPECT_DOUBLE_EQ(restored.quaternionRate.x(), 0.0);
  EXPECT_DOUBLE_EQ(restored.quaternionRate.y(), 0.0);
  EXPECT_DOUBLE_EQ(restored.quaternionRate.z(), 0.0);
  EXPECT_DOUBLE_EQ(restored.quaternionRate.w(), 0.0);

  // Default angular acceleration is zero
  EXPECT_DOUBLE_EQ(restored.angularAcceleration.pitch(), 0.0);
  EXPECT_DOUBLE_EQ(restored.angularAcceleration.roll(), 0.0);
  EXPECT_DOUBLE_EQ(restored.angularAcceleration.yaw(), 0.0);
}

TEST_F(InertialStateTransferTest, RoundTrip_NegativeValues)
{
  msd_sim::InertialState original;
  original.position = msd_sim::Coordinate{-100.5, -200.75, -300.125};
  original.velocity = msd_sim::Vector3D{-1.0, -2.0, -3.0};
  original.acceleration = msd_sim::Vector3D{-4.0, -5.0, -6.0};
  original.orientation = msd_sim::QuaternionD{-0.5, -0.5, -0.5, -0.5};
  original.quaternionRate = msd_sim::Vector4D{-0.1, -0.2, -0.3, -0.4};
  original.angularAcceleration = msd_sim::AngularRate{-7.0, -8.0, -9.0};

  auto record = original.toRecord();
  auto restored = msd_sim::InertialState::fromRecord(record);

  EXPECT_DOUBLE_EQ(original.position.x(), restored.position.x());
  EXPECT_DOUBLE_EQ(original.position.y(), restored.position.y());
  EXPECT_DOUBLE_EQ(original.position.z(), restored.position.z());

  EXPECT_DOUBLE_EQ(original.velocity.x(), restored.velocity.x());
  EXPECT_DOUBLE_EQ(original.velocity.y(), restored.velocity.y());
  EXPECT_DOUBLE_EQ(original.velocity.z(), restored.velocity.z());

  EXPECT_DOUBLE_EQ(original.acceleration.x(), restored.acceleration.x());
  EXPECT_DOUBLE_EQ(original.acceleration.y(), restored.acceleration.y());
  EXPECT_DOUBLE_EQ(original.acceleration.z(), restored.acceleration.z());

  EXPECT_DOUBLE_EQ(original.orientation.w(), restored.orientation.w());
  EXPECT_DOUBLE_EQ(original.orientation.x(), restored.orientation.x());
  EXPECT_DOUBLE_EQ(original.orientation.y(), restored.orientation.y());
  EXPECT_DOUBLE_EQ(original.orientation.z(), restored.orientation.z());

  EXPECT_DOUBLE_EQ(original.quaternionRate.x(), restored.quaternionRate.x());
  EXPECT_DOUBLE_EQ(original.quaternionRate.y(), restored.quaternionRate.y());
  EXPECT_DOUBLE_EQ(original.quaternionRate.z(), restored.quaternionRate.z());
  EXPECT_DOUBLE_EQ(original.quaternionRate.w(), restored.quaternionRate.w());

  EXPECT_DOUBLE_EQ(original.angularAcceleration.pitch(),
                   restored.angularAcceleration.pitch());
  EXPECT_DOUBLE_EQ(original.angularAcceleration.roll(),
                   restored.angularAcceleration.roll());
  EXPECT_DOUBLE_EQ(original.angularAcceleration.yaw(),
                   restored.angularAcceleration.yaw());
}

TEST_F(InertialStateTransferTest, RoundTrip_LargeValues)
{
  msd_sim::InertialState original;
  original.position = msd_sim::Coordinate{1.0e15, -2.5e12, 9.99e18};
  original.velocity = msd_sim::Vector3D{1.0e6, 2.0e6, 3.0e6};
  original.acceleration = msd_sim::Vector3D{1.0e3, 2.0e3, 3.0e3};
  original.quaternionRate = msd_sim::Vector4D{1.0e-6, 2.0e-6, 3.0e-6, 4.0e-6};
  original.angularAcceleration = msd_sim::AngularRate{1.0e2, 2.0e2, 3.0e2};

  auto record = original.toRecord();
  auto restored = msd_sim::InertialState::fromRecord(record);

  EXPECT_DOUBLE_EQ(original.position.x(), restored.position.x());
  EXPECT_DOUBLE_EQ(original.position.y(), restored.position.y());
  EXPECT_DOUBLE_EQ(original.position.z(), restored.position.z());

  EXPECT_DOUBLE_EQ(original.velocity.x(), restored.velocity.x());
  EXPECT_DOUBLE_EQ(original.velocity.y(), restored.velocity.y());
  EXPECT_DOUBLE_EQ(original.velocity.z(), restored.velocity.z());

  EXPECT_DOUBLE_EQ(original.quaternionRate.x(), restored.quaternionRate.x());
  EXPECT_DOUBLE_EQ(original.quaternionRate.y(), restored.quaternionRate.y());
  EXPECT_DOUBLE_EQ(original.quaternionRate.z(), restored.quaternionRate.z());
  EXPECT_DOUBLE_EQ(original.quaternionRate.w(), restored.quaternionRate.w());
}

// ============================================================================
// Serialization Tests
// ============================================================================

TEST_F(InertialStateTransferTest, ToRecord_MapsNestedRecordsCorrectly)
{
  msd_sim::InertialState state;
  state.position = msd_sim::Coordinate{1.0, 2.0, 3.0};
  state.velocity = msd_sim::Vector3D{4.0, 5.0, 6.0};

  auto record = state.toRecord();

  // Check that nested records are populated
  EXPECT_DOUBLE_EQ(record.position.x, 1.0);
  EXPECT_DOUBLE_EQ(record.position.y, 2.0);
  EXPECT_DOUBLE_EQ(record.position.z, 3.0);

  EXPECT_DOUBLE_EQ(record.velocity.x, 4.0);
  EXPECT_DOUBLE_EQ(record.velocity.y, 5.0);
  EXPECT_DOUBLE_EQ(record.velocity.z, 6.0);
}

TEST_F(InertialStateTransferTest, FromRecord_MapsNestedRecordsCorrectly)
{
  msd_transfer::InertialStateRecord record;
  record.position.x = 10.0;
  record.position.y = 20.0;
  record.position.z = 30.0;
  record.velocity.x = 40.0;
  record.velocity.y = 50.0;
  record.velocity.z = 60.0;
  record.orientation.w = 1.0;
  record.orientation.x = 0.0;
  record.orientation.y = 0.0;
  record.orientation.z = 0.0;

  auto state = msd_sim::InertialState::fromRecord(record);

  EXPECT_DOUBLE_EQ(state.position.x(), 10.0);
  EXPECT_DOUBLE_EQ(state.position.y(), 20.0);
  EXPECT_DOUBLE_EQ(state.position.z(), 30.0);

  EXPECT_DOUBLE_EQ(state.velocity.x(), 40.0);
  EXPECT_DOUBLE_EQ(state.velocity.y(), 50.0);
  EXPECT_DOUBLE_EQ(state.velocity.z(), 60.0);

  EXPECT_DOUBLE_EQ(state.orientation.w(), 1.0);
}

// ============================================================================
// Angular Velocity Preservation Test
// ============================================================================

TEST_F(InertialStateTransferTest, RoundTrip_AngularVelocityPreserved)
{
  // Set angular velocity, which internally sets quaternionRate
  msd_sim::InertialState original;
  original.orientation = msd_sim::QuaternionD{1.0, 0.0, 0.0, 0.0};
  original.setAngularVelocity(msd_sim::AngularRate{1.0, 2.0, 3.0});

  auto record = original.toRecord();
  auto restored = msd_sim::InertialState::fromRecord(record);

  // Compare the derived angular velocity
  auto originalOmega = original.getAngularVelocity();
  auto restoredOmega = restored.getAngularVelocity();

  EXPECT_NEAR(originalOmega.pitch(), restoredOmega.pitch(), 1e-10);
  EXPECT_NEAR(originalOmega.roll(), restoredOmega.roll(), 1e-10);
  EXPECT_NEAR(originalOmega.yaw(), restoredOmega.yaw(), 1e-10);
}
