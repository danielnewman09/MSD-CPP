// Transfer tests for AssetInertial::toDynamicStateRecord()

#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-transfer/src/AssetDynamicStateRecord.hpp"

namespace fs = std::filesystem;

namespace
{
std::vector<msd_sim::Coordinate> createCubePoints(double size)
{
  double h = size / 2.0;
  return {{-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
          {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
}
}  // namespace

class AssetDynamicStateTransferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_asset_dynamic_state_transfer.db";
    if (fs::exists(testDbPath_))
    {
      fs::remove(testDbPath_);
    }
    auto& logger = cpp_sqlite::Logger::getInstance();
    db_ = std::make_unique<cpp_sqlite::Database>(
      testDbPath_, true, logger.getLogger());

    // Create a 1m cube hull for test assets
    auto cubePoints = createCubePoints(1.0);
    hull_ = std::make_unique<msd_sim::ConvexHull>(cubePoints);
  }

  void TearDown() override
  {
    db_.reset();
  }

  std::string testDbPath_;
  std::unique_ptr<cpp_sqlite::Database> db_;
  std::unique_ptr<msd_sim::ConvexHull> hull_;
};

// ============================================================================
// Round-Trip Tests
// ============================================================================

TEST_F(AssetDynamicStateTransferTest, RoundTrip_AllFields)
{
  // Create asset with known properties
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{1.0, 2.0, 3.0}};
  msd_sim::AssetInertial asset{1, 42, *hull_, 50.0, frame, 0.8, 0.3};

  // Set dynamic state
  auto& state = asset.getInertialState();
  state.velocity = msd_sim::Vector3D{4.0, 5.0, 6.0};
  state.acceleration = msd_sim::Vector3D{7.0, 8.0, 9.0};
  state.setAngularVelocity(msd_sim::AngularRate{0.1, 0.2, 0.3});
  state.angularAcceleration = msd_sim::AngularRate{10.0, 11.0, 12.0};

  // Apply forces
  asset.applyForce(msd_sim::Vector3D{100.0, 200.0, 300.0});
  asset.applyTorque(msd_sim::Vector3D{40.0, 50.0, 60.0});

  // Serialize
  auto record = asset.toDynamicStateRecord();

  // Verify body ID
  EXPECT_EQ(record.body_id, 42u);

  // Verify kinematic state
  EXPECT_DOUBLE_EQ(record.position.x, 1.0);
  EXPECT_DOUBLE_EQ(record.position.y, 2.0);
  EXPECT_DOUBLE_EQ(record.position.z, 3.0);

  EXPECT_DOUBLE_EQ(record.velocity.x, 4.0);
  EXPECT_DOUBLE_EQ(record.velocity.y, 5.0);
  EXPECT_DOUBLE_EQ(record.velocity.z, 6.0);

  EXPECT_DOUBLE_EQ(record.acceleration.x, 7.0);
  EXPECT_DOUBLE_EQ(record.acceleration.y, 8.0);
  EXPECT_DOUBLE_EQ(record.acceleration.z, 9.0);

  EXPECT_DOUBLE_EQ(record.angularAcceleration.pitch, 10.0);
  EXPECT_DOUBLE_EQ(record.angularAcceleration.roll, 11.0);
  EXPECT_DOUBLE_EQ(record.angularAcceleration.yaw, 12.0);

  // Verify force/torque
  EXPECT_DOUBLE_EQ(record.accumulatedForce.x, 100.0);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.y, 200.0);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.z, 300.0);

  EXPECT_DOUBLE_EQ(record.accumulatedTorque.x, 40.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.y, 50.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.z, 60.0);

  // Database round-trip
  auto& dao =
    db_->getDAO<msd_transfer::AssetDynamicStateRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  EXPECT_EQ(retrieved->body_id, 42u);
  EXPECT_DOUBLE_EQ(retrieved->position.x, 1.0);
  EXPECT_DOUBLE_EQ(retrieved->velocity.y, 5.0);
  EXPECT_DOUBLE_EQ(retrieved->acceleration.z, 9.0);
  EXPECT_DOUBLE_EQ(retrieved->accumulatedForce.x, 100.0);
  EXPECT_DOUBLE_EQ(retrieved->accumulatedTorque.z, 60.0);
}

TEST_F(AssetDynamicStateTransferTest, RoundTrip_ZeroForcesTorques)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  msd_sim::AssetInertial asset{1, 7, *hull_, 10.0, frame};

  // No forces applied — should be zero
  auto record = asset.toDynamicStateRecord();

  EXPECT_EQ(record.body_id, 7u);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.x, 0.0);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.y, 0.0);
  EXPECT_DOUBLE_EQ(record.accumulatedForce.z, 0.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.x, 0.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.y, 0.0);
  EXPECT_DOUBLE_EQ(record.accumulatedTorque.z, 0.0);
}

TEST_F(AssetDynamicStateTransferTest, RoundTrip_OrientationPreserved)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  msd_sim::AssetInertial asset{1, 1, *hull_, 10.0, frame};

  // Set a non-trivial orientation
  auto& state = asset.getInertialState();
  state.orientation = msd_sim::QuaternionD{0.5, 0.5, 0.5, 0.5};

  auto record = asset.toDynamicStateRecord();

  EXPECT_DOUBLE_EQ(record.orientation.w, 0.5);
  EXPECT_DOUBLE_EQ(record.orientation.x, 0.5);
  EXPECT_DOUBLE_EQ(record.orientation.y, 0.5);
  EXPECT_DOUBLE_EQ(record.orientation.z, 0.5);
}

TEST_F(AssetDynamicStateTransferTest, SelectNonexistent_ReturnsNullopt)
{
  auto& dao =
    db_->getDAO<msd_transfer::AssetDynamicStateRecord>();

  auto result = dao.selectById(99999);
  EXPECT_FALSE(result.has_value());
}
