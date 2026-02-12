#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/DataTypes/AngularAcceleration.hpp"
#include "msd-transfer/src/AngularAccelerationRecord.hpp"

namespace fs = std::filesystem;

class AngularAccelerationTransferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_angular_acceleration_transfer.db";
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

TEST_F(AngularAccelerationTransferTest, RoundTrip_AllFields)
{
  msd_sim::AngularAcceleration original{0.5, -1.25, 3.75};

  auto record = original.toRecord();

  EXPECT_DOUBLE_EQ(record.pitch, 0.5);
  EXPECT_DOUBLE_EQ(record.roll, -1.25);
  EXPECT_DOUBLE_EQ(record.yaw, 3.75);

  auto& dao = db_->getDAO<msd_transfer::AngularAccelerationRecord>();
  dao.insert(record);
  EXPECT_GT(record.id, 0);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularAcceleration::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularAccelerationTransferTest, RoundTrip_ZeroValues)
{
  msd_sim::AngularAcceleration original{0.0, 0.0, 0.0};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularAccelerationRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularAcceleration::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularAccelerationTransferTest, RoundTrip_NegativeValues)
{
  msd_sim::AngularAcceleration original{-10.5, -20.75, -30.125};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularAccelerationRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularAcceleration::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularAccelerationTransferTest, RoundTrip_LargeValues)
{
  msd_sim::AngularAcceleration original{1.0e15, -2.5e12, 9.99e18};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularAccelerationRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularAcceleration::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

// ============================================================================
// Serialization Tests
// ============================================================================

TEST_F(AngularAccelerationTransferTest, ToRecord_MapsFieldsCorrectly)
{
  msd_sim::AngularAcceleration accel{4.0, 5.0, 6.0};
  auto record = accel.toRecord();

  EXPECT_DOUBLE_EQ(record.pitch, 4.0);
  EXPECT_DOUBLE_EQ(record.roll, 5.0);
  EXPECT_DOUBLE_EQ(record.yaw, 6.0);
}

TEST_F(AngularAccelerationTransferTest, FromRecord_MapsFieldsCorrectly)
{
  msd_transfer::AngularAccelerationRecord record;
  record.pitch = 7.0;
  record.roll = 8.0;
  record.yaw = 9.0;

  auto accel = msd_sim::AngularAcceleration::fromRecord(record);

  EXPECT_DOUBLE_EQ(accel.pitch(), 7.0);
  EXPECT_DOUBLE_EQ(accel.roll(), 8.0);
  EXPECT_DOUBLE_EQ(accel.yaw(), 9.0);
}

// ============================================================================
// Database Query Tests
// ============================================================================

TEST_F(AngularAccelerationTransferTest, SelectNonexistent_ReturnsNullopt)
{
  auto& dao = db_->getDAO<msd_transfer::AngularAccelerationRecord>();
  auto result = dao.selectById(99999);
  EXPECT_FALSE(result.has_value());
}

TEST_F(AngularAccelerationTransferTest, InsertMultiple_SelectAll)
{
  msd_sim::AngularAcceleration r1{1.0, 2.0, 3.0};
  msd_sim::AngularAcceleration r2{4.0, 5.0, 6.0};
  msd_sim::AngularAcceleration r3{7.0, 8.0, 9.0};

  auto rec1 = r1.toRecord();
  auto rec2 = r2.toRecord();
  auto rec3 = r3.toRecord();

  auto& dao = db_->getDAO<msd_transfer::AngularAccelerationRecord>();
  dao.insert(rec1);
  dao.insert(rec2);
  dao.insert(rec3);

  auto all = dao.selectAll();
  EXPECT_EQ(all.size(), 3);
}
