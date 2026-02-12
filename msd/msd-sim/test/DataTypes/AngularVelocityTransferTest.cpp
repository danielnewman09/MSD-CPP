#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/DataTypes/AngularVelocity.hpp"
#include "msd-transfer/src/AngularVelocityRecord.hpp"

namespace fs = std::filesystem;

class AngularVelocityTransferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_angular_velocity_transfer.db";
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

TEST_F(AngularVelocityTransferTest, RoundTrip_AllFields)
{
  msd_sim::AngularVelocity original{0.5, -1.25, 3.75};

  auto record = original.toRecord();

  EXPECT_DOUBLE_EQ(record.pitch, 0.5);
  EXPECT_DOUBLE_EQ(record.roll, -1.25);
  EXPECT_DOUBLE_EQ(record.yaw, 3.75);

  auto& dao = db_->getDAO<msd_transfer::AngularVelocityRecord>();
  dao.insert(record);
  EXPECT_GT(record.id, 0);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularVelocity::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularVelocityTransferTest, RoundTrip_ZeroValues)
{
  msd_sim::AngularVelocity original{0.0, 0.0, 0.0};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularVelocityRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularVelocity::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularVelocityTransferTest, RoundTrip_NegativeValues)
{
  msd_sim::AngularVelocity original{-10.5, -20.75, -30.125};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularVelocityRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularVelocity::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularVelocityTransferTest, RoundTrip_LargeValues)
{
  msd_sim::AngularVelocity original{1.0e15, -2.5e12, 9.99e18};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularVelocityRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularVelocity::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

// ============================================================================
// Serialization Tests
// ============================================================================

TEST_F(AngularVelocityTransferTest, ToRecord_MapsFieldsCorrectly)
{
  msd_sim::AngularVelocity vel{4.0, 5.0, 6.0};
  auto record = vel.toRecord();

  EXPECT_DOUBLE_EQ(record.pitch, 4.0);
  EXPECT_DOUBLE_EQ(record.roll, 5.0);
  EXPECT_DOUBLE_EQ(record.yaw, 6.0);
}

TEST_F(AngularVelocityTransferTest, FromRecord_MapsFieldsCorrectly)
{
  msd_transfer::AngularVelocityRecord record;
  record.pitch = 7.0;
  record.roll = 8.0;
  record.yaw = 9.0;

  auto vel = msd_sim::AngularVelocity::fromRecord(record);

  EXPECT_DOUBLE_EQ(vel.pitch(), 7.0);
  EXPECT_DOUBLE_EQ(vel.roll(), 8.0);
  EXPECT_DOUBLE_EQ(vel.yaw(), 9.0);
}

// ============================================================================
// Database Query Tests
// ============================================================================

TEST_F(AngularVelocityTransferTest, SelectNonexistent_ReturnsNullopt)
{
  auto& dao = db_->getDAO<msd_transfer::AngularVelocityRecord>();
  auto result = dao.selectById(99999);
  EXPECT_FALSE(result.has_value());
}

TEST_F(AngularVelocityTransferTest, InsertMultiple_SelectAll)
{
  msd_sim::AngularVelocity r1{1.0, 2.0, 3.0};
  msd_sim::AngularVelocity r2{4.0, 5.0, 6.0};
  msd_sim::AngularVelocity r3{7.0, 8.0, 9.0};

  auto rec1 = r1.toRecord();
  auto rec2 = r2.toRecord();
  auto rec3 = r3.toRecord();

  auto& dao = db_->getDAO<msd_transfer::AngularVelocityRecord>();
  dao.insert(rec1);
  dao.insert(rec2);
  dao.insert(rec3);

  auto all = dao.selectAll();
  EXPECT_EQ(all.size(), 3);
}
