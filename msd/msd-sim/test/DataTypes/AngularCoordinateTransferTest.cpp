#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/DataTypes/AngularCoordinate.hpp"
#include "msd-transfer/src/AngularCoordinateRecord.hpp"

namespace fs = std::filesystem;

class AngularCoordinateTransferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_angular_coordinate_transfer.db";
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

TEST_F(AngularCoordinateTransferTest, RoundTrip_AllFields)
{
  // Create domain object with known values
  msd_sim::AngularCoordinate original{0.5, -1.25, 3.0};

  // Serialize to record
  auto record = original.toRecord();

  // Verify record fields
  EXPECT_DOUBLE_EQ(record.pitch, 0.5);
  EXPECT_DOUBLE_EQ(record.roll, -1.25);
  EXPECT_DOUBLE_EQ(record.yaw, 3.0);

  // Insert into database
  auto& dao = db_->getDAO<msd_transfer::AngularCoordinateRecord>();
  dao.insert(record);
  EXPECT_GT(record.id, 0);

  // Retrieve from database
  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  // Deserialize back to domain object
  auto restored = msd_sim::AngularCoordinate::fromRecord(*retrieved);

  // Compare all components
  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularCoordinateTransferTest, RoundTrip_ZeroValues)
{
  msd_sim::AngularCoordinate original{0.0, 0.0, 0.0};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularCoordinateRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularCoordinate::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularCoordinateTransferTest, RoundTrip_NegativeValues)
{
  msd_sim::AngularCoordinate original{-1.0, -2.0, -3.0};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularCoordinateRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularCoordinate::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

TEST_F(AngularCoordinateTransferTest, RoundTrip_PiValues)
{
  // Test with values at the normalization boundary
  msd_sim::AngularCoordinate original{M_PI, -M_PI / 2, M_PI / 4};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::AngularCoordinateRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::AngularCoordinate::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.pitch(), restored.pitch());
  EXPECT_DOUBLE_EQ(original.roll(), restored.roll());
  EXPECT_DOUBLE_EQ(original.yaw(), restored.yaw());
}

// ============================================================================
// Serialization Tests
// ============================================================================

TEST_F(AngularCoordinateTransferTest, ToRecord_MapsFieldsCorrectly)
{
  msd_sim::AngularCoordinate coord{0.1, 0.2, 0.3};
  auto record = coord.toRecord();

  EXPECT_DOUBLE_EQ(record.pitch, 0.1);
  EXPECT_DOUBLE_EQ(record.roll, 0.2);
  EXPECT_DOUBLE_EQ(record.yaw, 0.3);
}

TEST_F(AngularCoordinateTransferTest, FromRecord_MapsFieldsCorrectly)
{
  msd_transfer::AngularCoordinateRecord record;
  record.pitch = 0.7;
  record.roll = 0.8;
  record.yaw = 0.9;

  auto coord = msd_sim::AngularCoordinate::fromRecord(record);

  EXPECT_DOUBLE_EQ(coord.pitch(), 0.7);
  EXPECT_DOUBLE_EQ(coord.roll(), 0.8);
  EXPECT_DOUBLE_EQ(coord.yaw(), 0.9);
}

// ============================================================================
// Database Query Tests
// ============================================================================

TEST_F(AngularCoordinateTransferTest, SelectNonexistent_ReturnsNullopt)
{
  auto& dao = db_->getDAO<msd_transfer::AngularCoordinateRecord>();
  auto result = dao.selectById(99999);
  EXPECT_FALSE(result.has_value());
}

TEST_F(AngularCoordinateTransferTest, InsertMultiple_SelectAll)
{
  msd_sim::AngularCoordinate c1{0.1, 0.2, 0.3};
  msd_sim::AngularCoordinate c2{0.4, 0.5, 0.6};
  msd_sim::AngularCoordinate c3{0.7, 0.8, 0.9};

  auto rec1 = c1.toRecord();
  auto rec2 = c2.toRecord();
  auto rec3 = c3.toRecord();

  auto& dao = db_->getDAO<msd_transfer::AngularCoordinateRecord>();
  dao.insert(rec1);
  dao.insert(rec2);
  dao.insert(rec3);

  auto all = dao.selectAll();
  EXPECT_EQ(all.size(), 3);
}
