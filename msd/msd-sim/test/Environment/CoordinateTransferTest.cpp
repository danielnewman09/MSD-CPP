#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-transfer/src/CoordinateRecord.hpp"

namespace fs = std::filesystem;

class CoordinateTransferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_coordinate_transfer.db";
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

TEST_F(CoordinateTransferTest, RoundTrip_AllFields)
{
  // Create domain object with known values
  msd_sim::Coordinate original{1.5, -2.75, 3.125};

  // Serialize to record
  auto record = original.toRecord();

  // Verify record fields
  EXPECT_DOUBLE_EQ(record.x, 1.5);
  EXPECT_DOUBLE_EQ(record.y, -2.75);
  EXPECT_DOUBLE_EQ(record.z, 3.125);

  // Insert into database
  auto& dao = db_->getDAO<msd_transfer::CoordinateRecord>();
  dao.insert(record);
  EXPECT_GT(record.id, 0);

  // Retrieve from database
  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  // Deserialize back to domain object
  auto restored = msd_sim::Coordinate::fromRecord(*retrieved);

  // Compare all components
  EXPECT_DOUBLE_EQ(original.x(), restored.x());
  EXPECT_DOUBLE_EQ(original.y(), restored.y());
  EXPECT_DOUBLE_EQ(original.z(), restored.z());
}

TEST_F(CoordinateTransferTest, RoundTrip_ZeroValues)
{
  msd_sim::Coordinate original{0.0, 0.0, 0.0};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::CoordinateRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::Coordinate::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.x(), restored.x());
  EXPECT_DOUBLE_EQ(original.y(), restored.y());
  EXPECT_DOUBLE_EQ(original.z(), restored.z());
}

TEST_F(CoordinateTransferTest, RoundTrip_NegativeValues)
{
  msd_sim::Coordinate original{-100.5, -200.75, -300.125};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::CoordinateRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::Coordinate::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.x(), restored.x());
  EXPECT_DOUBLE_EQ(original.y(), restored.y());
  EXPECT_DOUBLE_EQ(original.z(), restored.z());
}

TEST_F(CoordinateTransferTest, RoundTrip_LargeValues)
{
  msd_sim::Coordinate original{1.0e15, -2.5e12, 9.99e18};

  auto record = original.toRecord();
  auto& dao = db_->getDAO<msd_transfer::CoordinateRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());

  auto restored = msd_sim::Coordinate::fromRecord(*retrieved);

  EXPECT_DOUBLE_EQ(original.x(), restored.x());
  EXPECT_DOUBLE_EQ(original.y(), restored.y());
  EXPECT_DOUBLE_EQ(original.z(), restored.z());
}

// ============================================================================
// Serialization Tests
// ============================================================================

TEST_F(CoordinateTransferTest, ToRecord_MapsFieldsCorrectly)
{
  msd_sim::Coordinate coord{4.0, 5.0, 6.0};
  auto record = coord.toRecord();

  EXPECT_DOUBLE_EQ(record.x, 4.0);
  EXPECT_DOUBLE_EQ(record.y, 5.0);
  EXPECT_DOUBLE_EQ(record.z, 6.0);
}

TEST_F(CoordinateTransferTest, FromRecord_MapsFieldsCorrectly)
{
  msd_transfer::CoordinateRecord record;
  record.x = 7.0;
  record.y = 8.0;
  record.z = 9.0;

  auto coord = msd_sim::Coordinate::fromRecord(record);

  EXPECT_DOUBLE_EQ(coord.x(), 7.0);
  EXPECT_DOUBLE_EQ(coord.y(), 8.0);
  EXPECT_DOUBLE_EQ(coord.z(), 9.0);
}

// ============================================================================
// Database Query Tests
// ============================================================================

TEST_F(CoordinateTransferTest, SelectNonexistent_ReturnsNullopt)
{
  auto& dao = db_->getDAO<msd_transfer::CoordinateRecord>();
  auto result = dao.selectById(99999);
  EXPECT_FALSE(result.has_value());
}

TEST_F(CoordinateTransferTest, InsertMultiple_SelectAll)
{
  msd_sim::Coordinate c1{1.0, 2.0, 3.0};
  msd_sim::Coordinate c2{4.0, 5.0, 6.0};
  msd_sim::Coordinate c3{7.0, 8.0, 9.0};

  auto r1 = c1.toRecord();
  auto r2 = c2.toRecord();
  auto r3 = c3.toRecord();

  auto& dao = db_->getDAO<msd_transfer::CoordinateRecord>();
  dao.insert(r1);
  dao.insert(r2);
  dao.insert(r3);

  auto all = dao.selectAll();
  EXPECT_EQ(all.size(), 3);
}
