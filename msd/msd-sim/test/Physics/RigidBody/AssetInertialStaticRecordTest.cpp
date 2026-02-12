// Transfer tests for AssetInertial::toStaticRecord()

#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-transfer/src/AssetInertialStaticRecord.hpp"

namespace fs = std::filesystem;

namespace
{
std::vector<msd_sim::Coordinate> createCubePoints(double size)
{
  double h = size / 2.0;
  return {{-h, -h, -h},
          {h, -h, -h},
          {h, h, -h},
          {-h, h, -h},
          {-h, -h, h},
          {h, -h, h},
          {h, h, h},
          {-h, h, h}};
}
}  // namespace

class AssetInertialStaticRecordTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_asset_inertial_static_record.db";
    if (fs::exists(testDbPath_))
    {
      fs::remove(testDbPath_);
    }
    auto& logger = cpp_sqlite::Logger::getInstance();
    db_ = std::make_unique<cpp_sqlite::Database>(
      testDbPath_, true, logger.getLogger());

    auto cubePoints = createCubePoints(1.0);
    hull_ = std::make_unique<msd_sim::ConvexHull>(cubePoints);
  }

  void TearDown() override { db_.reset(); }

  std::string testDbPath_;
  std::unique_ptr<cpp_sqlite::Database> db_;
  std::unique_ptr<msd_sim::ConvexHull> hull_;
};

TEST_F(AssetInertialStaticRecordTest, FieldMapping_AllProperties)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  msd_sim::AssetInertial asset{1, 42, *hull_, 75.0, frame, 0.9, 0.6};

  auto record = asset.toStaticRecord();

  EXPECT_EQ(record.body_id, 42u);
  EXPECT_DOUBLE_EQ(record.mass, 75.0);
  EXPECT_DOUBLE_EQ(record.restitution, 0.9);
  EXPECT_DOUBLE_EQ(record.friction, 0.6);
}

TEST_F(AssetInertialStaticRecordTest, FieldMapping_DefaultRestitutionAndFriction)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  msd_sim::AssetInertial asset{1, 7, *hull_, 10.0, frame};

  auto record = asset.toStaticRecord();

  EXPECT_EQ(record.body_id, 7u);
  EXPECT_DOUBLE_EQ(record.mass, 10.0);
  EXPECT_DOUBLE_EQ(record.restitution, 0.5);   // Default
  EXPECT_DOUBLE_EQ(record.friction, 0.0);       // Default
}

TEST_F(AssetInertialStaticRecordTest, DatabaseRoundTrip)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  msd_sim::AssetInertial asset{2, 33, *hull_, 25.0, frame, 0.7, 0.4};

  auto record = asset.toStaticRecord();
  auto& dao = db_->getDAO<msd_transfer::AssetInertialStaticRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->body_id, 33u);
  EXPECT_DOUBLE_EQ(retrieved->mass, 25.0);
  EXPECT_DOUBLE_EQ(retrieved->restitution, 0.7);
  EXPECT_DOUBLE_EQ(retrieved->friction, 0.4);
}

TEST_F(AssetInertialStaticRecordTest, NaNDefaults_UnsetRecord)
{
  // A default-constructed record should have NaN for physics fields
  msd_transfer::AssetInertialStaticRecord record;
  EXPECT_TRUE(std::isnan(record.mass));
  EXPECT_TRUE(std::isnan(record.restitution));
  EXPECT_TRUE(std::isnan(record.friction));
  EXPECT_EQ(record.body_id, 0u);
}
