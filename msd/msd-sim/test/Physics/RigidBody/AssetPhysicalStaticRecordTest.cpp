// Transfer tests for AssetPhysical::toStaticRecord()

#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>

#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-transfer/src/AssetPhysicalStaticRecord.hpp"

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

class AssetPhysicalStaticRecordTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_asset_physical_static_record.db";
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

TEST_F(AssetPhysicalStaticRecordTest, InertialAsset_FieldMapping)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{1.0, 2.0, 3.0}};
  msd_sim::AssetInertial asset{5, 42, *hull_, 50.0, frame, 0.8};

  // Access base class method (hidden by AssetInertial::toStaticRecord())
  const msd_sim::AssetPhysical& base = asset;
  auto record = base.toStaticRecord(false);

  EXPECT_EQ(record.body_id, 42u);
  EXPECT_EQ(record.asset_id, 5u);
  EXPECT_EQ(record.is_environment, 0u);
}

TEST_F(AssetPhysicalStaticRecordTest, EnvironmentAsset_IsEnvironmentFlag)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  msd_sim::AssetEnvironment asset{3, 99, *hull_, frame};

  auto record = asset.toStaticRecord(true);

  EXPECT_EQ(record.body_id, 99u);
  EXPECT_EQ(record.asset_id, 3u);
  EXPECT_EQ(record.is_environment, 1u);
}

TEST_F(AssetPhysicalStaticRecordTest, DatabaseRoundTrip)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{0.0, 0.0, 0.0}};
  msd_sim::AssetInertial asset{7, 15, *hull_, 10.0, frame};

  const msd_sim::AssetPhysical& base = asset;
  auto record = base.toStaticRecord(false);
  auto& dao = db_->getDAO<msd_transfer::AssetPhysicalStaticRecord>();
  dao.insert(record);

  auto retrieved = dao.selectById(record.id);
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->body_id, 15u);
  EXPECT_EQ(retrieved->asset_id, 7u);
  EXPECT_EQ(retrieved->is_environment, 0u);
}
