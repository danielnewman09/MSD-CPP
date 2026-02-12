// Transfer tests for ReferenceFrame toRecord/fromRecord

#include <gtest/gtest.h>
#include <cmath>

#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-transfer/src/AssetPhysicalDynamicRecord.hpp"

TEST(ReferenceFrameTransferTest, RoundTrip_IdentityFrame)
{
  msd_sim::ReferenceFrame frame;

  auto record = frame.toRecord();

  // Position should be origin
  EXPECT_DOUBLE_EQ(record.position.x, 0.0);
  EXPECT_DOUBLE_EQ(record.position.y, 0.0);
  EXPECT_DOUBLE_EQ(record.position.z, 0.0);

  // Orientation should be identity quaternion
  EXPECT_DOUBLE_EQ(record.orientation.w, 1.0);
  EXPECT_NEAR(record.orientation.x, 0.0, 1e-10);
  EXPECT_NEAR(record.orientation.y, 0.0, 1e-10);
  EXPECT_NEAR(record.orientation.z, 0.0, 1e-10);

  // Round-trip
  auto restored = msd_sim::ReferenceFrame::fromRecord(record);
  EXPECT_DOUBLE_EQ(restored.getOrigin().x(), 0.0);
  EXPECT_DOUBLE_EQ(restored.getOrigin().y(), 0.0);
  EXPECT_DOUBLE_EQ(restored.getOrigin().z(), 0.0);

  auto quat = restored.getQuaternion();
  EXPECT_NEAR(quat.w(), 1.0, 1e-10);
  EXPECT_NEAR(quat.x(), 0.0, 1e-10);
  EXPECT_NEAR(quat.y(), 0.0, 1e-10);
  EXPECT_NEAR(quat.z(), 0.0, 1e-10);
}

TEST(ReferenceFrameTransferTest, RoundTrip_TranslationOnly)
{
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{10.0, 20.0, 30.0}};

  auto record = frame.toRecord();
  auto restored = msd_sim::ReferenceFrame::fromRecord(record);

  EXPECT_DOUBLE_EQ(restored.getOrigin().x(), 10.0);
  EXPECT_DOUBLE_EQ(restored.getOrigin().y(), 20.0);
  EXPECT_DOUBLE_EQ(restored.getOrigin().z(), 30.0);
}

TEST(ReferenceFrameTransferTest, RoundTrip_RotatedFrame)
{
  // 90-degree rotation about Z-axis
  Eigen::Quaterniond quat{
    Eigen::AngleAxisd{M_PI / 2.0, msd_sim::Vector3D::UnitZ()}};
  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{1.0, 2.0, 3.0}, quat};

  auto record = frame.toRecord();
  auto restored = msd_sim::ReferenceFrame::fromRecord(record);

  // Position preserved
  EXPECT_DOUBLE_EQ(restored.getOrigin().x(), 1.0);
  EXPECT_DOUBLE_EQ(restored.getOrigin().y(), 2.0);
  EXPECT_DOUBLE_EQ(restored.getOrigin().z(), 3.0);

  // Orientation preserved (quaternion comparison)
  auto restoredQuat = restored.getQuaternion();
  EXPECT_NEAR(restoredQuat.w(), quat.w(), 1e-10);
  EXPECT_NEAR(restoredQuat.x(), quat.x(), 1e-10);
  EXPECT_NEAR(restoredQuat.y(), quat.y(), 1e-10);
  EXPECT_NEAR(restoredQuat.z(), quat.z(), 1e-10);
}

TEST(ReferenceFrameTransferTest, RoundTrip_ArbitraryOrientation)
{
  // Compound rotation: 30° about X, then 45° about Y, then 60° about Z
  Eigen::Quaterniond quat =
    Eigen::AngleAxisd{M_PI / 3.0, msd_sim::Vector3D::UnitZ()} *
    Eigen::AngleAxisd{M_PI / 4.0, msd_sim::Vector3D::UnitY()} *
    Eigen::AngleAxisd{M_PI / 6.0, msd_sim::Vector3D::UnitX()};

  msd_sim::ReferenceFrame frame{msd_sim::Coordinate{-5.0, 10.5, 0.25}, quat};

  auto record = frame.toRecord();
  auto restored = msd_sim::ReferenceFrame::fromRecord(record);

  // Verify rotation matrices match
  auto originalRot = frame.getRotation();
  auto restoredRot = restored.getRotation();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      EXPECT_NEAR(restoredRot(i, j), originalRot(i, j), 1e-10)
        << "Rotation matrix mismatch at (" << i << ", " << j << ")";
    }
  }
}
