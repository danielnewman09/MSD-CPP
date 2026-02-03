// Ticket: 0001_link-gui-sim-object
// Tests for GPUInstanceManager template class

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <msd-assets/src/GeometryFactory.hpp>
#include <msd-gui/src/GPUInstanceManager.hpp>
#include <msd-sim/src/DataTypes/AngularCoordinate.hpp>
#include <msd-sim/src/DataTypes/Coordinate.hpp>
#include <msd-sim/src/Environment/ReferenceFrame.hpp>
#include <msd-sim/src/Physics/RigidBody/ConvexHull.hpp>

using namespace msd_gui;
using namespace msd_sim;

// ============================================================================
// Test Fixtures and Helpers
// ============================================================================

namespace
{

// Create a simple cube as a point cloud for ConvexHull
std::vector<Coordinate> createCubePoints(double size)
{
  double half = size / 2.0;
  return {Coordinate(-half, -half, -half),
          Coordinate(half, -half, -half),
          Coordinate(half, half, -half),
          Coordinate(-half, half, -half),
          Coordinate(-half, -half, half),
          Coordinate(half, -half, half),
          Coordinate(half, half, half),
          Coordinate(-half, half, half)};
}

// Helper to create an AssetInertial for testing
// Note: AssetInertial requires a ConvexHull reference, so we need to manage
// lifetime
class TestAssetFactory
{
public:
  AssetInertial createAsset(uint32_t assetId,
                            uint32_t instanceId,
                            const Coordinate& position,
                            double mass = 1.0)
  {
    ReferenceFrame frame{position};
    hulls_.emplace_back(createCubePoints(1.0));
    return AssetInertial{assetId, instanceId, hulls_.back(), mass, frame};
  }

  AssetInertial createAssetWithRotation(uint32_t assetId,
                                        uint32_t instanceId,
                                        const Coordinate& position,
                                        const AngularCoordinate& orientation,
                                        double mass = 1.0)
  {
    ReferenceFrame frame{position, orientation};
    hulls_.emplace_back(createCubePoints(1.0));
    return AssetInertial{assetId, instanceId, hulls_.back(), mass, frame};
  }

private:
  std::vector<ConvexHull> hulls_;
};

}  // anonymous namespace

// ============================================================================
// PositionOnlyShaderPolicy InstanceManager Tests
// ============================================================================

class PositionOnlyInstanceManagerTest : public ::testing::Test
{
protected:
  InstanceManager<PositionOnlyShaderPolicy> manager_;
  TestAssetFactory factory_;
};

TEST_F(PositionOnlyInstanceManagerTest, InitialStateIsEmpty)
{
  EXPECT_TRUE(manager_.getInstances().empty());
}

TEST_F(PositionOnlyInstanceManagerTest, BuildInstanceDataAddsInstance)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{1.0, 2.0, 3.0});

  size_t index = manager_.buildInstanceData(asset, 0, 1.0f, 0.5f, 0.25f);

  EXPECT_EQ(index, 0);
  EXPECT_EQ(manager_.getInstances().size(), 1);
}

TEST_F(PositionOnlyInstanceManagerTest, BuildInstanceDataExtractsPosition)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{5.0, 10.0, 15.0});

  manager_.buildInstanceData(asset, 0, 1.0f, 0.0f, 0.0f);

  const auto& instances = manager_.getInstances();
  ASSERT_EQ(instances.size(), 1);

  const auto& data = instances[0];
  EXPECT_FLOAT_EQ(data.position[0], 5.0f);
  EXPECT_FLOAT_EQ(data.position[1], 10.0f);
  EXPECT_FLOAT_EQ(data.position[2], 15.0f);
}

TEST_F(PositionOnlyInstanceManagerTest, BuildInstanceDataExtractsColor)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{0.0, 0.0, 0.0});

  manager_.buildInstanceData(asset, 0, 0.8f, 0.6f, 0.4f);

  const auto& instances = manager_.getInstances();
  ASSERT_EQ(instances.size(), 1);

  const auto& data = instances[0];
  EXPECT_FLOAT_EQ(data.color[0], 0.8f);
  EXPECT_FLOAT_EQ(data.color[1], 0.6f);
  EXPECT_FLOAT_EQ(data.color[2], 0.4f);
}

TEST_F(PositionOnlyInstanceManagerTest, BuildInstanceDataMultipleObjects)
{
  auto asset1 = factory_.createAsset(1, 100, Coordinate{1.0, 0.0, 0.0});
  auto asset2 = factory_.createAsset(2, 101, Coordinate{2.0, 0.0, 0.0});
  auto asset3 = factory_.createAsset(3, 102, Coordinate{3.0, 0.0, 0.0});

  size_t idx1 = manager_.buildInstanceData(asset1, 0, 1.0f, 0.0f, 0.0f);
  size_t idx2 = manager_.buildInstanceData(asset2, 0, 0.0f, 1.0f, 0.0f);
  size_t idx3 = manager_.buildInstanceData(asset3, 0, 0.0f, 0.0f, 1.0f);

  EXPECT_EQ(idx1, 0);
  EXPECT_EQ(idx2, 1);
  EXPECT_EQ(idx3, 2);
  EXPECT_EQ(manager_.getInstances().size(), 3);
}

TEST_F(PositionOnlyInstanceManagerTest, ClearObjectsRemovesAllInstances)
{
  auto asset1 = factory_.createAsset(1, 100, Coordinate{1.0, 0.0, 0.0});
  auto asset2 = factory_.createAsset(2, 101, Coordinate{2.0, 0.0, 0.0});

  manager_.buildInstanceData(asset1, 0, 1.0f, 0.0f, 0.0f);
  manager_.buildInstanceData(asset2, 0, 0.0f, 1.0f, 0.0f);

  EXPECT_EQ(manager_.getInstances().size(), 2);

  manager_.clearObjects();

  EXPECT_TRUE(manager_.getInstances().empty());
}

TEST_F(PositionOnlyInstanceManagerTest, RemoveObjectByInstanceId)
{
  auto asset1 = factory_.createAsset(1, 100, Coordinate{1.0, 0.0, 0.0});
  auto asset2 = factory_.createAsset(2, 101, Coordinate{2.0, 0.0, 0.0});
  auto asset3 = factory_.createAsset(3, 102, Coordinate{3.0, 0.0, 0.0});

  manager_.buildInstanceData(asset1, 0, 1.0f, 0.0f, 0.0f);
  manager_.buildInstanceData(asset2, 0, 0.0f, 1.0f, 0.0f);
  manager_.buildInstanceData(asset3, 0, 0.0f, 0.0f, 1.0f);

  EXPECT_EQ(manager_.getInstances().size(), 3);

  manager_.removeObject(101);  // Remove middle object

  EXPECT_EQ(manager_.getInstances().size(), 2);
}

TEST_F(PositionOnlyInstanceManagerTest, RemoveNonExistentObjectDoesNotCrash)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{1.0, 0.0, 0.0});
  manager_.buildInstanceData(asset, 0, 1.0f, 0.0f, 0.0f);

  // Removing non-existent object should not throw or crash
  EXPECT_NO_THROW(manager_.removeObject(999));
  EXPECT_EQ(manager_.getInstances().size(), 1);
}

// ============================================================================
// FullTransformShaderPolicy InstanceManager Tests
// ============================================================================

class FullTransformInstanceManagerTest : public ::testing::Test
{
protected:
  InstanceManager<FullTransformShaderPolicy> manager_;
  TestAssetFactory factory_;
};

TEST_F(FullTransformInstanceManagerTest, InitialStateIsEmpty)
{
  EXPECT_TRUE(manager_.getInstances().empty());
}

TEST_F(FullTransformInstanceManagerTest, BuildInstanceDataAddsInstance)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{1.0, 2.0, 3.0});

  size_t index = manager_.buildInstanceData(asset, 5, 1.0f, 0.5f, 0.25f);

  EXPECT_EQ(index, 0);
  EXPECT_EQ(manager_.getInstances().size(), 1);
}

TEST_F(FullTransformInstanceManagerTest, BuildInstanceDataSetsGeometryIndex)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{0.0, 0.0, 0.0});

  manager_.buildInstanceData(asset, 42, 1.0f, 0.0f, 0.0f);

  const auto& instances = manager_.getInstances();
  ASSERT_EQ(instances.size(), 1);
  EXPECT_EQ(instances[0].geometryIndex, 42);
}

TEST_F(FullTransformInstanceManagerTest, BuildInstanceDataExtractsColor)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{0.0, 0.0, 0.0});

  manager_.buildInstanceData(asset, 0, 0.9f, 0.7f, 0.3f);

  const auto& instances = manager_.getInstances();
  ASSERT_EQ(instances.size(), 1);

  const auto& data = instances[0];
  EXPECT_FLOAT_EQ(data.color[0], 0.9f);
  EXPECT_FLOAT_EQ(data.color[1], 0.7f);
  EXPECT_FLOAT_EQ(data.color[2], 0.3f);
}

TEST_F(FullTransformInstanceManagerTest,
       BuildInstanceDataCreatesIdentityMatrixForNoRotation)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{0.0, 0.0, 0.0});

  manager_.buildInstanceData(asset, 0, 1.0f, 1.0f, 1.0f);

  const auto& instances = manager_.getInstances();
  ASSERT_EQ(instances.size(), 1);

  const auto& matrix = instances[0].modelMatrix;

  // Column-major 4x4 identity-like matrix (no rotation, no translation)
  // Column 0 (X axis)
  EXPECT_FLOAT_EQ(matrix[0], 1.0f);
  EXPECT_FLOAT_EQ(matrix[1], 0.0f);
  EXPECT_FLOAT_EQ(matrix[2], 0.0f);
  EXPECT_FLOAT_EQ(matrix[3], 0.0f);

  // Column 1 (Y axis)
  EXPECT_FLOAT_EQ(matrix[4], 0.0f);
  EXPECT_FLOAT_EQ(matrix[5], 1.0f);
  EXPECT_FLOAT_EQ(matrix[6], 0.0f);
  EXPECT_FLOAT_EQ(matrix[7], 0.0f);

  // Column 2 (Z axis)
  EXPECT_FLOAT_EQ(matrix[8], 0.0f);
  EXPECT_FLOAT_EQ(matrix[9], 0.0f);
  EXPECT_FLOAT_EQ(matrix[10], 1.0f);
  EXPECT_FLOAT_EQ(matrix[11], 0.0f);

  // Column 3 (Translation)
  EXPECT_FLOAT_EQ(matrix[12], 0.0f);
  EXPECT_FLOAT_EQ(matrix[13], 0.0f);
  EXPECT_FLOAT_EQ(matrix[14], 0.0f);
  EXPECT_FLOAT_EQ(matrix[15], 1.0f);
}

TEST_F(FullTransformInstanceManagerTest, BuildInstanceDataExtractsTranslation)
{
  auto asset = factory_.createAsset(1, 100, Coordinate{10.0, 20.0, 30.0});

  manager_.buildInstanceData(asset, 0, 1.0f, 1.0f, 1.0f);

  const auto& instances = manager_.getInstances();
  ASSERT_EQ(instances.size(), 1);

  const auto& matrix = instances[0].modelMatrix;

  // Translation is in column 3 (indices 12, 13, 14)
  EXPECT_FLOAT_EQ(matrix[12], 10.0f);
  EXPECT_FLOAT_EQ(matrix[13], 20.0f);
  EXPECT_FLOAT_EQ(matrix[14], 30.0f);
}

TEST_F(FullTransformInstanceManagerTest, BuildInstanceDataMultipleObjects)
{
  auto asset1 = factory_.createAsset(1, 100, Coordinate{1.0, 0.0, 0.0});
  auto asset2 = factory_.createAsset(2, 101, Coordinate{2.0, 0.0, 0.0});

  size_t idx1 = manager_.buildInstanceData(asset1, 0, 1.0f, 0.0f, 0.0f);
  size_t idx2 = manager_.buildInstanceData(asset2, 1, 0.0f, 1.0f, 0.0f);

  EXPECT_EQ(idx1, 0);
  EXPECT_EQ(idx2, 1);
  EXPECT_EQ(manager_.getInstances().size(), 2);

  // Verify each has correct geometry index
  EXPECT_EQ(manager_.getInstances()[0].geometryIndex, 0);
  EXPECT_EQ(manager_.getInstances()[1].geometryIndex, 1);
}

TEST_F(FullTransformInstanceManagerTest, ClearObjectsRemovesAllInstances)
{
  auto asset1 = factory_.createAsset(1, 100, Coordinate{1.0, 0.0, 0.0});
  auto asset2 = factory_.createAsset(2, 101, Coordinate{2.0, 0.0, 0.0});

  manager_.buildInstanceData(asset1, 0, 1.0f, 0.0f, 0.0f);
  manager_.buildInstanceData(asset2, 1, 0.0f, 1.0f, 0.0f);

  EXPECT_EQ(manager_.getInstances().size(), 2);

  manager_.clearObjects();

  EXPECT_TRUE(manager_.getInstances().empty());
}

TEST_F(FullTransformInstanceManagerTest, RemoveObjectByInstanceId)
{
  auto asset1 = factory_.createAsset(1, 100, Coordinate{1.0, 0.0, 0.0});
  auto asset2 = factory_.createAsset(2, 101, Coordinate{2.0, 0.0, 0.0});

  manager_.buildInstanceData(asset1, 0, 1.0f, 0.0f, 0.0f);
  manager_.buildInstanceData(asset2, 1, 0.0f, 1.0f, 0.0f);

  EXPECT_EQ(manager_.getInstances().size(), 2);

  manager_.removeObject(100);

  EXPECT_EQ(manager_.getInstances().size(), 1);
}

// ============================================================================
// GeometryInfo Struct Tests
// ============================================================================

TEST(GeometryInfoTest, DefaultInitialization)
{
  GeometryInfo info;
  EXPECT_EQ(info.baseVertex, 0);
  EXPECT_EQ(info.vertexCount, 0);
}

TEST(GeometryInfoTest, CustomInitialization)
{
  GeometryInfo info{100, 500};
  EXPECT_EQ(info.baseVertex, 100);
  EXPECT_EQ(info.vertexCount, 500);
}

// ============================================================================
// Instance Data Structure Alignment Tests (inherited from shader_policy_test)
// ============================================================================

TEST(InstanceManagerDataAlignment, PositionOnlyInstanceDataSizeIs32Bytes)
{
  // Critical for GPU buffer layout - must be 32 bytes
  EXPECT_EQ(sizeof(PositionOnlyInstanceData), 32);
}

TEST(InstanceManagerDataAlignment, FullTransformInstanceDataSizeIs96Bytes)
{
  // Critical for GPU buffer layout - must be 96 bytes
  EXPECT_EQ(sizeof(FullTransformInstanceData), 96);
}

TEST(InstanceManagerDataAlignment, PositionOnlyDataIs16ByteAligned)
{
  EXPECT_EQ(sizeof(PositionOnlyInstanceData) % 16, 0);
}

TEST(InstanceManagerDataAlignment, FullTransformDataIs16ByteAligned)
{
  EXPECT_EQ(sizeof(FullTransformInstanceData) % 16, 0);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(InstanceManagerEdgeCases, PositionOnlyClearEmptyManagerDoesNotCrash)
{
  InstanceManager<PositionOnlyShaderPolicy> manager;
  EXPECT_NO_THROW(manager.clearObjects());
  EXPECT_TRUE(manager.getInstances().empty());
}

TEST(InstanceManagerEdgeCases, FullTransformClearEmptyManagerDoesNotCrash)
{
  InstanceManager<FullTransformShaderPolicy> manager;
  EXPECT_NO_THROW(manager.clearObjects());
  EXPECT_TRUE(manager.getInstances().empty());
}

TEST(InstanceManagerEdgeCases, RemoveFromEmptyManagerDoesNotCrash)
{
  InstanceManager<PositionOnlyShaderPolicy> manager;
  EXPECT_NO_THROW(manager.removeObject(0));
  EXPECT_NO_THROW(manager.removeObject(999));
}

// ============================================================================
// Large Dataset Tests
// ============================================================================

TEST(InstanceManagerPerformance, PositionOnlyHandlesManyInstances)
{
  InstanceManager<PositionOnlyShaderPolicy> manager;
  TestAssetFactory factory;

  const size_t count = 1000;
  for (size_t i = 0; i < count; ++i)
  {
    auto asset =
      factory.createAsset(static_cast<uint32_t>(i),
                          static_cast<uint32_t>(i + 1000),
                          Coordinate{static_cast<double>(i), 0.0, 0.0});
    manager.buildInstanceData(
      asset, 0, static_cast<float>(i) / count, 0.5f, 0.5f);
  }

  EXPECT_EQ(manager.getInstances().size(), count);
}

TEST(InstanceManagerPerformance, FullTransformHandlesManyInstances)
{
  InstanceManager<FullTransformShaderPolicy> manager;
  TestAssetFactory factory;

  const size_t count = 1000;
  for (size_t i = 0; i < count; ++i)
  {
    auto asset =
      factory.createAsset(static_cast<uint32_t>(i),
                          static_cast<uint32_t>(i + 1000),
                          Coordinate{static_cast<double>(i), 0.0, 0.0});
    manager.buildInstanceData(asset,
                              static_cast<uint32_t>(i % 10),
                              static_cast<float>(i) / count,
                              0.5f,
                              0.5f);
  }

  EXPECT_EQ(manager.getInstances().size(), count);
}
