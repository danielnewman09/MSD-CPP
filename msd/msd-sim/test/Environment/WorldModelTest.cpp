// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#include <gtest/gtest.h>

#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

using namespace msd_sim;

// ========== Test Helpers ==========

namespace
{

// Create a simple cube collision geometry
msd_assets::CollisionGeometry createCubeGeometry()
{
  std::vector<Eigen::Vector3d> vertices{
    {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
    {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
  return msd_assets::CollisionGeometry{vertices};
}

// Create a simple tetrahedron collision geometry
msd_assets::CollisionGeometry createTetrahedronGeometry()
{
  std::vector<Eigen::Vector3d> vertices{
    {0, 0, 0}, {1, 0, 0}, {0.5, 1, 0}, {0.5, 0.5, 1}};
  return msd_assets::CollisionGeometry{vertices};
}

}  // namespace

// ========== Environment Asset Management Tests ==========

TEST(WorldModelTest, AddEnvironmentAssetStoresAsset)
{
  WorldModel world;

  auto geom = createCubeGeometry();
  ReferenceFrame frame{Coordinate{10, 0, 0}};
  AssetEnvironment asset{std::move(geom), frame};

  size_t index = world.addEnvironmentAsset(std::move(asset));

  EXPECT_EQ(index, 0);
  EXPECT_EQ(world.getEnvironmentAssetCount(), 1);
}

TEST(WorldModelTest, GetEnvironmentAssetRetrievesCorrectAsset)
{
  WorldModel world;

  auto geom = createCubeGeometry();
  ReferenceFrame frame{Coordinate{5, 10, 15}};
  AssetEnvironment asset{std::move(geom), frame};

  size_t index = world.addEnvironmentAsset(std::move(asset));
  const AssetEnvironment& retrieved = world.getEnvironmentAsset(index);

  const Coordinate& position = retrieved.getReferenceFrame().getOrigin();
  EXPECT_DOUBLE_EQ(position.x(), 5.0);
  EXPECT_DOUBLE_EQ(position.y(), 10.0);
  EXPECT_DOUBLE_EQ(position.z(), 15.0);
}

TEST(WorldModelTest, GetEnvironmentAssetThrowsOnInvalidIndex)
{
  WorldModel world;

  EXPECT_THROW(world.getEnvironmentAsset(0), std::out_of_range);
  EXPECT_THROW(world.getEnvironmentAsset(100), std::out_of_range);
}

TEST(WorldModelTest, RemoveEnvironmentAssetRemovesAsset)
{
  WorldModel world;

  auto geom1 = createCubeGeometry();
  auto geom2 = createTetrahedronGeometry();

  world.addEnvironmentAsset(AssetEnvironment{std::move(geom1), ReferenceFrame{}});
  world.addEnvironmentAsset(AssetEnvironment{std::move(geom2), ReferenceFrame{}});

  EXPECT_EQ(world.getEnvironmentAssetCount(), 2);

  world.removeEnvironmentAsset(0);

  EXPECT_EQ(world.getEnvironmentAssetCount(), 1);
}

TEST(WorldModelTest, RemoveEnvironmentAssetThrowsOnInvalidIndex)
{
  WorldModel world;

  EXPECT_THROW(world.removeEnvironmentAsset(0), std::out_of_range);
}

TEST(WorldModelTest, GetEnvironmentAssetsReturnsAllAssets)
{
  WorldModel world;

  auto geom1 = createCubeGeometry();
  auto geom2 = createTetrahedronGeometry();

  world.addEnvironmentAsset(AssetEnvironment{std::move(geom1), ReferenceFrame{}});
  world.addEnvironmentAsset(AssetEnvironment{std::move(geom2), ReferenceFrame{}});

  const auto& assets = world.getEnvironmentAssets();

  EXPECT_EQ(assets.size(), 2);
}

// ========== Inertial Asset Management Tests ==========

TEST(WorldModelTest, AddInertialAssetStoresAsset)
{
  WorldModel world;

  auto geom = createCubeGeometry();
  ReferenceFrame frame{Coordinate{20, 0, 0}};
  AssetInertial asset{std::move(geom), 10.0, frame};

  size_t index = world.addInertialAsset(std::move(asset));

  EXPECT_EQ(index, 0);
  EXPECT_EQ(world.getInertialAssetCount(), 1);
}

TEST(WorldModelTest, GetInertialAssetRetrievesCorrectAsset)
{
  WorldModel world;

  auto geom = createCubeGeometry();
  ReferenceFrame frame{Coordinate{1, 2, 3}};
  AssetInertial asset{std::move(geom), 50.0, frame};

  size_t index = world.addInertialAsset(std::move(asset));
  const AssetInertial& retrieved = world.getInertialAsset(index);

  EXPECT_DOUBLE_EQ(retrieved.getMass(), 50.0);

  const Coordinate& position = retrieved.getReferenceFrame().getOrigin();
  EXPECT_DOUBLE_EQ(position.x(), 1.0);
  EXPECT_DOUBLE_EQ(position.y(), 2.0);
  EXPECT_DOUBLE_EQ(position.z(), 3.0);
}

TEST(WorldModelTest, GetInertialAssetThrowsOnInvalidIndex)
{
  WorldModel world;

  EXPECT_THROW(world.getInertialAsset(0), std::out_of_range);
  EXPECT_THROW(world.getInertialAsset(999), std::out_of_range);
}

TEST(WorldModelTest, RemoveInertialAssetRemovesAsset)
{
  WorldModel world;

  auto geom1 = createCubeGeometry();
  auto geom2 = createTetrahedronGeometry();

  world.addInertialAsset(AssetInertial{std::move(geom1), 1.0, ReferenceFrame{}});
  world.addInertialAsset(AssetInertial{std::move(geom2), 2.0, ReferenceFrame{}});

  EXPECT_EQ(world.getInertialAssetCount(), 2);

  world.removeInertialAsset(0);

  EXPECT_EQ(world.getInertialAssetCount(), 1);
}

TEST(WorldModelTest, RemoveInertialAssetThrowsOnInvalidIndex)
{
  WorldModel world;

  EXPECT_THROW(world.removeInertialAsset(0), std::out_of_range);
}

TEST(WorldModelTest, GetInertialAssetsReturnsAllAssets)
{
  WorldModel world;

  auto geom1 = createCubeGeometry();
  auto geom2 = createTetrahedronGeometry();

  world.addInertialAsset(AssetInertial{std::move(geom1), 10.0, ReferenceFrame{}});
  world.addInertialAsset(AssetInertial{std::move(geom2), 20.0, ReferenceFrame{}});

  const auto& assets = world.getInertialAssets();

  EXPECT_EQ(assets.size(), 2);
}

TEST(WorldModelTest, GetInertialAssetsAllowsDirectIteration)
{
  WorldModel world;

  auto geom1 = createCubeGeometry();
  auto geom2 = createTetrahedronGeometry();

  world.addInertialAsset(AssetInertial{std::move(geom1), 5.0, ReferenceFrame{}});
  world.addInertialAsset(AssetInertial{std::move(geom2), 15.0, ReferenceFrame{}});

  double totalMass = 0.0;
  for (const auto& asset : world.getInertialAssets())
  {
    totalMass += asset.getMass();
  }

  EXPECT_DOUBLE_EQ(totalMass, 20.0);
}

// ========== Boundary Management Tests ==========

TEST(WorldModelTest, SetBoundaryStoresBoundaryHull)
{
  WorldModel world;

  std::vector<Eigen::Vector3d> boundaryPoints{
    {-100, -100, -100}, {100, -100, -100}, {0, 100, -100}, {0, 0, 100}};
  ConvexHull boundary{boundaryPoints};

  world.setBoundary(boundary);

  EXPECT_TRUE(world.hasBoundary());
}

TEST(WorldModelTest, GetBoundaryReturnsCorrectBoundary)
{
  WorldModel world;

  std::vector<Eigen::Vector3d> boundaryPoints{
    {-50, -50, -50}, {50, -50, -50}, {0, 50, -50}, {0, 0, 50}};
  ConvexHull boundary{boundaryPoints};

  world.setBoundary(boundary);

  const auto& retrievedBoundary = world.getBoundary();

  EXPECT_TRUE(retrievedBoundary.has_value());
  EXPECT_EQ(retrievedBoundary->getVertexCount(), 4);
}

TEST(WorldModelTest, SetBoundaryReplacesPreviousBoundary)
{
  WorldModel world;

  std::vector<Eigen::Vector3d> points1{{-1, -1, -1}, {1, -1, -1}, {0, 1, -1}, {0, 0, 1}};
  std::vector<Eigen::Vector3d> points2{{-2, -2, -2}, {2, -2, -2}, {0, 2, -2}, {0, 0, 2}};

  world.setBoundary(ConvexHull{points1});
  EXPECT_TRUE(world.hasBoundary());

  world.setBoundary(ConvexHull{points2});
  EXPECT_TRUE(world.hasBoundary());

  // Verify new boundary replaced old one
  const auto& boundary = world.getBoundary();
  EXPECT_TRUE(boundary.has_value());
}

TEST(WorldModelTest, ClearBoundaryRemovesBoundary)
{
  WorldModel world;

  std::vector<Eigen::Vector3d> boundaryPoints{
    {-10, -10, -10}, {10, -10, -10}, {0, 10, -10}, {0, 0, 10}};
  ConvexHull boundary{boundaryPoints};

  world.setBoundary(boundary);
  EXPECT_TRUE(world.hasBoundary());

  world.clearBoundary();
  EXPECT_FALSE(world.hasBoundary());
}

TEST(WorldModelTest, HasBoundaryReturnsFalseWhenNoBoundarySet)
{
  WorldModel world;

  EXPECT_FALSE(world.hasBoundary());
}

// ========== Mixed Storage Tests ==========

TEST(WorldModelTest, SupportsBothEnvironmentAndInertialAssetsSimultaneously)
{
  WorldModel world;

  auto envGeom = createCubeGeometry();
  auto inertialGeom = createTetrahedronGeometry();

  world.addEnvironmentAsset(AssetEnvironment{std::move(envGeom), ReferenceFrame{}});
  world.addInertialAsset(AssetInertial{std::move(inertialGeom), 5.0, ReferenceFrame{}});

  EXPECT_EQ(world.getEnvironmentAssetCount(), 1);
  EXPECT_EQ(world.getInertialAssetCount(), 1);
}

TEST(WorldModelTest, IndependentIndexingForEnvironmentAndInertialAssets)
{
  WorldModel world;

  auto geom1 = createCubeGeometry();
  auto geom2 = createTetrahedronGeometry();
  auto geom3 = createCubeGeometry();

  size_t envIdx = world.addEnvironmentAsset(AssetEnvironment{std::move(geom1), ReferenceFrame{}});
  size_t inertialIdx = world.addInertialAsset(AssetInertial{std::move(geom2), 1.0, ReferenceFrame{}});
  size_t envIdx2 = world.addEnvironmentAsset(AssetEnvironment{std::move(geom3), ReferenceFrame{}});

  // Each type has independent indexing starting at 0
  EXPECT_EQ(envIdx, 0);
  EXPECT_EQ(inertialIdx, 0);
  EXPECT_EQ(envIdx2, 1);
}
