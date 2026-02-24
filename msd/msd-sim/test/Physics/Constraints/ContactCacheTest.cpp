// Ticket: 0040d_contact_persistence_warm_starting
// Ticket: 0075a_unified_constraint_data_structure
// Test: ContactCache unit tests

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/ContactCache.hpp"

using namespace msd_sim;

// ============================================================================
// ContactCache Unit Tests
// Ticket: 0075a_unified_constraint_data_structure
// All tests updated to use update()/getWarmStart() with Eigen::Vector3d
// impulse storage: {lambda_n, lambda_t1, lambda_t2}.
// ============================================================================

TEST(ContactCache, StoreAndRetrieve_MatchingContact)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  // Two contacts: frictionless (normal-only impulse)
  std::vector<Eigen::Vector3d> const impulses{
    Eigen::Vector3d{5.0, 0.0, 0.0},
    Eigen::Vector3d{3.0, 0.0, 0.0}};
  std::vector<Coordinate> const points{
    Coordinate{1.0, 0.0, 0.0}, Coordinate{-1.0, 0.0, 0.0}};

  cache.update(1, 2, normal, impulses, points);

  // Same normal, same points → should return cached impulses
  auto result = cache.getWarmStart(1, 2, normal, points);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_NEAR(result[0].x(), 5.0, 1e-10);  // lambda_n
  EXPECT_NEAR(result[0].y(), 0.0, 1e-10);  // lambda_t1
  EXPECT_NEAR(result[0].z(), 0.0, 1e-10);  // lambda_t2
  EXPECT_NEAR(result[1].x(), 3.0, 1e-10);
}

TEST(ContactCache, BodyOrderIndependent)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> const impulses{Eigen::Vector3d{7.0, 0.0, 0.0}};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  // Store with (1, 2)
  cache.update(1, 2, normal, impulses, points);

  // Retrieve with (2, 1) — should still find it
  auto result = cache.getWarmStart(2, 1, normal, points);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x(), 7.0, 1e-10);
}

TEST(ContactCache, NormalChange_InvalidatesCache)
{
  ContactCache cache;

  Vector3D const normal1{0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> const impulses{Eigen::Vector3d{5.0, 0.0, 0.0}};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal1, impulses, points);

  // Rotate normal by 20 degrees (> 15 degree threshold)
  double const angle = 20.0 * M_PI / 180.0;
  Vector3D const normal2{std::sin(angle), 0.0, std::cos(angle)};

  auto result = cache.getWarmStart(1, 2, normal2, points);
  EXPECT_TRUE(result.empty());
}

TEST(ContactCache, NormalSmallChange_PreservesCache)
{
  ContactCache cache;

  Vector3D const normal1{0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> const impulses{Eigen::Vector3d{5.0, 0.0, 0.0}};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal1, impulses, points);

  // Rotate normal by 10 degrees (< 15 degree threshold)
  double const angle = 10.0 * M_PI / 180.0;
  Vector3D const normal2{std::sin(angle), 0.0, std::cos(angle)};

  auto result = cache.getWarmStart(1, 2, normal2, points);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x(), 5.0, 1e-10);
}

TEST(ContactCache, PointMatching_NearestNeighbor)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  // Three contacts with distinct normal impulses for identification
  std::vector<Eigen::Vector3d> const impulses{
    Eigen::Vector3d{10.0, 0.0, 0.0},
    Eigen::Vector3d{20.0, 0.0, 0.0},
    Eigen::Vector3d{30.0, 0.0, 0.0}};
  std::vector<Coordinate> const cachedPoints{
    Coordinate{1.0, 0.0, 0.0},
    Coordinate{-1.0, 0.0, 0.0},
    Coordinate{0.0, 1.0, 0.0}};

  cache.update(1, 2, normal, impulses, cachedPoints);

  // Query with slightly shifted points in different order
  std::vector<Coordinate> const currentPoints{
    Coordinate{0.005, 1.005, 0.0},    // Near cachedPoints[2] → lambda_n=30
    Coordinate{1.005, 0.005, 0.0},    // Near cachedPoints[0] → lambda_n=10
    Coordinate{-0.995, 0.005, 0.0}};  // Near cachedPoints[1] → lambda_n=20

  auto result = cache.getWarmStart(1, 2, normal, currentPoints);

  ASSERT_EQ(result.size(), 3u);
  EXPECT_NEAR(result[0].x(), 30.0, 1e-10);  // Matched to cachedPoints[2]
  EXPECT_NEAR(result[1].x(), 10.0, 1e-10);  // Matched to cachedPoints[0]
  EXPECT_NEAR(result[2].x(), 20.0, 1e-10);  // Matched to cachedPoints[1]
}

TEST(ContactCache, PointMatching_NoMatchBeyondRadius)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> const impulses{Eigen::Vector3d{10.0, 0.0, 0.0}};
  std::vector<Coordinate> const cachedPoints{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, impulses, cachedPoints);

  // Query with point far away (> 10cm matching radius)
  std::vector<Coordinate> const currentPoints{Coordinate{1.0, 0.0, 0.0}};

  auto result = cache.getWarmStart(1, 2, normal, currentPoints);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x(), 0.0, 1e-10);  // No match → impulse = {0,0,0}
  EXPECT_NEAR(result[0].y(), 0.0, 1e-10);
  EXPECT_NEAR(result[0].z(), 0.0, 1e-10);
}

TEST(ContactCache, Expiry_RemovesOldEntries)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> const impulses{Eigen::Vector3d{5.0, 0.0, 0.0}};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, impulses, points);
  EXPECT_EQ(cache.size(), 1u);

  // Advance 11 frames without refreshing
  for (int i = 0; i < 11; ++i)
  {
    cache.advanceFrame();
  }
  cache.expireOldEntries(10);

  // Should be expired
  EXPECT_EQ(cache.size(), 0u);
  auto result = cache.getWarmStart(1, 2, normal, points);
  EXPECT_TRUE(result.empty());
}

TEST(ContactCache, Expiry_RefreshResetsAge)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> const impulses{Eigen::Vector3d{5.0, 0.0, 0.0}};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, impulses, points);

  // Advance 5 frames, then refresh, then advance 5 more
  for (int i = 0; i < 5; ++i)
  {
    cache.advanceFrame();
  }
  // Refresh the entry (resets age to 0)
  cache.update(1, 2, normal, impulses, points);

  for (int i = 0; i < 5; ++i)
  {
    cache.advanceFrame();
  }
  cache.expireOldEntries(10);

  // Should still be alive (age=5, not >10)
  EXPECT_EQ(cache.size(), 1u);
}

TEST(ContactCache, Clear_RemovesAllEntries)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<Eigen::Vector3d> const impulses{Eigen::Vector3d{5.0, 0.0, 0.0}};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, impulses, points);
  cache.update(3, 4, normal, impulses, points);
  EXPECT_EQ(cache.size(), 2u);

  cache.clear();
  EXPECT_EQ(cache.size(), 0u);
}

TEST(ContactCache, NoCachedEntry_ReturnsEmpty)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  auto result = cache.getWarmStart(1, 2, normal, points);
  EXPECT_TRUE(result.empty());
}

TEST(ContactCache, PartialMatch_MixedInitialization)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};

  // Cache with 2 contact points
  std::vector<Eigen::Vector3d> const impulses{
    Eigen::Vector3d{10.0, 0.0, 0.0},
    Eigen::Vector3d{20.0, 0.0, 0.0}};
  std::vector<Coordinate> const cachedPoints{
    Coordinate{1.0, 0.0, 0.0}, Coordinate{-1.0, 0.0, 0.0}};
  cache.update(1, 2, normal, impulses, cachedPoints);

  // Query with 3 points: 2 match, 1 new
  std::vector<Coordinate> const currentPoints{
    Coordinate{1.005, 0.0, 0.0},   // Near cached[0] → lambda_n=10
    Coordinate{0.0, 5.0, 0.0},     // Far from both → 0.0
    Coordinate{-1.005, 0.0, 0.0}}; // Near cached[1] → lambda_n=20

  auto result = cache.getWarmStart(1, 2, normal, currentPoints);

  ASSERT_EQ(result.size(), 3u);
  EXPECT_NEAR(result[0].x(), 10.0, 1e-10);  // Matched
  EXPECT_NEAR(result[1].x(), 0.0, 1e-10);   // New contact → zero impulse
  EXPECT_NEAR(result[2].x(), 20.0, 1e-10);  // Matched
}

TEST(ContactCache, FrictionImpulse_StoredAndRetrieved)
{
  // Ticket: 0075a — verify that friction components (t1, t2) are preserved
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  // One contact with friction impulses
  std::vector<Eigen::Vector3d> const impulses{
    Eigen::Vector3d{8.0, 2.5, -1.2}};  // {lambda_n, lambda_t1, lambda_t2}
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, impulses, points);

  auto result = cache.getWarmStart(1, 2, normal, points);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x(), 8.0, 1e-10);   // lambda_n
  EXPECT_NEAR(result[0].y(), 2.5, 1e-10);   // lambda_t1
  EXPECT_NEAR(result[0].z(), -1.2, 1e-10);  // lambda_t2
}
