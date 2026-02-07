// Ticket: 0040d_contact_persistence_warm_starting
// Test: ContactCache unit tests

#include <gtest/gtest.h>

#include <cmath>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/ContactCache.hpp"

using namespace msd_sim;

// ============================================================================
// ContactCache Unit Tests
// ============================================================================

TEST(ContactCache, StoreAndRetrieve_MatchingContact)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<double> const lambdas{5.0, 3.0};
  std::vector<Coordinate> const points{
    Coordinate{1.0, 0.0, 0.0}, Coordinate{-1.0, 0.0, 0.0}};

  cache.update(1, 2, normal, lambdas, points);

  // Same normal, same points → should return cached lambdas
  auto result = cache.getWarmStart(1, 2, normal, points);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_NEAR(result[0], 5.0, 1e-10);
  EXPECT_NEAR(result[1], 3.0, 1e-10);
}

TEST(ContactCache, BodyOrderIndependent)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<double> const lambdas{7.0};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  // Store with (1, 2)
  cache.update(1, 2, normal, lambdas, points);

  // Retrieve with (2, 1) — should still find it
  auto result = cache.getWarmStart(2, 1, normal, points);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0], 7.0, 1e-10);
}

TEST(ContactCache, NormalChange_InvalidatesCache)
{
  ContactCache cache;

  Vector3D const normal1{0.0, 0.0, 1.0};
  std::vector<double> const lambdas{5.0};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal1, lambdas, points);

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
  std::vector<double> const lambdas{5.0};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal1, lambdas, points);

  // Rotate normal by 10 degrees (< 15 degree threshold)
  double const angle = 10.0 * M_PI / 180.0;
  Vector3D const normal2{std::sin(angle), 0.0, std::cos(angle)};

  auto result = cache.getWarmStart(1, 2, normal2, points);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0], 5.0, 1e-10);
}

TEST(ContactCache, PointMatching_NearestNeighbor)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<double> const lambdas{10.0, 20.0, 30.0};
  std::vector<Coordinate> const cachedPoints{
    Coordinate{1.0, 0.0, 0.0},
    Coordinate{-1.0, 0.0, 0.0},
    Coordinate{0.0, 1.0, 0.0}};

  cache.update(1, 2, normal, lambdas, cachedPoints);

  // Query with slightly shifted points in different order
  std::vector<Coordinate> const currentPoints{
    Coordinate{0.005, 1.005, 0.0},    // Near cachedPoints[2] → lambda=30
    Coordinate{1.005, 0.005, 0.0},    // Near cachedPoints[0] → lambda=10
    Coordinate{-0.995, 0.005, 0.0}};  // Near cachedPoints[1] → lambda=20

  auto result = cache.getWarmStart(1, 2, normal, currentPoints);

  ASSERT_EQ(result.size(), 3u);
  EXPECT_NEAR(result[0], 30.0, 1e-10);  // Matched to cachedPoints[2]
  EXPECT_NEAR(result[1], 10.0, 1e-10);  // Matched to cachedPoints[0]
  EXPECT_NEAR(result[2], 20.0, 1e-10);  // Matched to cachedPoints[1]
}

TEST(ContactCache, PointMatching_NoMatchBeyondRadius)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<double> const lambdas{10.0};
  std::vector<Coordinate> const cachedPoints{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, lambdas, cachedPoints);

  // Query with point far away (> 2cm matching radius)
  std::vector<Coordinate> const currentPoints{Coordinate{1.0, 0.0, 0.0}};

  auto result = cache.getWarmStart(1, 2, normal, currentPoints);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0], 0.0, 1e-10);  // No match → lambda = 0
}

TEST(ContactCache, Expiry_RemovesOldEntries)
{
  ContactCache cache;

  Vector3D const normal{0.0, 0.0, 1.0};
  std::vector<double> const lambdas{5.0};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, lambdas, points);
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
  std::vector<double> const lambdas{5.0};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, lambdas, points);

  // Advance 5 frames, then refresh, then advance 5 more
  for (int i = 0; i < 5; ++i)
  {
    cache.advanceFrame();
  }
  // Refresh the entry (resets age to 0)
  cache.update(1, 2, normal, lambdas, points);

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
  std::vector<double> const lambdas{5.0};
  std::vector<Coordinate> const points{Coordinate{0.0, 0.0, 0.0}};

  cache.update(1, 2, normal, lambdas, points);
  cache.update(3, 4, normal, lambdas, points);
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
  std::vector<double> const lambdas{10.0, 20.0};
  std::vector<Coordinate> const cachedPoints{
    Coordinate{1.0, 0.0, 0.0}, Coordinate{-1.0, 0.0, 0.0}};
  cache.update(1, 2, normal, lambdas, cachedPoints);

  // Query with 3 points: 2 match, 1 new
  std::vector<Coordinate> const currentPoints{
    Coordinate{1.005, 0.0, 0.0},   // Near cached[0] → 10.0
    Coordinate{0.0, 5.0, 0.0},     // Far from both → 0.0
    Coordinate{-1.005, 0.0, 0.0}}; // Near cached[1] → 20.0

  auto result = cache.getWarmStart(1, 2, normal, currentPoints);

  ASSERT_EQ(result.size(), 3u);
  EXPECT_NEAR(result[0], 10.0, 1e-10);  // Matched
  EXPECT_NEAR(result[1], 0.0, 1e-10);   // New contact
  EXPECT_NEAR(result[2], 20.0, 1e-10);  // Matched
}
