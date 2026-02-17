// Ticket: 0040d_contact_persistence_warm_starting
// Ticket: 0052d_solver_integration_ecos_removal
// Ticket: 0069_friction_velocity_reversal

#include "msd-sim/src/Physics/Constraints/ContactCache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace msd_sim
{

std::vector<double> ContactCache::getWarmStart(
  uint32_t bodyA,
  uint32_t bodyB,
  const Vector3D& currentNormal,
  const std::vector<Coordinate>& currentPoints) const
{
  auto key = makeKey(bodyA, bodyB);
  auto it = cache_.find(key);
  if (it == cache_.end())
  {
    return {};
  }

  const CachedContact& cached = it->second;

  // Check normal similarity: dot product > cos(15 degrees)
  double const dotProduct = currentNormal.dot(cached.normal);
  if (dotProduct < kNormalThreshold)
  {
    return {};
  }

  // Match current contact points to cached points by nearest-neighbor
  const size_t numCurrent = currentPoints.size();
  const size_t numCached = cached.points.size();

  if (numCurrent == 0 || numCached == 0)
  {
    return {};
  }

  // Ticket 0052d: Determine if cache stores 3 lambdas per contact (friction)
  // or 1 lambda per contact (no friction / legacy)
  const bool hasFrictionLambdas = (cached.lambdas.size() == numCached * 3);
  const size_t lambdasPerContact = hasFrictionLambdas ? 3 : 1;

  // Return vector: lambdasPerContact values per current contact
  std::vector<double> warmLambdas(numCurrent * lambdasPerContact, 0.0);

  // Track which cached points have been matched (prevent double-matching)
  std::vector<bool> cachedUsed(numCached, false);

  for (size_t i = 0; i < numCurrent; ++i)
  {
    double bestDist = std::numeric_limits<double>::max();
    std::optional<size_t> bestIdx = std::nullopt;

    for (size_t j = 0; j < numCached; ++j)
    {
      if (cachedUsed[j])
      {
        continue;
      }

      double const dist = (currentPoints[i] - cached.points[j]).norm();
      if (dist < bestDist)
      {
        bestDist = dist;
        bestIdx = j;
      }
    }

    if (bestIdx && bestDist < kPointMatchRadius)
    {
      // Copy lambdasPerContact values for the matched contact
      for (size_t k = 0; k < lambdasPerContact; ++k)
      {
        warmLambdas[i * lambdasPerContact + k] =
          cached.lambdas[*bestIdx * lambdasPerContact + k];
      }
      cachedUsed[*bestIdx] = true;
    }
    // Unmatched points keep lambda = 0
  }

  return warmLambdas;
}

void ContactCache::update(uint32_t bodyA,
                           uint32_t bodyB,
                           const Vector3D& normal,
                           const std::vector<double>& lambdas,
                           const std::vector<Coordinate>& points)
{
  auto key = makeKey(bodyA, bodyB);
  cache_[key] = CachedContact{
    .bodyA_id = std::min(bodyA, bodyB),
    .bodyB_id = std::max(bodyA, bodyB),
    .normal = normal,
    .lambdas = lambdas,
    .points = points,
    .age = 0};
}

void ContactCache::expireOldEntries(uint32_t maxAge)
{
  for (auto it = cache_.begin(); it != cache_.end();)
  {
    if (it->second.age > maxAge)
    {
      it = cache_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void ContactCache::advanceFrame()
{
  for (auto& [key, entry] : cache_)
  {
    ++entry.age;
  }
}

bool ContactCache::hasEntry(uint32_t bodyA, uint32_t bodyB) const
{
  auto key = makeKey(bodyA, bodyB);
  return cache_.find(key) != cache_.end();
}

void ContactCache::clear()
{
  cache_.clear();
}

size_t ContactCache::size() const
{
  return cache_.size();
}

ContactCache::BodyPairKey ContactCache::makeKey(uint32_t a, uint32_t b)
{
  return {std::min(a, b), std::max(a, b)};
}

void ContactCache::updateSlidingState(uint32_t bodyA,
                                       uint32_t bodyB,
                                       const Vector3D& tangentVelocity,
                                       double velocityThreshold)
{
  auto key = makeKey(bodyA, bodyB);
  auto it = cache_.find(key);
  if (it == cache_.end())
  {
    return;  // No cache entry for this pair
  }

  CachedContact& cached = it->second;

  double const speed = tangentVelocity.norm();
  if (speed >= velocityThreshold)
  {
    // Sustained sliding — update direction and increment count
    cached.slidingDirection = tangentVelocity / speed;  // Normalize
    ++cached.slidingFrameCount;
  }
  else
  {
    // Velocity dropped below threshold — reset sliding state
    cached.slidingDirection = std::nullopt;
    cached.slidingFrameCount = 0;
  }
}

std::pair<std::optional<Vector3D>, bool> ContactCache::getSlidingState(
  uint32_t bodyA,
  uint32_t bodyB,
  int minFrames) const
{
  auto key = makeKey(bodyA, bodyB);
  auto it = cache_.find(key);
  if (it == cache_.end())
  {
    return {std::nullopt, false};
  }

  const CachedContact& cached = it->second;

  // Sliding mode is active if direction is set and frame count meets threshold
  bool const isActive =
    cached.slidingDirection.has_value() && (cached.slidingFrameCount >= minFrames);

  return {cached.slidingDirection, isActive};
}

}  // namespace msd_sim
