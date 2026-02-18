// Ticket: 0071a_constraint_solver_scalability
// Design: docs/designs/0071a_constraint_solver_scalability/design.md

#include "msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace msd_sim
{

// ============================================================================
// Union-Find (Disjoint Set Union) helper — local to this translation unit
// ============================================================================

namespace
{

/**
 * @brief Path-compressing union-find over a fixed-size range [0, n)
 *
 * Uses path compression and union-by-rank for near-O(1) amortised operations.
 * Only operates on inertial body indices [0, numInertial). Environment bodies
 * are excluded by the caller (they do not create connectivity).
 */
class UnionFind
{
public:
  explicit UnionFind(size_t n) : parent_(n), rank_(n, 0)
  {
    std::iota(parent_.begin(), parent_.end(), size_t{0});
  }

  /// Find root representative with path compression.
  size_t find(size_t x)
  {
    if (parent_[x] != x)
    {
      parent_[x] = find(parent_[x]);  // Path compression
    }
    return parent_[x];
  }

  /// Union two elements by rank.
  void unite(size_t a, size_t b)
  {
    size_t const ra = find(a);
    size_t const rb = find(b);
    if (ra == rb)
    {
      return;
    }
    // Union by rank
    if (rank_[ra] < rank_[rb])
    {
      parent_[ra] = rb;
    }
    else if (rank_[ra] > rank_[rb])
    {
      parent_[rb] = ra;
    }
    else
    {
      parent_[rb] = ra;
      ++rank_[ra];
    }
  }

private:
  std::vector<size_t> parent_;
  std::vector<size_t> rank_;
};

}  // anonymous namespace

// ============================================================================
// ConstraintIslandBuilder::buildIslands
// ============================================================================

std::vector<ConstraintIslandBuilder::Island>
ConstraintIslandBuilder::buildIslands(
  const std::vector<Constraint*>& constraints,
  size_t numInertialBodies)
{
  if (constraints.empty() || numInertialBodies == 0)
  {
    return {};
  }

  // Step 1: Build union-find over inertial body indices only.
  // Environment bodies (index >= numInertialBodies) are excluded — they have
  // infinite mass and do not create mechanical coupling between inertial bodies.
  UnionFind uf{numInertialBodies};

  for (const auto* c : constraints)
  {
    if (c == nullptr)
    {
      continue;
    }
    const size_t a = c->bodyAIndex();
    const size_t b = c->bodyBIndex();

    const bool aIsInertial = (a < numInertialBodies);
    const bool bIsInertial = (b < numInertialBodies);

    if (aIsInertial && bIsInertial)
    {
      // Inertial-inertial: create connectivity
      uf.unite(a, b);
    }
    // Inertial-environment: do NOT create connectivity.
    // The environment body is infinite mass and does not couple two inertial
    // bodies sharing the same floor/wall.
  }

  // Step 2: Group constraints by their inertial root representative.
  // Key: root representative of the inertial side.
  // For inertial-environment constraints, the inertial body's root is used.
  // For inertial-inertial constraints, both bodies share a root after union.
  std::unordered_map<size_t, std::vector<Constraint*>> islandConstraints;
  islandConstraints.reserve(numInertialBodies);

  for (auto* c : constraints)
  {
    if (c == nullptr)
    {
      continue;
    }
    const size_t a = c->bodyAIndex();
    const size_t b = c->bodyBIndex();

    const bool aIsInertial = (a < numInertialBodies);
    const bool bIsInertial = (b < numInertialBodies);

    size_t root = 0;
    if (aIsInertial)
    {
      root = uf.find(a);
    }
    else if (bIsInertial)
    {
      root = uf.find(b);
    }
    else
    {
      // Both bodies are environment — this should not happen in practice
      // (environment bodies don't collide with each other), but handle it
      // gracefully by skipping.
      continue;
    }

    islandConstraints[root].push_back(c);
  }

  // Step 3: Collect unique body indices per island (inertial + environment).
  std::vector<Island> islands;
  islands.reserve(islandConstraints.size());

  for (auto& [root, constraintList] : islandConstraints)
  {
    Island island;
    island.constraints = std::move(constraintList);

    // Collect unique body indices from this island's constraints
    std::unordered_set<size_t> bodySet;
    for (const auto* c : island.constraints)
    {
      bodySet.insert(c->bodyAIndex());
      bodySet.insert(c->bodyBIndex());
    }
    island.bodyIndices.assign(bodySet.begin(), bodySet.end());
    std::sort(island.bodyIndices.begin(), island.bodyIndices.end());

    islands.push_back(std::move(island));
  }

  return islands;
}

}  // namespace msd_sim
