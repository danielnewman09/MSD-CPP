// Ticket: 0071a_constraint_solver_scalability
// Design: docs/designs/0071a_constraint_solver_scalability/design.md
// Test: ConstraintIslandBuilder unit tests

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.hpp"
#include "msd-sim/src/Physics/Constraints/LambdaBounds.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

using namespace msd_sim;

// ============================================================================
// Minimal Constraint stub for testing — only bodyAIndex()/bodyBIndex() matter
// ============================================================================

namespace
{

class StubConstraint : public Constraint
{
public:
  StubConstraint(size_t bodyA, size_t bodyB)
    : Constraint(bodyA, bodyB)
  {
  }

  int dimension() const override
  {
    return 1;
  }

  int bodyCount() const override
  {
    return 2;
  }

  Eigen::VectorXd evaluate(const InertialState&,
                           const InertialState&,
                           double) const override
  {
    return Eigen::VectorXd::Zero(1);
  }

  Eigen::MatrixXd jacobian(const InertialState&,
                           const InertialState&,
                           double) const override
  {
    return Eigen::MatrixXd::Zero(1, 12);
  }

  LambdaBounds lambdaBounds() const override
  {
    return LambdaBounds::unilateral();
  }

  std::string typeName() const override
  {
    return "StubConstraint";
  }

  void recordState(msd_transfer::ConstraintRecordVisitor&,
                   uint32_t,
                   uint32_t) const override
  {
    // No-op for tests
  }
};

// Helper: check that island contains a specific body index
bool islandContainsBody(const ConstraintIslandBuilder::Island& island, size_t bodyIdx)
{
  return std::find(island.bodyIndices.begin(), island.bodyIndices.end(), bodyIdx) !=
         island.bodyIndices.end();
}

// Helper: total constraint count across all islands
size_t totalConstraints(const std::vector<ConstraintIslandBuilder::Island>& islands)
{
  size_t total = 0;
  for (const auto& island : islands)
  {
    total += island.constraints.size();
  }
  return total;
}

}  // anonymous namespace

// ============================================================================
// ConstraintIslandBuilder Unit Tests
// ============================================================================

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_empty)
{
  std::vector<Constraint*> empty;
  auto islands = ConstraintIslandBuilder::buildIslands(empty, 4);
  EXPECT_TRUE(islands.empty());
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_emptyWithZeroBodies)
{
  StubConstraint c{0, 1};
  std::vector<Constraint*> constraints{&c};
  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 0);
  EXPECT_TRUE(islands.empty());
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_singlePair)
{
  // Two bodies, one constraint → one island
  StubConstraint c{0, 1};
  std::vector<Constraint*> constraints{&c};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 2);

  ASSERT_EQ(islands.size(), 1u);
  EXPECT_EQ(islands[0].constraints.size(), 1u);
  EXPECT_EQ(islands[0].constraints[0], &c);
  EXPECT_TRUE(islandContainsBody(islands[0], 0));
  EXPECT_TRUE(islandContainsBody(islands[0], 1));
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_twoIndependentPairs)
{
  // Four bodies: A-B contact, C-D contact (disjoint) → two islands
  StubConstraint c0{0, 1};
  StubConstraint c1{2, 3};
  std::vector<Constraint*> constraints{&c0, &c1};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 4);

  ASSERT_EQ(islands.size(), 2u);
  EXPECT_EQ(totalConstraints(islands), 2u);

  // Each island has exactly 2 body indices
  for (const auto& island : islands)
  {
    EXPECT_EQ(island.bodyIndices.size(), 2u);
    EXPECT_EQ(island.constraints.size(), 1u);
  }

  // Islands collectively cover all four bodies
  std::vector<size_t> allBodies;
  for (const auto& island : islands)
  {
    allBodies.insert(allBodies.end(),
                     island.bodyIndices.begin(),
                     island.bodyIndices.end());
  }
  std::sort(allBodies.begin(), allBodies.end());
  EXPECT_EQ(allBodies, (std::vector<size_t>{0, 1, 2, 3}));
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_chainOfThree)
{
  // A-B and B-C → transitive connectivity → one island {A, B, C}
  StubConstraint c0{0, 1};  // A-B
  StubConstraint c1{1, 2};  // B-C
  std::vector<Constraint*> constraints{&c0, &c1};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 3);

  ASSERT_EQ(islands.size(), 1u);
  EXPECT_EQ(islands[0].constraints.size(), 2u);
  EXPECT_EQ(islands[0].bodyIndices.size(), 3u);
  EXPECT_TRUE(islandContainsBody(islands[0], 0));
  EXPECT_TRUE(islandContainsBody(islands[0], 1));
  EXPECT_TRUE(islandContainsBody(islands[0], 2));
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_starTopology)
{
  // One central body (0) contacting N others → one island containing all
  StubConstraint c0{0, 1};
  StubConstraint c1{0, 2};
  StubConstraint c2{0, 3};
  StubConstraint c3{0, 4};
  std::vector<Constraint*> constraints{&c0, &c1, &c2, &c3};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 5);

  ASSERT_EQ(islands.size(), 1u);
  EXPECT_EQ(islands[0].constraints.size(), 4u);
  EXPECT_EQ(islands[0].bodyIndices.size(), 5u);
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_environmentDoesNotConnect)
{
  // Body 0 touches floor (env=2), Body 1 touches floor (env=2).
  // numInertialBodies = 2, so index 2 is an environment body.
  // The two inertial bodies share only the floor — they should NOT be in the
  // same island. Critical correctness condition for the ClusterDrop scenario.
  StubConstraint c0{0, 2};  // Body 0 vs floor (env)
  StubConstraint c1{1, 2};  // Body 1 vs floor (env)
  std::vector<Constraint*> constraints{&c0, &c1};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 2);

  // Two separate islands: {body 0 + env 2} and {body 1 + env 2}
  ASSERT_EQ(islands.size(), 2u);
  EXPECT_EQ(totalConstraints(islands), 2u);

  // Each island has exactly one constraint
  for (const auto& island : islands)
  {
    EXPECT_EQ(island.constraints.size(), 1u);
  }

  // Body 0 and body 1 are in different islands
  bool body0Found = false;
  bool body1Found = false;
  for (const auto& island : islands)
  {
    if (islandContainsBody(island, 0))
    {
      body0Found = true;
      // This island must NOT contain body 1
      EXPECT_FALSE(islandContainsBody(island, 1));
    }
    if (islandContainsBody(island, 1))
    {
      body1Found = true;
      // This island must NOT contain body 0
      EXPECT_FALSE(islandContainsBody(island, 0));
    }
  }
  EXPECT_TRUE(body0Found);
  EXPECT_TRUE(body1Found);
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_mixedConnectivity)
{
  // A(0)-B(1) inertial-inertial → connected
  // B(1)-env(3) inertial-env → B in same island as A
  // C(2)-env(3) inertial-env → C separate island
  // numInertialBodies = 3, so body 3 is environment
  StubConstraint c0{0, 1};  // A-B inertial-inertial
  StubConstraint c1{1, 3};  // B-env
  StubConstraint c2{2, 3};  // C-env
  std::vector<Constraint*> constraints{&c0, &c1, &c2};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 3);

  // Island {A, B, env} and island {C, env}
  ASSERT_EQ(islands.size(), 2u);
  EXPECT_EQ(totalConstraints(islands), 3u);

  // Find the island containing A and B
  const ConstraintIslandBuilder::Island* abIsland = nullptr;
  const ConstraintIslandBuilder::Island* cIsland = nullptr;
  for (const auto& island : islands)
  {
    if (islandContainsBody(island, 0) && islandContainsBody(island, 1))
    {
      abIsland = &island;
    }
    else if (islandContainsBody(island, 2))
    {
      cIsland = &island;
    }
  }
  ASSERT_NE(abIsland, nullptr) << "A-B island not found";
  ASSERT_NE(cIsland, nullptr) << "C island not found";

  EXPECT_EQ(abIsland->constraints.size(), 2u);  // c0 and c1
  EXPECT_EQ(cIsland->constraints.size(), 1u);   // c2

  // C's island must not contain A or B
  EXPECT_FALSE(islandContainsBody(*cIsland, 0));
  EXPECT_FALSE(islandContainsBody(*cIsland, 1));
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_bodyIndicesCorrect)
{
  // Verify each island's bodyIndices contains exactly the bodies in its
  // constraints, including environment bodies
  StubConstraint c0{0, 1};  // Inertial-inertial
  StubConstraint c1{0, 3};  // Inertial-env (env idx = 3, numInertial = 2)
  std::vector<Constraint*> constraints{&c0, &c1};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 2);

  ASSERT_EQ(islands.size(), 1u);

  const auto& island = islands[0];
  EXPECT_EQ(island.constraints.size(), 2u);

  // Island must contain all bodies referenced by its constraints
  EXPECT_TRUE(islandContainsBody(island, 0));
  EXPECT_TRUE(islandContainsBody(island, 1));
  EXPECT_TRUE(islandContainsBody(island, 3));  // Environment body also included

  // Body indices are sorted
  for (size_t i = 1; i < island.bodyIndices.size(); ++i)
  {
    EXPECT_LT(island.bodyIndices[i - 1], island.bodyIndices[i])
      << "bodyIndices should be sorted";
  }
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_multipleConstraintsSamePair)
{
  // Multiple contacts between same pair (e.g., 4-point manifold) → one island
  StubConstraint c0{0, 1};
  StubConstraint c1{0, 1};
  StubConstraint c2{0, 1};
  StubConstraint c3{0, 1};
  std::vector<Constraint*> constraints{&c0, &c1, &c2, &c3};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 2);

  ASSERT_EQ(islands.size(), 1u);
  EXPECT_EQ(islands[0].constraints.size(), 4u);
  EXPECT_EQ(islands[0].bodyIndices.size(), 2u);
}

// [0071a_constraint_solver_scalability]
TEST(ConstraintIslandBuilder, buildIslands_constraintOrderPreserved)
{
  // The relative ordering of constraints within an island must match the
  // input order. This preserves the CC-before-FC interleaving used by
  // ConstraintSolver (design R2 integration note).
  StubConstraint cc0{0, 1};  // ContactConstraint for contact 0
  StubConstraint fc0{0, 1};  // FrictionConstraint for contact 0 (interleaved)
  StubConstraint cc1{0, 1};  // ContactConstraint for contact 1
  StubConstraint fc1{0, 1};  // FrictionConstraint for contact 1
  std::vector<Constraint*> constraints{&cc0, &fc0, &cc1, &fc1};

  auto islands = ConstraintIslandBuilder::buildIslands(constraints, 2);

  ASSERT_EQ(islands.size(), 1u);
  ASSERT_EQ(islands[0].constraints.size(), 4u);

  // Order must be preserved: cc0, fc0, cc1, fc1
  EXPECT_EQ(islands[0].constraints[0], &cc0);
  EXPECT_EQ(islands[0].constraints[1], &fc0);
  EXPECT_EQ(islands[0].constraints[2], &cc1);
  EXPECT_EQ(islands[0].constraints[3], &fc1);
}
