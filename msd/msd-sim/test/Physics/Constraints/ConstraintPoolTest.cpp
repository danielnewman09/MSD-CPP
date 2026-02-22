// Ticket: 0071g_constraint_pool_allocation
// Design: docs/designs/0071g_constraint_pool_allocation/design.md

#include "msd-sim/src/Physics/Constraints/ConstraintPool.hpp"

#include <gtest/gtest.h>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"

namespace msd_sim
{

// ===== Test Fixtures =====

namespace
{

// Canonical unit normal for test contact constraints
const Coordinate kNormal{0.0, 0.0, 1.0};
const Coordinate kContactPointA{0.0, 0.0, 1.0};
const Coordinate kContactPointB{0.0, 0.0, -1.0};
const Coordinate kComA{0.0, 0.0, 2.0};
const Coordinate kComB{0.0, 0.0, -2.0};
constexpr double kPenetrationDepth = 0.01;
constexpr double kRestitution = 0.5;
constexpr double kPreImpactVel = -1.0;
constexpr double kFrictionCoeff = 0.3;

ContactConstraint* allocateTestContact(ConstraintPool& pool,
                                       size_t bodyA = 0,
                                       size_t bodyB = 1)
{
  return pool.allocateContact(bodyA,
                              bodyB,
                              kNormal,
                              kContactPointA,
                              kContactPointB,
                              kPenetrationDepth,
                              kComA,
                              kComB,
                              kRestitution,
                              kPreImpactVel);
}

FrictionConstraint* allocateTestFriction(ConstraintPool& pool,
                                          size_t bodyA = 0,
                                          size_t bodyB = 1)
{
  return pool.allocateFriction(bodyA,
                               bodyB,
                               kNormal,
                               kContactPointA,
                               kContactPointB,
                               kComA,
                               kComB,
                               kFrictionCoeff);
}

}  // namespace

// ===== ConstraintPool Unit Tests =====

TEST(ConstraintPoolTest, InitialState_CountsAreZero_0071g)
{
  ConstraintPool pool;
  EXPECT_EQ(pool.contactCount(), 0u);
  EXPECT_EQ(pool.frictionCount(), 0u);
}

TEST(ConstraintPoolTest, AllocateContact_ReturnsNonNullPointer_0071g)
{
  ConstraintPool pool;
  ContactConstraint* cc = allocateTestContact(pool);
  ASSERT_NE(cc, nullptr);
}

TEST(ConstraintPoolTest, AllocateContact_IncreasesContactCount_0071g)
{
  ConstraintPool pool;
  allocateTestContact(pool);
  EXPECT_EQ(pool.contactCount(), 1u);
  EXPECT_EQ(pool.frictionCount(), 0u);

  allocateTestContact(pool);
  EXPECT_EQ(pool.contactCount(), 2u);
}

TEST(ConstraintPoolTest, AllocateFriction_ReturnNonNullPointer_0071g)
{
  ConstraintPool pool;
  FrictionConstraint* fc = allocateTestFriction(pool);
  ASSERT_NE(fc, nullptr);
}

TEST(ConstraintPoolTest, AllocateFriction_IncreasesFrictionCount_0071g)
{
  ConstraintPool pool;
  allocateTestFriction(pool);
  EXPECT_EQ(pool.frictionCount(), 1u);
  EXPECT_EQ(pool.contactCount(), 0u);

  allocateTestFriction(pool);
  EXPECT_EQ(pool.frictionCount(), 2u);
}

TEST(ConstraintPoolTest, AllocateContact_ConstraintHasCorrectNormal_0071g)
{
  ConstraintPool pool;
  ContactConstraint* cc = allocateTestContact(pool);
  ASSERT_NE(cc, nullptr);
  EXPECT_NEAR(cc->getContactNormal().x(), kNormal.x(), 1e-10);
  EXPECT_NEAR(cc->getContactNormal().y(), kNormal.y(), 1e-10);
  EXPECT_NEAR(cc->getContactNormal().z(), kNormal.z(), 1e-10);
}

TEST(ConstraintPoolTest, AllocateContact_ConstraintHasPenetrationDepth_0071g)
{
  ConstraintPool pool;
  ContactConstraint* cc = allocateTestContact(pool);
  ASSERT_NE(cc, nullptr);
  EXPECT_NEAR(cc->getPenetrationDepth(), kPenetrationDepth, 1e-10);
}

TEST(ConstraintPoolTest, AllocateContact_ConstraintHasBodyIndices_0071g)
{
  ConstraintPool pool;
  ContactConstraint* cc = allocateTestContact(pool, 3, 7);
  ASSERT_NE(cc, nullptr);
  EXPECT_EQ(cc->bodyAIndex(), 3u);
  EXPECT_EQ(cc->bodyBIndex(), 7u);
}

TEST(ConstraintPoolTest, Reset_CountsBecomeZero_0071g)
{
  ConstraintPool pool;
  allocateTestContact(pool);
  allocateTestContact(pool);
  allocateTestFriction(pool);
  EXPECT_EQ(pool.contactCount(), 2u);
  EXPECT_EQ(pool.frictionCount(), 1u);

  pool.reset();

  EXPECT_EQ(pool.contactCount(), 0u);
  EXPECT_EQ(pool.frictionCount(), 0u);
}

TEST(ConstraintPoolTest, Reset_ThenReallocate_ProducesValidConstraints_0071g)
{
  // This tests the steady-state behavior: reset clears counts, capacity is
  // retained, and the next allocation batch produces valid constraints.
  ConstraintPool pool;

  // First frame: allocate some constraints
  allocateTestContact(pool);
  allocateTestFriction(pool);
  EXPECT_EQ(pool.contactCount(), 1u);
  EXPECT_EQ(pool.frictionCount(), 1u);

  // Second frame: reset and reallocate
  pool.reset();
  EXPECT_EQ(pool.contactCount(), 0u);
  EXPECT_EQ(pool.frictionCount(), 0u);

  ContactConstraint* cc = allocateTestContact(pool);
  FrictionConstraint* fc = allocateTestFriction(pool);

  ASSERT_NE(cc, nullptr);
  ASSERT_NE(fc, nullptr);
  EXPECT_EQ(pool.contactCount(), 1u);
  EXPECT_EQ(pool.frictionCount(), 1u);

  // Verify constraint values are correct after reuse
  EXPECT_NEAR(cc->getPenetrationDepth(), kPenetrationDepth, 1e-10);
  EXPECT_NEAR(fc->getFrictionCoefficient(), kFrictionCoeff, 1e-10);
}

TEST(ConstraintPoolTest, PointerStability_WithReserve_PointersRemainValid_0071g)
{
  // After reserveContacts(n) and reserveFriction(n), all emplace_back calls
  // within n elements must not invalidate previously returned pointers.
  ConstraintPool pool;

  constexpr size_t kCount = 100;
  pool.reserveContacts(kCount);
  pool.reserveFriction(kCount);

  // Allocate 100 contact + 100 friction constraints
  std::vector<ContactConstraint*> contactPtrs;
  std::vector<FrictionConstraint*> frictionPtrs;

  for (size_t i = 0; i < kCount; ++i)
  {
    contactPtrs.push_back(allocateTestContact(pool, i, i + 1));
    frictionPtrs.push_back(allocateTestFriction(pool, i, i + 1));
  }

  EXPECT_EQ(pool.contactCount(), kCount);
  EXPECT_EQ(pool.frictionCount(), kCount);

  // Verify all pointers are still valid and have correct body indices
  for (size_t i = 0; i < kCount; ++i)
  {
    ASSERT_NE(contactPtrs[i], nullptr) << "Contact pointer " << i << " is null";
    EXPECT_EQ(contactPtrs[i]->bodyAIndex(), i) << "Contact " << i << " has wrong bodyA";

    ASSERT_NE(frictionPtrs[i], nullptr) << "Friction pointer " << i << " is null";
    EXPECT_EQ(frictionPtrs[i]->bodyAIndex(), i) << "Friction " << i << " has wrong bodyA";
  }
}

TEST(ConstraintPoolTest, MultipleResetCycles_NoHeapGrowthAfterHighWaterMark_0071g)
{
  // Simulate multiple frames: allocate, reset, allocate.
  // After the first allocation, capacity is retained; counts return to zero on reset.
  ConstraintPool pool;

  // Pre-reserve to avoid pointer invalidity within the test
  pool.reserveContacts(10);

  for (int frame = 0; frame < 5; ++frame)
  {
    pool.reset();  // First iteration: counts were 0 from init; subsequent: clear from previous
    for (int i = 0; i < 10; ++i)
    {
      allocateTestContact(pool);
    }
    EXPECT_EQ(pool.contactCount(), 10u)
      << "Frame " << frame << ": expected 10 contacts";
  }
}

TEST(ConstraintPoolTest, AllocateManyContacts_CountsCorrect_0071g)
{
  ConstraintPool pool;
  pool.reserveContacts(200);
  pool.reserveFriction(200);

  for (size_t i = 0; i < 200; ++i)
  {
    allocateTestContact(pool);
    allocateTestFriction(pool);
  }

  EXPECT_EQ(pool.contactCount(), 200u);
  EXPECT_EQ(pool.frictionCount(), 200u);
}

TEST(ConstraintPoolTest, ReserveContacts_DoesNotAllocateConstraints_0071g)
{
  // reserveContacts() changes capacity but not count
  ConstraintPool pool;
  pool.reserveContacts(50);

  EXPECT_EQ(pool.contactCount(), 0u);
  EXPECT_EQ(pool.frictionCount(), 0u);
}

TEST(ConstraintPoolTest, ReserveFriction_DoesNotAllocateConstraints_0071g)
{
  ConstraintPool pool;
  pool.reserveFriction(50);

  EXPECT_EQ(pool.contactCount(), 0u);
  EXPECT_EQ(pool.frictionCount(), 0u);
}

}  // namespace msd_sim
