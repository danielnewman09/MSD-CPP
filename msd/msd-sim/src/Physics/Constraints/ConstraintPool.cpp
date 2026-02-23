// Ticket: 0071g_constraint_pool_allocation
// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0071g_constraint_pool_allocation/design.md
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

#include "msd-sim/src/Physics/Constraints/ConstraintPool.hpp"

namespace msd_sim
{

ContactConstraint* ConstraintPool::allocateContact(
  size_t bodyAIndex,
  size_t bodyBIndex,
  const Coordinate& normal,
  const Coordinate& contactPointA,
  const Coordinate& contactPointB,
  double penetrationDepth,
  const Coordinate& comA,
  const Coordinate& comB,
  double restitution,
  double preImpactRelVelNormal,
  double frictionCoefficient)
{
  // Ticket: 0075a — pass frictionCoefficient to unified ContactConstraint
  contactStorage_.emplace_back(bodyAIndex,
                                bodyBIndex,
                                normal,
                                contactPointA,
                                contactPointB,
                                penetrationDepth,
                                comA,
                                comB,
                                restitution,
                                preImpactRelVelNormal,
                                frictionCoefficient);
  return &contactStorage_.back();
}

FrictionConstraint* ConstraintPool::allocateFriction(
  size_t bodyAIndex,
  size_t bodyBIndex,
  const Coordinate& normal,
  const Coordinate& contactPointA,
  const Coordinate& contactPointB,
  const Coordinate& comA,
  const Coordinate& comB,
  double frictionCoefficient)
{
  frictionStorage_.emplace_back(bodyAIndex,
                                 bodyBIndex,
                                 normal,
                                 contactPointA,
                                 contactPointB,
                                 comA,
                                 comB,
                                 frictionCoefficient);
  return &frictionStorage_.back();
}

void ConstraintPool::reset()
{
  // clear() destructs all active constraints but preserves vector capacity.
  // After the first frame, subsequent emplace_back() calls reuse the existing
  // allocation with zero heap activity.
  contactStorage_.clear();
  frictionStorage_.clear();
}

void ConstraintPool::reserveContacts(size_t n)
{
  contactStorage_.reserve(n);
}

void ConstraintPool::reserveFriction(size_t n)
{
  frictionStorage_.reserve(n);
}

size_t ConstraintPool::contactCount() const
{
  return contactStorage_.size();
}

size_t ConstraintPool::frictionCount() const
{
  return frictionStorage_.size();
}

}  // namespace msd_sim
