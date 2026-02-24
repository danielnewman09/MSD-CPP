// Ticket: 0071g_constraint_pool_allocation
// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0071g_constraint_pool_allocation/design.md
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

#ifndef MSD_SIM_PHYSICS_CONSTRAINT_POOL_HPP
#define MSD_SIM_PHYSICS_CONSTRAINT_POOL_HPP

#include <cstddef>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"

namespace msd_sim
{

/**
 * @brief Per-frame typed free-list pool for constraint objects.
 *
 * Maintains two contiguous backing stores — one for ContactConstraint,
 * one for FrictionConstraint. After the first frame the backing vectors
 * have grown to their high-water mark; subsequent reset() calls clear
 * the vectors without releasing capacity, achieving zero heap allocation
 * for constraint objects in steady state.
 *
 * Implementation note: Uses reserve() + emplace_back() + clear() rather
 * than resize() + assignment because ContactConstraint and FrictionConstraint
 * have no default constructors. clear() calls trivial virtual destructors
 * (no owned resources) and preserves capacity, so subsequent emplace_back()
 * calls reuse the allocated memory.
 *
 * Pointer stability: Pointers returned by allocate*() are valid until
 * the next reset() call. Callers must not store pointers across frames.
 *
 * Frame lifecycle (pointer safety guarantee):
 * 1. reset() — clear counts, preserve capacity
 * 2. allocate*() batch — all emplace_back calls for this frame
 * 3. Solver, position correction, recording — read-only pointer usage
 * 4. Next frame — goto 1
 *
 * There is no interleaving of grow and use within a frame. When the
 * vector must grow (first frame or new high-water mark), all pointers
 * already stored in allConstraints_ from this frame would be invalidated —
 * but because all allocations happen in the createConstraints() batch before
 * any pointer is stored, this is safe.
 *
 * Extensibility: New constraint types (joints, limits) add a new
 * typed backing vector and a corresponding allocate*() method.
 * No changes are needed to existing allocation paths.
 *
 * Thread safety: Not thread-safe (single-threaded physics loop assumed).
 *
 * @see docs/designs/0071g_constraint_pool_allocation/design.md
 * @ticket 0071g_constraint_pool_allocation
 */
class ConstraintPool
{
public:
  ConstraintPool() = default;

  /**
   * @brief Construct a ContactConstraint in-pool and return a pointer to it.
   *
   * Grows the backing vector on first call or when capacity is exceeded.
   * After the first frame, capacity is sufficient and no allocation occurs.
   *
   * @return Pointer to the constructed ContactConstraint.
   *         Valid until next reset().
   */
  /**
   * @brief Construct a unified ContactConstraint in-pool and return a pointer.
   *
   * @param frictionCoefficient Combined friction coefficient μ [0, ∞)
   *        Default 0.0 → frictionless (dimension=1, backward-compatible)
   *
   * @ticket 0075a_unified_constraint_data_structure
   */
  ContactConstraint* allocateContact(size_t bodyAIndex,
                                     size_t bodyBIndex,
                                     const Coordinate& normal,
                                     const Coordinate& contactPointA,
                                     const Coordinate& contactPointB,
                                     double penetrationDepth,
                                     const Coordinate& comA,
                                     const Coordinate& comB,
                                     double restitution,
                                     double preImpactRelVelNormal,
                                     double frictionCoefficient = 0.0);

  /**
   * @brief Construct a FrictionConstraint in-pool and return a pointer.
   *
   * @deprecated As of ticket 0075a_unified_constraint_data_structure,
   *   friction data is embedded in ContactConstraint. This method is
   *   retained for backward compatibility but should not be called
   *   from new code. Will be removed in ticket 0075c.
   *
   * Grows the backing vector on first call or when capacity is exceeded.
   * After the first frame, capacity is sufficient and no allocation occurs.
   *
   * @return Pointer to the constructed FrictionConstraint.
   *         Valid until next reset().
   */
  FrictionConstraint* allocateFriction(size_t bodyAIndex,
                                       size_t bodyBIndex,
                                       const Coordinate& normal,
                                       const Coordinate& contactPointA,
                                       const Coordinate& contactPointB,
                                       const Coordinate& comA,
                                       const Coordinate& comB,
                                       double frictionCoefficient);

  /**
   * @brief Reset all constraint pools for the next frame.
   *
   * Calls clear() on both backing vectors: destructs all active constraints
   * but preserves allocated capacity. After the first frame, subsequent
   * emplace_back() calls reuse existing capacity with zero heap allocation.
   *
   * All previously returned pointers become invalid after this call.
   */
  void reset();

  /**
   * @brief Pre-reserve capacity for ContactConstraints this frame.
   *
   * Call this before any allocateContact() calls within a frame to ensure
   * the backing vector will not reallocate during the allocation batch.
   * When capacity >= n no reallocation occurs, so returned pointers remain
   * valid for the entire frame.
   *
   * @param n Total number of ContactConstraints expected this frame
   */
  void reserveContacts(size_t n);

  /**
   * @brief Pre-reserve capacity for FrictionConstraints this frame.
   *
   * Call this before any allocateFriction() calls within a frame.
   *
   * @param n Total number of FrictionConstraints expected this frame
   */
  void reserveFriction(size_t n);

  /** @return Number of active ContactConstraints in the pool. */
  [[nodiscard]] size_t contactCount() const;

  /**
   * @brief Number of active FrictionConstraints in the pool.
   *
   * @deprecated As of ticket 0075a, friction is embedded in ContactConstraint.
   *   This always returns 0 in normal operation. Will be removed in ticket 0075c.
   */
  [[nodiscard]] size_t frictionCount() const;

  // Rule of Zero: compiler-generated copy/move are correct.
  // (backing vectors own their memory; copying a pool copies all constraints)

private:
  std::vector<ContactConstraint> contactStorage_;
  std::vector<FrictionConstraint> frictionStorage_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_POOL_HPP
