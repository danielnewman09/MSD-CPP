// Ticket: 0071a_constraint_solver_scalability
// Design: docs/designs/0071a_constraint_solver_scalability/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINT_ISLAND_BUILDER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINT_ISLAND_BUILDER_HPP

#include <cstddef>
#include <vector>

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"

namespace msd_sim
{

/**
 * @brief Partitions constraints into independent contact islands via union-find
 *
 * Uses union-find (disjoint set) to partition a flat constraint list into
 * independent connected components. Two bodies are connected if they share
 * at least one constraint. Environment bodies (index >= numInertialBodies)
 * are intentionally excluded from connectivity: two inertial bodies touching
 * the same floor remain in separate islands, because environment bodies have
 * infinite mass and are unperturbed by contacts.
 *
 * This is a pure orchestration utility for CollisionPipeline. It produces
 * island subsets that are passed to ConstraintSolver::solve() individually,
 * reducing the effective mass matrix from O((3C)³) global to O(Σᵢ (3Cᵢ)³)
 * per-island.
 *
 * Complexity: O(n · α(n)) where α is the inverse Ackermann function
 * (effectively O(n) total for all operations).
 *
 * Thread safety: Stateless static utility — safe to call from any thread.
 * Error handling: Returns empty vector for empty input. No exceptions thrown.
 *
 * @see docs/designs/0071a_constraint_solver_scalability/design.md
 * @see docs/designs/0071a_constraint_solver_scalability/0071a_constraint_solver_scalability.puml
 * @ticket 0071a_constraint_solver_scalability
 */
class ConstraintIslandBuilder
{
public:
  /**
   * @brief One independent contact island
   *
   * An island is a connected component in the contact graph. All constraints
   * in an island share at least one body with another constraint in the same
   * island (via transitive connectivity). Bodies with no contacts do not
   * appear in any island.
   */
  struct Island
  {
    std::vector<Constraint*> constraints;  ///< Non-owning pointers into owning container
    std::vector<size_t> bodyIndices;       ///< Unique body indices (inertial + environment)
  };

  /**
   * @brief Partition constraints into independent contact islands
   *
   * Implements union-find over inertial body indices to determine connected
   * components. Environment bodies (index >= numInertialBodies) do NOT create
   * connectivity between inertial bodies — two cubes touching the same floor
   * form two separate islands because the floor has infinite mass and cannot
   * transmit impulses between them.
   *
   * The constraint ordering within each island preserves the relative order
   * from the input list, which maintains the CC-before-paired-FC interleaving
   * required by ConstraintSolver.
   *
   * @param constraints All constraints for the current frame (non-owning pointers)
   * @param numInertialBodies Number of inertial (dynamic) bodies. Environment
   *        body indices satisfy: bodyIndex >= numInertialBodies.
   * @return List of independent islands. Each island contains its constraint
   *         subset and all unique body indices (including environment indices)
   *         referenced by those constraints. Returns empty vector if
   *         constraints is empty.
   */
  [[nodiscard]] static std::vector<Island> buildIslands(
    const std::vector<Constraint*>& constraints,
    size_t numInertialBodies);

  ConstraintIslandBuilder() = delete;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINT_ISLAND_BUILDER_HPP
