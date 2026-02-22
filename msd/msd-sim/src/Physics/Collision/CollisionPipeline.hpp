// Ticket: 0044_collision_pipeline_integration
// Ticket: 0071g_constraint_pool_allocation
// Design: docs/designs/0044_collision_pipeline_integration/design.md
// Design: docs/designs/0071g_constraint_pool_allocation/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP
#define MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP

#include <Eigen/Dense>

#include <cstdint>
#include <span>
#include <unordered_set>
#include <vector>

#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/Constraints/ContactCache.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintIslandBuilder.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintPool.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/PositionCorrector.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Orchestrates collision response pipeline with warm-starting and
 * position correction
 *
 * Extracts the collision detection, constraint creation, solver invocation,
 * cache management, and position correction logic from WorldModel into a
 * dedicated orchestrator class with independently testable phases.
 *
 * The pipeline executes the following phases:
 * 1. Collision Detection — O(n²) pairwise narrow-phase checks
 * 2. Contact Constraint Creation — Factory calls to build ContactConstraint
 *    objects
 * 3. Solver Input Assembly — Gathering states, masses, inertias
 * 3.5. Warm-Starting — Query ContactCache for previous frame's lambdas
 * 4. Constraint Solving — Active Set Method with initial lambda
 * 4.5. Cache Update — Store solved lambdas in ContactCache
 * 5. Force Application — Apply solved constraint forces to inertial bodies
 * 6. Position Correction — Split-impulse penetration correction
 *
 * Design rationale: Separates concerns (WorldModel manages simulation
 * lifecycle, CollisionPipeline implements collision response algorithm),
 * improves testability (each phase can be unit-tested independently), and
 * enhances extensibility (easier to add broadphase culling or friction).
 *
 * Thread safety: Not thread-safe (contains mutable state in member vectors)
 *
 * Error handling:
 * - Empty asset spans handled gracefully (early return)
 * - Solver non-convergence propagated from ConstraintSolver
 * - No exceptions thrown for normal operation
 *
 * @see
 * docs/designs/0044_collision_pipeline_integration/0044_collision_pipeline_integration.puml
 * @ticket 0044_collision_pipeline_integration
 */
class CollisionPipeline
{
public:
  /**
   * @brief Solver diagnostic data captured after constraint solving
   *
   * @ticket 0056b_collision_pipeline_data_extraction
   */
  struct SolverData
  {
    int iterations{0};
    double residual{0.0};
    bool converged{false};
    size_t numConstraints{0};
    size_t numContacts{0};
  };

  /**
   * @brief Construct collision pipeline with handler and solver references
   *
   * @param collisionHandler Collision detection subsystem (GJK/EPA)
   * @param constraintSolver Contact constraint solver (ASM/FrictionCone)
   */
  explicit CollisionPipeline();

  /**
   * @brief Execute full collision response pipeline
   *
   * Performs:
   * 1. Collision detection (O(n²) pairwise narrow phase)
   * 2. Contact constraint creation via ContactConstraintFactory
   * 3. Solver input assembly (states, masses, inertias)
   * 3.5. Warm-starting query from ContactCache
   * 4. Constraint solving with initial lambda (Active Set Method)
   * 4.5. Cache update with solved lambdas
   * 5. Force application to inertial bodies
   * 6. Position correction via PositionCorrector
   *
   * Frame data is cleared at both the start and end of this method to prevent
   * dangling references when the pipeline is idle between frames.
   *
   * @param inertialAssets Dynamic objects (non-owning span)
   * @param environmentalAssets Static objects (non-owning span)
   * @param dt Timestep [s]
   */
  void execute(std::span<AssetInertial> inertialAssets,
               std::span<const AssetEnvironment> environmentalAssets,
               double dt);

  /**
   * @brief Advance contact cache age by one frame
   *
   * Must be called once per frame before execute() to increment cache age.
   * This enables contact persistence tracking for warm-starting.
   *
   * @ticket 0044_collision_pipeline_integration
   */
  void advanceFrame();

  /**
   * @brief Expire stale contact cache entries
   *
   * Removes cached contacts older than maxAge frames. Typically called after
   * advanceFrame() and before execute() each frame.
   *
   * @param maxAge Maximum age in frames (default: 10)
   * @ticket 0044_collision_pipeline_integration
   */
  void expireOldEntries(uint32_t maxAge = 10);

  /**
   * @brief Check if collisions occurred this frame
   *
   * Returns true if collision detection found any colliding pairs.
   * Used for energy tracking diagnostics.
   *
   * @return true if collisions occurred, false otherwise
   * @ticket 0044_collision_pipeline_integration
   */
  bool hadCollisions() const;

  /**
   * @brief Collision pair data from last execute()
   *
   * Holds body indices, instance IDs, CollisionResult, and material
   * coefficients for a single colliding pair.
   *
   * Data is valid from end of execute() until start of next execute().
   * collisions_ is owned by the pipeline and cleared at the start of
   * each frame — no intermediate snapshot copy needed.
   *
   * @ticket 0056b_collision_pipeline_data_extraction
   */
  struct CollisionPair
  {
    size_t bodyAIndex;
    size_t bodyBIndex;
    uint32_t bodyAId;  // Instance ID for cache keying
    uint32_t bodyBId;  // Instance ID for cache keying
    CollisionResult result;
    double restitution;
    double frictionCoefficient;
  };

  /**
   * @brief Get collision pairs from last execute()
   *
   * Returns const reference to collision pairs detected during the most
   * recent execute() call. Data is valid from end of execute() until
   * start of next execute().
   *
   * @return const reference to collision pair vector
   * @ticket 0056b_collision_pipeline_data_extraction
   */
  const std::vector<CollisionPair>& getCollisions() const;

  /**
   * @brief Get solver diagnostics from last execute()
   *
   * @return const reference to solver diagnostic data
   * @ticket 0056b_collision_pipeline_data_extraction
   */
  const SolverData& getSolverData() const;

  /**
   * @brief Record constraint states to DataRecorder
   *
   * Iterates all constraints (both ContactConstraint and FrictionConstraint),
   * calls constraint->recordState() with a DataRecorderVisitor, and buffers
   * the typed records to the appropriate DAOs.
   *
   * Uses visitor pattern for type-safe dispatching without dynamic_cast.
   * Each constraint builds its specific record (ContactConstraintRecord or
   * FrictionConstraintRecord) and dispatches to visitor.visit(record).
   *
   * Must be called AFTER execute() and BEFORE clearEphemeralState() to ensure
   * constraints are still alive.
   *
   * @param recorder DataRecorder instance to buffer records to
   * @param frameId Frame ID for FK references in constraint records
   *
   * Thread safety: Not thread-safe (calls DataRecorder methods)
   * Error handling: Exceptions from DAO operations propagate to caller
   *
   * @see DataRecorderVisitor
   * @see Constraint::recordState()
   * @ticket 0057_contact_tangent_recording
   */
  void recordConstraints(class DataRecorder& recorder, uint32_t frameId) const;

  CollisionPipeline(const CollisionPipeline&) = delete;
  CollisionPipeline& operator=(const CollisionPipeline&) = delete;
  CollisionPipeline(CollisionPipeline&&) = delete;
  CollisionPipeline& operator=(CollisionPipeline&&) = delete;
  ~CollisionPipeline() = default;

protected:
  // Sub-phase methods accessible for unit testing (friend test classes)

  /**
   * @brief Phase 1: Collision detection (O(n²) pairwise)
   *
   * Populates collisions_ member with detected collision pairs
   * and their metadata (body indices, restitution).
   *
   * Performs:
   * - Inertial vs Inertial: O(n²) pairwise checks
   * - Inertial vs Environment: O(n*m) checks
   *
   * Early return if no collisions detected.
   */
  void detectCollisions(std::span<AssetInertial> inertialAssets,
                        std::span<const AssetEnvironment> environmentalAssets);

  /**
   * @brief Phase 2: Create contact constraints from collision results
   *
   * Populates constraints_ member via ContactConstraintFactory.
   * One constraint per contact point in manifold.
   *
   * Early return if collisions_ is empty.
   *
   * @param inertialAssets Dynamic objects (non-owning span)
   * @param environmentalAssets Static objects (non-owning span)
   * @param dt Timestep [s]
   */
  void createConstraints(std::span<AssetInertial> inertialAssets,
                         std::span<const AssetEnvironment> environmentalAssets,
                         double dt);

  /**
   * @brief Phase 3: Assemble solver input arrays
   *
   * Populates states_, inverseMasses_, inverseInertias_, constraintPtrs_
   * from asset data in solver-compatible layout.
   *
   * Body indexing convention (preserved for solver compatibility):
   * - [0 .. numInertial-1] = inertial bodies
   * - [numInertial .. numInertial+numEnv-1] = environment bodies
   *
   * @param inertialAssets Dynamic objects (non-owning span)
   * @param environmentalAssets Static objects (non-owning span)
   * @param numBodies Total number of bodies (inertial + environmental)
   */
  void assembleSolverInput(
    std::span<AssetInertial> inertialAssets,
    std::span<const AssetEnvironment> environmentalAssets,
    size_t numBodies);

  /**
   * @brief Phase 4: Solve contact constraint system with warm-starting
   *
   * Invokes ConstraintSolver::solve() with initial lambda from
   * ContactCache for faster convergence on persistent contacts.
   *
   * @param dt Timestep [s]
   * @return SolveResult with per-body forces
   * @ticket 0045_constraint_solver_unification
   */
  ConstraintSolver::SolveResult solveConstraintsWithWarmStart(double dt);

  /**
   * @brief Phase 5: Apply constraint forces to inertial bodies
   *
   * Calls applyForce() and applyTorque() on each inertial asset.
   * Skips environment bodies (infinite mass).
   * Skips forces below threshold (1e-12) per current implementation.
   *
   * @param inertialAssets Dynamic objects (mutable span)
   * @param solveResult Solver output with per-body forces
   * @ticket 0045_constraint_solver_unification
   */
  static void applyForces(
    std::span<AssetInertial> inertialAssets,
    const ConstraintSolver::SolveResult& solveResult);

  /**
   * @brief Phase 6: Position correction via split-impulse method
   *
   * Corrects penetrating contacts using pseudo-velocities without injecting
   * kinetic energy into real velocities.
   *
   * @param inertialAssets Dynamic objects (mutable span)
   * @param environmentalAssets Static objects (non-owning span)
   * @param numBodies Total body count (inertial + environmental)
   */
  void correctPositions(std::span<AssetInertial> inertialAssets,
                        std::span<const AssetEnvironment> environmentalAssets,
                        size_t numBodies);

private:
  /**
   * @brief Clear ephemeral solver state (references, pointers, constraints)
   *
   * Clears vectors that hold references or pointers to external objects
   * (states_, etc.) to prevent dangling references when assets are modified
   * between frames. Also clears constraints.
   *
   * Does NOT clear collisions_ — those are value types (owned by pipeline)
   * and are cleared at the start of the next execute() call. This allows
   * WorldModel to read collision data between frames without an intermediate
   * snapshot copy.
   *
   * Preserves capacity to avoid reallocation (NFR-1).
   *
   * Ticket: 0058_constraint_ownership_cleanup
   */
  void clearEphemeralState();

  /**
   * @brief Propagate solved normal lambdas to FrictionConstraints.
   *
   * After the solver runs, each FrictionConstraint needs the corresponding
   * ContactConstraint's solved lambda so that recordState() can write the
   * actual normal force value (in Newtons) for visualization.
   *
   * Ticket: 0057_contact_tangent_recording
   */
  void propagateSolvedLambdas(const ConstraintSolver::SolveResult& solveResult);

  /**
   * @brief Build solver view (all constraints as Constraint*)
   *
   * Generates a vector of raw pointers for ConstraintSolver::solve().
   * The view is temporary (not stored) and valid only within the calling
   * scope.
   *
   * @param interleaved If true (friction active), returns [CC_0, FC_0, CC_1,
   * FC_1, ...]. If false, returns all constraints in storage order.
   * @return Vector of non-owning Constraint* pointers
   *
   * Ticket: 0058_constraint_ownership_cleanup
   */
  std::vector<Constraint*> buildSolverView(bool interleaved) const;

  /**
   * @brief Build contact-only view (ContactConstraint* via dynamic_cast)
   *
   * Generates a vector of Constraint* pointers containing only
   * ContactConstraint instances for PositionCorrector::correctPositions().
   * Filters allConstraints_ using dynamic_cast to extract only
   * ContactConstraint instances.
   *
   * The view is temporary (not stored) and valid only within the calling
   * scope.
   *
   * @return Vector of non-owning Constraint* pointers (ContactConstraints only)
   *
   * Ticket: 0058_constraint_ownership_cleanup
   */
  std::vector<Constraint*> buildContactView() const;

  /**
   * @brief Map constraint index to collision pair index
   *
   * Iterates pairRanges_ to find which collision pair owns the given
   * constraint index. Used by recordConstraints() to extract body IDs.
   *
   * @param constraintIdx Index into allConstraints_
   * @return Index into collisions_ vector
   * @throws std::logic_error if constraint index not found in pairRanges_
   *
   * Ticket: 0057_contact_tangent_recording
   */
  size_t findPairIndexForConstraint(size_t constraintIdx) const;

  CollisionHandler collisionHandler_;
  ConstraintSolver constraintSolver_;

  // Collision results — owned by pipeline, valid from end of execute()
  // until start of next execute(). WorldModel reads directly via
  // getCollisions() — no snapshot copy needed.
  std::vector<CollisionPair> collisions_;

  // Solver diagnostics — lightweight, captured after solve
  SolverData solverData_;

  // Ephemeral solver state (cleared at end of execute to prevent dangling refs)
  // Ticket: 0058_constraint_ownership_cleanup
  // Ticket: 0071g_constraint_pool_allocation
  //
  // constraintPool_ must be declared BEFORE allConstraints_ to ensure the pool
  // outlives the non-owning pointer view during destruction.
  ConstraintPool constraintPool_;                  // Owns constraint objects (pooled)
  std::vector<Constraint*> allConstraints_;        // Non-owning view into pool
  std::vector<std::reference_wrapper<const InertialState>> states_;
  std::vector<double> inverseMasses_;
  std::vector<Eigen::Matrix3d> inverseInertias_;

  // Cache and position correction (ticket 0044)
  ContactCache contactCache_;
  PositionCorrector positionCorrector_;

  // Track collision-active status (ticket 0044)
  bool collisionOccurred_{false};

  // Cache interaction data structures (ticket 0044)
  struct PairConstraintRange
  {
    size_t startIdx;
    size_t count;
    size_t pairIdx;
  };
  std::vector<PairConstraintRange> pairRanges_;

  // Ticket: 0071e_trivial_allocation_elimination
  // Per-pair contact points, indexed by collision pair index.
  // Pre-computed once in solveConstraintsWithWarmStart() and reused for
  // both the warm-start query and the cache-update loop, eliminating
  // one extractContactPoints() call per pair per frame.
  std::vector<std::vector<Coordinate>> pairContactPoints_;

  // Ticket: 0071g_constraint_pool_allocation
  // Member workspaces to eliminate per-frame local vector allocations.
  // solvedLambdas_: reused in the cache-update loop (replaces local variable)
  // islandConstraintSet_: reused per island in warm-start (replaces local variable)
  std::vector<double> solvedLambdas_;
  std::unordered_set<const Constraint*> islandConstraintSet_;

  // Friend declarations for unit testing
  friend class CollisionPipelineTest;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP
