// Ticket: 0044_collision_pipeline_integration
// Design: docs/designs/0044_collision_pipeline_integration/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP
#define MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP

#include <Eigen/Dense>

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/Constraints/ContactCache.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
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
   * (states_, constraintPtrs_, etc.) to prevent dangling references when
   * assets are modified between frames. Also clears constraints.
   *
   * Does NOT clear collisions_ — those are value types (owned by pipeline)
   * and are cleared at the start of the next execute() call. This allows
   * WorldModel to read collision data between frames without an intermediate
   * snapshot copy.
   *
   * Preserves capacity to avoid reallocation (NFR-1).
   */
  void clearEphemeralState();

  CollisionHandler collisionHandler_;
  ConstraintSolver constraintSolver_;

  // Collision results — owned by pipeline, valid from end of execute()
  // until start of next execute(). WorldModel reads directly via
  // getCollisions() — no snapshot copy needed.
  std::vector<CollisionPair> collisions_;

  // Solver diagnostics — lightweight, captured after solve
  SolverData solverData_;

  // Ephemeral solver state (cleared at end of execute to prevent dangling refs)
  std::vector<std::unique_ptr<ContactConstraint>> constraints_;
  std::vector<std::unique_ptr<FrictionConstraint>> frictionConstraints_;
  std::vector<std::reference_wrapper<const InertialState>> states_;
  std::vector<double> inverseMasses_;
  std::vector<Eigen::Matrix3d> inverseInertias_;
  std::vector<Constraint*> constraintPtrs_;
  std::vector<Constraint*> normalConstraintPtrs_;

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

  // Friend declarations for unit testing
  friend class CollisionPipelineTest;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP
