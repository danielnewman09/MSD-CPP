// Ticket: 0036_collision_pipeline_extraction
// Design: docs/designs/0036_collision_pipeline_extraction/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP
#define MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP

#include <Eigen/Dense>

#include <memory>
#include <span>
#include <vector>

#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Orchestrates collision response pipeline
 *
 * Extracts the collision detection, constraint creation, solver invocation,
 * and force application logic from WorldModel::updateCollisions() into a
 * dedicated orchestrator class with independently testable phases.
 *
 * The pipeline executes five distinct phases:
 * 1. Collision Detection — O(n²) pairwise narrow-phase checks
 * 2. Contact Constraint Creation — Factory calls to build ContactConstraint
 * objects
 * 3. Solver Input Assembly — Gathering states, masses, inertias into
 * solver-compatible arrays
 * 4. Constraint Solving — Invoking ConstraintSolver (Active Set Method or
 * ECOS)
 * 5. Force Application — Applying solved constraint forces back to inertial
 * bodies
 *
 * Design rationale: Separates concerns (WorldModel manages simulation
 * lifecycle, CollisionPipeline implements collision response algorithm),
 * improves testability (each phase can be unit-tested independently), and
 * enhances extensibility (easier to add broadphase culling, warm-starting, or
 * friction).
 *
 * Thread safety: Not thread-safe (contains mutable state in member vectors)
 *
 * Error handling:
 * - Empty asset spans handled gracefully (early return)
 * - Solver non-convergence propagated from ConstraintSolver
 * - No exceptions thrown for normal operation
 *
 * @see
 * docs/designs/0036_collision_pipeline_extraction/0036_collision_pipeline_extraction.puml
 * @ticket 0036_collision_pipeline_extraction
 */
class CollisionPipeline
{
public:
  /**
   * @brief Construct collision pipeline with handler and solver references
   *
   * @param collisionHandler Collision detection subsystem (GJK/EPA)
   * @param constraintSolver Contact constraint solver (ASM/ECOS)
   */
  explicit CollisionPipeline();

  /**
   * @brief Execute full collision response pipeline
   *
   * Performs:
   * 1. Collision detection (O(n²) pairwise narrow phase)
   * 2. Contact constraint creation via ContactConstraintFactory
   * 3. Solver input assembly (states, masses, inertias)
   * 4. Constraint solving (ASM or ECOS based on friction presence)
   * 5. Force application to inertial bodies
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
   * @brief Phase 4: Solve contact constraint system
   *
   * Invokes ConstraintSolver::solveWithContacts() to compute contact impulses.
   *
   * @param dt Timestep [s]
   * @return MultiBodySolveResult with per-body forces
   */
  ConstraintSolver::MultiBodySolveResult solveConstraints(double dt);

  /**
   * @brief Phase 5: Apply constraint forces to inertial bodies
   *
   * Calls applyForce() and applyTorque() on each inertial asset.
   * Skips environment bodies (infinite mass).
   * Skips forces below threshold (1e-12) per current implementation.
   *
   * @param inertialAssets Dynamic objects (mutable span)
   * @param solveResult Solver output with per-body forces
   */
  static void applyForces(
    std::span<AssetInertial> inertialAssets,
    const ConstraintSolver::MultiBodySolveResult& solveResult);

private:
  /**
   * @brief Clear frame-persistent data
   *
   * Clears all member vectors to prepare for new frame or leave pipeline
   * empty when idle. Called at both start and end of execute().
   * Preserves capacity to avoid reallocation (NFR-1).
   *
   * Prevents dangling references: If WorldModel modifies asset vectors
   * between frames (e.g., removes a body), cached references in states_
   * and constraintPtrs_ would become dangling. Clearing at end ensures
   * the pipeline is "empty" and safe when idle.
   */
  void clearFrameData();

  CollisionHandler collisionHandler_;
  ConstraintSolver constraintSolver_;

  // Frame-persistent storage (reused across frames for NFR-1: zero additional
  // allocations)
  struct CollisionPair
  {
    size_t bodyAIndex;
    size_t bodyBIndex;
    CollisionResult result;
    double restitution;
  };

  std::vector<CollisionPair> collisions_;
  std::vector<std::unique_ptr<ContactConstraint>> constraints_;
  std::vector<std::reference_wrapper<const InertialState>> states_;
  std::vector<double> inverseMasses_;
  std::vector<Eigen::Matrix3d> inverseInertias_;
  std::vector<TwoBodyConstraint*> constraintPtrs_;

  // Friend declarations for unit testing
  friend class CollisionPipelineTest;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_PIPELINE_HPP
