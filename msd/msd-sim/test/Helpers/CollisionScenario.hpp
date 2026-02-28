// Ticket: 0087_collision_solver_lambda_test_suite
// Design: docs/designs/0087_collision_solver_lambda_test_suite/design.md

#ifndef MSD_SIM_TEST_HELPERS_COLLISION_SCENARIO_HPP
#define MSD_SIM_TEST_HELPERS_COLLISION_SCENARIO_HPP

#include <memory>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim::test
{

/**
 * @brief Owns the assets, hulls, and pipeline for a single isolated
 * collision scenario.
 *
 * Provides stepOnce() which calls the pipeline's protected sub-phases
 * in sequence via friend access and returns the ConstraintSolver::SolveResult
 * for direct lambda inspection.
 *
 * CollisionPipeline deletes all copy and move operations, so CollisionScenario
 * owns it via std::unique_ptr<CollisionPipeline>. This makes CollisionScenario
 * itself movable (the unique_ptr transfers ownership) while keeping the
 * pipeline non-movable at the type level.
 *
 * CollisionScenarioBuilder factory methods return CollisionScenario by value;
 * NRVO applies, and if a move is needed the unique_ptr member enables it.
 *
 * Thread safety: Not thread-safe; single-threaded test use only.
 *
 * @ticket 0087_collision_solver_lambda_test_suite
 */
class CollisionScenario
{
public:
  /**
   * @brief Construct an empty scenario with the given time step.
   * @param dt Simulation time step in seconds (default: 0.016s = 60 FPS)
   */
  explicit CollisionScenario(double dt = 0.016);

  // CollisionPipeline deletes all copy/move — own via unique_ptr so
  // CollisionScenario itself can be moved (unique_ptr transfers ownership).
  CollisionScenario(const CollisionScenario&) = delete;
  CollisionScenario& operator=(const CollisionScenario&) = delete;
  CollisionScenario(CollisionScenario&&) noexcept = default;
  CollisionScenario& operator=(CollisionScenario&&) noexcept = default;
  ~CollisionScenario() = default;

  // ===== Spawn helpers =====

  /**
   * @brief Add a dynamic inertial body to the scenario.
   *
   * The hull is built from the provided vertices. The scenario owns the hull
   * storage and the asset. The returned index can be used with
   * getInertialState() after stepOnce().
   *
   * @param points Convex hull vertices in local frame
   * @param frame Initial reference frame (position + orientation)
   * @param mass Body mass [kg]
   * @param restitution Coefficient of restitution [0, 1] (default: 0.0)
   * @param friction Friction coefficient [0, inf) (default: 0.5)
   * @param velocity Initial linear velocity (default: {0,0,0})
   * @return Index for later use with getInertialState()
   */
  size_t addInertial(const std::vector<Coordinate>& points,
                     const ReferenceFrame& frame,
                     double mass,
                     double restitution = 0.0,
                     double friction = 0.5,
                     const Coordinate& velocity = Coordinate{0.0, 0.0, 0.0});

  /**
   * @brief Add a static environment body to the scenario.
   *
   * Environment bodies are immovable (infinite mass). Restitution defaults
   * to 0.0; friction to 0.5.
   *
   * @param points Convex hull vertices in local frame
   * @param frame Reference frame (position + orientation)
   * @param restitution Coefficient of restitution [0, 1] (default: 0.0)
   * @param friction Friction coefficient [0, inf) (default: 0.5)
   */
  void addEnvironment(const std::vector<Coordinate>& points,
                      const ReferenceFrame& frame,
                      double restitution = 0.0,
                      double friction = 0.5);

  // ===== Pipeline execution =====

  /**
   * @brief Run one pipeline step and capture the SolveResult.
   *
   * Calls the protected pipeline sub-phases in order:
   *   1. detectCollisions()
   *   2. createConstraints()
   *   3. assembleSolverInput()
   *   4. solveConstraintsWithWarmStart()  → captures SolveResult
   *   5. applyForces()                    (if applyForces == true)
   *   6. correctPositions()               (if correctPositions == true)
   *
   * The solved result is stored internally and accessible via
   * getLastSolveResult() for tests that need to re-inspect after
   * further state queries.
   *
   * @param applyForces If true, apply constraint forces to inertial bodies
   *        after solving (default: true)
   * @param correctPositions If true, run split-impulse position correction
   *        after solving (default: false)
   * @return SolveResult with lambdas, bodyForces, converged flag
   */
  ConstraintSolver::SolveResult stepOnce(bool applyForces = true,
                                         bool correctPositions = false);

  /**
   * @brief Re-fetch the SolveResult from the most recent stepOnce() call.
   *
   * For tests that want to inspect the result after additional state queries
   * without re-running the pipeline.
   *
   * @return Const reference to stored SolveResult
   */
  const ConstraintSolver::SolveResult& getLastSolveResult() const;

  // ===== State access =====

  /**
   * @brief Get the kinematic state of an inertial body by index.
   * @param idx Index returned by addInertial()
   * @return Const reference to InertialState
   * @throws std::out_of_range if idx >= number of inertial bodies
   */
  const InertialState& getInertialState(size_t idx) const;

  /**
   * @brief Get the inertial asset by index (mutable, for pre-step setup).
   * @param idx Index returned by addInertial()
   * @return Mutable reference to AssetInertial
   */
  AssetInertial& getInertialAsset(size_t idx);

  /**
   * @brief Get a const reference to the underlying pipeline.
   *
   * Provides read access to pipeline diagnostics (getCollisions(),
   * getSolverData(), hadCollisions()) after stepOnce().
   */
  const CollisionPipeline& pipeline() const;

  /**
   * @brief Return true if the most recent stepOnce() detected collisions.
   */
  bool hadCollisions() const;

  /**
   * @brief Return the number of inertial bodies in this scenario.
   */
  size_t numInertials() const;

private:
  double dt_;

  // Hull and asset storage — both use std::vector pre-reserved to
  // kMaxBodies (defined in CollisionScenario.cpp) at construction time.
  // Pre-reservation prevents any reallocation; without it, push_back would
  // move elements to a new buffer, invalidating the non-owning ConvexHull
  // references inside AssetInertial / AssetEnvironment.
  // The vectors also remain contiguous, which is required for building
  // std::span<AssetInertial> in stepOnce().
  std::vector<ConvexHull> inertialHulls_;
  std::vector<ConvexHull> envHulls_;

  std::vector<AssetInertial> inertials_;
  std::vector<AssetEnvironment> environments_;

  // Pipeline owned via unique_ptr (non-movable type)
  std::unique_ptr<CollisionPipeline> pipeline_;

  // Most recent SolveResult captured in stepOnce()
  ConstraintSolver::SolveResult lastResult_;

  // Running instance-ID counter (globally unique within this scenario)
  uint32_t nextInstanceId_{1};
};

}  // namespace msd_sim::test

#endif  // MSD_SIM_TEST_HELPERS_COLLISION_SCENARIO_HPP
