// Ticket: 0087_collision_solver_lambda_test_suite
// Design: docs/designs/0087_collision_solver_lambda_test_suite/design.md

#include "msd-sim/test/Helpers/CollisionScenario.hpp"

#include <span>
#include <stdexcept>

namespace msd_sim::test
{

// Maximum number of bodies in a single test scenario.
// Reserving at construction prevents reallocation, which would invalidate
// the non-owning ConvexHull references held inside AssetInertial /
// AssetEnvironment objects.
static constexpr size_t kMaxBodies{16};

// ============================================================================
// Constructor
// ============================================================================

CollisionScenario::CollisionScenario(double dt)
  : dt_{dt}
  , pipeline_{std::make_unique<CollisionPipeline>()}
{
  // Pre-reserve all storage vectors to prevent reallocation.
  // AssetInertial holds a non-owning reference to its ConvexHull.
  // If inertialHulls_ (or inertials_) ever reallocates, previously
  // stored hull references become dangling. Reserving kMaxBodies upfront
  // avoids this for all normal test scenarios.
  inertialHulls_.reserve(kMaxBodies);
  envHulls_.reserve(kMaxBodies);
  inertials_.reserve(kMaxBodies);
  environments_.reserve(kMaxBodies);
}

// ============================================================================
// Spawn helpers
// ============================================================================

size_t CollisionScenario::addInertial(const std::vector<Coordinate>& points,
                                      const ReferenceFrame& frame,
                                      double mass,
                                      double restitution,
                                      double friction,
                                      const Coordinate& velocity)
{
  // Safety check — should never trigger for normal test scenarios.
  if (inertialHulls_.size() >= kMaxBodies)
  {
    throw std::out_of_range{"CollisionScenario::addInertial: exceeded kMaxBodies"};
  }

  // Build hull first. Because inertialHulls_ is reserved to kMaxBodies,
  // push_back never reallocates, keeping all existing hull addresses stable.
  inertialHulls_.emplace_back(points);

  const size_t idx{inertials_.size()};
  const uint32_t assetId{static_cast<uint32_t>(idx)};
  const uint32_t instanceId{nextInstanceId_++};

  // Construct asset with reference to the hull we just emplaced.
  // No reallocation: inertials_ is also reserved to kMaxBodies.
  inertials_.emplace_back(assetId,
                           instanceId,
                           inertialHulls_.back(),
                           mass,
                           frame,
                           restitution,
                           friction);

  inertials_.back().getInertialState().velocity = velocity;

  return idx;
}

void CollisionScenario::addEnvironment(const std::vector<Coordinate>& points,
                                       const ReferenceFrame& frame,
                                       double restitution,
                                       double friction)
{
  if (envHulls_.size() >= kMaxBodies)
  {
    throw std::out_of_range{"CollisionScenario::addEnvironment: exceeded kMaxBodies"};
  }

  envHulls_.emplace_back(points);

  const uint32_t assetId{static_cast<uint32_t>(environments_.size())};
  const uint32_t instanceId{nextInstanceId_++};

  environments_.emplace_back(assetId,
                              instanceId,
                              envHulls_.back(),
                              frame,
                              restitution,
                              friction);
}

// ============================================================================
// Pipeline execution
// ============================================================================

ConstraintSolver::SolveResult CollisionScenario::stepOnce(
  bool doApplyForces,
  bool doCorrectPositions)
{
  // Build spans for pipeline calls.
  // std::vector<T> is contiguous, so std::span<T> construction is valid.
  std::span<AssetInertial> inertialSpan{inertials_};
  std::span<const AssetEnvironment> envSpan{environments_};

  const size_t numBodies{inertials_.size() + environments_.size()};

  // Clear collision state from any previous stepOnce() call.
  // Mirrors the clearing that execute() does at the start of each frame.
  pipeline_->collisions_.clear();
  pipeline_->clearEphemeralState();

  // Phase 1: Collision detection
  pipeline_->detectCollisions(inertialSpan, envSpan);

  // Mirror execute()'s post-detection flag update.
  // collisionOccurred_ is private; friend access allows direct assignment.
  pipeline_->collisionOccurred_ = !pipeline_->collisions_.empty();

  // Early return if no collisions — result is empty (default-constructed)
  if (pipeline_->collisions_.empty())
  {
    return lastResult_;
  }

  // Phase 2: Create contact constraints from collision results
  pipeline_->createConstraints(inertialSpan, envSpan, dt_);

  // Phase 3: Assemble solver input arrays
  pipeline_->assembleSolverInput(inertialSpan, envSpan, numBodies);

  // Phase 4: Solve with warm-start and capture SolveResult
  lastResult_ = pipeline_->solveConstraintsWithWarmStart(dt_);

  // Phase 5 (optional): Apply constraint forces to inertial bodies
  if (doApplyForces)
  {
    CollisionPipeline::applyForces(inertialSpan, lastResult_);
  }

  // Phase 6 (optional): Position correction via split-impulse
  if (doCorrectPositions)
  {
    pipeline_->correctPositions(inertialSpan, envSpan, numBodies);
  }

  return lastResult_;
}

const ConstraintSolver::SolveResult& CollisionScenario::getLastSolveResult() const
{
  return lastResult_;
}

// ============================================================================
// State access
// ============================================================================

const InertialState& CollisionScenario::getInertialState(size_t idx) const
{
  if (idx >= inertials_.size())
  {
    throw std::out_of_range{"CollisionScenario::getInertialState: index " +
                            std::to_string(idx) + " out of range [0, " +
                            std::to_string(inertials_.size()) + ")"};
  }
  return inertials_[idx].getInertialState();
}

AssetInertial& CollisionScenario::getInertialAsset(size_t idx)
{
  if (idx >= inertials_.size())
  {
    throw std::out_of_range{"CollisionScenario::getInertialAsset: index " +
                            std::to_string(idx) + " out of range [0, " +
                            std::to_string(inertials_.size()) + ")"};
  }
  return inertials_[idx];
}

const CollisionPipeline& CollisionScenario::pipeline() const
{
  return *pipeline_;
}

bool CollisionScenario::hadCollisions() const
{
  return pipeline_->hadCollisions();
}

size_t CollisionScenario::numInertials() const
{
  return inertials_.size();
}

}  // namespace msd_sim::test
