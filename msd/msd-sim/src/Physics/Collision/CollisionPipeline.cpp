// Ticket: 0044_collision_pipeline_integration
// Design: docs/designs/0044_collision_pipeline_integration/design.md

#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <utility>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// Environment ID flag: high bit set to distinguish from inertial IDs
constexpr uint32_t kEnvIdFlag = 0x80000000u;

CollisionPipeline::CollisionPipeline() : collisionHandler_{1e-6}
{
}

void CollisionPipeline::advanceFrame()
{
  contactCache_.advanceFrame();
}

void CollisionPipeline::expireOldEntries(uint32_t maxAge)
{
  contactCache_.expireOldEntries(maxAge);
}

bool CollisionPipeline::hadCollisions() const
{
  return collisionOccurred_;
}

void CollisionPipeline::execute(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets,
  double dt)
{
  // Clear frame-persistent data at start (fail-fast for stale data)
  clearFrameData();

  const size_t numInertial = inertialAssets.size();
  const size_t numEnvironment = environmentalAssets.size();
  const size_t numBodies = numInertial + numEnvironment;

  // Early return for empty scenes or invalid timestep
  if (numBodies == 0 || dt <= 0.0)
  {
    collisionOccurred_ = false;
    return;
  }

  // ===== Phase 1: Collision Detection =====
  detectCollisions(inertialAssets, environmentalAssets);

  // Update collision-active flag for energy tracking
  collisionOccurred_ = !collisions_.empty();

  // Early return if no collisions
  if (collisions_.empty())
  {
    // Clear at end to leave pipeline empty when idle (prevents dangling refs)
    clearFrameData();
    return;
  }

  // ===== Phase 2: Create Contact Constraints =====
  createConstraints(inertialAssets, environmentalAssets, dt);

  // Early return if no constraints created
  if (constraints_.empty())
  {
    clearFrameData();
    return;
  }

  // ===== Phase 3: Assemble Solver Input Arrays =====
  assembleSolverInput(inertialAssets, environmentalAssets, numBodies);

  // ===== Phase 4: Solve Contact Constraint System with Warm-Starting =====
  auto solveResult = solveConstraintsWithWarmStart(dt);

  // ===== Phase 5: Apply Constraint Forces to Inertial Bodies =====
  applyForces(inertialAssets, solveResult);

  // ===== Phase 6: Position Correction =====
  correctPositions(inertialAssets, environmentalAssets, numBodies);

  // Clear at end to leave pipeline empty when idle (prevents dangling refs)
  clearFrameData();
}

void CollisionPipeline::detectCollisions(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets)
{
  const size_t numInertial = inertialAssets.size();
  const size_t numEnvironment = environmentalAssets.size();

  // Inertial vs Inertial (O(n²) pairwise)
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t j = i + 1; j < numInertial; ++j)
    {
      auto result =
        collisionHandler_.checkCollision(inertialAssets[i], inertialAssets[j]);
      if (!result)
      {
        continue;
      }

      double const combinedE = contact_constraint_factory::combineRestitution(
        inertialAssets[i].getCoefficientOfRestitution(),
        inertialAssets[j].getCoefficientOfRestitution());

      collisions_.push_back({i,
                             j,
                             inertialAssets[i].getInstanceId(),
                             inertialAssets[j].getInstanceId(),
                             *result,
                             combinedE});
    }
  }

  // Inertial vs Environment
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t e = 0; e < numEnvironment; ++e)
    {
      auto result = collisionHandler_.checkCollision(inertialAssets[i],
                                                     environmentalAssets[e]);
      if (!result)
      {
        continue;
      }

      double const combinedE = contact_constraint_factory::combineRestitution(
        inertialAssets[i].getCoefficientOfRestitution(),
        environmentalAssets[e].getCoefficientOfRestitution());

      // Environment body index offset by numInertial
      collisions_.emplace_back(
        CollisionPair{.bodyAIndex = i,
                      .bodyBIndex = numInertial + e,
                      .bodyAId = inertialAssets[i].getInstanceId(),
                      .bodyBId = environmentalAssets[e].getInstanceId() |
                                 kEnvIdFlag,
                      .result = std::move(*result),
                      .restitution = combinedE});
    }
  }
}

void CollisionPipeline::createConstraints(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets,
  double /* dt */)
{
  const size_t numInertial = inertialAssets.size();

  for (size_t p = 0; p < collisions_.size(); ++p)
  {
    const auto& pair = collisions_[p];
    size_t const rangeStart = constraints_.size();

    // Extract states based on body index mapping
    const InertialState& stateA =
      (pair.bodyAIndex < numInertial)
        ? inertialAssets[pair.bodyAIndex].getInertialState()
        : environmentalAssets[pair.bodyAIndex - numInertial].getInertialState();

    const InertialState& stateB =
      (pair.bodyBIndex < numInertial)
        ? inertialAssets[pair.bodyBIndex].getInertialState()
        : environmentalAssets[pair.bodyBIndex - numInertial].getInertialState();

    const Coordinate& comA = stateA.position;
    const Coordinate& comB = stateB.position;

    // Create constraints via factory
    auto constraints =
      contact_constraint_factory::createFromCollision(pair.bodyAIndex,
                                                      pair.bodyBIndex,
                                                      pair.result,
                                                      stateA,
                                                      stateB,
                                                      comA,
                                                      comB,
                                                      pair.restitution);

    // Move constraints into pipeline storage
    for (auto& c : constraints)
    {
      constraints_.push_back(std::move(c));
    }

    // Track constraint range for cache mapping
    pairRanges_.push_back(
      {rangeStart, constraints_.size() - rangeStart, p});
  }
}

void CollisionPipeline::assembleSolverInput(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets,
  size_t numBodies)
{
  // Reserve capacity for efficiency
  states_.reserve(numBodies);
  inverseMasses_.reserve(numBodies);
  inverseInertias_.reserve(numBodies);

  // Gather inertial asset data
  for (auto& asset : inertialAssets)
  {
    states_.push_back(std::cref(asset.getInertialState()));
    inverseMasses_.push_back(asset.getInverseMass());
    inverseInertias_.push_back(asset.getInverseInertiaTensorWorld());
  }

  // Gather environmental asset data
  for (const auto& envAsset : environmentalAssets)
  {
    states_.push_back(std::cref(envAsset.getInertialState()));
    inverseMasses_.push_back(
      msd_sim::AssetEnvironment::getInverseMass());  // 0.0
    inverseInertias_.push_back(
      msd_sim::AssetEnvironment::getInverseInertiaTensor());  // Zero matrix
  }
}

ConstraintSolver::MultiBodySolveResult
CollisionPipeline::solveConstraintsWithWarmStart(double dt)
{
  // Build non-owning Constraint* vector for solver
  constraintPtrs_.reserve(constraints_.size());
  for (auto& c : constraints_)
  {
    constraintPtrs_.push_back(c.get());
  }

  // ===== Phase 3.5: Warm-Start from Contact Cache =====
  const size_t totalConstraints = constraints_.size();
  Eigen::VectorXd initialLambda =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(totalConstraints));

  // Helper: extract contact world-space points for a collision pair's
  // constraint range. Contact point ≈ comA + leverArmA.
  auto extractContactPoints =
    [this](const PairConstraintRange& range,
           const CollisionPair& pair) -> std::vector<Coordinate>
  {
    std::vector<Coordinate> points;
    points.reserve(range.count);
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      const auto& contact = constraints_[range.startIdx + ci];
      const InertialState& stateA = states_[pair.bodyAIndex].get();
      points.push_back(Coordinate{stateA.position + contact->getLeverArmA()});
    }
    return points;
  };

  for (const auto& range : pairRanges_)
  {
    const auto& pair = collisions_[range.pairIdx];

    auto currentPoints = extractContactPoints(range, pair);

    // Look up cache
    Vector3D const normalVec{pair.result.normal.x(),
                             pair.result.normal.y(),
                             pair.result.normal.z()};
    auto cachedLambdas = contactCache_.getWarmStart(
      pair.bodyAId, pair.bodyBId, normalVec, currentPoints);

    if (!cachedLambdas.empty() && cachedLambdas.size() == range.count)
    {
      for (size_t ci = 0; ci < range.count; ++ci)
      {
        initialLambda(static_cast<Eigen::Index>(range.startIdx + ci)) =
          cachedLambdas[ci];
      }
    }
  }

  // ===== Phase 4: Solve with warm-starting =====
  auto solveResult = constraintSolver_.solveWithContacts(constraintPtrs_,
                                                          states_,
                                                          inverseMasses_,
                                                          inverseInertias_,
                                                          states_.size(),
                                                          dt,
                                                          initialLambda);

  // ===== Phase 4.5: Update Contact Cache =====
  for (const auto& range : pairRanges_)
  {
    const auto& pair = collisions_[range.pairIdx];

    std::vector<double> solvedLambdas;
    solvedLambdas.reserve(range.count);
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      solvedLambdas.push_back(
        solveResult.lambdas(static_cast<Eigen::Index>(range.startIdx + ci)));
    }

    auto contactPoints = extractContactPoints(range, pair);

    Vector3D const normalVec{pair.result.normal.x(),
                             pair.result.normal.y(),
                             pair.result.normal.z()};
    contactCache_.update(
      pair.bodyAId, pair.bodyBId, normalVec, solvedLambdas, contactPoints);
  }

  return solveResult;
}

void CollisionPipeline::applyForces(
  std::span<AssetInertial> inertialAssets,
  const ConstraintSolver::MultiBodySolveResult& solveResult)
{
  // Environment bodies (indices >= numInertial) are skipped because they
  // have infinite mass and cannot be moved.
  const size_t numInertial = inertialAssets.size();

  for (size_t k = 0; k < numInertial; ++k)
  {
    const auto& forces = solveResult.bodyForces[k];

    // Skip bodies with no constraint force
    if (forces.linearForce.norm() < 1e-12 &&
        forces.angularTorque.norm() < 1e-12)
    {
      continue;
    }

    inertialAssets[k].applyForce(forces.linearForce);
    inertialAssets[k].applyTorque(forces.angularTorque);
  }
}

void CollisionPipeline::correctPositions(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets,
  size_t numBodies)
{
  const size_t numInertial = inertialAssets.size();

  // Build mutable state pointer list for position correction
  std::vector<InertialState*> mutableStates;
  mutableStates.reserve(numBodies);
  for (auto& asset : inertialAssets)
  {
    mutableStates.push_back(&asset.getInertialState());
  }
  for (auto& envAsset : environmentalAssets)
  {
    // Environment states are conceptually immutable, but position corrector
    // handles this via zero inverse mass (no position change applied)
    mutableStates.push_back(
      const_cast<InertialState*>(&envAsset.getInertialState()));
  }

  positionCorrector_.correctPositions(constraintPtrs_,
                                      mutableStates,
                                      inverseMasses_,
                                      inverseInertias_,
                                      numBodies,
                                      numInertial,
                                      /* dt = */ 0.016);  // Placeholder, not used in position correction
}

void CollisionPipeline::clearFrameData()
{
  collisions_.clear();
  constraints_.clear();
  states_.clear();
  inverseMasses_.clear();
  inverseInertias_.clear();
  constraintPtrs_.clear();
  pairRanges_.clear();
}

}  // namespace msd_sim
