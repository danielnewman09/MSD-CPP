// Ticket: 0036_collision_pipeline_extraction
// Design: docs/designs/0036_collision_pipeline_extraction/design.md

#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include <cstddef>
#include <functional>
#include <span>
#include <utility>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/Constraints/ConstraintSolver.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

CollisionPipeline::CollisionPipeline()
  : collisionHandler_{1e-6} 
{
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
    return;
  }

  // ===== Phase 1: Collision Detection =====
  detectCollisions(inertialAssets, environmentalAssets);

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

  // ===== Phase 4: Solve Contact Constraint System =====
  auto solveResult = solveConstraints(dt);

  // ===== Phase 5: Apply Constraint Forces to Inertial Bodies =====
  applyForces(inertialAssets, solveResult);

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

      collisions_.push_back({i, j, *result, combinedE});
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
        CollisionPair{.bodyAIndex=i, .bodyBIndex=numInertial + e, .result=std::move(*result), .restitution=combinedE});
    }
  }
}

void CollisionPipeline::createConstraints(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets,
  double /* dt */)
{
  const size_t numInertial = inertialAssets.size();

  for (const auto& pair : collisions_)
  {
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
    inverseMasses_.push_back(msd_sim::AssetEnvironment::getInverseMass());  // 0.0
    inverseInertias_.push_back(
      msd_sim::AssetEnvironment::getInverseInertiaTensor());  // Zero matrix
  }
}

ConstraintSolver::MultiBodySolveResult CollisionPipeline::solveConstraints(
  double dt)
{
  // Build non-owning TwoBodyConstraint* vector for solver
  constraintPtrs_.reserve(constraints_.size());
  for (auto& c : constraints_)
  {
    constraintPtrs_.push_back(c.get());
  }

  // Invoke solver
  return constraintSolver_.solveWithContacts(constraintPtrs_,
                                             states_,
                                             inverseMasses_,
                                             inverseInertias_,
                                             states_.size(),
                                             dt);
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

void CollisionPipeline::clearFrameData()
{
  collisions_.clear();
  constraints_.clear();
  states_.clear();
  inverseMasses_.clear();
  inverseInertias_.clear();
  constraintPtrs_.clear();
}

}  // namespace msd_sim
