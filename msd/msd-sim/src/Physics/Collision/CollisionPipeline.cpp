// Ticket: 0044_collision_pipeline_integration
// Ticket: 0052d_solver_integration_ecos_removal
// Design: docs/designs/0044_collision_pipeline_integration/design.md
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include <cmath>
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
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
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

      // Ticket 0052d: Combined friction coefficient (geometric mean)
      double const combinedMu = std::sqrt(
        inertialAssets[i].getFrictionCoefficient() *
        inertialAssets[j].getFrictionCoefficient());

      collisions_.push_back({i,
                             j,
                             inertialAssets[i].getInstanceId(),
                             inertialAssets[j].getInstanceId(),
                             *result,
                             combinedE,
                             combinedMu});
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

      // Ticket 0052d: Combined friction coefficient (geometric mean)
      double const combinedMu = std::sqrt(
        inertialAssets[i].getFrictionCoefficient() *
        environmentalAssets[e].getFrictionCoefficient());

      // Environment body index offset by numInertial
      collisions_.emplace_back(
        CollisionPair{.bodyAIndex = i,
                      .bodyBIndex = numInertial + e,
                      .bodyAId = inertialAssets[i].getInstanceId(),
                      .bodyBId = environmentalAssets[e].getInstanceId() |
                                 kEnvIdFlag,
                      .result = std::move(*result),
                      .restitution = combinedE,
                      .frictionCoefficient = combinedMu});
    }
  }
}

void CollisionPipeline::createConstraints(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets,
  double /* dt */)
{
  const size_t numInertial = inertialAssets.size();

  // Ticket 0052d: Determine if ANY collision pair has non-zero friction.
  // If so, create FrictionConstraints for ALL contacts (1:1 correspondence).
  // If none have friction, skip FrictionConstraints entirely → ASM path.
  bool anyFriction = false;
  for (const auto& pair : collisions_)
  {
    if (pair.frictionCoefficient > 0.0)
    {
      anyFriction = true;
      break;
    }
  }

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

    // Create normal constraints via factory
    auto contactConstraints =
      contact_constraint_factory::createFromCollision(pair.bodyAIndex,
                                                      pair.bodyBIndex,
                                                      pair.result,
                                                      stateA,
                                                      stateB,
                                                      comA,
                                                      comB,
                                                      pair.restitution);

    for (size_t ci = 0; ci < contactConstraints.size(); ++ci)
    {
      const auto& cc = contactConstraints[ci];

      if (anyFriction)
      {
        // Create matching FrictionConstraint with same contact geometry.
        // Uses per-pair friction coefficient (may be 0 for frictionless pairs).
        auto fc = std::make_unique<FrictionConstraint>(
          pair.bodyAIndex,
          pair.bodyBIndex,
          pair.result.normal,
          comA + cc->getLeverArmA(),   // contactPointA = comA + leverArmA
          comB + cc->getLeverArmB(),   // contactPointB = comB + leverArmB
          comA,
          comB,
          pair.frictionCoefficient);

        frictionConstraints_.push_back(std::move(fc));
      }

      // Move contact constraint into pipeline storage
      constraints_.push_back(std::move(contactConstraints[ci]));
    }

    // Track constraint range (number of contact points, not total constraints)
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

ConstraintSolver::SolveResult
CollisionPipeline::solveConstraintsWithWarmStart(double dt)
{
  const bool hasFriction = !frictionConstraints_.empty();

  // Build constraint pointer lists
  normalConstraintPtrs_.reserve(constraints_.size());
  for (size_t i = 0; i < constraints_.size(); ++i)
  {
    normalConstraintPtrs_.push_back(constraints_[i].get());
  }

  if (hasFriction)
  {
    // Ticket 0052d: Build interleaved constraint pointer list
    // [CC_0, FC_0, CC_1, FC_1, ...]
    // constraints_ and frictionConstraints_ have 1:1 correspondence
    constraintPtrs_.reserve(constraints_.size() + frictionConstraints_.size());
    for (size_t i = 0; i < constraints_.size(); ++i)
    {
      constraintPtrs_.push_back(constraints_[i].get());
      constraintPtrs_.push_back(frictionConstraints_[i].get());
    }
  }
  else
  {
    // No friction: solver sees only ContactConstraints → ASM path
    constraintPtrs_ = normalConstraintPtrs_;
  }

  // Lambdas per contact: 3 with friction (n, t1, t2), 1 without
  const size_t lambdasPerContact = hasFriction ? 3 : 1;

  // ===== Phase 3.5: Warm-Start from Contact Cache =====
  const size_t totalRows = constraints_.size() * lambdasPerContact;
  Eigen::VectorXd initialLambda =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(totalRows));

  // Helper: extract contact world-space points for a collision pair's
  // constraint range. Contact point = comA + leverArmA.
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

    Vector3D const normalVec{pair.result.normal.x(),
                             pair.result.normal.y(),
                             pair.result.normal.z()};
    auto cachedLambdas = contactCache_.getWarmStart(
      pair.bodyAId, pair.bodyBId, normalVec, currentPoints);

    if (!cachedLambdas.empty() &&
        cachedLambdas.size() == range.count * lambdasPerContact)
    {
      for (size_t ci = 0; ci < range.count; ++ci)
      {
        auto const flatBase = static_cast<Eigen::Index>(
          (range.startIdx + ci) * lambdasPerContact);
        for (size_t k = 0; k < lambdasPerContact; ++k)
        {
          initialLambda(flatBase + static_cast<Eigen::Index>(k)) =
            cachedLambdas[ci * lambdasPerContact + k];
        }
      }
    }
  }

  // ===== Phase 4: Solve with warm-starting =====
  auto solveResult = constraintSolver_.solve(constraintPtrs_,
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
    solvedLambdas.reserve(range.count * lambdasPerContact);
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      auto const flatBase = static_cast<Eigen::Index>(
        (range.startIdx + ci) * lambdasPerContact);
      for (size_t k = 0; k < lambdasPerContact; ++k)
      {
        solvedLambdas.push_back(
          solveResult.lambdas(flatBase + static_cast<Eigen::Index>(k)));
      }
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
  const ConstraintSolver::SolveResult& solveResult)
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

  // Ticket 0052d: Pass only normal constraints to position corrector
  positionCorrector_.correctPositions(normalConstraintPtrs_,
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
  frictionConstraints_.clear();
  states_.clear();
  inverseMasses_.clear();
  inverseInertias_.clear();
  constraintPtrs_.clear();
  normalConstraintPtrs_.clear();
  pairRanges_.clear();
}

}  // namespace msd_sim
