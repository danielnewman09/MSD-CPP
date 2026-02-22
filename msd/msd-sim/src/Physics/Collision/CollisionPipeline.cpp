// Ticket: 0044_collision_pipeline_integration
// Ticket: 0052d_solver_integration_ecos_removal
// Ticket: 0071a_constraint_solver_scalability
// Ticket: 0071e_trivial_allocation_elimination
// Design: docs/designs/0044_collision_pipeline_integration/design.md
// Design: docs/designs/0052_custom_friction_cone_solver/design.md
// Design: docs/designs/0071a_constraint_solver_scalability/design.md

#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/DataRecorder/DataRecorder.hpp"
#include "msd-sim/src/DataRecorder/DataRecorderVisitor.hpp"
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

const std::vector<CollisionPipeline::CollisionPair>&
CollisionPipeline::getCollisions() const
{
  return collisions_;
}

const CollisionPipeline::SolverData&
CollisionPipeline::getSolverData() const
{
  return solverData_;
}

void CollisionPipeline::execute(
  std::span<AssetInertial> inertialAssets,
  std::span<const AssetEnvironment> environmentalAssets,
  double dt)
{
  // Clear all state from previous frame
  // Ticket: 0071e_trivial_allocation_elimination
  // Capture previous collision count before clearing so we can re-reserve below.
  // clear() preserves capacity, so this reserve is a no-op after frame 1.
  const size_t prevCollisionCount = collisions_.size();
  collisions_.clear();
  if (prevCollisionCount == 0)
  {
    // First frame: reserve a reasonable estimate (inertial × environment pairs)
    const size_t numInertialEst = inertialAssets.size();
    const size_t numEnvEst = environmentalAssets.size();
    collisions_.reserve(numInertialEst * (numInertialEst + numEnvEst));
  }
  solverData_ = SolverData{};
  clearEphemeralState();

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
    clearEphemeralState();
    return;
  }

  // ===== Phase 2: Create Contact Constraints =====
  createConstraints(inertialAssets, environmentalAssets, dt);

  // Early return if no constraints created
  // Ticket: 0058_constraint_ownership_cleanup
  if (allConstraints_.empty())
  {
    clearEphemeralState();
    return;
  }

  // ===== Phase 3: Assemble Solver Input Arrays =====
  assembleSolverInput(inertialAssets, environmentalAssets, numBodies);

  // ===== Phase 4: Solve Contact Constraint System with Warm-Starting =====
  auto solveResult = solveConstraintsWithWarmStart(dt);

  // ===== Phase 4.7: Propagate solved normal lambdas to FrictionConstraints =====
  // The solver returns all lambdas in a flat vector (3 per contact when friction
  // is active: normal, tangent1, tangent2). FrictionConstraint::recordState()
  // needs the normal lambda to record actual force values for visualization.
  // Ticket: 0057_contact_tangent_recording
  propagateSolvedLambdas(solveResult);

  // ===== Phase 5: Apply Constraint Forces to Inertial Bodies =====
  applyForces(inertialAssets, solveResult);

  // ===== Phase 6: Position Correction =====
  correctPositions(inertialAssets, environmentalAssets, numBodies);

  // Capture solver diagnostics (lightweight value copy)
  // Ticket: 0058_constraint_ownership_cleanup
  solverData_.iterations = solveResult.iterations;
  solverData_.residual = solveResult.residual;
  solverData_.converged = solveResult.converged;
  solverData_.numConstraints = allConstraints_.size();
  solverData_.numContacts = collisions_.size();

  // Ephemeral state (allConstraints_, states_, etc.) is NOT cleared here.
  // It persists until the next execute() call (line 69) so that
  // WorldModel::recordCurrentFrame() can iterate allConstraints_ for
  // constraint recording (FrictionConstraintRecord, ContactConstraintRecord).
  // Ticket: 0057_contact_tangent_recording
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
      // Ticket 0053d: Skip SAT for persistent contacts with stable normals.
      // ContactCache entries indicate the pair was in contact last frame,
      // meaning EPA produced valid results then. SAT is only critical for
      // new contacts where EPA may fail on degenerate simplices.
      bool const skipSAT = contactCache_.hasEntry(
        inertialAssets[i].getInstanceId(),
        inertialAssets[j].getInstanceId());

      auto result = collisionHandler_.checkCollision(
        inertialAssets[i], inertialAssets[j], skipSAT);
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
      // Ticket 0053d: Skip SAT for persistent contacts (see above)
      bool const skipSAT = contactCache_.hasEntry(
        inertialAssets[i].getInstanceId(),
        environmentalAssets[e].getInstanceId() | kEnvIdFlag);

      auto result = collisionHandler_.checkCollision(
        inertialAssets[i], environmentalAssets[e], skipSAT);
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

  // Ticket: 0071e_trivial_allocation_elimination
  // Worst case: 1 contact + 1 friction constraint per collision pair.
  // Each CollisionResult may have up to 4 contact points, so the actual
  // bound is collisions_.size() * 4 * (anyFriction ? 2 : 1).
  // Use a conservative estimate of 2 constraints per pair (1 contact point
  // is most common) to avoid excessive over-reservation.
  allConstraints_.reserve(collisions_.size() * 2 * (anyFriction ? 2 : 1));
  pairRanges_.reserve(collisions_.size());

  // Ticket: 0058_constraint_ownership_cleanup
  // Store all constraints in allConstraints_ with interleaved pattern when
  // friction is active: [CC_0, FC_0, CC_1, FC_1, ...]
  for (size_t p = 0; p < collisions_.size(); ++p)
  {
    const auto& pair = collisions_[p];
    size_t const rangeStart = allConstraints_.size();

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
      // Capture lever arms BEFORE moving the constraint (avoid use-after-move)
      const auto leverArmA = contactConstraints[ci]->getLeverArmA();
      const auto leverArmB = contactConstraints[ci]->getLeverArmB();

      // Store ContactConstraint in allConstraints_
      allConstraints_.push_back(std::move(contactConstraints[ci]));

      if (anyFriction)
      {
        // Create matching FrictionConstraint with same contact geometry.
        // Store immediately after its ContactConstraint for interleaved pattern.
        // Uses per-pair friction coefficient (may be 0 for frictionless pairs).
        auto fc = std::make_unique<FrictionConstraint>(
          pair.bodyAIndex,
          pair.bodyBIndex,
          pair.result.normal,
          comA + leverArmA,   // contactPointA = comA + leverArmA
          comB + leverArmB,   // contactPointB = comB + leverArmB
          comA,
          comB,
          pair.frictionCoefficient);

        // Ticket 0069: Check if contact is in sliding mode
        auto [slidingDir, isSliding] = contactCache_.getSlidingState(
          pair.bodyAId, pair.bodyBId, /* minFrames = */ 3);
        if (isSliding && slidingDir.has_value())
        {
          fc->setSlidingMode(*slidingDir);
        }

        allConstraints_.push_back(std::move(fc));
      }
    }

    // Track constraint range (number of ContactConstraints for this pair)
    size_t const numContacts = contactConstraints.size();
    pairRanges_.push_back({rangeStart, numContacts, p});
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
  // Ticket: 0058_constraint_ownership_cleanup
  // Ticket: 0071a_constraint_solver_scalability — island decomposition
  //
  // Determine friction mode based on constraint types in allConstraints_.
  // This is a global flag: if any pair has friction, all pairs have FCs.
  bool hasFriction = false;
  for (const auto& c : allConstraints_)
  {
    if (dynamic_cast<FrictionConstraint*>(c.get()))
    {
      hasFriction = true;
      break;
    }
  }

  // Lambdas per contact: 3 with friction (n, t1, t2), 1 without.
  const size_t lambdasPerContact = hasFriction ? 3 : 1;

  // With interleaved storage [CC, FC, CC, FC, ...], each contact group
  // occupies stride entries in allConstraints_ but only lambdasPerContact
  // lambdas in the solver's flat lambda vector.
  const size_t stride = hasFriction ? 2 : 1;

  // Helper: extract contact world-space points for a collision pair's range.
  // Contact point = comA + leverArmA.
  auto extractContactPoints =
    [this, hasFriction](const PairConstraintRange& range,
                        const CollisionPair& pair) -> std::vector<Coordinate>
  {
    std::vector<Coordinate> points;
    points.reserve(range.count);
    const size_t s = hasFriction ? 2 : 1;
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      const size_t idx = range.startIdx + ci * s;
      auto* contact = dynamic_cast<ContactConstraint*>(allConstraints_[idx].get());
      if (!contact)
      {
        continue;
      }
      const InertialState& stateA = states_[pair.bodyAIndex].get();
      points.push_back(Coordinate{stateA.position + contact->getLeverArmA()});
    }
    return points;
  };

  // ===== Phase 3.5 + 4 + 4.5: Island-Decomposed Solve =====
  //
  // Partition allConstraints_ into independent contact islands. Each island is
  // solved independently with its own warm-start vector. This reduces the
  // effective mass matrix from O((3C)³) global to O(Σᵢ (3Cᵢ)³) per-island.
  //
  // Design: docs/designs/0071a_constraint_solver_scalability/design.md
  // Option A: pass global body arrays (states_, inverseMasses_, inverseInertias_)
  // unchanged. ConstraintSolver naturally handles sparse body participation.

  auto constraintPtrs = buildSolverView(hasFriction);

  // Ticket: 0071e_trivial_allocation_elimination
  // Pre-compute contact points for all pairs once. Both the warm-start query
  // loop and the cache-update loop need these, so computing them once here
  // eliminates one extractContactPoints() call per pair per frame.
  // pairContactPoints_[i] corresponds to collisions_[i] / pairRanges entry
  // with pairIdx == i.
  pairContactPoints_.clear();
  pairContactPoints_.resize(collisions_.size());
  for (const auto& range : pairRanges_)
  {
    const auto& pair = collisions_[range.pairIdx];
    pairContactPoints_[range.pairIdx] = extractContactPoints(range, pair);
  }

  // Determine numInertial: inertial bodies have non-zero inverse mass,
  // environment bodies have inverseMass == 0.0. Find the highest index
  // with positive inverse mass + 1.
  size_t numInertial = 0;
  for (size_t i = 0; i < inverseMasses_.size(); ++i)
  {
    if (inverseMasses_[i] > 0.0)
    {
      numInertial = i + 1;  // Track highest inertial index + 1
    }
  }
  // If no inertial bodies found (degenerate), fall back to full solve
  if (numInertial == 0)
  {
    numInertial = states_.size();
  }

  // Build islands — O(n · α(n)), negligible overhead
  auto islands = ConstraintIslandBuilder::buildIslands(constraintPtrs, numInertial);

  // Build a reverse map: Constraint* → its contact group index in the global
  // flat lambda vector. This enables per-island lambda assembly and cache update.
  //
  // Contact group index: the index of a ContactConstraint in the ordered sequence
  // of ContactConstraints (ignoring FrictionConstraints). For the global lambda
  // vector, contact group g has lambdas at [g*lambdasPerContact .. (g+1)*lambdasPerContact).
  //
  // We compute the global contact group index for each ContactConstraint by
  // iterating allConstraints_ in order.
  std::unordered_map<const Constraint*, size_t> constraintToGlobalGroup;
  {
    size_t globalGroup = 0;
    for (size_t i = 0; i < allConstraints_.size(); i += stride)
    {
      constraintToGlobalGroup[allConstraints_[i].get()] = globalGroup;
      ++globalGroup;
    }
  }

  // Initialize global result accumulator.
  // bodyForces is indexed by global body index. Each island solve returns
  // a bodyForces vector of size numBodies (global), with non-zero entries only
  // for the island's bodies. We accumulate by summation.
  ConstraintSolver::SolveResult globalResult;
  globalResult.bodyForces.resize(states_.size());
  globalResult.converged = true;
  globalResult.iterations = 0;
  globalResult.residual = 0.0;

  // globalLambdas: flattened lambda vector for all contacts (global ordering).
  // After all islands are solved, this is used by propagateSolvedLambdas()
  // and the cache update path.
  size_t const totalContacts = constraintPtrs.size() / stride;
  Eigen::VectorXd globalLambdas =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(totalContacts * lambdasPerContact));

  for (const auto& island : islands)
  {
    // Per-island constraint list (already in global pointer order, so CC/FC
    // interleaving is preserved — design R2 integration note).
    const auto& islandConstraints = island.constraints;

    // Count ContactConstraints in this island.
    // With no friction (stride==1): all constraints are CCs.
    // With friction (stride==2): interleaved [CC, FC, CC, FC, ...], so every
    // other entry starting at index 0 is a CC.
    size_t islandContactCount = 0;
    if (stride == 1)
    {
      islandContactCount = islandConstraints.size();
    }
    else
    {
      // With friction, count CCs at even positions in the island list
      for (size_t i = 0; i < islandConstraints.size(); i += stride)
      {
        if (dynamic_cast<const ContactConstraint*>(islandConstraints[i]))
        {
          ++islandContactCount;
        }
      }
    }

    const size_t islandRows = islandContactCount * lambdasPerContact;

    // Assemble per-island warm-start vector by querying the cache for each
    // pair whose constraints appear in this island (design R1 integration note).
    // We identify pairs by checking pairRanges_ against this island's constraint set.
    Eigen::VectorXd islandLambda =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(islandRows));

    // Build a lookup set of constraint pointers in this island for O(1) membership test
    std::unordered_set<const Constraint*> islandConstraintSet(
      islandConstraints.begin(), islandConstraints.end());

    // Island-local contact group index counter
    size_t islandGroupOffset = 0;

    for (const auto& range : pairRanges_)
    {
      // Check if this pair's first ContactConstraint is in this island
      const Constraint* firstCC = allConstraints_[range.startIdx].get();
      if (islandConstraintSet.find(firstCC) == islandConstraintSet.end())
      {
        continue;  // This pair belongs to a different island
      }

      const auto& pair = collisions_[range.pairIdx];
      // Ticket: 0071e_trivial_allocation_elimination — reuse pre-computed points
      const auto& currentPoints = pairContactPoints_[range.pairIdx];

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
          auto const islandBase = static_cast<Eigen::Index>(
            (islandGroupOffset + ci) * lambdasPerContact);
          for (size_t k = 0; k < lambdasPerContact; ++k)
          {
            islandLambda(islandBase + static_cast<Eigen::Index>(k)) =
              cachedLambdas[ci * lambdasPerContact + k];
          }
        }
      }
      islandGroupOffset += range.count;
    }

    // Solve this island — passing global body arrays (Option A from design)
    auto islandResult = constraintSolver_.solve(islandConstraints,
                                                states_,
                                                inverseMasses_,
                                                inverseInertias_,
                                                states_.size(),
                                                dt,
                                                islandLambda);

    // Accumulate per-island body forces into global result
    for (size_t b = 0; b < islandResult.bodyForces.size() && b < globalResult.bodyForces.size(); ++b)
    {
      globalResult.bodyForces[b].linearForce = globalResult.bodyForces[b].linearForce +
                                               islandResult.bodyForces[b].linearForce;
      globalResult.bodyForces[b].angularTorque = globalResult.bodyForces[b].angularTorque +
                                                 islandResult.bodyForces[b].angularTorque;
    }
    globalResult.iterations += islandResult.iterations;
    globalResult.converged = globalResult.converged && islandResult.converged;
    globalResult.residual = std::max(globalResult.residual, islandResult.residual);

    // Map per-island lambdas back to global lambda vector using the
    // constraintToGlobalGroup map (design R3 integration note).
    // Iterate the island's CC list in order to determine island-local group offsets.
    size_t islandLocalGroup = 0;
    for (size_t i = 0; i < islandConstraints.size(); i += stride)
    {
      const Constraint* cc = islandConstraints[i];
      auto it = constraintToGlobalGroup.find(cc);
      if (it == constraintToGlobalGroup.end())
      {
        ++islandLocalGroup;
        continue;
      }
      const size_t globalGroup = it->second;
      const auto globalBase = static_cast<Eigen::Index>(globalGroup * lambdasPerContact);
      const auto islandBase = static_cast<Eigen::Index>(islandLocalGroup * lambdasPerContact);

      if (islandBase + static_cast<Eigen::Index>(lambdasPerContact) <= islandResult.lambdas.size() &&
          globalBase + static_cast<Eigen::Index>(lambdasPerContact) <= globalLambdas.size())
      {
        for (size_t k = 0; k < lambdasPerContact; ++k)
        {
          globalLambdas(globalBase + static_cast<Eigen::Index>(k)) =
            islandResult.lambdas(islandBase + static_cast<Eigen::Index>(k));
        }
      }
      ++islandLocalGroup;
    }
  }

  globalResult.lambdas = globalLambdas;

  // ===== Phase 4.5: Update Contact Cache =====
  // Write solved lambdas back to cache for next frame's warm-start.
  // Cache writes happen after all islands are solved (design: avoid intra-frame
  // cross-island contamination).
  for (const auto& range : pairRanges_)
  {
    const auto& pair = collisions_[range.pairIdx];

    std::vector<double> solvedLambdas;
    solvedLambdas.reserve(range.count * lambdasPerContact);
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      auto const contactGroupIdx = range.startIdx / stride + ci;
      auto const flatBase = static_cast<Eigen::Index>(
        contactGroupIdx * lambdasPerContact);
      for (size_t k = 0; k < lambdasPerContact; ++k)
      {
        solvedLambdas.push_back(
          globalResult.lambdas(flatBase + static_cast<Eigen::Index>(k)));
      }
    }

    // Ticket: 0071e_trivial_allocation_elimination
    // Contact points for this pair were already computed in the warm-start
    // query loop above (stored in pairContactPoints_). Reuse them here to
    // avoid a second extractContactPoints() call per pair per frame.
    auto contactPoints = pairContactPoints_[range.pairIdx];

    Vector3D const normalVec{pair.result.normal.x(),
                             pair.result.normal.y(),
                             pair.result.normal.z()};
    contactCache_.update(
      pair.bodyAId, pair.bodyBId, normalVec, solvedLambdas, contactPoints);

    // Ticket 0069: Update sliding state based on tangent velocity
    if (!contactPoints.empty() && hasFriction)
    {
      const InertialState& stateA = states_[pair.bodyAIndex].get();
      const InertialState& stateB = states_[pair.bodyBIndex].get();

      const Coordinate& contactPt = contactPoints[0];
      const Coordinate& comA = stateA.position;
      const Coordinate& comB = stateB.position;

      Coordinate const leverArmA = contactPt - comA;
      Coordinate const leverArmB = contactPt - comB;

      Coordinate const vContactA = stateA.velocity + stateA.getAngularVelocity().cross(leverArmA);
      Coordinate const vContactB = stateB.velocity + stateB.getAngularVelocity().cross(leverArmB);
      Coordinate const vRel = vContactA - vContactB;

      Vector3D const normalVecSliding{pair.result.normal.x(),
                                      pair.result.normal.y(),
                                      pair.result.normal.z()};
      Coordinate const vTangent = vRel - normalVecSliding * vRel.dot(normalVecSliding);
      Vector3D const tangentVel{vTangent.x(), vTangent.y(), vTangent.z()};

      contactCache_.updateSlidingState(
        pair.bodyAId, pair.bodyBId, tangentVel, /* velocityThreshold = */ 0.01);
    }
  }

  return globalResult;
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
  // Ticket: 0058_constraint_ownership_cleanup
  // Ticket: 0071a_constraint_solver_scalability — island decomposition
  //
  // Per design Open Question 1 (Option A): reuse the same island partition from
  // the velocity solve. Each island's constraint subset is filtered to
  // contact-only before passing to PositionCorrector.
  // The PositionCorrector receives global state arrays (same as velocity solve
  // Option A approach — no body index remapping needed).

  auto contactView = buildContactView();

  // Build islands from contact-only view (same partition, contact-only subset)
  // Use numInertial derived from inverseMasses_ (bodies with non-zero inv mass)
  size_t posNumInertial = 0;
  for (size_t i = 0; i < inverseMasses_.size(); ++i)
  {
    if (inverseMasses_[i] > 0.0)
    {
      posNumInertial = i + 1;
    }
  }
  if (posNumInertial == 0)
  {
    posNumInertial = numInertial;
  }

  auto posIslands = ConstraintIslandBuilder::buildIslands(contactView, posNumInertial);

  if (posIslands.empty())
  {
    // Fall back to global solve if island detection found nothing
    positionCorrector_.correctPositions(contactView,
                                        mutableStates,
                                        inverseMasses_,
                                        inverseInertias_,
                                        numBodies,
                                        numInertial,
                                        /* dt = */ 0.016);
    return;
  }

  for (const auto& island : posIslands)
  {
    // island.constraints contains only ContactConstraints (built from contactView)
    positionCorrector_.correctPositions(island.constraints,
                                        mutableStates,
                                        inverseMasses_,
                                        inverseInertias_,
                                        numBodies,
                                        numInertial,
                                        /* dt = */ 0.016);
  }
}

void CollisionPipeline::propagateSolvedLambdas(
  const ConstraintSolver::SolveResult& solveResult)
{
  // Detect friction presence and stride
  bool hasFriction = false;
  for (const auto& c : allConstraints_)
  {
    if (dynamic_cast<FrictionConstraint*>(c.get()))
    {
      hasFriction = true;
      break;
    }
  }

  if (!hasFriction)
  {
    return;  // Nothing to propagate
  }

  // With friction, lambdas are grouped as 3 per contact: [n, t1, t2].
  // allConstraints_ is interleaved: [CC_0, FC_0, CC_1, FC_1, ...].
  // For contact group g (at allConstraints_[g*2] and allConstraints_[g*2+1]),
  // the normal lambda is at solveResult.lambdas[g * 3].
  const size_t stride = 2;  // CC + FC per contact

  for (size_t i = 1; i < allConstraints_.size(); i += stride)
  {
    auto* fc = dynamic_cast<FrictionConstraint*>(allConstraints_[i].get());
    if (!fc)
    {
      continue;
    }

    const size_t contactGroupIdx = i / stride;
    const auto lambdaIdx = static_cast<Eigen::Index>(contactGroupIdx * 3);

    if (lambdaIdx + 2 < solveResult.lambdas.size())
    {
      fc->setNormalLambda(solveResult.lambdas(lambdaIdx));
      fc->setTangentLambdas(solveResult.lambdas(lambdaIdx + 1),
                            solveResult.lambdas(lambdaIdx + 2));
    }
  }
}

void CollisionPipeline::clearEphemeralState()
{
  // Clear vectors holding references/pointers to external objects to prevent
  // dangling refs when assets are modified between frames.
  // Does NOT clear collisions_ or solverData_ — those are value types owned
  // by the pipeline, safe to read between frames via getCollisions()/getSolverData().
  // Ticket: 0058_constraint_ownership_cleanup
  allConstraints_.clear();
  states_.clear();
  inverseMasses_.clear();
  inverseInertias_.clear();
  pairRanges_.clear();
  pairContactPoints_.clear();
}

std::vector<Constraint*>
CollisionPipeline::buildSolverView(bool interleaved) const
{
  std::vector<Constraint*> view;

  if (!interleaved)
  {
    // No friction: return all constraints in storage order
    view.reserve(allConstraints_.size());
    for (const auto& c : allConstraints_)
    {
      view.push_back(c.get());
    }
  }
  else
  {
    // Friction active: return interleaved [CC_0, FC_0, CC_1, FC_1, ...]
    // Relies on constraint storage pattern: ContactConstraint followed by
    // its matching FrictionConstraint (established in createConstraints)
    view.reserve(allConstraints_.size());
    for (const auto& c : allConstraints_)
    {
      view.push_back(c.get());
    }
  }

  return view;
}

std::vector<Constraint*> CollisionPipeline::buildContactView() const
{
  std::vector<Constraint*> view;
  view.reserve(allConstraints_.size() / 2);  // Rough estimate

  for (const auto& c : allConstraints_)
  {
    if (dynamic_cast<ContactConstraint*>(c.get()))
    {
      view.push_back(c.get());
    }
  }

  return view;
}

void CollisionPipeline::recordConstraints(DataRecorder& recorder, uint32_t frameId) const
{
  // Create visitor wrapping DataRecorder
  DataRecorderVisitor visitor{recorder, frameId};

  // Iterate all constraints (interleaved [CC, FC, CC, FC, ...] or all CC if no friction)
  for (size_t i = 0; i < allConstraints_.size(); ++i)
  {
    const auto& constraint = allConstraints_[i];

    // Find collision pair for this constraint (via pairRanges_ lookup)
    size_t pairIdx = findPairIndexForConstraint(i);
    const auto& pair = collisions_[pairIdx];

    // Constraint builds record and dispatches to visitor (no casting here)
    constraint->recordState(visitor, pair.bodyAId, pair.bodyBId);
  }
}

size_t CollisionPipeline::findPairIndexForConstraint(size_t constraintIdx) const
{
  // Detect interleaved layout: [CC, FC, CC, FC, ...] when friction is active.
  // Stride is 2 (CC+FC per contact) with friction, 1 (CC only) without.
  bool hasFriction = false;
  for (const auto& c : allConstraints_)
  {
    if (dynamic_cast<FrictionConstraint*>(c.get()))
    {
      hasFriction = true;
      break;
    }
  }
  const size_t stride = hasFriction ? 2 : 1;

  // range.count is the number of ContactConstraints; actual allConstraints_
  // span is count * stride entries starting at startIdx.
  for (const auto& range : pairRanges_)
  {
    const size_t rangeEnd = range.startIdx + range.count * stride;
    if (constraintIdx >= range.startIdx && constraintIdx < rangeEnd)
    {
      return range.pairIdx;
    }
  }

  // Should never reach here if pairRanges_ is correct
  throw std::logic_error("Constraint index not found in pairRanges_");
}

}  // namespace msd_sim
