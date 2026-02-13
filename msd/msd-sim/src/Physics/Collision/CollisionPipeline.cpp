// Ticket: 0044_collision_pipeline_integration
// Ticket: 0052d_solver_integration_ecos_removal
// Design: docs/designs/0044_collision_pipeline_integration/design.md
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#include "msd-sim/src/Physics/Collision/CollisionPipeline.hpp"
#include <cstddef>
#include <cstdint>
#include <functional>
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
  collisions_.clear();
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

  // Clear ephemeral state (references/pointers) but keep collisions_ and
  // solverData_ alive for WorldModel to read between frames
  clearEphemeralState();
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
  // Determine friction mode based on constraint types in allConstraints_
  bool hasFriction = false;
  for (const auto& c : allConstraints_)
  {
    if (dynamic_cast<FrictionConstraint*>(c.get()))
    {
      hasFriction = true;
      break;
    }
  }

  // Lambdas per contact: 3 with friction (n, t1, t2), 1 without
  const size_t lambdasPerContact = hasFriction ? 3 : 1;

  // ===== Phase 3.5: Warm-Start from Contact Cache =====
  // Ticket: 0058_constraint_ownership_cleanup
  // Count ContactConstraints (half of total if friction active, all otherwise)
  size_t numContacts = 0;
  for (const auto& c : allConstraints_)
  {
    if (dynamic_cast<ContactConstraint*>(c.get()))
    {
      ++numContacts;
    }
  }
  const size_t totalRows = numContacts * lambdasPerContact;
  Eigen::VectorXd initialLambda =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(totalRows));

  // Helper: extract contact world-space points for a collision pair's
  // constraint range. Contact point = comA + leverArmA.
  // Ticket: 0058_constraint_ownership_cleanup
  // With interleaved pattern [CC, FC, CC, FC, ...], ContactConstraints are
  // at even indices when friction is active, sequential otherwise.
  auto extractContactPoints =
    [this, hasFriction](const PairConstraintRange& range,
                        const CollisionPair& pair) -> std::vector<Coordinate>
  {
    std::vector<Coordinate> points;
    points.reserve(range.count);
    const size_t stride = hasFriction ? 2 : 1;
    for (size_t ci = 0; ci < range.count; ++ci)
    {
      const size_t idx = range.startIdx + ci * stride;
      auto* contact = dynamic_cast<ContactConstraint*>(allConstraints_[idx].get());
      if (!contact)
      {
        continue;  // Skip non-contact constraints
      }
      const InertialState& stateA = states_[pair.bodyAIndex].get();
      points.push_back(Coordinate{stateA.position + contact->getLeverArmA()});
    }
    return points;
  };

  // With interleaved storage [CC, FC, CC, FC, ...], each contact group occupies
  // stride entries in allConstraints_ but only lambdasPerContact lambdas.
  // range.startIdx is the allConstraints_ index; convert to contact group index.
  const size_t stride = hasFriction ? 2 : 1;

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
        // Convert allConstraints_ index to contact group index for lambda mapping
        auto const contactGroupIdx = range.startIdx / stride + ci;
        auto const flatBase = static_cast<Eigen::Index>(
          contactGroupIdx * lambdasPerContact);
        for (size_t k = 0; k < lambdasPerContact; ++k)
        {
          initialLambda(flatBase + static_cast<Eigen::Index>(k)) =
            cachedLambdas[ci * lambdasPerContact + k];
        }
      }
    }
  }

  // ===== Phase 4: Solve with warm-starting =====
  // Ticket: 0058_constraint_ownership_cleanup
  auto constraintPtrs = buildSolverView(hasFriction);
  auto solveResult = constraintSolver_.solve(constraintPtrs,
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
      // Convert allConstraints_ index to contact group index for lambda mapping
      auto const contactGroupIdx = range.startIdx / stride + ci;
      auto const flatBase = static_cast<Eigen::Index>(
        contactGroupIdx * lambdasPerContact);
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
  // Ticket: 0058_constraint_ownership_cleanup
  auto contactView = buildContactView();
  positionCorrector_.correctPositions(contactView,
                                      mutableStates,
                                      inverseMasses_,
                                      inverseInertias_,
                                      numBodies,
                                      numInertial,
                                      /* dt = */ 0.016);  // Placeholder, not used in position correction
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

}  // namespace msd_sim
