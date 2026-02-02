#include "msd-sim/src/Environment/WorldModel.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include "msd-sim/src/Physics/Constraints/ContactConstraintFactory.hpp"
#include "msd-sim/src/Physics/Constraints/FrictionConstraint.hpp"
#include "msd-sim/src/Physics/Integration/SemiImplicitEulerIntegrator.hpp"
#include "msd-sim/src/Physics/PotentialEnergy/GravityPotential.hpp"

namespace msd_sim
{

// Constructor initializes default integrator and gravity potential
WorldModel::WorldModel()
{
  // Initialize default integrator (semi-implicit Euler)
  integrator_ = std::make_unique<SemiImplicitEulerIntegrator>();

  // Initialize default gravity potential
  potentialEnergies_.push_back(
    std::make_unique<GravityPotential>(Coordinate{0.0, 0.0, -9.81}));
  // Ticket: 0030_lagrangian_quaternion_physics
}


// ========== Object Management ==========

const AssetInertial& WorldModel::spawnObject(uint32_t assetId,
                                             ConvexHull& hull,
                                             const ReferenceFrame& origin)
{
  auto instanceId = getInertialAssetId();
  inertialAssets_.emplace_back(assetId, instanceId, hull, 10.0, origin);
  return inertialAssets_.back();
}

const AssetEnvironment& WorldModel::spawnEnvironmentObject(
  uint32_t assetId,
  ConvexHull& hull,
  const ReferenceFrame& origin)
{
  auto instanceId = ++environmentAssetIdCounter_;
  environmentalAssets_.emplace_back(assetId, instanceId, hull, origin);
  return environmentalAssets_.back();
}

const AssetInertial& WorldModel::getObject(uint32_t instanceId) const
{
  auto it = std::find_if(inertialAssets_.begin(),
                         inertialAssets_.end(),
                         [instanceId](const AssetInertial& asset)
                         { return asset.getInstanceId() == instanceId; });
  if (it == inertialAssets_.end())
  {
    throw std::out_of_range("Object with instanceId not found: " +
                            std::to_string(instanceId));
  }
  return *it;
}

AssetInertial& WorldModel::getObject(uint32_t instanceId)
{
  auto it = std::find_if(inertialAssets_.begin(),
                         inertialAssets_.end(),
                         [instanceId](const AssetInertial& asset)
                         { return asset.getInstanceId() == instanceId; });
  if (it == inertialAssets_.end())
  {
    throw std::out_of_range("Object with instanceId not found: " +
                            std::to_string(instanceId));
  }
  return *it;
}

void WorldModel::clearObjects()
{
  inertialAssets_.clear();
}

// ========== Platform Management (Legacy) ==========

void WorldModel::addPlatform(Platform&& platform)
{
  platforms_.push_back(std::move(platform));
}

// ========== Simulation Update ==========

void WorldModel::update(std::chrono::milliseconds simTime)
{
  // Convert to seconds for physics calculations
  double dt = (simTime.count() - time_.count()) / 1000.0;

  // Update all platforms (agent logic + visual sync)
  for (auto& platform : platforms_)
  {
    platform.update(simTime);
  }

  // IMPORTANT: Contact constraint forces BEFORE physics integration
  // Order matters: constraint forces must be accumulated before velocity
  // integration so they are included in the force-based integration step
  // Ticket: 0032_contact_constraint_refactor
  updateCollisions(dt);

  // Update physics for all dynamic objects
  updatePhysics(dt);

  // Update simulation time
  time_ = simTime;
}

// ========== Private Methods ==========

void WorldModel::updatePhysics(double dt)
{
  for (auto& asset : inertialAssets_)
  {
    // ===== Step 1: Compute Generalized Forces from Potential Energies =====
    Coordinate netForce{0.0, 0.0, 0.0};
    Coordinate netTorque{0.0, 0.0, 0.0};

    InertialState& state = asset.getInertialState();
    double mass = asset.getMass();
    const Eigen::Matrix3d& inertiaTensor = asset.getInertiaTensor();

    for (const auto& potential : potentialEnergies_)
    {
      netForce += potential->computeForce(state, mass);
      netTorque += potential->computeTorque(state, inertiaTensor);
    }

    // Add accumulated forces (from collisions, thrusters, etc.)
    netForce += asset.getAccumulatedForce();
    netTorque += asset.getAccumulatedTorque();

    // ===== Step 2: Gather Constraints =====
    // Get all constraints attached to this asset
    std::vector<Constraint*> constraints = asset.getConstraints();

    // ===== Step 3: Delegate Integration to Integrator =====
    // Integrator handles: velocity update, position update, constraint
    // enforcement
    integrator_->step(state,
                      netForce,
                      netTorque,
                      mass,
                      asset.getInverseInertiaTensorWorld(),
                      constraints,
                      dt);

    // ===== Step 4: Apply Velocity Threshold =====
    // Clamp near-zero velocities to prevent stick-slip jitter
    // Ticket: 0035d_friction_hardening_and_validation
    const double velocityMagnitude = state.velocity.norm();
    if (velocityMagnitude < velocity_rest_threshold_)
    {
      state.velocity = Coordinate{0.0, 0.0, 0.0};
    }

    const double angularVelocityMagnitude = state.getAngularVelocity().norm();
    if (angularVelocityMagnitude < angularVelocityRestThreshold)
    {
      state.setAngularVelocity(AngularRate{0.0, 0.0, 0.0});
    }

    // ===== Step 5: Synchronize ReferenceFrame =====
    // ReferenceFrame must match InertialState for collision detection and
    // rendering
    ReferenceFrame& frame = asset.getReferenceFrame();
    frame.setOrigin(state.position);
    frame.setQuaternion(state.orientation);

    // ===== Step 6: Clear Forces for Next Frame =====
    asset.clearForces();
  }
  // Ticket: 0030_lagrangian_quaternion_physics
}

void WorldModel::updateCollisions(double dt)
{
  // Constraint-based collision response using Active Set Method / ECOS
  // two-phase solver Replaces per-pair impulse-based CollisionResponse with a
  // unified solver.
  //
  // Solver body indexing:
  //   [0 .. numInertial-1]                   = inertial bodies
  //   [numInertial .. numInertial+numEnv-1]   = environment bodies (infinite
  //   mass)
  //
  // Ticket: 0032_contact_constraint_refactor

  const size_t numInertial = inertialAssets_.size();
  const size_t numEnvironment = environmentalAssets_.size();
  const size_t numBodies = numInertial + numEnvironment;

  if (numBodies == 0 || dt <= 0.0)
  {
    return;
  }

  // ===== Phase 1: Collision Detection =====
  // Collect all collision pairs with their body indices

  struct CollisionPair
  {
    size_t bodyAIndex;
    size_t bodyBIndex;
    CollisionResult result;
    double restitution;
    double frictionCoefficientA;  // Ticket: 0035c_friction_pipeline_integration
    double frictionCoefficientB;  // Ticket: 0035c_friction_pipeline_integration
  };

  std::vector<CollisionPair> collisions;

  // Inertial vs Inertial (O(n²) pairwise)
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t j = i + 1; j < numInertial; ++j)
    {
      auto result = collisionHandler_.checkCollision(inertialAssets_[i],
                                                     inertialAssets_[j]);
      if (!result)
      {
        continue;
      }

      double combinedE = ContactConstraintFactory::combineRestitution(
        inertialAssets_[i].getCoefficientOfRestitution(),
        inertialAssets_[j].getCoefficientOfRestitution());

      collisions.push_back({i,
                            j,
                            *result,
                            combinedE,
                            inertialAssets_[i].getFrictionCoefficient(),
                            inertialAssets_[j].getFrictionCoefficient()});
    }
  }

  // Inertial vs Environment
  for (size_t i = 0; i < numInertial; ++i)
  {
    for (size_t e = 0; e < numEnvironment; ++e)
    {
      auto result = collisionHandler_.checkCollision(inertialAssets_[i],
                                                     environmentalAssets_[e]);
      if (!result)
      {
        continue;
      }

      double combinedE = ContactConstraintFactory::combineRestitution(
        inertialAssets_[i].getCoefficientOfRestitution(),
        environmentalAssets_[e].getCoefficientOfRestitution());

      // Environment body index offset by numInertial
      collisions.push_back({i,
                            numInertial + e,
                            *result,
                            combinedE,
                            inertialAssets_[i].getFrictionCoefficient(),
                            environmentalAssets_[e].getFrictionCoefficient()});
    }
  }

  if (collisions.empty())
  {
    return;
  }

  // ===== Phase 2: Create Contact Constraints (Two-Phase Approach) =====
  // Ticket: 0035d5_two_phase_friction_solve
  // Phase A (ASM): N normal constraints per collision pair
  // Phase B (ECOS): 1 centroid normal + 1 centroid friction per collision pair
  std::vector<std::unique_ptr<TwoBodyConstraint>> normalConstraints;
  std::vector<std::unique_ptr<TwoBodyConstraint>> frictionPairs;

  // Ticket: 0035d9_two_phase_pipeline_fix
  // Diagnostic logging for two-phase pipeline investigation
  spdlog::debug("[0035d9] Phase 2: Creating constraints for {} collision pairs",
                collisions.size());

  for (const auto& pair : collisions)
  {
    const InertialState& stateA =
      (pair.bodyAIndex < numInertial)
        ? inertialAssets_[pair.bodyAIndex].getInertialState()
        : environmentalAssets_[pair.bodyAIndex - numInertial]
            .getInertialState();

    const InertialState& stateB =
      (pair.bodyBIndex < numInertial)
        ? inertialAssets_[pair.bodyBIndex].getInertialState()
        : environmentalAssets_[pair.bodyBIndex - numInertial]
            .getInertialState();

    const Coordinate& comA = stateA.position;
    const Coordinate& comB = stateB.position;

    // N normal constraints for ASM
    auto contactConstraints =
      ContactConstraintFactory::createFromCollision(pair.bodyAIndex,
                                                    pair.bodyBIndex,
                                                    pair.result,
                                                    stateA,
                                                    stateB,
                                                    comA,
                                                    comB,
                                                    pair.restitution);

    for (auto& c : contactConstraints)
    {
      normalConstraints.push_back(std::move(c));
    }

    // 1 centroid normal + 1 centroid friction for ECOS (if μ > 0)
    auto centroidNormal =
      ContactConstraintFactory::createCentroidContactConstraint(
        pair.bodyAIndex,
        pair.bodyBIndex,
        pair.result,
        stateA,
        stateB,
        comA,
        comB,
        pair.restitution);

    auto centroidFriction =
      ContactConstraintFactory::createCentroidFrictionConstraint(
        pair.bodyAIndex,
        pair.bodyBIndex,
        pair.result,
        comA,
        comB,
        pair.frictionCoefficientA,
        pair.frictionCoefficientB);

    if (centroidNormal && centroidFriction)
    {
      // Must be first (ECOS cone ordering: normal before tangential)
      frictionPairs.push_back(std::move(centroidNormal));
      frictionPairs.push_back(std::move(centroidFriction));
    }
  }

  if (normalConstraints.empty())
  {
    return;
  }

  // Ticket: 0035d9_two_phase_pipeline_fix
  // Log constraint counts after creation
  spdlog::debug("[0035d9] Phase 2 complete: {} normal constraints, {} friction "
                "pair constraints",
                normalConstraints.size(),
                frictionPairs.size());

  // ===== Phase 3: Build Solver Input Arrays =====
  std::vector<std::reference_wrapper<const InertialState>> states;
  std::vector<double> inverseMasses;
  std::vector<Eigen::Matrix3d> inverseInertias;

  states.reserve(numBodies);
  inverseMasses.reserve(numBodies);
  inverseInertias.reserve(numBodies);

  for (auto& asset : inertialAssets_)
  {
    states.push_back(std::cref(asset.getInertialState()));
    inverseMasses.push_back(asset.getInverseMass());
    inverseInertias.push_back(asset.getInverseInertiaTensorWorld());
  }

  for (auto& envAsset : environmentalAssets_)
  {
    states.push_back(std::cref(envAsset.getInertialState()));
    inverseMasses.push_back(envAsset.getInverseMass());  // 0.0
    inverseInertias.push_back(
      envAsset.getInverseInertiaTensor());  // Zero matrix
  }

  // ===== Phase 4a: Solve Normals via ASM =====
  // Ticket: 0035d5_two_phase_friction_solve
  std::vector<TwoBodyConstraint*> normalPtrs;
  normalPtrs.reserve(normalConstraints.size());
  for (auto& c : normalConstraints)
  {
    normalPtrs.push_back(c.get());
  }

  auto normalResult = contactSolver_.solveWithContacts(
    normalPtrs, states, inverseMasses, inverseInertias, numBodies, dt);
  // No friction detected → dispatches to ASM automatically

  // Ticket: 0035d9_two_phase_pipeline_fix
  // Log ASM normal solve results
  spdlog::debug("[0035d9] Phase 4a (ASM) complete: converged={}, "
                "iterations={}, residual={:.6e}",
                normalResult.converged,
                normalResult.iterations,
                normalResult.residual);
  spdlog::debug("[0035d9] Phase 4a: ASM lambda vector size={}, norm={:.6e}",
                normalResult.lambdas.size(),
                normalResult.lambdas.norm());
  if (normalResult.lambdas.size() > 0 && normalResult.lambdas.size() <= 10)
  {
    spdlog::debug(
      "[0035d9] Phase 4a: ASM lambda values: {}",
      fmt::join(normalResult.lambdas.data(),
                normalResult.lambdas.data() + normalResult.lambdas.size(),
                ", "));
  }

  // ===== Phase 4b: Solve Friction via ECOS (if friction present) =====
  std::vector<ConstraintSolver::BodyForces> frictionBodyForces;
  if (!frictionPairs.empty())
  {
    // Ticket: 0035d9_two_phase_pipeline_fix
    // Fix H1: Pass ASM-computed normal lambda to ECOS friction solve
    // Sum ASM normal lambdas for each collision pair and set on
    // FrictionConstraints Assumption: normalConstraints and frictionPairs are
    // in same collision pair order normalConstraints = [CC₀, CC₁, CC₂, CC₃] (4
    // normals for 1 pair) frictionPairs = [CentroidCC₀, CentroidFC₀] (1
    // centroid normal + 1 centroid friction)

    const size_t numPairs = collisions.size();
    const size_t contactsPerPair = normalConstraints.size() / numPairs;

    spdlog::debug(
      "[0035d9] Phase 4b: Setting ASM normal lambdas on FrictionConstraints");
    spdlog::debug("[0035d9] Phase 4b: {} collision pairs, {} contacts/pair, {} "
                  "normal lambdas",
                  numPairs,
                  contactsPerPair,
                  normalResult.lambdas.size());

    for (size_t pairIdx = 0; pairIdx < numPairs; ++pairIdx)
    {
      // Sum ASM normal lambdas for this collision pair
      double asmNormalSum = 0.0;
      for (size_t contactIdx = 0; contactIdx < contactsPerPair; ++contactIdx)
      {
        size_t lambdaIdx = pairIdx * contactsPerPair + contactIdx;
        if (static_cast<Eigen::Index>(lambdaIdx) < normalResult.lambdas.size())
        {
          asmNormalSum +=
            normalResult.lambdas(static_cast<Eigen::Index>(lambdaIdx));
        }
      }

      spdlog::debug("[0035d9] Phase 4b: Pair[{}] ASM normal sum = {:.6e}",
                    pairIdx,
                    asmNormalSum);

      // Find the FrictionConstraint for this pair (should be at pairIdx * 2 +
      // 1)
      size_t frictionIdx = pairIdx * 2 + 1;  // Skip centroid normal
      if (frictionIdx < frictionPairs.size())
      {
        auto* friction =
          dynamic_cast<FrictionConstraint*>(frictionPairs[frictionIdx].get());
        if (friction != nullptr)
        {
          friction->setNormalLambda(asmNormalSum);
          spdlog::debug("[0035d9] Phase 4b: Set FrictionConstraint[{}] normal "
                        "lambda to {:.6e}",
                        pairIdx,
                        asmNormalSum);
        }
      }
    }

    std::vector<TwoBodyConstraint*> frictionPtrs;
    frictionPtrs.reserve(frictionPairs.size());
    for (auto& c : frictionPairs)
    {
      frictionPtrs.push_back(c.get());
    }

    auto frictionResult = contactSolver_.solveWithContacts(
      frictionPtrs, states, inverseMasses, inverseInertias, numBodies, dt);
    // FrictionConstraint detected → dispatches to ECOS

    // Ticket: 0035d9_two_phase_pipeline_fix
    // Extract friction-only body forces (zeroes centroid normal contribution)
    // Then scale by ratio of ASM normal to ECOS centroid normal to correct
    // friction bound
    frictionBodyForces = contactSolver_.extractFrictionOnlyBodyForces(
      frictionPtrs, states, frictionResult.lambdas, numBodies, dt);

    // Scale friction forces to use correct ASM normal bound
    for (size_t pairIdx = 0; pairIdx < numPairs; ++pairIdx)
    {
      // Get ASM normal sum for this pair
      double asmNormalSum = 0.0;
      for (size_t contactIdx = 0; contactIdx < contactsPerPair; ++contactIdx)
      {
        size_t lambdaIdx = pairIdx * contactsPerPair + contactIdx;
        if (static_cast<Eigen::Index>(lambdaIdx) < normalResult.lambdas.size())
        {
          asmNormalSum +=
            normalResult.lambdas(static_cast<Eigen::Index>(lambdaIdx));
        }
      }

      // Get ECOS centroid normal from friction result (first component of each
      // pair)
      size_t ecosNormalIdx =
        pairIdx * 3;  // 3 components per contact (1 normal + 2 friction)
      double ecosCentroidNormal = 0.0;
      if (static_cast<Eigen::Index>(ecosNormalIdx) <
          frictionResult.lambdas.size())
      {
        ecosCentroidNormal =
          frictionResult.lambdas(static_cast<Eigen::Index>(ecosNormalIdx));
      }

      spdlog::debug("[0035d9] Phase 4b: Pair[{}] ASM normal sum={:.6e}, ECOS "
                    "centroid normal={:.6e}",
                    pairIdx,
                    asmNormalSum,
                    ecosCentroidNormal);

      // Scale friction forces if ECOS centroid normal is significantly
      // different from ASM sum
      if (std::abs(ecosCentroidNormal) > 1e-10 &&
          std::abs(asmNormalSum) > 1e-10)
      {
        double scaleFactor = asmNormalSum / ecosCentroidNormal;
        spdlog::debug(
          "[0035d9] Phase 4b: Pair[{}] scaling friction forces by {:.6f}",
          pairIdx,
          scaleFactor);

        // Scale all body forces (assuming single body for now - needs
        // generalization for multi-body)
        for (size_t bodyIdx = 0; bodyIdx < frictionBodyForces.size(); ++bodyIdx)
        {
          frictionBodyForces[bodyIdx].linearForce *= scaleFactor;
          frictionBodyForces[bodyIdx].angularTorque *= scaleFactor;
        }
      }
    }

    // Ticket: 0035d9_two_phase_pipeline_fix
    // Log ECOS friction solve results
    spdlog::debug("[0035d9] Phase 4b (ECOS) complete: converged={}, "
                  "iterations={}, residual={:.6e}",
                  frictionResult.converged,
                  frictionResult.iterations,
                  frictionResult.residual);
    spdlog::debug("[0035d9] Phase 4b: ECOS lambda vector size={}, norm={:.6e}",
                  frictionResult.lambdas.size(),
                  frictionResult.lambdas.norm());
    if (frictionResult.lambdas.size() > 0 &&
        frictionResult.lambdas.size() <= 10)
    {
      spdlog::debug(
        "[0035d9] Phase 4b: ECOS lambda values: {}",
        fmt::join(frictionResult.lambdas.data(),
                  frictionResult.lambdas.data() + frictionResult.lambdas.size(),
                  ", "));
    }
  }

  // ===== Phase 5: Apply Combined Constraint Forces to Inertial Bodies =====
  // Ticket: 0035d5_two_phase_friction_solve
  // Combines ASM normal forces + ECOS friction-only forces
  // Environment bodies (indices >= numInertial) are skipped because they
  // have infinite mass and cannot be moved.

  // Ticket: 0035d9_two_phase_pipeline_fix
  spdlog::debug("[0035d9] Phase 5: Combining forces for {} inertial bodies",
                numInertial);

  for (size_t k = 0; k < numInertial; ++k)
  {
    Coordinate totalLinear{0.0, 0.0, 0.0};
    Coordinate totalAngular{0.0, 0.0, 0.0};

    // Add ASM normal forces
    totalLinear += normalResult.bodyForces[k].linearForce;
    totalAngular += normalResult.bodyForces[k].angularTorque;

    // Ticket: 0035d9_two_phase_pipeline_fix
    spdlog::debug("[0035d9] Phase 5: Body[{}] ASM normal force=({:.6f}, "
                  "{:.6f}, {:.6f}), torque=({:.6f}, {:.6f}, {:.6f})",
                  k,
                  normalResult.bodyForces[k].linearForce.x(),
                  normalResult.bodyForces[k].linearForce.y(),
                  normalResult.bodyForces[k].linearForce.z(),
                  normalResult.bodyForces[k].angularTorque.x(),
                  normalResult.bodyForces[k].angularTorque.y(),
                  normalResult.bodyForces[k].angularTorque.z());

    // Add ECOS friction-only forces
    if (!frictionPairs.empty())
    {
      totalLinear += frictionBodyForces[k].linearForce;
      totalAngular += frictionBodyForces[k].angularTorque;

      // Ticket: 0035d9_two_phase_pipeline_fix
      spdlog::debug("[0035d9] Phase 5: Body[{}] ECOS friction force=({:.6f}, "
                    "{:.6f}, {:.6f}), torque=({:.6f}, {:.6f}, {:.6f})",
                    k,
                    frictionBodyForces[k].linearForce.x(),
                    frictionBodyForces[k].linearForce.y(),
                    frictionBodyForces[k].linearForce.z(),
                    frictionBodyForces[k].angularTorque.x(),
                    frictionBodyForces[k].angularTorque.y(),
                    frictionBodyForces[k].angularTorque.z());
    }

    // Ticket: 0035d9_two_phase_pipeline_fix
    spdlog::debug("[0035d9] Phase 5: Body[{}] TOTAL force=({:.6f}, {:.6f}, "
                  "{:.6f}), torque=({:.6f}, {:.6f}, {:.6f})",
                  k,
                  totalLinear.x(),
                  totalLinear.y(),
                  totalLinear.z(),
                  totalAngular.x(),
                  totalAngular.y(),
                  totalAngular.z());

    // Skip bodies with no constraint force
    if (totalLinear.norm() < 1e-12 && totalAngular.norm() < 1e-12)
    {
      continue;
    }

    // ===== Post-Impulse Energy Clamping =====
    // Ticket: 0035da_post_impulse_energy_clamping
    //
    // Contact constraint forces are reactive — they should never inject
    // energy into the system. The restitution term in the contact RHS
    // amplifies angular velocity contributions at the contact point,
    // creating impulses that increase total kinetic energy at each bounce.
    //
    // Strategy: Only clamp the angular component of the impulse.
    // The translational impulse (normal push) must be preserved fully
    // to maintain momentum conservation (Newton's 3rd law). The energy
    // injection specifically comes from angular velocity amplification
    // through the restitution + lever arm interaction.
    //
    // For resting contacts, the translational impulse counteracts gravity
    // and must not be reduced. The angular torque in resting contacts is
    // typically small and clamping it has no adverse effect.
    {
      InertialState& state = inertialAssets_[k].getInertialState();
      const double mass = inertialAssets_[k].getMass();
      const Eigen::Matrix3d I_inv_world =
        inertialAssets_[k].getInverseInertiaTensorWorld();
      const Eigen::Matrix3d R = state.orientation.toRotationMatrix();
      const Eigen::Matrix3d I_world =
        R * inertialAssets_[k].getInertiaTensor() * R.transpose();

      const Eigen::Vector3d v = state.velocity;
      const Eigen::Vector3d omega = state.getAngularVelocity();

      // Velocity change from constraint impulse
      const Eigen::Vector3d dv =
        Eigen::Vector3d{totalLinear} * dt / mass;
      const Eigen::Vector3d domega =
        I_inv_world * Eigen::Vector3d{totalAngular} * dt;

      // Pre-impulse kinetic energy
      const double KE_pre =
        0.5 * mass * v.squaredNorm() + 0.5 * omega.dot(I_world * omega);

      // Post-impulse velocities (constraint only)
      const Eigen::Vector3d v_post = v + dv;
      const Eigen::Vector3d omega_post = omega + domega;
      const double KE_post = 0.5 * mass * v_post.squaredNorm() +
                             0.5 * omega_post.dot(I_world * omega_post);

      // Compute post-impulse KE with only the translational component
      // to determine if the angular impulse is the source of injection
      const double KE_trans_only = 0.5 * mass * v_post.squaredNorm() +
                                   0.5 * omega.dot(I_world * omega);

      double angularScale = 1.0;
      if (KE_post > KE_pre && KE_post > KE_trans_only && KE_pre > 1e-12)
      {
        // The angular impulse is injecting energy beyond what the
        // translational impulse produces. Scale it down so total KE
        // matches what we'd have with translational impulse alone.
        //
        // Target: KE_target = max(KE_pre, KE_trans_only)
        // This allows the translational impulse to set the energy budget,
        // and the angular impulse cannot add to it.
        const double KE_target = std::max(KE_pre, KE_trans_only);

        // Rotational KE after angular clamp must equal:
        // KE_target - KE_trans = KE_target - 0.5*m*v_post²
        const double KE_trans_post = 0.5 * mass * v_post.squaredNorm();
        const double KE_rot_target = KE_target - KE_trans_post;
        const double KE_rot_post =
          0.5 * omega_post.dot(I_world * omega_post);

        if (KE_rot_post > 1e-12 && KE_rot_target > 0.0)
        {
          // Scale angular velocity change to achieve target rotational KE
          // omega_clamped = omega + angularScale * domega
          // We need: 0.5 * omega_clamped^T I omega_clamped = KE_rot_target
          // Approximate: scale the domega contribution
          angularScale = std::sqrt(KE_rot_target / KE_rot_post);
          // Ensure we don't amplify
          angularScale = std::min(angularScale, 1.0);
        }
        else
        {
          // Zero out angular impulse entirely
          angularScale = 0.0;
        }

        spdlog::debug("[0035da] Energy clamp: Body[{}] KE_pre={:.6f}, "
                      "KE_post={:.6f}, KE_trans_only={:.6f}, "
                      "angularScale={:.6f}",
                      k,
                      KE_pre,
                      KE_post,
                      KE_trans_only,
                      angularScale);
      }

      // Apply translational impulse at full strength (preserves momentum)
      const Eigen::Vector3d impulse =
        Eigen::Vector3d{totalLinear} * dt;
      // Apply angular impulse (scaled if it would inject energy)
      const Eigen::Vector3d deltaOmega =
        I_inv_world * Eigen::Vector3d{totalAngular} * dt * angularScale;

      inertialAssets_[k].applyImpulse(Coordinate{impulse});
      inertialAssets_[k].applyAngularImpulse(AngularRate{deltaOmega});
    }
  }
  // Ticket: 0032_contact_constraint_refactor
  // Constraint forces accumulated here are integrated in updatePhysics()
}


uint32_t WorldModel::getInertialAssetId()
{
  return ++inertialAssetIdCounter_;
}

// ========== Potential Energy Configuration (ticket 0030) ==========

void WorldModel::addPotentialEnergy(std::unique_ptr<PotentialEnergy> energy)
{
  potentialEnergies_.push_back(std::move(energy));
}

void WorldModel::clearPotentialEnergies()
{
  potentialEnergies_.clear();
}

// ========== Integrator Configuration (ticket 0030) ==========

void WorldModel::setIntegrator(std::unique_ptr<Integrator> integrator)
{
  integrator_ = std::move(integrator);
}

}  // namespace msd_sim
