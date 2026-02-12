// Ticket: 0039a_energy_tracking_diagnostic_infrastructure

#include "msd-sim/src/Diagnostics/EnergyTracker.hpp"

#include <cmath>

#include "msd-sim/src/Physics/PotentialEnergy/PotentialEnergy.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

// ========== BodyEnergy ==========

msd_transfer::EnergyRecord EnergyTracker::BodyEnergy::toRecord(
  uint32_t frameId,
  uint32_t bodyId) const
{
  msd_transfer::EnergyRecord record{};
  record.body.id = bodyId;  // Ticket: 0056i_static_asset_recording_and_fk
  record.linear_ke = linearKE;
  record.rotational_ke = rotationalKE;
  record.potential_e = potentialE;
  record.total_e = total();
  record.frame.id = frameId;
  return record;
}

// ========== SystemEnergy ==========

msd_transfer::SystemEnergyRecord EnergyTracker::SystemEnergy::toRecord(
  uint32_t frameId,
  double previousSystemEnergy,
  bool collisionActive) const
{
  double const currentTotal = total();
  double const deltaE = currentTotal - previousSystemEnergy;

  msd_transfer::SystemEnergyRecord record{};
  record.total_linear_ke = totalLinearKE;
  record.total_rotational_ke = totalRotationalKE;
  record.total_potential_e = totalPotentialE;
  record.total_system_e = currentTotal;
  record.delta_e = deltaE;
  record.energy_injection =
    isEnergyInjection(currentTotal, previousSystemEnergy) ? 1U : 0U;
  record.collision_active = collisionActive ? 1U : 0U;
  record.frame.id = frameId;
  return record;
}

// ========== EnergyTracker ==========

EnergyTracker::BodyEnergy EnergyTracker::computeBodyEnergy(
  const InertialState& state,
  double mass,
  const Eigen::Matrix3d& bodyInertia,
  std::span<const std::unique_ptr<PotentialEnergy>> potentialEnergies)
{
  BodyEnergy result{};

  // Linear KE: 0.5 * m * v^2
  // Ticket: 0039a_energy_tracking_diagnostic_infrastructure
  Eigen::Vector3d const velocity{state.velocity.x(),
                                 state.velocity.y(),
                                 state.velocity.z()};
  result.linearKE = 0.5 * mass * velocity.squaredNorm();

  // Rotational KE: 0.5 * omega^T * I_world * omega
  // CRITICAL: Transform inertia tensor to world frame
  // I_world = R * I_body * R^T
  Eigen::Matrix3d const R = state.orientation.toRotationMatrix();
  Eigen::Matrix3d const I_world = R * bodyInertia * R.transpose();
  AngularVelocity const omega = state.getAngularVelocity();
  Eigen::Vector3d const omegaVec{omega.x(), omega.y(), omega.z()};
  result.rotationalKE = 0.5 * omegaVec.transpose() * I_world * omegaVec;

  // Potential energy: sum of all potential energy fields
  for (const auto& potential : potentialEnergies)
  {
    result.potentialE += potential->computeEnergy(state, mass);
  }

  return result;
}

EnergyTracker::SystemEnergy EnergyTracker::computeSystemEnergy(
  std::span<const AssetInertial> bodies,
  std::span<const std::unique_ptr<PotentialEnergy>> potentialEnergies)
{
  SystemEnergy result{};

  for (const auto& asset : bodies)
  {
    BodyEnergy const bodyEnergy = computeBodyEnergy(asset.getInertialState(),
                                                    asset.getMass(),
                                                    asset.getInertiaTensor(),
                                                    potentialEnergies);
    result.totalLinearKE += bodyEnergy.linearKE;
    result.totalRotationalKE += bodyEnergy.rotationalKE;
    result.totalPotentialE += bodyEnergy.potentialE;
  }

  return result;
}

bool EnergyTracker::isEnergyInjection(double currentEnergy,
                                      double previousEnergy,
                                      double relativeTolerance,
                                      double absoluteTolerance)
{
  double const deltaE = currentEnergy - previousEnergy;

  // Only flag energy increases (injection), not decreases (dissipation)
  if (deltaE <= 0.0)
  {
    return false;
  }

  // Use the larger of relative and absolute tolerance
  double const relativeThreshold =
    relativeTolerance * std::abs(currentEnergy);
  double const threshold = std::max(relativeThreshold, absoluteTolerance);

  return deltaE > threshold;
}

}  // namespace msd_sim
