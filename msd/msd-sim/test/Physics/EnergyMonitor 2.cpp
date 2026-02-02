// Ticket: 0035d_friction_hardening_and_validation
// Design: docs/designs/0035d_friction_hardening_and_validation/design.md

#include "EnergyMonitor.hpp"
#include <cmath>

namespace msd_sim
{

double EnergyMonitor::computeLinearKE(
    const std::vector<InertialState>& states,
    const std::vector<double>& masses)
{
  double totalKE = 0.0;

  for (size_t i = 0; i < states.size(); ++i)
  {
    const Coordinate& v = states[i].velocity;
    const double m = masses[i];

    // KE_linear = 0.5 * m * v^2
    const double vSquared = v.squaredNorm();
    totalKE += 0.5 * m * vSquared;
  }

  return totalKE;
}

double EnergyMonitor::computeAngularKE(
    const std::vector<InertialState>& states,
    const std::vector<Eigen::Matrix3d>& inertias)
{
  double totalKE = 0.0;

  for (size_t i = 0; i < states.size(); ++i)
  {
    const AngularRate omega = states[i].getAngularVelocity();
    const Eigen::Matrix3d& I = inertias[i];

    // KE_angular = 0.5 * omega^T * I * omega
    const Eigen::Vector3d omegaVec{omega.x(), omega.y(), omega.z()};
    const double keAngular = 0.5 * omegaVec.dot(I * omegaVec);
    totalKE += keAngular;
  }

  return totalKE;
}

double EnergyMonitor::computeTotalKE(
    const std::vector<InertialState>& states,
    const std::vector<double>& masses,
    const std::vector<Eigen::Matrix3d>& inertias)
{
  return computeLinearKE(states, masses) + computeAngularKE(states, inertias);
}

bool EnergyMonitor::validateNonIncreasing(
    double E_before,
    double E_after,
    double tolerance)
{
  // Energy should never increase (friction dissipates energy)
  // E_after <= E_before + tolerance
  return E_after <= (E_before + tolerance);
}

}  // namespace msd_sim
