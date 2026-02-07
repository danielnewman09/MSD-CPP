// Ticket: 0039a_energy_tracking_diagnostic_infrastructure

#ifndef MSD_SIM_DIAGNOSTICS_ENERGY_TRACKER_HPP
#define MSD_SIM_DIAGNOSTICS_ENERGY_TRACKER_HPP

#include <Eigen/Dense>
#include <cstdint>
#include <span>

#include "msd-transfer/src/EnergyRecord.hpp"
#include "msd-transfer/src/SystemEnergyRecord.hpp"

namespace msd_sim
{

// Forward declaration
class AssetInertial;
struct InertialState;
class PotentialEnergy;

/**
 * @brief Energy computation utility for rigid body diagnostics
 *
 * Provides static methods to compute kinetic and potential energy for
 * individual bodies and the entire system. Used by WorldModel to record
 * energy data for collision debugging and anomaly detection.
 *
 * CRITICAL: Rotational kinetic energy uses world-frame inertia tensor:
 *   KE_rot = 0.5 * omega^T * I_world * omega
 *   where I_world = R * I_body * R^T
 *
 * Using body-frame inertia with world-frame angular velocity produces
 * chaotic-looking energy even when physics are correct.
 *
 * @ticket 0039a_energy_tracking_diagnostic_infrastructure
 */
class EnergyTracker
{
public:
  /**
   * @brief Per-body energy breakdown
   */
  struct BodyEnergy
  {
    double linearKE{0.0};      // Linear kinetic energy [J]
    double rotationalKE{0.0};  // Rotational kinetic energy [J]
    double potentialE{0.0};    // Potential energy [J]

    /**
     * @brief Total mechanical energy for this body
     * @return linearKE + rotationalKE + potentialE [J]
     */
    [[nodiscard]] double total() const
    {
      return linearKE + rotationalKE + potentialE;
    }

    /**
     * @brief Convert to database record
     * @param frameId Frame ID for foreign key
     * @param bodyId Stable body identifier
     * @return EnergyRecord for database insertion
     */
    [[nodiscard]] msd_transfer::EnergyRecord toRecord(uint32_t frameId,
                                                       uint32_t bodyId) const;
  };

  /**
   * @brief System-level energy summary
   */
  struct SystemEnergy
  {
    double totalLinearKE{0.0};      // Sum of all body linear KE [J]
    double totalRotationalKE{0.0};  // Sum of all body rotational KE [J]
    double totalPotentialE{0.0};    // Sum of all body PE [J]

    /**
     * @brief Total system mechanical energy
     * @return totalLinearKE + totalRotationalKE + totalPotentialE [J]
     */
    [[nodiscard]] double total() const
    {
      return totalLinearKE + totalRotationalKE + totalPotentialE;
    }

    /**
     * @brief Convert to database record
     * @param frameId Frame ID for foreign key
     * @param previous Previous frame's system energy (for delta calculation)
     * @param collisionActive Whether any collision was detected this frame
     * @return SystemEnergyRecord for database insertion
     */
    [[nodiscard]] msd_transfer::SystemEnergyRecord toRecord(
      uint32_t frameId,
      double previousSystemEnergy,
      bool collisionActive) const;
  };

  /**
   * @brief Compute energy for a single rigid body
   *
   * @param state Current kinematic state
   * @param mass Body mass [kg]
   * @param bodyInertia Inertia tensor in body frame [kg*m^2]
   * @param potentialEnergies Potential energy fields to evaluate
   * @return BodyEnergy with all energy components
   */
  static BodyEnergy computeBodyEnergy(
    const InertialState& state,
    double mass,
    const Eigen::Matrix3d& bodyInertia,
    std::span<const std::unique_ptr<PotentialEnergy>> potentialEnergies);

  /**
   * @brief Compute total system energy across all bodies
   *
   * @param bodies All inertial assets in the simulation
   * @param potentialEnergies Potential energy fields to evaluate
   * @return SystemEnergy with aggregated energy values
   */
  static SystemEnergy computeSystemEnergy(
    std::span<const AssetInertial> bodies,
    std::span<const std::unique_ptr<PotentialEnergy>> potentialEnergies);

  /**
   * @brief Check if energy change exceeds tolerance (anomaly detection)
   *
   * Uses the larger of relative and absolute tolerance:
   * - Relative: relativeTolerance * |currentEnergy|
   * - Absolute: absoluteTolerance
   *
   * Energy increase beyond tolerance indicates energy injection (anomaly).
   * Energy decrease is allowed (dissipation is physical).
   *
   * @param currentEnergy Current total system energy [J]
   * @param previousEnergy Previous frame total system energy [J]
   * @param relativeTolerance Relative tolerance (default 1e-6)
   * @param absoluteTolerance Absolute tolerance [J] (default 1e-9)
   * @return true if energy increased beyond tolerance
   */
  static bool isEnergyInjection(double currentEnergy,
                                double previousEnergy,
                                double relativeTolerance = 1e-6,
                                double absoluteTolerance = 1e-9);
};

}  // namespace msd_sim

#endif  // MSD_SIM_DIAGNOSTICS_ENERGY_TRACKER_HPP
