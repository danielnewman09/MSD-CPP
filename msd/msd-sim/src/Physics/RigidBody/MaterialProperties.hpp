// Ticket: 0056a_collision_force_transfer_records

#ifndef MSD_SIM_PHYSICS_MATERIAL_PROPERTIES_HPP
#define MSD_SIM_PHYSICS_MATERIAL_PROPERTIES_HPP

#include <stdexcept>
#include <string>

namespace msd_sim
{

/**
 * @brief Surface material properties shared by inertial and environment assets.
 *
 * Consolidates coefficient of restitution and friction coefficient with
 * validated setters, eliminating duplicated validation logic between
 * AssetInertial and AssetEnvironment.
 *
 * @ticket 0056a_collision_force_transfer_records
 */
struct MaterialProperties
{
  double coefficientOfRestitution{0.5};
  double frictionCoefficient{0.0};

  /**
   * @brief Set the coefficient of restitution.
   *
   * @param e Coefficient of restitution [0, 1]
   * @throws std::invalid_argument if e not in [0, 1]
   */
  void setCoefficientOfRestitution(double e)
  {
    if (e < 0.0 || e > 1.0)
    {
      throw std::invalid_argument(
        "Coefficient of restitution must be in [0, 1], got: " +
        std::to_string(e));
    }
    coefficientOfRestitution = e;
  }

  /**
   * @brief Set the friction coefficient.
   *
   * @param mu Friction coefficient [0, inf)
   * @throws std::invalid_argument if mu < 0
   */
  void setFrictionCoefficient(double mu)
  {
    if (mu < 0.0)
    {
      throw std::invalid_argument(
        "Friction coefficient must be non-negative, got: " +
        std::to_string(mu));
    }
    frictionCoefficient = mu;
  }
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_MATERIAL_PROPERTIES_HPP
