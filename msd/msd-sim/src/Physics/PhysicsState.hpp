#ifndef MSD_SIM_PHYSICS_PHYSICS_STATE_HPP
#define MSD_SIM_PHYSICS_PHYSICS_STATE_HPP

#include "msd-sim/src/Physics/ForceAccumulator.hpp"
#include "msd-sim/src/Physics/RigidBodyProperties.hpp"

namespace msd_sim {

/**
 * @brief Extension to Platform for physics simulation capabilities.
 *
 * PhysicsState contains all physics-related state that can be added
 * to a Platform object:
 * - Rigid body properties (mass, inertia)
 * - Force accumulation
 *
 * This allows Platform objects to opt-in to physics simulation while
 * maintaining backward compatibility.
 */
struct PhysicsState
{
  /**
   * @brief Default constructor - creates a unit mass point.
   */
  PhysicsState();

  /**
   * @brief Construct with specified rigid body properties.
   *
   * @param properties Rigid body properties (mass, inertia, center of mass)
   */
  explicit PhysicsState(const RigidBodyProperties& properties);

  /**
   * @brief Rigid body properties (mass, inertia, center of mass).
   */
  RigidBodyProperties properties;

  /**
   * @brief Force accumulator for current simulation step.
   */
  ForceAccumulator forceAccumulator;

  /**
   * @brief Apply a force to the accumulator.
   *
   * Convenience method that delegates to forceAccumulator.
   *
   * @param force The force to apply
   */
  void applyForce(const Force& force);

  /**
   * @brief Apply multiple forces to the accumulator.
   *
   * @param forces Vector of forces to apply
   */
  void applyForces(const std::vector<Force>& forces);

  /**
   * @brief Clear all accumulated forces.
   */
  void clearForces();

  /**
   * @brief Calculate linear and angular accelerations from accumulated forces.
   *
   * Uses Newton's laws:
   * a = F_net / m
   * α = I^(-1) * τ_net
   *
   * @param centerOfMass Center of mass for torque calculation
   * @param outLinearAccel Output parameter for linear acceleration [m/s²]
   * @param outAngularAccel Output parameter for angular acceleration [rad/s²]
   */
  void calculateAccelerations(const Coordinate& centerOfMass,
                               Coordinate& outLinearAccel,
                               Coordinate& outAngularAccel) const;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_PHYSICS_STATE_HPP
