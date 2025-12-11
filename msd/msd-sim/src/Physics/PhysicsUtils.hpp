#ifndef MSD_SIM_PHYSICS_PHYSICS_UTILS_HPP
#define MSD_SIM_PHYSICS_PHYSICS_UTILS_HPP

#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"
#include "msd-sim/src/Environment/InertialState.hpp"

namespace msd_sim {
namespace physics {

/**
 * @brief Convert angular acceleration vector to EulerAngles rate.
 *
 * Converts an angular acceleration vector (in rad/s²) to the corresponding
 * Euler angle acceleration rates. This is needed because the inertial state
 * stores angular quantities as Euler angles.
 *
 * @param angularAccel Angular acceleration vector [rad/s²]
 * @return EulerAngles representing angular acceleration rates
 */
EulerAngles angularAccelerationToEulerRates(const Coordinate& angularAccel);

/**
 * @brief Convert EulerAngles to angular velocity vector.
 *
 * Converts Euler angle rates to an angular velocity vector in body frame.
 *
 * @param eulerRates Euler angle rates (pitch, roll, yaw rates)
 * @return Angular velocity vector [rad/s]
 */
Coordinate eulerRatesToAngularVelocity(const EulerAngles& eulerRates);

/**
 * @brief Integrate inertial state forward using Euler method.
 *
 * Simple first-order integration:
 * position += velocity * dt
 * velocity += acceleration * dt
 *
 * @param state Current inertial state (modified in place)
 * @param linearAccel Linear acceleration [m/s²]
 * @param angularAccel Angular acceleration [rad/s²]
 * @param dt Time step [s]
 */
void integrateStateEuler(InertialState& state,
                         const Coordinate& linearAccel,
                         const Coordinate& angularAccel,
                         float dt);

/**
 * @brief Integrate inertial state forward using semi-implicit Euler method.
 *
 * Also known as symplectic Euler or Euler-Cromer method:
 * velocity += acceleration * dt
 * position += velocity * dt  (uses updated velocity)
 *
 * More stable than explicit Euler for physics simulation.
 *
 * @param state Current inertial state (modified in place)
 * @param linearAccel Linear acceleration [m/s²]
 * @param angularAccel Angular acceleration [rad/s²]
 * @param dt Time step [s]
 */
void integrateStateSemiImplicitEuler(InertialState& state,
                                      const Coordinate& linearAccel,
                                      const Coordinate& angularAccel,
                                      float dt);

/**
 * @brief Integrate inertial state forward using RK4 method.
 *
 * Fourth-order Runge-Kutta integration for higher accuracy.
 * Requires evaluating accelerations at multiple intermediate points.
 *
 * Note: This signature assumes constant acceleration. For full RK4,
 * you would need to re-evaluate forces at intermediate states.
 *
 * @param state Current inertial state (modified in place)
 * @param linearAccel Linear acceleration [m/s²]
 * @param angularAccel Angular acceleration [rad/s²]
 * @param dt Time step [s]
 */
void integrateStateRK4(InertialState& state,
                       const Coordinate& linearAccel,
                       const Coordinate& angularAccel,
                       float dt);

/**
 * @brief Apply linear acceleration to inertial state.
 *
 * Updates the acceleration field in the state. Typically called before
 * integration.
 *
 * @param state Inertial state to update
 * @param linearAccel Linear acceleration [m/s²]
 */
void applyLinearAcceleration(InertialState& state,
                             const Coordinate& linearAccel);

/**
 * @brief Apply angular acceleration to inertial state.
 *
 * Updates the angular acceleration field in the state.
 *
 * @param state Inertial state to update
 * @param angularAccel Angular acceleration [rad/s²]
 */
void applyAngularAcceleration(InertialState& state,
                              const Coordinate& angularAccel);

/**
 * @brief Check if two coordinates are equal within tolerance.
 *
 * @param c1 First coordinate
 * @param c2 Second coordinate
 * @param tolerance Comparison tolerance [m]
 * @return true if coordinates are equal within tolerance
 */
bool coordinatesEqual(const Coordinate& c1,
                      const Coordinate& c2,
                      float tolerance = 1e-6f);

}  // namespace physics
}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_PHYSICS_UTILS_HPP
