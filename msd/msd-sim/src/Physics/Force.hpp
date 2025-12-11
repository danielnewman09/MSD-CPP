#ifndef MSD_SIM_PHYSICS_FORCE_HPP
#define MSD_SIM_PHYSICS_FORCE_HPP

#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim {

/**
 * @brief Represents a force vector applied at a specific point in 3D space.
 *
 * A Force consists of a force vector (magnitude and direction) and a
 * point of application. The point of application is specified in the
 * local reference frame of the object's geometry (i.e., relative to
 * the object's center of mass or geometric origin).
 */
class Force
{
public:
  /**
   * @brief Default constructor - zero force at origin.
   */
  Force();

  /**
   * @brief Construct a force with specified vector and application point.
   *
   * @param forceVector The force vector in Newtons [N]
   * @param applicationPoint The point where the force is applied,
   *                         specified in the object's local frame
   */
  Force(const Coordinate& forceVector, const Coordinate& applicationPoint);

  /**
   * @brief Get the force vector.
   * @return The force vector in Newtons [N]
   */
  const Coordinate& getForceVector() const;

  /**
   * @brief Get the point of application.
   * @return The application point in the object's local frame
   */
  const Coordinate& getApplicationPoint() const;

  /**
   * @brief Set the force vector.
   * @param forceVector The new force vector in Newtons [N]
   */
  void setForceVector(const Coordinate& forceVector);

  /**
   * @brief Set the point of application.
   * @param applicationPoint The new application point in local frame
   */
  void setApplicationPoint(const Coordinate& applicationPoint);

  /**
   * @brief Calculate the torque (moment) produced by this force about a point.
   *
   * Torque is computed as: τ = r × F
   * where r is the vector from the reference point to the application point,
   * and F is the force vector.
   *
   * @param referencePoint The point about which to calculate torque (typically
   * center of mass)
   * @return The torque vector in Newton-meters [N⋅m]
   */
  Coordinate calculateTorque(const Coordinate& referencePoint) const;

  /**
   * @brief Transform this force to a different reference frame.
   *
   * @param rotation Rotation matrix from current frame to target frame
   * @param translation Translation vector from current origin to target origin
   */
  void transformToFrame(const Eigen::Matrix3f& rotation,
                        const Coordinate& translation);

private:
  Coordinate forceVector_;       // Force vector in Newtons [N]
  Coordinate applicationPoint_;  // Point of application in local frame [m]
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_FORCE_HPP
