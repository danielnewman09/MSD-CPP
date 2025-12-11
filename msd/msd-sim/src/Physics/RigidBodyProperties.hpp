#ifndef MSD_SIM_PHYSICS_RIGID_BODY_PROPERTIES_HPP
#define MSD_SIM_PHYSICS_RIGID_BODY_PROPERTIES_HPP

#include "msd-sim/src/Environment/Coordinate.hpp"
#include <Eigen/Dense>

namespace msd_sim {

/**
 * @brief Physical properties of a rigid body for dynamics calculations.
 *
 * Contains mass, inertia tensor, and center of mass information needed
 * to compute how forces and torques affect a rigid body's motion.
 * Assumes homogeneous density.
 */
class RigidBodyProperties
{
public:
  /**
   * @brief Default constructor - creates a unit mass point at origin.
   */
  RigidBodyProperties();

  /**
   * @brief Construct with specified mass and inertia.
   *
   * @param mass Total mass in kilograms [kg]
   * @param inertiaTensor 3x3 inertia tensor in body frame [kg⋅m²]
   * @param centerOfMass Center of mass in local geometric frame [m]
   */
  RigidBodyProperties(float mass,
                      const Eigen::Matrix3f& inertiaTensor,
                      const Coordinate& centerOfMass = Coordinate(0, 0, 0));

  /**
   * @brief Get the mass.
   * @return Mass in kilograms [kg]
   */
  float getMass() const;

  /**
   * @brief Get the inertia tensor.
   * @return 3x3 inertia tensor in body frame [kg⋅m²]
   */
  const Eigen::Matrix3f& getInertiaTensor() const;

  /**
   * @brief Get the inverse inertia tensor (cached for efficiency).
   * @return 3x3 inverse inertia tensor [1/(kg⋅m²)]
   */
  const Eigen::Matrix3f& getInverseInertiaTensor() const;

  /**
   * @brief Get the center of mass position.
   * @return Center of mass in local frame [m]
   */
  const Coordinate& getCenterOfMass() const;

  /**
   * @brief Get the inverse mass (cached for efficiency).
   * @return 1/mass [1/kg], or 0 if mass is infinite
   */
  float getInverseMass() const;

  /**
   * @brief Set the mass.
   * @param mass New mass in kilograms [kg]
   */
  void setMass(float mass);

  /**
   * @brief Set the inertia tensor.
   *
   * The inertia tensor should be specified in the body's local frame,
   * typically with axes aligned to principal axes for a diagonal tensor.
   *
   * @param inertiaTensor New 3x3 inertia tensor [kg⋅m²]
   */
  void setInertiaTensor(const Eigen::Matrix3f& inertiaTensor);

  /**
   * @brief Set the center of mass position.
   * @param centerOfMass New center of mass in local frame [m]
   */
  void setCenterOfMass(const Coordinate& centerOfMass);

  /**
   * @brief Check if this body has infinite mass (immovable).
   * @return true if mass is infinite, false otherwise
   */
  bool isImmovable() const;

  /**
   * @brief Create a rigid body with infinite mass (for static objects).
   * @return RigidBodyProperties representing an immovable object
   */
  static RigidBodyProperties createImmovable();

  /**
   * @brief Create properties for a solid sphere.
   *
   * Inertia tensor: I = (2/5) * m * r² * Identity
   *
   * @param mass Mass in kilograms [kg]
   * @param radius Sphere radius in meters [m]
   * @return RigidBodyProperties for the sphere
   */
  static RigidBodyProperties createSphere(float mass, float radius);

  /**
   * @brief Create properties for a solid box.
   *
   * Inertia tensor diagonal: I_xx = (1/12) * m * (h² + d²), etc.
   *
   * @param mass Mass in kilograms [kg]
   * @param width Box width (x-dimension) in meters [m]
   * @param height Box height (y-dimension) in meters [m]
   * @param depth Box depth (z-dimension) in meters [m]
   * @return RigidBodyProperties for the box
   */
  static RigidBodyProperties createBox(float mass,
                                       float width,
                                       float height,
                                       float depth);

  /**
   * @brief Create properties for a solid cylinder.
   *
   * Assumes cylinder axis is along z-axis.
   * I_xx = I_yy = (1/12) * m * (3*r² + h²)
   * I_zz = (1/2) * m * r²
   *
   * @param mass Mass in kilograms [kg]
   * @param radius Cylinder radius in meters [m]
   * @param height Cylinder height in meters [m]
   * @return RigidBodyProperties for the cylinder
   */
  static RigidBodyProperties createCylinder(float mass,
                                            float radius,
                                            float height);

  /**
   * @brief Create properties from a convex hull.
   *
   * Computes mass properties (inertia tensor and center of mass) from a
   * convex hull by treating it as a solid polyhedron with uniform density.
   * Uses tetrahedral decomposition for accurate inertia calculation.
   *
   * @param hull Convex hull to compute properties from
   * @param mass Total mass in kilograms [kg]
   * @return RigidBodyProperties computed from the hull geometry
   * @throws std::invalid_argument if hull is invalid or has zero volume
   */
  static RigidBodyProperties fromConvexHull(const class ConvexHull& hull,
                                            float mass);

private:
  float mass_;                      // Mass [kg]
  float inverseMass_;               // 1/mass [1/kg] (cached)
  Eigen::Matrix3f inertiaTensor_;   // Inertia tensor [kg⋅m²]
  Eigen::Matrix3f inverseInertia_;  // Inverse inertia tensor (cached)
  Coordinate centerOfMass_;         // Center of mass in local frame [m]

  /**
   * @brief Update cached inverse values.
   *
   * Called automatically when mass or inertia tensor changes.
   */
  void updateInverseValues();
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_RIGID_BODY_PROPERTIES_HPP
