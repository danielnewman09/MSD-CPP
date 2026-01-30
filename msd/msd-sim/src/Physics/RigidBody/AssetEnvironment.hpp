#ifndef MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP
#define MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP

#include <memory>
#include <Eigen/Dense>
#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Stationary geometric element with no rigid body dynamics.
 *
 * AssetEnvironment represents static, immovable objects in the simulation
 * environment. These assets inherit visual geometry, convex hull, and
 * reference frame from AssetPhysical, but have no rigid body properties
 * and cannot be moved by forces or interactions.
 *
 * Key features:
 * - Visual geometry for rendering (inherited)
 * - Convex hull for collision detection (inherited)
 * - Reference frame defining position and orientation (inherited)
 * - Always stationary (no dynamics)
 * - Lightweight compared to InertialAsset (no mass properties)
 * - Provides zero mass/inertia for unified solver path (Ticket 0032)
 *
 * Typical use cases:
 * - Terrain and ground surfaces
 * - Buildings and structures
 * - Static obstacles
 * - Environmental boundaries
 * - Walls and barriers
 *
 * Usage pattern:
 * @code
 * auto geometry = std::make_shared<msd_assets::Geometry>(...);
 * ReferenceFrame frame(Coordinate(10, 0, 0));  // Position at x=10
 * auto asset = AssetEnvironment::create(geometry, frame);
 *
 * // Use for rendering
 * renderer.draw(asset->getVisualGeometry(), asset->getReferenceFrame());
 *
 * // Use for collision detection
 * if (asset->getCollisionHull().contains(point)) {
 *   // Handle collision with static environment
 * }
 * @endcode
 *
 * @ticket 0032_contact_constraint_refactor
 */
class AssetEnvironment : public AssetPhysical
{
public:
  /**
   * @brief Constructor with default restitution
   *
   * @param assetId Asset type ID
   * @param instanceId Unique instance ID
   * @param hull Collision hull reference
   * @param frame Reference frame (position and orientation)
   */
  AssetEnvironment(uint32_t assetId,
                   uint32_t instanceId,
                   ConvexHull& hull,
                   const ReferenceFrame& frame);

  /**
   * @brief Constructor with custom restitution
   *
   * @param assetId Asset type ID
   * @param instanceId Unique instance ID
   * @param hull Collision hull reference
   * @param frame Reference frame (position and orientation)
   * @param coefficientOfRestitution Coefficient of restitution [0, 1]
   */
  AssetEnvironment(uint32_t assetId,
                   uint32_t instanceId,
                   ConvexHull& hull,
                   const ReferenceFrame& frame,
                   double coefficientOfRestitution);

  ~AssetEnvironment() = default;

  // ===== Mass properties for unified solver path (Ticket 0032) =====

  /**
   * @brief Get inverse mass (infinite mass representation)
   * @return 0.0 (infinite mass)
   */
  double getInverseMass() const { return 0.0; }

  /**
   * @brief Get inverse inertia tensor (infinite inertia representation)
   * @return Zero matrix (infinite inertia)
   */
  const Eigen::Matrix3d& getInverseInertiaTensor() const { return kZeroInertia; }

  /**
   * @brief Get static inertial state (zero velocity)
   * @return Static state with position from frame, zero velocities
   */
  const InertialState& getInertialState() const { return static_state_; }

  // ===== Restitution for collision response =====

  /**
   * @brief Get coefficient of restitution
   * @return Coefficient of restitution [0, 1]
   */
  double getCoefficientOfRestitution() const { return coefficient_of_restitution_; }

  /**
   * @brief Set coefficient of restitution
   * @param e Coefficient of restitution [0, 1]
   * @throws std::invalid_argument if e not in [0, 1]
   */
  void setCoefficientOfRestitution(double e);

  // Rule of Five
  AssetEnvironment(const AssetEnvironment&) = delete;
  AssetEnvironment& operator=(const AssetEnvironment&) = delete;
  AssetEnvironment(AssetEnvironment&&) noexcept = default;
  AssetEnvironment& operator=(AssetEnvironment&&) noexcept = delete;

private:
  static const Eigen::Matrix3d kZeroInertia;
  InertialState static_state_;  // Zero velocity, position from frame
  double coefficient_of_restitution_{0.5};
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ENVIRONMENT_ASSET_HPP
