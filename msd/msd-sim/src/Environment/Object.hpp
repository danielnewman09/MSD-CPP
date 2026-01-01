#ifndef MSD_Object_HPP
#define MSD_Object_HPP

#include <functional>
#include <optional>
#include <msd-assets/src/Asset.hpp>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/EulerAngles.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"
#include "msd-sim/src/Physics/RigidBody/PhysicsComponent.hpp"

namespace msd_sim
{

/**
 * @brief Unified object type supporting various simulation roles
 *
 * Object uses a component-based design to support four distinct types:
 *
 * 1. Graphical-Only: Visual rendering without physics
 *    - Has: Asset reference, ReferenceFrame, Color
 *    - No: Physics, Collision
 *
 * 2. Inertial: Full dynamic physics simulation
 *    - Has: Asset, ReferenceFrame, Color, PhysicsComponent, ConvexHull
 *    - Participates in: Rendering, physics integration, collision detection
 *
 * 3. Environmental: Static collision objects
 *    - Has: Asset, ReferenceFrame, Color, ConvexHull
 *    - No: PhysicsComponent (cannot move)
 *    - Participates in: Rendering, collision detection
 *
 * 4. Boundary: Invisible collision-only boundaries
 *    - Has: ReferenceFrame, ConvexHull
 *    - No: Asset (no visual), PhysicsComponent
 *    - Participates in: Collision detection only
 *
 * The component-based design allows efficient iteration:
 * - Physics loop: iterate only objects with PhysicsComponent
 * - Collision: iterate only objects with ConvexHull
 * - Rendering: iterate only objects with valid Asset
 *
 * Usage:
 * @code
 * // Graphical-only object
 * auto visual = Object::createGraphical(asset, frame, color);
 *
 * // Dynamic physics object
 * auto dynamic = Object::createInertial(asset, frame, mass, color);
 *
 * // Static environment object
 * auto environment = Object::createEnvironmental(asset, frame, color);
 *
 * // Invisible boundary wall
 * auto boundary = Object::createBoundary(collisionHull, frame);
 * @endcode
 */
class Object
{
public:
  /**
   * @brief Object type tags for runtime identification
   */
  enum class Type : uint8_t
  {
    Graphical,     // Visual only
    Inertial,      // Dynamic physics + collision + visual
    Environmental, // Static collision + visual
    Boundary       // Collision only (invisible)
  };

  // ========== Factory Methods ==========

  /**
   * @brief Create a graphical-only object (no physics or collision)
   *
   * @param asset Asset reference for visual geometry
   * @param frame Initial position and orientation
   * @param r Red component [0-1]
   * @param g Green component [0-1]
   * @param b Blue component [0-1]
   * @return Graphical object
   */
  static Object createGraphical(const msd_assets::Asset& asset,
                                const ReferenceFrame& frame = ReferenceFrame(),
                                float r = 1.0f,
                                float g = 1.0f,
                                float b = 1.0f);

  /**
   * @brief Create a dynamic inertial object (full physics simulation)
   *
   * Creates an object with:
   * - Visual geometry from asset
   * - Collision hull computed from asset's collision geometry
   * - Physics properties (mass, inertia) computed from collision hull
   * - Dynamic state initialized at rest
   *
   * @param asset Asset reference (must have collision geometry)
   * @param frame Initial position and orientation
   * @param mass Mass in kg (must be > 0)
   * @param r Red component [0-1]
   * @param g Green component [0-1]
   * @param b Blue component [0-1]
   * @return Inertial object
   * @throws std::invalid_argument if asset lacks collision geometry or mass <=
   * 0
   */
  static Object createInertial(const msd_assets::Asset& asset,
                               const ReferenceFrame& frame,
                               double mass,
                               float r = 1.0f,
                               float g = 1.0f,
                               float b = 1.0f);

  /**
   * @brief Create a static environmental object (collision but no dynamics)
   *
   * @param asset Asset reference (must have collision geometry)
   * @param frame Position and orientation
   * @param r Red component [0-1]
   * @param g Green component [0-1]
   * @param b Blue component [0-1]
   * @return Environmental object
   * @throws std::invalid_argument if asset lacks collision geometry
   */
  static Object createEnvironmental(const msd_assets::Asset& asset,
                                    const ReferenceFrame& frame =
                                      ReferenceFrame(),
                                    float r = 1.0f,
                                    float g = 1.0f,
                                    float b = 1.0f);

  /**
   * @brief Create an invisible boundary (collision only, no visual)
   *
   * @param collisionHull Convex hull for collision detection
   * @param frame Position and orientation
   * @return Boundary object
   */
  static Object createBoundary(const ConvexHull& collisionHull,
                               const ReferenceFrame& frame =
                                 ReferenceFrame());

  // ========== Component Access ==========

  /**
   * @brief Get object type
   */
  Type getType() const { return type_; }

  /**
   * @brief Check if object has visual geometry
   */
  bool hasVisualGeometry() const;

  /**
   * @brief Check if object has collision hull
   */
  bool hasCollision() const { return collisionHull_.has_value(); }

  /**
   * @brief Check if object has physics component
   */
  bool hasPhysics() const { return physics_.has_value(); }

  /**
   * @brief Get asset reference (if present)
   * @return Optional asset reference
   */
  std::optional<std::reference_wrapper<const msd_assets::Asset>>
  getAsset() const;

  /**
   * @brief Get visual geometry (if present)
   * @return Pointer to visual geometry, or nullptr
   */
  const msd_assets::VisualGeometry* getVisualGeometry() const;

  /**
   * @brief Get collision hull (const)
   * @throws std::runtime_error if no collision hull
   */
  const ConvexHull& getCollisionHull() const;

  /**
   * @brief Get collision hull (mutable)
   * @throws std::runtime_error if no collision hull
   */
  ConvexHull& getCollisionHull();

  /**
   * @brief Get physics component (const)
   * @throws std::runtime_error if no physics component
   */
  const PhysicsComponent& getPhysics() const;

  /**
   * @brief Get physics component (mutable)
   * @throws std::runtime_error if no physics component
   */
  PhysicsComponent& getPhysics();

  // ========== Transform and Appearance ==========

  ReferenceFrame& getTransform() { return transform_; }
  const ReferenceFrame& getTransform() const { return transform_; }

  void setPosition(const Coordinate& position)
  {
    transform_.setOrigin(position);
  }
  const Coordinate& getPosition() const { return transform_.getOrigin(); }

  void setRotation(const EulerAngles& euler) { transform_.setRotation(euler); }
  EulerAngles& getRotation() { return transform_.getEulerAngles(); }

  void setColor(float r, float g, float b);
  void getColor(float& r, float& g, float& b) const;

private:
  // Private constructor - use factory methods
  Object(Type type,
         std::optional<std::reference_wrapper<const msd_assets::Asset>> asset,
         const ReferenceFrame& frame,
         std::optional<PhysicsComponent> physics,
         std::optional<ConvexHull> collisionHull,
         float r,
         float g,
         float b);

  Type type_;

  // Asset reference (optional - not present for boundary objects)
  std::optional<std::reference_wrapper<const msd_assets::Asset>> asset_;

  // Transform (always present)
  ReferenceFrame transform_;

  // Optional components
  std::optional<PhysicsComponent> physics_;
  std::optional<ConvexHull> collisionHull_;

  // Appearance
  float color_[3];
};

}  // namespace msd_sim

#endif  // MSD_Object_HPP
