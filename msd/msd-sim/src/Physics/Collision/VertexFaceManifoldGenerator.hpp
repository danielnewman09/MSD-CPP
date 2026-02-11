// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0055c_friction_direction_fix/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_VERTEX_FACE_MANIFOLD_GENERATOR_HPP
#define MSD_SIM_PHYSICS_COLLISION_VERTEX_FACE_MANIFOLD_GENERATOR_HPP

#include <array>
#include <cstddef>
#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"

namespace msd_sim
{

/**
 * @brief Generates multi-point contact manifolds for vertex-face geometry.
 *
 * When a vertex from one object contacts a face from another object, generating
 * a single contact point at the vertex location creates an uncompensated yaw
 * torque through friction Jacobian angular coupling (rA × t). This causes
 * oscillating friction direction → net positive work → energy injection.
 *
 * **Solution**: Project the reference face vertices onto the contact plane
 * (plane containing incident vertex, perpendicular to contact normal) to
 * generate 3-4 contact points spanning the contact patch. Friction forces at
 * these distributed points have opposing lever arms, ensuring yaw torques cancel.
 *
 * **Algorithm** (from design document):
 * 1. Project reference face vertices onto contact plane
 * 2. Validate projected points form valid convex polygon (3-4 vertices)
 * 3. Compute penetration depth for each projected point (Option A: uniform EPA
 * depth) 4. Build ContactPoint pairs (refPoint, incidentVertex, depth)
 * 5. Return min(projectedPoints.size(), 4) contacts
 *
 * **Design decision** (human-approved): All contact points share the same EPA
 * depth (Option A from design). Per-point depth computation (Option B) can be
 * added later if constraint solver stability requires it.
 *
 * **Thread safety**: Stateless (config is const), safe for concurrent use.
 *
 * @see docs/designs/0055c_friction_direction_fix/design.md
 * @see docs/investigations/0055b_friction_direction_root_cause/investigation-log.md
 * @ticket 0055c_friction_direction_fix
 */
class VertexFaceManifoldGenerator
{
public:
  /**
   * @brief Configuration for manifold generation.
   */
  struct Config
  {
    double depthTolerance{1e-6};  ///< Depth consistency tolerance [m]
    size_t maxContacts{4};        ///< Maximum contacts to generate
  };

  /**
   * @brief Construct generator with default configuration.
   */
  VertexFaceManifoldGenerator() = default;

  /**
   * @brief Construct generator with specified configuration.
   *
   * @param config Generator configuration
   */
  explicit VertexFaceManifoldGenerator(Config config);

  /**
   * @brief Generate contact manifold for vertex-face geometry.
   *
   * Projects reference face vertices onto the contact plane (plane containing
   * incident vertex, perpendicular to contact normal) and builds contact pairs.
   *
   * **Design decision**: All contacts use uniform EPA depth (Option A from
   * design). This is simpler than per-point depth and appropriate for constraint
   * solver enforcement. Per-point depth (Option B) can be added later if needed.
   *
   * @param refFaceVertices Vertices of the reference face (3-4 points, world
   * space)
   * @param incidentVertex The single incident vertex (world space)
   * @param contactNormal Contact normal (from incident toward reference)
   * @param epaDepth EPA penetration depth for consistency [m]
   * @param contacts Output array for generated contacts (preallocated, size 4)
   * @return Number of valid contacts generated (0-4)
   *
   * **Returns 0** if:
   * - Reference face has < 3 vertices (degenerate)
   * - Projection produces invalid geometry
   */
  [[nodiscard]] size_t generate(const std::vector<Coordinate>& refFaceVertices,
                                 const Coordinate& incidentVertex,
                                 const Vector3D& contactNormal,
                                 double epaDepth,
                                 std::array<ContactPoint, 4>& contacts) const;

  VertexFaceManifoldGenerator(const VertexFaceManifoldGenerator&) = default;
  VertexFaceManifoldGenerator& operator=(const VertexFaceManifoldGenerator&) = default;
  VertexFaceManifoldGenerator(VertexFaceManifoldGenerator&&) noexcept = default;
  VertexFaceManifoldGenerator& operator=(VertexFaceManifoldGenerator&&) noexcept =
    default;
  ~VertexFaceManifoldGenerator() = default;

private:
  Config config_;

  /**
   * @brief Project reference face vertices onto contact plane.
   *
   * Contact plane: contains incidentVertex, perpendicular to contactNormal.
   * Projection formula: projectedPoint = faceVertex - ((faceVertex -
   * incidentVertex) · normal) * normal
   *
   * @param faceVertices Reference face vertices (world space)
   * @param planePoint Point on contact plane (incident vertex)
   * @param planeNormal Contact plane normal (contact normal)
   * @return Projected vertices (3-4 points if valid, empty if degenerate)
   */
  [[nodiscard]] std::vector<Coordinate>
  projectFaceOntoPlane(const std::vector<Coordinate>& faceVertices,
                       const Coordinate& planePoint,
                       const Vector3D& planeNormal) const;

  /**
   * @brief Compute per-point penetration depth from incident vertex to face.
   *
   * **Note**: Currently unused (Option A uses uniform EPA depth). Included for
   * future Option B implementation if per-point depth is needed.
   *
   * @param refPoint Reference point on face (world space)
   * @param incidentVertex Incident vertex (world space)
   * @param contactNormal Contact normal
   * @return Penetration depth [m]
   */
  [[nodiscard]] double computePointDepth(const Coordinate& refPoint,
                                          const Coordinate& incidentVertex,
                                          const Vector3D& contactNormal) const;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_VERTEX_FACE_MANIFOLD_GENERATOR_HPP
