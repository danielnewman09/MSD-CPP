// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0055c_friction_direction_fix/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_VERTEX_FACE_DETECTOR_HPP
#define MSD_SIM_PHYSICS_COLLISION_VERTEX_FACE_DETECTOR_HPP

#include <cstddef>

namespace msd_sim
{

/**
 * @brief Contact geometry classification for manifold generation.
 *
 * Classifies the geometry of a collision contact based on the number of
 * vertices in the reference and incident polygons extracted during EPA
 * clipping or SAT witness point computation.
 *
 * This classification determines which contact manifold generation strategy
 * to use:
 * - FaceFace: Sutherland-Hodgman clipping (3-4 contacts)
 * - EdgeEdge: Edge manifold generation (2 contacts)
 * - VertexFace: Vertex-face manifold generation (3-4 contacts)
 * - VertexVertex: Single-point fallback (1 contact)
 *
 * @see docs/designs/0055c_friction_direction_fix/0055c_friction_direction_fix.puml
 * @ticket 0055c_friction_direction_fix
 */
enum class ContactType
{
  FaceFace,     ///< Both sides have >= 3 vertices
  EdgeEdge,     ///< Both sides have exactly 2 vertices
  VertexFace,   ///< One side has 1 vertex, other has >= 3
  VertexVertex, ///< Both sides have exactly 1 vertex
  Unknown       ///< Fallback case (unexpected geometry)
};

/**
 * @brief Detects vertex-face contact geometry for manifold generation.
 *
 * Analyzes polygon sizes from EPA clipping or SAT witness points to classify
 * the contact type. Used by CollisionHandler and EPA to determine whether
 * to generate multi-point vertex-face manifolds instead of single-point
 * fallbacks.
 *
 * **Design rationale** (from 0055b investigation):
 * Single-point vertex-face contacts with friction create uncompensated yaw
 * torque through Jacobian angular coupling (rA × t). This causes oscillating
 * friction direction → net positive work → energy injection. Multi-point
 * manifolds ensure friction yaw torques cancel.
 *
 * **Thread safety**: Stateless, safe for concurrent use.
 *
 * @see docs/designs/0055c_friction_direction_fix/design.md
 * @ticket 0055c_friction_direction_fix
 */
class VertexFaceDetector
{
public:
  /**
   * @brief Construct a vertex-face detector (stateless).
   */
  VertexFaceDetector() = default;

  /**
   * @brief Analyze polygon sizes and classify contact geometry.
   *
   * Classification rules:
   * - FaceFace: Both >= 3 vertices
   * - EdgeEdge: Both == 2 vertices
   * - VertexFace: One == 1 vertex, other >= 3
   * - VertexVertex: Both == 1 vertex
   * - Unknown: Any other combination (should be rare)
   *
   * @param refVertCount Number of vertices in reference polygon
   * @param incVertCount Number of vertices in incident polygon
   * @return Classified contact type
   */
  [[nodiscard]] ContactType detectContactType(size_t refVertCount,
                                               size_t incVertCount) const;

  /**
   * @brief Convenience method for vertex-face check.
   *
   * @param refVertCount Number of vertices in reference polygon
   * @param incVertCount Number of vertices in incident polygon
   * @return True if contact is vertex-face geometry
   */
  [[nodiscard]] bool isVertexFaceContact(size_t refVertCount,
                                          size_t incVertCount) const;

  VertexFaceDetector(const VertexFaceDetector&) = default;
  VertexFaceDetector& operator=(const VertexFaceDetector&) = default;
  VertexFaceDetector(VertexFaceDetector&&) noexcept = default;
  VertexFaceDetector& operator=(VertexFaceDetector&&) noexcept = default;
  ~VertexFaceDetector() = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_VERTEX_FACE_DETECTOR_HPP
