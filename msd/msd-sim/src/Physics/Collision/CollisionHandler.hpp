// Ticket: 0027a_expanding_polytope_algorithm
// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_HANDLER_HPP
#define MSD_SIM_PHYSICS_COLLISION_HANDLER_HPP

#include <optional>

#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/Collision/VertexFaceDetector.hpp"
#include "msd-sim/src/Physics/Collision/VertexFaceManifoldGenerator.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"

namespace msd_sim
{

/**
 * @brief Orchestrates collision detection algorithms (GJK/EPA).
 *
 * Provides a unified interface for collision detection that:
 * - Runs GJK to detect intersection
 * - If collision detected, runs EPA to compute contact info
 * - Returns std::optional to indicate collision presence
 *
 * This abstraction allows future enhancements (broadphase,
 * continuous collision detection, etc.) without changing callers.
 *
 * @see
 * docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml
 * @ticket 0027a_expanding_polytope_algorithm
 */
class CollisionHandler
{
public:
  /**
   * @brief Construct handler with specified tolerance.
   *
   * @param epsilon Numerical tolerance for GJK/EPA (default: 1e-6)
   */
  explicit CollisionHandler(double epsilon = 1e-6);

  /**
   * @brief Check for collision between two physical assets.
   *
   * Runs GJK to detect intersection. If collision detected,
   * runs EPA to compute penetration depth, contact normal,
   * and contact point.
   *
   * @param assetA First physical asset
   * @param assetB Second physical asset
   * @param skipSATValidation If true, skip SAT validation of EPA result.
   *        Use when ContactCache confirms persistent contact with stable
   *        normal, making EPA failure unlikely.
   *        Ticket: 0053d_sat_fallback_cost_reduction
   * @return std::nullopt if no collision, CollisionResult if collision
   */
  [[nodiscard]] std::optional<CollisionResult> checkCollision(
    const AssetPhysical& assetA,
    const AssetPhysical& assetB,
    bool skipSATValidation = false) const;

  CollisionHandler(const CollisionHandler&) = default;
  CollisionHandler(CollisionHandler&&) noexcept = default;
  CollisionHandler& operator=(const CollisionHandler&) = default;
  CollisionHandler& operator=(CollisionHandler&&) noexcept = default;
  ~CollisionHandler() = default;

private:
  /// @brief SAT result: minimum penetration depth and corresponding normal
  ///
  /// Ticket: 0047_face_contact_manifold_generation
  struct SATResult
  {
    double depth;
    Vector3D normal;
  };

  /// @brief Compute minimum penetration depth and direction using SAT
  ///
  /// Ticket: 0047_face_contact_manifold_generation
  ///
  /// Iterates over all unique face normals of both hulls and computes the
  /// overlap (penetration depth) along each. Returns the minimum depth and
  /// the corresponding face normal.
  ///
  /// Used to validate and correct EPA results — EPA can produce wrong
  /// results at zero or near-zero penetration when the Minkowski difference
  /// origin is on the boundary.
  [[nodiscard]] SATResult computeSATMinPenetration(
    const AssetPhysical& assetA,
    const AssetPhysical& assetB) const;

  /// @brief Build a CollisionResult from SAT data when EPA fails
  ///
  /// Ticket: 0047_face_contact_manifold_generation
  /// Ticket: 0055c_friction_direction_fix
  ///
  /// When EPA picks the wrong face (depth wildly inconsistent with SAT),
  /// construct a valid contact using the SAT normal and depth with witness
  /// points from the support function.
  ///
  /// For vertex-face contacts, generates multi-point manifold to prevent
  /// energy injection from single-point friction torques.
  [[nodiscard]] CollisionResult buildSATContact(
    const AssetPhysical& assetA,
    const AssetPhysical& assetB,
    const SATResult& sat) const;

  /// @brief Extract face vertices from an asset hull given a normal direction
  ///
  /// Ticket: 0055c_friction_direction_fix
  ///
  /// Finds the face most aligned with the given normal and returns its vertices.
  /// Used to detect vertex-face geometry and generate contact manifolds.
  ///
  /// @param asset Asset to extract vertices from
  /// @param normal Face normal direction (world space)
  /// @return Vertices of the best-matching face (world space)
  [[nodiscard]] std::vector<Coordinate> extractFaceVertices(
    const AssetPhysical& asset,
    const Vector3D& normal) const;

  double epsilon_;
  VertexFaceDetector vertexFaceDetector_;
  VertexFaceManifoldGenerator vertexFaceManifoldGenerator_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_HANDLER_HPP
