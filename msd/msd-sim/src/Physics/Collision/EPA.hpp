// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#ifndef MSD_SIM_PHYSICS_EPA_HPP
#define MSD_SIM_PHYSICS_EPA_HPP

#include <array>
#include <utility>
#include <vector>
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Facet.hpp"
#include "msd-sim/src/Physics/Collision/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"

namespace msd_sim
{

/**
 * @brief Minkowski difference vertex with witness point tracking.
 *
 * Used internally by EPA to maintain the association between
 * Minkowski space points and their contributing surface points.
 * This enables extraction of physical contact locations after
 * EPA convergence.
 *
 * @ticket 0028_epa_witness_points
 */
struct MinkowskiVertex
{
  Coordinate point;     // Minkowski difference point (A - B)
  Coordinate witnessA;  // Support point on A that contributed
  Coordinate witnessB;  // Support point on B that contributed

  MinkowskiVertex() = default;
  MinkowskiVertex(Coordinate p, Coordinate wA, Coordinate wB)
    : point{std::move(p)}, witnessA{std::move(wA)}, witnessB{std::move(wB)}
  {
  }
};

/**
 * @brief Expanding Polytope Algorithm (EPA) for contact information extraction.
 *
 * EPA computes penetration depth, contact normal, and contact point from a GJK
 * terminating simplex. When GJK detects an intersection, EPA expands the
 * terminating simplex (tetrahedron containing the origin) into a polytope until
 * the closest face to the origin is found, yielding complete collision data.
 *
 * Algorithm steps:
 * 1. Initialize polytope from GJK simplex (4 vertices, 4 triangular faces)
 * 2. Find closest face to origin
 * 3. Query support point in direction of face normal
 * 4. If new point within tolerance, found closest face → terminate
 * 5. Else, remove visible faces, build horizon edges, add new faces
 * 6. Repeat until convergence
 *
 * Contact extraction:
 * - Contact normal = closest face normal (normalized, A→B)
 * - Penetration depth = distance from origin to closest face
 * - Contact point = barycentric centroid of closest face
 *
 * @see
 * docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml
 * @ticket 0027a_expanding_polytope_algorithm
 */
class EPA
{
public:
  /**
   * @brief Construct EPA solver for two physical assets.
   *
   * @param assetA First physical asset (includes hull and reference frame)
   * @param assetB Second physical asset (includes hull and reference frame)
   * @param epsilon Numerical tolerance for convergence (default: 1e-6)
   */
  EPA(const AssetPhysical& assetA,
      const AssetPhysical& assetB,
      double epsilon = 1e-6);

  /**
   * @brief Compute contact information from GJK terminating simplex.
   *
   * Assumes simplex contains the origin (GJK returned true).
   * Expands polytope iteratively until closest face found within tolerance.
   *
   * @param simplex GJK terminating simplex (4 vertices in Minkowski space)
   * @param maxIterations Maximum expansion iterations (default: 64)
   * @return CollisionResult with penetration depth, normal, contact point.
   *         If EPA does not fully converge, returns the best approximation
   *         from the closest polytope face found so far.
   * @throws std::invalid_argument if simplex size != 4
   */
  CollisionResult computeContactInfo(const std::vector<Coordinate>& simplex,
                                     int maxIterations = 64);

  EPA(const EPA&) = default;
  EPA(EPA&&) noexcept = default;
  EPA& operator=(const EPA&) = delete;      // Cannot reassign reference members
  EPA& operator=(EPA&&) noexcept = delete;  // Cannot reassign reference members
  ~EPA() = default;


  /**
   * @brief Edge in polytope for horizon construction.
   */
  struct EPAEdge
  {
    size_t v0;  // First vertex index
    size_t v1;  // Second vertex index

    EPAEdge() = default;
    EPAEdge(size_t a, size_t b) : v0{a}, v1{b}
    {
    }

    // Equality for duplicate detection (order-independent)
    bool operator==(const EPAEdge& other) const
    {
      return (v0 == other.v0 && v1 == other.v1) ||
             (v0 == other.v1 && v1 == other.v0);
    }
  };

  // Core algorithm
  bool expandPolytope(int maxIterations);
  [[nodiscard]] size_t findClosestFace() const;

  // Topology management
  [[nodiscard]] bool isVisible(const Facet& face,
                               const Coordinate& point) const;
  std::vector<EPAEdge> buildHorizonEdges(const Coordinate& newVertex);
  void addFace(size_t v0, size_t v1, size_t v2);

  // Contact extraction
  static Coordinate computeContactPoint(const Facet& face);
  // Contact manifold extraction (Ticket: 0029_contact_manifold_generation)
  size_t extractContactManifold(size_t faceIndex,
                                std::array<ContactPoint, 4>& contacts) const;

  // Edge contact generation (Ticket: 0040c_edge_contact_manifold)
  size_t generateEdgeContacts(const Facet& epaFace,
                              std::array<ContactPoint, 4>& contacts) const;

  const AssetPhysical& assetA_;
  const AssetPhysical& assetB_;
  double epsilon_;

  std::vector<MinkowskiVertex>
    vertices_;                // Minkowski vertices with witness tracking
  std::vector<Facet> faces_;  // Triangular faces
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_EPA_HPP
