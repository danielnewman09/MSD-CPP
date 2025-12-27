#ifndef MSD_TRANSFER_MESH_RECORD_HPP
#define MSD_TRANSFER_MESH_RECORD_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for complete geometry (visual mesh + collision hull)
 *
 * Stores both the visual rendering mesh and the collision hull in a single
 * record. This ensures that mesh and hull are always synchronized by ID.
 *
 * Visual mesh data contains positions and normals for rendering.
 * Collision hull data contains simplified convex hull vertices for physics.
 */
struct MeshRecord : public cpp_sqlite::BaseTransferObject
{
  std::string name;      // Unique mesh name (e.g., "pyramid")
  std::string category;  // Category for grouping (e.g., "primitives")

  // Visual mesh data (for rendering)
  // BLOB format: array of Vertex structs (position[3], color[3], normal[3])
  std::vector<uint8_t> vertex_data;
  uint32_t vertex_count{0};
  uint32_t triangle_count{0};
};


// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(
  msd_transfer::MeshRecord,
  (cpp_sqlite::BaseTransferObject),
  (name, category, vertex_data, vertex_count, triangle_count));

struct CollisionMeshRecord : public MeshRecord
{
  // Collision hull data (for physics)
  // BLOB format: array of ConvexHull vertices (x, y, z doubles)
  std::vector<uint8_t> hull_data;
  uint32_t hull_vertex_count{0};

  // Bounding volume metadata (computed from visual mesh)
  float aabb_min_x{0.0f};
  float aabb_min_y{0.0f};
  float aabb_min_z{0.0f};
  float aabb_max_x{0.0f};
  float aabb_max_y{0.0f};
  float aabb_max_z{0.0f};
  float bounding_radius{0.0f};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(msd_transfer::CollisionMeshRecord,
                      (msd_transfer::MeshRecord),
                      (hull_data,
                       hull_vertex_count,
                       aabb_min_x,
                       aabb_min_y,
                       aabb_min_z,
                       aabb_max_x,
                       aabb_max_y,
                       aabb_max_z,
                       bounding_radius));


}  // namespace msd_transfer


#endif  // MSD_TRANSFER_MESH_RECORD_HPP
