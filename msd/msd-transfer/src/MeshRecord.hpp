#ifndef MSD_TRANSFER_MESH_RECORD_HPP
#define MSD_TRANSFER_MESH_RECORD_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <boost/describe.hpp>
#include <cpp_sqlite/sqlite_db/DBBaseTransferObject.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for visual mesh geometry
 *
 * Stores rendering mesh data (vertices, normals, colors) as a BLOB.
 * Used for GPU rendering only - NOT for physics collision.
 */
struct MeshRecord : public cpp_sqlite::BaseTransferObject
{
  std::string name;            // Unique mesh name (e.g., "pyramid_visual")
  std::string category;        // Category for grouping (e.g., "primitive")
  std::vector<uint8_t> vertex_data;  // Serialized Vertex[] array (BLOB)
  uint32_t vertex_count;       // Number of vertices in the mesh
  uint32_t triangle_count;     // Number of triangles (vertex_count / 3)

  // Bounding box for culling/spatial queries
  float aabb_min_x{0.0f};
  float aabb_min_y{0.0f};
  float aabb_min_z{0.0f};
  float aabb_max_x{0.0f};
  float aabb_max_y{0.0f};
  float aabb_max_z{0.0f};
  float bounding_radius{0.0f};
};

}  // namespace msd_transfer

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(msd_transfer::MeshRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (name,
                       category,
                       vertex_data,
                       vertex_count,
                       triangle_count,
                       aabb_min_x,
                       aabb_min_y,
                       aabb_min_z,
                       aabb_max_x,
                       aabb_max_y,
                       aabb_max_z,
                       bounding_radius));

#endif  // MSD_TRANSFER_MESH_RECORD_HPP
