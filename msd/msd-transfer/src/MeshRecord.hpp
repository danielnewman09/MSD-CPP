#ifndef MSD_TRANSFER_MESH_RECORD_HPP
#define MSD_TRANSFER_MESH_RECORD_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

namespace msd_transfer
{

/**
 * @brief Database record for visual mesh geometry
 *
 * Stores visual rendering mesh data (positions, normals, colors).
 * This is a standalone table referenced by ObjectRecord.
 */
struct MeshRecord : public cpp_sqlite::BaseTransferObject
{
  // Visual mesh data (for rendering)
  // BLOB format: array of Vertex structs (position[3], color[3], normal[3])
  std::vector<uint8_t> vertex_data;
  uint32_t vertex_count{0};
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(MeshRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (vertex_data, vertex_count));

/**
 * @brief Database record representing a complete object definition
 *
 * This is the main record that ties together visual and collision geometry.
 * It contains foreign keys to MeshRecord (visual) and CollisionMeshRecord
 * (collision). Objects can have:
 *   - Both visual and collision mesh (most common for physics objects)
 *   - Only visual mesh (visual-only objects, no physics)
 *   - Only collision mesh (invisible collision volumes)
 */
struct ObjectRecord : public cpp_sqlite::BaseTransferObject
{
  std::string name;      // Unique object name (e.g., "pyramid")
  std::string category;  // Category for grouping (e.g., "primitives")

  // Foreign keys (nullable - can be 0 if not applicable)

  cpp_sqlite::ForeignKey<MeshRecord>
    meshRecord;  // FK to MeshRecord (visual geometry)
  cpp_sqlite::ForeignKey<MeshRecord>
    collisionMeshRecord;  // FK to CollisionMeshRecord (collision geometry)
};

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(ObjectRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (name, category, meshRecord, collisionMeshRecord));


}  // namespace msd_transfer


#endif  // MSD_TRANSFER_MESH_RECORD_HPP
