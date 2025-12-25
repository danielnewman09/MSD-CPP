#ifndef MSD_TRANSFER_COLLISION_HULL_RECORD_HPP
#define MSD_TRANSFER_COLLISION_HULL_RECORD_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <boost/describe.hpp>
#include <cpp_sqlite/sqlite_db/DBBaseTransferObject.hpp>
#include <cpp_sqlite/sqlite_db/DBForeignKey.hpp>

#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Database record for collision hull geometry
 *
 * Stores physics collision hull (convex hull vertices) as a BLOB.
 * Stored separately from visual meshes for independent updates and
 * optimal storage (collision hulls are much smaller than visual meshes).
 */
struct CollisionHullRecord : public cpp_sqlite::BaseTransferObject
{
  std::string name;  // Unique hull name (e.g., "pyramid_collision")
  std::vector<uint8_t> hull_data;  // Serialized ConvexHull vertices (BLOB)
  uint32_t vertex_count;           // Number of hull vertices

  // Optional: Reference to the mesh that generated this hull
  cpp_sqlite::ForeignKey<MeshRecord> source_mesh;
};

}  // namespace msd_transfer

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(msd_transfer::CollisionHullRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (name, hull_data, vertex_count, source_mesh));

#endif  // MSD_TRANSFER_COLLISION_HULL_RECORD_HPP
