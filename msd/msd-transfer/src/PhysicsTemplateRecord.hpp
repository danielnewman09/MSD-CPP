#ifndef MSD_TRANSFER_PHYSICS_TEMPLATE_RECORD_HPP
#define MSD_TRANSFER_PHYSICS_TEMPLATE_RECORD_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <boost/describe.hpp>
#include <cpp_sqlite/sqlite_db/DBBaseTransferObject.hpp>
#include <cpp_sqlite/sqlite_db/DBForeignKey.hpp>

#include "msd-transfer/src/CollisionHullRecord.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_transfer
{

/**
 * @brief Database record for complete physics object templates
 *
 * Combines visual mesh reference, collision hull reference, and physical
 * properties (mass, friction, etc.) into a reusable template for creating
 * rigid body instances.
 */
struct PhysicsTemplateRecord : public cpp_sqlite::BaseTransferObject
{
  std::string name;  // Unique template name (e.g., "pyramid_standard")

  // References to geometry data (lazy-loaded via ForeignKey)
  cpp_sqlite::ForeignKey<MeshRecord> visual_mesh;
  cpp_sqlite::ForeignKey<CollisionHullRecord> collision_hull;

  // Physical properties
  double mass{1.0};
  double friction{0.5};
  double restitution{0.3};
  double linear_damping{0.1};
  double angular_damping{0.1};

  // Inertia tensor (3x3 matrix stored as BLOB: 9 doubles = 72 bytes)
  std::vector<uint8_t> inertia_tensor;
};

}  // namespace msd_transfer

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(msd_transfer::PhysicsTemplateRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (name,
                       visual_mesh,
                       collision_hull,
                       mass,
                       friction,
                       restitution,
                       linear_damping,
                       angular_damping,
                       inertia_tensor));

#endif  // MSD_TRANSFER_PHYSICS_TEMPLATE_RECORD_HPP
