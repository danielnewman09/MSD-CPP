// Ticket: 0060a_replay_enabled_test_fixture
// Design: Direct specification in ticket (no separate design doc)

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <cpp_sqlite/src/utils/Logger.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

/**
 * @brief Test asset generation tool
 * @ticket 0060a_replay_enabled_test_fixture
 *
 * This executable creates a database with ObjectRecords for standard test
 * primitives:
 * - unit_cube (1.0 m cube for standard dynamic objects)
 * - large_cube (2.0 m cube for larger objects)
 * - floor_slab (100.0 m cube for environment/floor objects)
 *
 * Each asset gets both visual and collision MeshRecords plus an ObjectRecord
 * with FK references to both.
 *
 * Usage: generate_test_assets <output_database_path>
 */

void createCubeAsset(cpp_sqlite::Database& db,
                     const std::string& name,
                     float size)
{
  std::cout << "Creating " << name << " asset (size=" << size << ")..."
            << "\n";

  // Create visual MeshRecord
  auto visualRecord = msd_assets::GeometryFactory::createCube(size);
  auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();
  meshDAO.insert(visualRecord);
  const uint32_t visualId = visualRecord.id;
  std::cout << "  Visual mesh inserted with ID: " << visualId << "\n";

  // Create collision MeshRecord (same geometry)
  auto collisionRecord = msd_assets::GeometryFactory::createCube(size);
  meshDAO.insert(collisionRecord);
  const uint32_t collisionId = collisionRecord.id;
  std::cout << "  Collision mesh inserted with ID: " << collisionId << "\n";

  // Create ObjectRecord
  msd_transfer::ObjectRecord objectRecord;
  objectRecord.name = name;
  objectRecord.category = "test_primitives";
  objectRecord.meshRecord.id = visualId;
  objectRecord.collisionMeshRecord.id = collisionId;

  // Insert ObjectRecord
  auto& objectDAO = db.getDAO<msd_transfer::ObjectRecord>();
  auto objectId = objectDAO.insert(objectRecord);
  std::cout << "  Object record inserted with ID: " << objectId << "\n";
}

int main(int argc, char* argv[])
{
  // Check command line arguments
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " <output_database_path>" << "\n";
    std::cerr << "Example: " << argv[0] << " test_assets.db" << "\n";
    return 1;
  }

  const std::string dbPath = argv[1];

  try
  {
    std::cout << "Creating test asset database: " << dbPath << "\n";

    // Create logger
    auto& logger = cpp_sqlite::Logger::getInstance();

    // Create database (read-write mode, creates if doesn't exist)
    cpp_sqlite::Database db{dbPath, true, logger.getLogger()};

    // Create the test primitive assets
    createCubeAsset(db, "unit_cube", 1.0f);
    createCubeAsset(db, "large_cube", 2.0f);
    createCubeAsset(db, "floor_slab", 100.0f);

    std::cout << "\nTest asset database created successfully!" << "\n";
    std::cout << "Database location: " << dbPath << "\n";

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
