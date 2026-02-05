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
 * @brief Asset generation tool
 *
 * This executable creates a database with ObjectRecords for common primitives:
 * - Cube (with visual and collision geometry)
 * - Pyramid (with visual and collision geometry)
 *
 * Usage: generate_assets <output_database_path>
 */

void createCubeAsset(cpp_sqlite::Database& db)
{
  std::cout << "Creating cube asset..." << "\n";

  // Create MeshRecord directly from factory
  auto meshRecord = msd_assets::GeometryFactory::createCube(1.0);

  // Insert MeshRecord into database
  auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();
  meshDAO.insert(meshRecord);
  const uint32_t meshId = meshRecord.id;
  std::cout << "  Visual mesh inserted with ID: " << meshId << "\n";

  // Create CollisionMeshRecord directly from factory
  auto collisionRecord = msd_assets::GeometryFactory::createCube(1.0);

  // Insert CollisionMeshRecord into database
  auto& collisionDAO = db.getDAO<msd_transfer::MeshRecord>();
  collisionDAO.insert(collisionRecord);
  const uint32_t collisionId = collisionRecord.id;
  std::cout << "  Collision mesh inserted with ID: " << collisionId << "\n";

  // Create ObjectRecord
  msd_transfer::ObjectRecord objectRecord;
  objectRecord.name = "cube";
  objectRecord.category = "primitives";
  objectRecord.meshRecord.id = meshId;
  objectRecord.collisionMeshRecord.id = collisionId;

  // Insert ObjectRecord into database
  auto& objectDAO = db.getDAO<msd_transfer::ObjectRecord>();
  auto objectId = objectDAO.insert(objectRecord);
  std::cout << "  Object record inserted with ID: " << objectId << "\n";

  // Optional: Reconstruct geometry objects from records for verification
  // auto visualGeometry =
  // msd_assets::VisualGeometry::fromMeshRecord(meshRecord, objectId); auto
  // collisionGeometry =
  // msd_assets::CollisionGeometry::fromMeshRecord(collisionRecord, objectId);
}

void createPyramidAsset(cpp_sqlite::Database& db)
{
  std::cout << "Creating pyramid asset..." << "\n";

  // Create MeshRecord directly from factory
  auto meshRecord = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);

  // Insert MeshRecord into database
  auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();
  meshDAO.insert(meshRecord);
  const uint32_t meshId = meshRecord.id;
  std::cout << "  Visual mesh inserted with ID: " << meshId << "\n";

  // Create CollisionMeshRecord directly from factory
  auto collisionRecord = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);

  // Insert CollisionMeshRecord into database
  auto& collisionDAO = db.getDAO<msd_transfer::MeshRecord>();
  collisionDAO.insert(collisionRecord);
  const uint32_t collisionId = collisionRecord.id;
  std::cout << "  Collision mesh inserted with ID: " << collisionId << "\n";

  // Create ObjectRecord
  msd_transfer::ObjectRecord objectRecord;
  objectRecord.name = "pyramid";
  objectRecord.category = "primitives";
  objectRecord.meshRecord.id = meshId;
  objectRecord.collisionMeshRecord.id = collisionId;

  // Insert ObjectRecord into database
  auto& objectDAO = db.getDAO<msd_transfer::ObjectRecord>();
  auto objectId = objectDAO.insert(objectRecord);
  std::cout << "  Object record inserted with ID: " << objectId << "\n";

  // Optional: Reconstruct geometry objects from records for verification
  // auto visualGeometry =
  // msd_assets::VisualGeometry::fromMeshRecord(meshRecord, objectId); auto
  // collisionGeometry =
  // msd_assets::CollisionGeometry::fromMeshRecord(collisionRecord, objectId);
}

int main(int argc, char* argv[])
{
  // Check command line arguments
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " <output_database_path>" << "\n";
    std::cerr << "Example: " << argv[0] << " assets.db" << "\n";
    return 1;
  }

  const std::string dbPath = argv[1];

  try
  {
    std::cout << "Creating asset database: " << dbPath << "\n";

    // Create logger
    auto& logger = cpp_sqlite::Logger::getInstance();

    // Create database (read-write mode, creates if doesn't exist)
    cpp_sqlite::Database db(dbPath, true, logger.getLogger());

    // Create the assets
    createCubeAsset(db);
    createPyramidAsset(db);

    std::cout << "\nAsset database created successfully!" << "\n";
    std::cout << "Database location: " << dbPath << "\n";

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
