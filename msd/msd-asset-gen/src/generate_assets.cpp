// Ticket: 0060a_replay_enabled_test_fixture (unified asset DB)
// Ticket: 0062a_extend_test_asset_generator (sphere primitives)

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <cpp_sqlite/src/utils/Logger.hpp>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

/**
 * @brief Unified asset generation tool
 *
 * Creates a database with ObjectRecords for all primitives used by both
 * test recordings and production:
 *
 * Test primitives (IDs 1-7, matching generate_test_assets order):
 * - unit_cube (1.0 m cube)
 * - large_cube (2.0 m cube)
 * - tiny_cube (0.5 m cube)
 * - floor_slab (100.0 m cube)
 * - unit_sphere (1.0 m radius icosphere)
 * - small_sphere (0.5 m radius icosphere)
 * - large_sphere (2.0 m radius icosphere)
 *
 * Production primitives (IDs 8+):
 * - cube (1.0 m cube with visual geometry)
 * - pyramid (1.0 m pyramid with visual geometry)
 *
 * Usage: generate_assets <output_database_path>
 */

// ============================================================================
// Helper Functions (from generate_test_assets.cpp)
// ============================================================================

/**
 * @brief Create icosphere point cloud with given radius
 *
 * Uses 2 levels of subdivision from an icosahedron (~162 vertices).
 * Spheres avoid rotational coupling in linear collision tests.
 */
std::vector<msd_sim::Coordinate> createSpherePoints(double radius)
{
  // Start with icosahedron vertices
  double const phi = (1.0 + std::sqrt(5.0)) / 2.0;  // Golden ratio

  // 12 base icosahedron vertices (normalized to unit sphere, then scaled)
  std::vector<Eigen::Vector3d> vertices = {
    {-1,  phi, 0}, { 1,  phi, 0}, {-1, -phi, 0}, { 1, -phi, 0},
    { 0, -1,  phi}, { 0,  1,  phi}, { 0, -1, -phi}, { 0,  1, -phi},
    { phi, 0, -1}, { phi, 0,  1}, {-phi, 0, -1}, {-phi, 0,  1}
  };

  // Normalize to unit sphere
  for (auto& v : vertices)
  {
    v.normalize();
  }

  // 20 icosahedron faces (triangles)
  std::vector<std::array<size_t, 3>> faces = {
    {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
    {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
    {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
    {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
  };

  // Subdivide twice for ~162 vertices
  auto getMidpoint = [&](size_t i1, size_t i2) -> size_t
  {
    Eigen::Vector3d mid = (vertices[i1] + vertices[i2]) * 0.5;
    mid.normalize();
    vertices.push_back(mid);
    return vertices.size() - 1;
  };

  for (int sub = 0; sub < 2; ++sub)
  {
    std::vector<std::array<size_t, 3>> newFaces;
    for (const auto& face : faces)
    {
      size_t a = getMidpoint(face[0], face[1]);
      size_t b = getMidpoint(face[1], face[2]);
      size_t c = getMidpoint(face[2], face[0]);

      newFaces.push_back({face[0], a, c});
      newFaces.push_back({face[1], b, a});
      newFaces.push_back({face[2], c, b});
      newFaces.push_back({a, b, c});
    }
    faces = newFaces;
  }

  // Convert to Coordinate with radius scaling
  std::vector<msd_sim::Coordinate> points;
  points.reserve(vertices.size());
  for (const auto& v : vertices)
  {
    points.emplace_back(v.x() * radius, v.y() * radius, v.z() * radius);
  }

  return points;
}

/**
 * @brief Serialize collision vertices to BLOB format
 *
 * Converts Vector3D array to raw byte data for database storage.
 */
std::vector<uint8_t> serializeCollisionVertices(
  const std::vector<msd_sim::Coordinate>& vertices)
{
  std::vector<uint8_t> blob(vertices.size() * sizeof(msd_sim::Vector3D));
  std::memcpy(
    blob.data(), vertices.data(), vertices.size() * sizeof(msd_sim::Vector3D));
  return blob;
}

// ============================================================================
// Asset Creation Functions
// ============================================================================

/**
 * @brief Create a named cube asset with given size
 */
void createNamedCubeAsset(cpp_sqlite::Database& db,
                          const std::string& name,
                          double size,
                          const std::string& category)
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
  objectRecord.category = category;
  objectRecord.meshRecord.id = visualId;
  objectRecord.collisionMeshRecord.id = collisionId;

  // Insert ObjectRecord
  auto& objectDAO = db.getDAO<msd_transfer::ObjectRecord>();
  auto objectId = objectDAO.insert(objectRecord);
  std::cout << "  Object record inserted with ID: " << objectId << "\n";
}

/**
 * @brief Create a sphere asset with icosphere collision mesh
 */
void createSphereAsset(cpp_sqlite::Database& db,
                       const std::string& name,
                       double radius)
{
  std::cout << "Creating " << name << " asset (radius=" << radius << ")..."
            << "\n";

  // Create sphere vertices using icosphere algorithm
  auto sphereVertices = createSpherePoints(radius);

  // Create visual MeshRecord
  msd_transfer::MeshRecord visualRecord;
  visualRecord.vertex_data = serializeCollisionVertices(sphereVertices);
  visualRecord.vertex_count = static_cast<uint32_t>(sphereVertices.size());

  auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();
  meshDAO.insert(visualRecord);
  const uint32_t visualId = visualRecord.id;
  std::cout << "  Visual mesh inserted with ID: " << visualId
            << " (" << sphereVertices.size() << " vertices)\n";

  // Create collision MeshRecord (same icosphere geometry)
  msd_transfer::MeshRecord collisionRecord;
  collisionRecord.vertex_data = serializeCollisionVertices(sphereVertices);
  collisionRecord.vertex_count = static_cast<uint32_t>(sphereVertices.size());

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

/**
 * @brief Create a pyramid asset with visual geometry
 */
void createPyramidAsset(cpp_sqlite::Database& db)
{
  std::cout << "Creating pyramid asset..." << "\n";

  auto meshRecord = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);
  auto& meshDAO = db.getDAO<msd_transfer::MeshRecord>();
  meshDAO.insert(meshRecord);
  const uint32_t meshId = meshRecord.id;
  std::cout << "  Visual mesh inserted with ID: " << meshId << "\n";

  auto collisionRecord = msd_assets::GeometryFactory::createPyramid(1.0, 1.0);
  meshDAO.insert(collisionRecord);
  const uint32_t collisionId = collisionRecord.id;
  std::cout << "  Collision mesh inserted with ID: " << collisionId << "\n";

  msd_transfer::ObjectRecord objectRecord;
  objectRecord.name = "pyramid";
  objectRecord.category = "primitives";
  objectRecord.meshRecord.id = meshId;
  objectRecord.collisionMeshRecord.id = collisionId;

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
    cpp_sqlite::Database db{dbPath, true, logger.getLogger()};

    // ====================================================================
    // Test primitives FIRST (IDs 1-7, matching generate_test_assets order)
    // This ensures test recording asset IDs resolve correctly.
    // ====================================================================
    createNamedCubeAsset(db, "unit_cube", 1.0, "test_primitives");
    createNamedCubeAsset(db, "large_cube", 2.0, "test_primitives");
    createNamedCubeAsset(db, "tiny_cube", 0.5, "test_primitives");
    createNamedCubeAsset(db, "floor_slab", 100.0, "test_primitives");
    createSphereAsset(db, "unit_sphere", 1.0);
    createSphereAsset(db, "small_sphere", 0.5);
    createSphereAsset(db, "large_sphere", 2.0);

    // ====================================================================
    // Production primitives (IDs 8+)
    // ====================================================================
    createNamedCubeAsset(db, "cube", 1.0, "primitives");
    createPyramidAsset(db);

    std::cout << "\nAsset database created successfully!" << "\n";
    std::cout << "Database location: " << dbPath << "\n";
    std::cout << "Assets: 7 test primitives + 2 production primitives = 9 total\n";

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
