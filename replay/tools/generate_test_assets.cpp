// Ticket: 0060a_replay_enabled_test_fixture
// Ticket: 0062a_extend_test_asset_generator
// Design: Direct specification in tickets (no separate design doc)

#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <cpp_sqlite/src/utils/Logger.hpp>
#include <cmath>
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
 * @brief Test asset generation tool
 * @ticket 0060a_replay_enabled_test_fixture
 * @ticket 0062a_extend_test_asset_generator
 *
 * This executable creates a database with ObjectRecords for standard test
 * primitives:
 * - unit_cube (1.0 m cube for standard dynamic objects)
 * - large_cube (2.0 m cube for larger objects)
 * - floor_slab (100.0 m cube for environment/floor objects)
 * - tiny_cube (0.5 m cube for size asymmetry tests)
 * - unit_sphere (1.0 m radius sphere for rotation-free collision tests)
 * - small_sphere (0.5 m radius sphere for mass ratio tests)
 * - large_sphere (2.0 m radius sphere for environment-scale objects)
 *
 * Each asset gets both visual and collision MeshRecords plus an ObjectRecord
 * with FK references to both.
 *
 * Usage: generate_test_assets <output_database_path>
 */

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Create icosphere point cloud with given radius
 * @ticket 0062a_extend_test_asset_generator
 *
 * Uses 2 levels of subdivision from an icosahedron (~162 vertices).
 * Spheres avoid rotational coupling in linear collision tests.
 *
 * Algorithm from LinearCollisionTest.cpp (ticket 0039b).
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
 * @ticket 0062a_extend_test_asset_generator
 *
 * Converts Vector3D array to raw byte data for database storage.
 * Pattern from EngineIntegrationTest.cpp.
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

void createCubeAsset(cpp_sqlite::Database& db,
                     const std::string& name,
                     double size)
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

/**
 * @brief Create a sphere asset with icosphere collision mesh
 * @ticket 0062a_extend_test_asset_generator
 *
 * @param db Database connection
 * @param name Asset name (e.g., "unit_sphere", "small_sphere")
 * @param radius Sphere radius in meters
 *
 * Creates visual and collision MeshRecords using icosphere point cloud
 * (~162 vertices from 2 levels of subdivision). Collision mesh stores raw
 * Vector3D vertex data (not visual Vertex format).
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
  // For now, use same icosphere geometry for visual (later could use smoother
  // tessellation)
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

    // Cube assets (ticket 0060a + 0062a)
    createCubeAsset(db, "unit_cube", 1.0);
    createCubeAsset(db, "large_cube", 2.0);
    createCubeAsset(db, "tiny_cube", 0.5);    // Ticket 0062a
    createCubeAsset(db, "floor_slab", 100.0);

    // Sphere assets (ticket 0062a)
    createSphereAsset(db, "unit_sphere", 1.0);   // Standard collision test object
    createSphereAsset(db, "small_sphere", 0.5);  // Asymmetric mass ratio tests
    createSphereAsset(db, "large_sphere", 2.0);  // Environment-scale objects

    std::cout << "\nTest asset database created successfully!" << "\n";
    std::cout << "Database location: " << dbPath << "\n";
    std::cout << "Assets: 4 cubes + 3 spheres = 7 total primitives\n";

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
