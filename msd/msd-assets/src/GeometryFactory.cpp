#include "msd-assets/src/GeometryFactory.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace msd_assets
{

std::array<Eigen::Vector3d, 8> GeometryFactory::getCubeCorners(double size)
{
  float half = static_cast<float>(size) / 2.0f;

  // Define 8 corners of the cube
  // Naming: (Front/Back)(Top/Bottom)(Left/Right)
  return {
    Eigen::Vector3d{-half, -half, -half},  // 0: FTL (Front Top Left)
    Eigen::Vector3d{half, -half, -half},   // 1: FTR (Front Top Right)
    Eigen::Vector3d{half, half, -half},    // 2: FBR (Front Bottom Right)
    Eigen::Vector3d{-half, half, -half},   // 3: FBL (Front Bottom Left)
    Eigen::Vector3d{-half, -half, half},   // 4: BTL (Back Top Left)
    Eigen::Vector3d{half, -half, half},    // 5: BTR (Back Top Right)
    Eigen::Vector3d{half, half, half},     // 6: BBR (Back Bottom Right)
    Eigen::Vector3d{-half, half, half}     // 7: BBL (Back Bottom Left)
  };
}


msd_transfer::MeshRecord GeometryFactory::verticesToMeshRecord(
  const std::vector<Eigen::Vector3d>& vertices)
{
  msd_transfer::MeshRecord record;

  // Calculate vertex count and triangle count
  const size_t vertexCount = vertices.size();
  record.vertex_count = static_cast<uint32_t>(vertexCount);

  // Allocate vertex_data BLOB
  const size_t blobSize = vertexCount * sizeof(Vertex);
  record.vertex_data.resize(blobSize);

  // Get pointer to BLOB data for writing
  Vertex* vertexData = reinterpret_cast<Vertex*>(record.vertex_data.data());

  // Process triangles and compute normals
  for (size_t i = 0; i + 2 < vertexCount; i += 3)
  {
    const auto& v0 = vertices[i];
    const auto& v1 = vertices[i + 1];
    const auto& v2 = vertices[i + 2];

    // Calculate two edge vectors
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;

    // Calculate normal using cross product and normalize
    auto normal = edge1.cross(edge2).normalized();

    // Convert to float and populate vertices with normal and default white
    // color
    vertexData[i] = {{static_cast<float>(v0.x()),
                      static_cast<float>(v0.y()),
                      static_cast<float>(v0.z())},
                     {1.0f, 1.0f, 1.0f},  // Default white color
                     {static_cast<float>(normal.x()),
                      static_cast<float>(normal.y()),
                      static_cast<float>(normal.z())}};

    vertexData[i + 1] = {{static_cast<float>(v1.x()),
                          static_cast<float>(v1.y()),
                          static_cast<float>(v1.z())},
                         {1.0f, 1.0f, 1.0f},  // Default white color
                         {static_cast<float>(normal.x()),
                          static_cast<float>(normal.y()),
                          static_cast<float>(normal.z())}};

    vertexData[i + 2] = {{static_cast<float>(v2.x()),
                          static_cast<float>(v2.y()),
                          static_cast<float>(v2.z())},
                         {1.0f, 1.0f, 1.0f},  // Default white color
                         {static_cast<float>(normal.x()),
                          static_cast<float>(normal.y()),
                          static_cast<float>(normal.z())}};
  }

  return record;
}

msd_transfer::MeshRecord GeometryFactory::createCube(double size)
{
  auto corners = getCubeCorners(size);
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(36);  // 6 faces × 2 triangles × 3 vertices

  // Each face is composed of 2 triangles
  // Vertices are ordered counter-clockwise for front-facing

  // Front face (z = -half)
  vertices.push_back(corners[0]);  // Triangle 1
  vertices.push_back(corners[1]);
  vertices.push_back(corners[2]);
  vertices.push_back(corners[0]);  // Triangle 2
  vertices.push_back(corners[2]);
  vertices.push_back(corners[3]);

  // Back face (z = +half)
  vertices.push_back(corners[5]);  // Triangle 1
  vertices.push_back(corners[4]);
  vertices.push_back(corners[7]);
  vertices.push_back(corners[5]);  // Triangle 2
  vertices.push_back(corners[7]);
  vertices.push_back(corners[6]);

  // Left face (x = -half)
  vertices.push_back(corners[4]);  // Triangle 1
  vertices.push_back(corners[0]);
  vertices.push_back(corners[3]);
  vertices.push_back(corners[4]);  // Triangle 2
  vertices.push_back(corners[3]);
  vertices.push_back(corners[7]);

  // Right face (x = +half)
  vertices.push_back(corners[1]);  // Triangle 1
  vertices.push_back(corners[5]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[1]);  // Triangle 2
  vertices.push_back(corners[6]);
  vertices.push_back(corners[2]);

  // Top face (y = -half)
  vertices.push_back(corners[4]);  // Triangle 1
  vertices.push_back(corners[5]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[4]);  // Triangle 2
  vertices.push_back(corners[1]);
  vertices.push_back(corners[0]);

  // Bottom face (y = +half)
  vertices.push_back(corners[3]);  // Triangle 1
  vertices.push_back(corners[2]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[3]);  // Triangle 2
  vertices.push_back(corners[6]);
  vertices.push_back(corners[7]);

  return verticesToMeshRecord(vertices);
}

msd_transfer::MeshRecord GeometryFactory::createPyramid(double baseSize,
                                                        double height)
{
  float half = static_cast<float>(baseSize) / 2.0f;
  float halfHeight = static_cast<float>(height) / 2.0f;
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(
    18);  // 4 side faces (triangles) + 2 base triangles = 6 triangles

  // Base corners (y = -height/2)
  Eigen::Vector3d base_fl{-half, -halfHeight, -half};  // front-left
  Eigen::Vector3d base_fr{half, -halfHeight, -half};   // front-right
  Eigen::Vector3d base_br{half, -halfHeight, half};    // back-right
  Eigen::Vector3d base_bl{-half, -halfHeight, half};   // back-left

  // Apex (top of pyramid)
  Eigen::Vector3d apex{0.0f, halfHeight, 0.0f};

  // Front face
  vertices.push_back(base_fl);
  vertices.push_back(base_fr);
  vertices.push_back(apex);

  // Right face
  vertices.push_back(base_fr);
  vertices.push_back(base_br);
  vertices.push_back(apex);

  // Back face
  vertices.push_back(base_br);
  vertices.push_back(base_bl);
  vertices.push_back(apex);

  // Left face
  vertices.push_back(base_bl);
  vertices.push_back(base_fl);
  vertices.push_back(apex);

  // Base bottom (2 triangles)
  vertices.push_back(base_fl);
  vertices.push_back(base_br);
  vertices.push_back(base_fr);

  vertices.push_back(base_fl);
  vertices.push_back(base_bl);
  vertices.push_back(base_br);

  return verticesToMeshRecord(vertices);
}


msd_transfer::MeshRecord GeometryFactory::createCubeWireframe(double size)
{
  auto corners = GeometryFactory::getCubeCorners(size);
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(24);  // 12 edges × 2 vertices

  // Front face edges
  vertices.push_back(corners[0]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[2]);
  vertices.push_back(corners[2]);
  vertices.push_back(corners[3]);
  vertices.push_back(corners[3]);
  vertices.push_back(corners[0]);

  // Back face edges
  vertices.push_back(corners[4]);
  vertices.push_back(corners[5]);
  vertices.push_back(corners[5]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[7]);
  vertices.push_back(corners[7]);
  vertices.push_back(corners[4]);

  // Connecting edges between front and back
  vertices.push_back(corners[0]);
  vertices.push_back(corners[4]);
  vertices.push_back(corners[1]);
  vertices.push_back(corners[5]);
  vertices.push_back(corners[2]);
  vertices.push_back(corners[6]);
  vertices.push_back(corners[3]);
  vertices.push_back(corners[7]);

  return verticesToMeshRecord(vertices);
}


// Template implementations are now in the header file

}  // namespace msd_assets
