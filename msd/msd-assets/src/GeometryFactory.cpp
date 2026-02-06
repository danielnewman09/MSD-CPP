#include "msd-assets/src/GeometryFactory.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <cstring>
#include <vector>
#include "msd-assets/src/Geometry.hpp"
#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_assets
{

std::array<msd_sim::Vector3D, 8> GeometryFactory::getCubeCorners(double size)
{
  double const half = size / 2.0;

  // Define 8 corners of the cube
  // Naming: (Front/Back)(Top/Bottom)(Left/Right)
  return {
    msd_sim::Vector3D{-half, -half, -half},  // 0: FTL (Front Top Left)
    msd_sim::Vector3D{half, -half, -half},   // 1: FTR (Front Top Right)
    msd_sim::Vector3D{half, half, -half},    // 2: FBR (Front Bottom Right)
    msd_sim::Vector3D{-half, half, -half},   // 3: FBL (Front Bottom Left)
    msd_sim::Vector3D{-half, -half, half},   // 4: BTL (Back Top Left)
    msd_sim::Vector3D{half, -half, half},    // 5: BTR (Back Top Right)
    msd_sim::Vector3D{half, half, half},     // 6: BBR (Back Bottom Right)
    msd_sim::Vector3D{-half, half, half}     // 7: BBL (Back Bottom Left)
  };
}


msd_transfer::MeshRecord GeometryFactory::verticesToMeshRecord(
  const std::vector<msd_sim::Vector3D>& vertices)
{
  // Ticket: 0003_geometry-factory-type-safety
  // Create VisualGeometry from raw coordinates
  // - Computes normals via computeVertexData()
  // - Stores result in cachedVertices_ as vector<Vertex>
  CollisionGeometry const geometry{vertices, 0};

  // Serialize using existing populateMeshRecord()
  // - Correctly serializes vector<Vertex> to BLOB
  // - Sets vertex_count appropriately
  return geometry.populateMeshRecord();
}

msd_transfer::MeshRecord GeometryFactory::createCube(double size)
{
  auto corners = getCubeCorners(size);
  std::vector<msd_sim::Vector3D> vertices;
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
  double const half = baseSize / 2.0;
  double const halfHeight = height / 2.0;
  std::vector<msd_sim::Vector3D> vertices;
  vertices.reserve(
    18);  // 4 side faces (triangles) + 2 base triangles = 6 triangles

  // Base corners (y = -height/2)
  msd_sim::Vector3D const baseFl{-half, -halfHeight, -half};  // front-left
  msd_sim::Vector3D const baseFr{half, -halfHeight, -half};   // front-right
  msd_sim::Vector3D const baseBr{half, -halfHeight, half};    // back-right
  msd_sim::Vector3D const baseBl{-half, -halfHeight, half};   // back-left

  // Apex (top of pyramid)
  msd_sim::Vector3D const apex{0.0, halfHeight, 0.0};

  // Front face
  vertices.push_back(baseFl);
  vertices.push_back(baseFr);
  vertices.push_back(apex);

  // Right face
  vertices.push_back(baseFr);
  vertices.push_back(baseBr);
  vertices.push_back(apex);

  // Back face
  vertices.push_back(baseBr);
  vertices.push_back(baseBl);
  vertices.push_back(apex);

  // Left face
  vertices.push_back(baseBl);
  vertices.push_back(baseFl);
  vertices.push_back(apex);

  // Base bottom (2 triangles)
  vertices.push_back(baseFl);
  vertices.push_back(baseBr);
  vertices.push_back(baseFr);

  vertices.push_back(baseFl);
  vertices.push_back(baseBl);
  vertices.push_back(baseBr);

  return verticesToMeshRecord(vertices);
}


msd_transfer::MeshRecord GeometryFactory::createCubeWireframe(double size)
{
  auto corners = GeometryFactory::getCubeCorners(size);
  std::vector<msd_sim::Vector3D> vertices;
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
