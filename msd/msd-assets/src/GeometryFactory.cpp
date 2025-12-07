#include "msd-assets/src/GeometryFactory.hpp"
#include <array>

namespace msd_assets
{

std::array<msd_sim::Coordinate, 8> GeometryFactory::getCubeCorners(double size)
{
  float half = static_cast<float>(size) / 2.0f;

  // Define 8 corners of the cube
  // Naming: (Front/Back)(Top/Bottom)(Left/Right)
  return {
    msd_sim::Coordinate{-half, -half, -half},  // 0: FTL (Front Top Left)
    msd_sim::Coordinate{half, -half, -half},   // 1: FTR (Front Top Right)
    msd_sim::Coordinate{half, half, -half},    // 2: FBR (Front Bottom Right)
    msd_sim::Coordinate{-half, half, -half},   // 3: FBL (Front Bottom Left)
    msd_sim::Coordinate{-half, -half, half},   // 4: BTL (Back Top Left)
    msd_sim::Coordinate{half, -half, half},    // 5: BTR (Back Top Right)
    msd_sim::Coordinate{half, half, half},     // 6: BBR (Back Bottom Right)
    msd_sim::Coordinate{-half, half, half}     // 7: BBL (Back Bottom Left)
  };
}

Geometry GeometryFactory::createCube(double size)
{
  auto corners = getCubeCorners(size);
  std::vector<msd_sim::Coordinate> vertices;
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

  return Geometry{vertices};
}

Geometry GeometryFactory::createCubeWireframe(double size)
{
  auto corners = getCubeCorners(size);
  std::vector<msd_sim::Coordinate> vertices;
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

  return Geometry{vertices};
}

Geometry GeometryFactory::createPyramid(double baseSize, double height)
{
  float half = static_cast<float>(baseSize) / 2.0f;
  float halfHeight = static_cast<float>(height) / 2.0f;
  std::vector<msd_sim::Coordinate> vertices;
  vertices.reserve(18);  // 4 side faces (triangles) + 2 base triangles = 6 triangles

  // Base corners (y = -height/2)
  msd_sim::Coordinate base_fl{-half, -halfHeight, -half};  // front-left
  msd_sim::Coordinate base_fr{half, -halfHeight, -half};   // front-right
  msd_sim::Coordinate base_br{half, -halfHeight, half};    // back-right
  msd_sim::Coordinate base_bl{-half, -halfHeight, half};   // back-left

  // Apex (top of pyramid)
  msd_sim::Coordinate apex{0.0f, halfHeight, 0.0f};

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

  return Geometry{vertices};
}

}  // namespace msd_assets
