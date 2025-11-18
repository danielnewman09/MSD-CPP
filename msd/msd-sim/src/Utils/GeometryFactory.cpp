#include "msd-sim/src/Utils/GeometryFactory.hpp"
#include <array>

namespace msd_sim
{

std::array<Coordinate, 8> GeometryFactory::getCubeCorners(double size)
{
  double half = size / 2.0;

  // Define 8 corners of the cube
  // Naming: (Front/Back)(Top/Bottom)(Left/Right)
  return {
    Coordinate{-half, -half, -half},  // 0: FTL (Front Top Left)
    Coordinate{half, -half, -half},   // 1: FTR (Front Top Right)
    Coordinate{half, half, -half},    // 2: FBR (Front Bottom Right)
    Coordinate{-half, half, -half},   // 3: FBL (Front Bottom Left)
    Coordinate{-half, -half, half},   // 4: BTL (Back Top Left)
    Coordinate{half, -half, half},    // 5: BTR (Back Top Right)
    Coordinate{half, half, half},     // 6: BBR (Back Bottom Right)
    Coordinate{-half, half, half}     // 7: BBL (Back Bottom Left)
  };
}

std::vector<Coordinate> GeometryFactory::createCube(double size)
{
  auto corners = getCubeCorners(size);
  std::vector<Coordinate> vertices;
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

  return vertices;
}

std::vector<Coordinate> GeometryFactory::createCubeWireframe(double size)
{
  auto corners = getCubeCorners(size);
  std::vector<Coordinate> vertices;
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

  return vertices;
}

}  // namespace msd_sim
