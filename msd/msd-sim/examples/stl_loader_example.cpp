/**
 * @file stl_loader_example.cpp
 * @brief Example demonstrating how to load STL files into Polyhedron objects
 */

#include "msd-sim/src/Geometry/STLLoader.hpp"
#include "msd-sim/src/Environment/Polyhedron.hpp"
#include <iostream>

using namespace msd_sim;

int main()
{
  // Example 1: Load an STL file with automatic format detection
  auto polyhedron1 = STLLoader::loadSTL("path/to/model.stl");

  if (polyhedron1)
  {
    std::cout << "Successfully loaded model with "
              << polyhedron1->getVertexCount() << " vertices" << std::endl;

    // Set color for rendering
    polyhedron1->setColor(100, 150, 255);  // Light blue
  }
  else
  {
    std::cerr << "Failed to load STL file" << std::endl;
  }

  // Example 2: Load with a specific reference frame (positioned/rotated)
  ReferenceFrame frame;
  // Position the object at (10, 5, 0) in global coordinates
  frame.setOrigin(Coordinate(10.0, 5.0, 0.0));

  // Rotate around Z-axis by 45 degrees (pi/4 radians)
  Eigen::Matrix3d rotation;
  double angle = M_PI / 4.0;
  rotation << std::cos(angle), -std::sin(angle), 0,
              std::sin(angle),  std::cos(angle), 0,
              0,                0,               1;
  frame.setOrientation(rotation);

  auto polyhedron2 = STLLoader::loadSTL("path/to/another_model.stl", frame);

  // Example 3: Load specifically as binary STL
  auto polyhedron3 = STLLoader::loadBinarySTL("path/to/binary_model.stl");

  // Example 4: Check format before loading
  std::string filename = "path/to/model.stl";
  if (STLLoader::isBinarySTL(filename))
  {
    std::cout << filename << " is in binary format" << std::endl;
  }
  else
  {
    std::cout << filename << " is in ASCII format" << std::endl;
  }

  // Example 5: Low-level access to triangle data
  auto triangles = STLLoader::readBinarySTLTriangles("path/to/model.stl");
  std::cout << "Read " << triangles.size() << " triangles" << std::endl;

  for (size_t i = 0; i < std::min(size_t(3), triangles.size()); ++i)
  {
    const auto& tri = triangles[i];
    std::cout << "Triangle " << i << ":" << std::endl;
    std::cout << "  Normal: (" << tri.normal.transpose() << ")" << std::endl;
    std::cout << "  V1: (" << tri.vertex1.transpose() << ")" << std::endl;
    std::cout << "  V2: (" << tri.vertex2.transpose() << ")" << std::endl;
    std::cout << "  V3: (" << tri.vertex3.transpose() << ")" << std::endl;
  }

  return 0;
}
