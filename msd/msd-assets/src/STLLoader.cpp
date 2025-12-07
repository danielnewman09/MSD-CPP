#include "msd-assets/src/STLLoader.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace msd_assets
{

std::unique_ptr<Geometry> STLLoader::loadSTL(const std::string& filename)
{
  if (isBinarySTL(filename))
  {
    return loadBinarySTL(filename);
  }
  else
  {
    return loadASCIISTL(filename);
  }
}

std::unique_ptr<Geometry> STLLoader::loadBinarySTL(const std::string& filename)
{
  std::vector<STLTriangle> triangles = readBinarySTLTriangles(filename);

  if (triangles.empty())
  {
    std::cerr << "Failed to read binary STL file: " << filename << std::endl;
    return nullptr;
  }

  return std::make_unique<Geometry>(trianglesToGeometry(triangles));
}

std::unique_ptr<Geometry> STLLoader::loadASCIISTL(const std::string& filename)
{
  std::vector<STLTriangle> triangles = readASCIISTLTriangles(filename);

  if (triangles.empty())
  {
    std::cerr << "Failed to read ASCII STL file: " << filename << std::endl;
    return nullptr;
  }

  return std::make_unique<Geometry>(trianglesToGeometry(triangles));
}

std::vector<STLTriangle> STLLoader::readBinarySTLTriangles(
  const std::string& filename)
{
  std::vector<STLTriangle> triangles;

  // Open file in binary mode
  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open())
  {
    std::cerr << "Cannot open file: " << filename << std::endl;
    return triangles;
  }

  // Read 80-byte header (usually ignored)
  char header[80];
  file.read(header, 80);

  // Read number of triangles (4 bytes, unsigned int, little-endian)
  uint32_t triangleCount;
  file.read(reinterpret_cast<char*>(&triangleCount), sizeof(uint32_t));

  // Get file size for validation
  file.seekg(0, std::ios::end);
  size_t fileSize = file.tellg();
  file.seekg(84);  // Back to position after header and count

  // Validate file size
  if (!validateBinarySTLSize(fileSize, triangleCount))
  {
    std::cerr << "Invalid binary STL file size for: " << filename << std::endl;
    std::cerr << "Expected: " << (80 + 4 + triangleCount * 50)
              << " bytes, got: " << fileSize << " bytes" << std::endl;
    return triangles;
  }

  triangles.reserve(triangleCount);

  // Read each triangle
  for (uint32_t i = 0; i < triangleCount; ++i)
  {
    STLTriangle triangle;

    // Read normal (3 floats = 12 bytes)
    float normal[3];
    file.read(reinterpret_cast<char*>(normal), 3 * sizeof(float));
    triangle.normal = Eigen::Vector3f(normal[0], normal[1], normal[2]);

    // Read vertex 1 (3 floats = 12 bytes)
    float v1[3];
    file.read(reinterpret_cast<char*>(v1), 3 * sizeof(float));
    triangle.vertex1 = Eigen::Vector3f(v1[0], v1[1], v1[2]);

    // Read vertex 2 (3 floats = 12 bytes)
    float v2[3];
    file.read(reinterpret_cast<char*>(v2), 3 * sizeof(float));
    triangle.vertex2 = Eigen::Vector3f(v2[0], v2[1], v2[2]);

    // Read vertex 3 (3 floats = 12 bytes)
    float v3[3];
    file.read(reinterpret_cast<char*>(v3), 3 * sizeof(float));
    triangle.vertex3 = Eigen::Vector3f(v3[0], v3[1], v3[2]);

    // Read attribute byte count (2 bytes, usually 0)
    uint16_t attributeByteCount;
    file.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(uint16_t));

    triangles.push_back(triangle);
  }

  file.close();

  std::cout << "Loaded " << triangles.size()
            << " triangles from binary STL: " << filename << std::endl;

  return triangles;
}

std::vector<STLTriangle> STLLoader::readASCIISTLTriangles(
  const std::string& filename)
{
  std::vector<STLTriangle> triangles;

  std::ifstream file(filename);
  if (!file.is_open())
  {
    std::cerr << "Cannot open file: " << filename << std::endl;
    return triangles;
  }

  std::string line;
  STLTriangle currentTriangle;
  int vertexIndex = 0;

  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::string keyword;
    iss >> keyword;

    if (keyword == "facet")
    {
      // facet normal nx ny nz
      std::string normalKeyword;
      float nx, ny, nz;
      iss >> normalKeyword >> nx >> ny >> nz;
      currentTriangle.normal = Eigen::Vector3f(nx, ny, nz);
      vertexIndex = 0;
    }
    else if (keyword == "vertex")
    {
      // vertex x y z
      float x, y, z;
      iss >> x >> y >> z;

      if (vertexIndex == 0)
      {
        currentTriangle.vertex1 = Eigen::Vector3f(x, y, z);
      }
      else if (vertexIndex == 1)
      {
        currentTriangle.vertex2 = Eigen::Vector3f(x, y, z);
      }
      else if (vertexIndex == 2)
      {
        currentTriangle.vertex3 = Eigen::Vector3f(x, y, z);
      }
      vertexIndex++;
    }
    else if (keyword == "endfacet")
    {
      triangles.push_back(currentTriangle);
    }
  }

  file.close();

  std::cout << "Loaded " << triangles.size()
            << " triangles from ASCII STL: " << filename << std::endl;

  return triangles;
}

bool STLLoader::isBinarySTL(const std::string& filename)
{
  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open())
  {
    return false;
  }

  // Read first 80 bytes (header)
  char header[81] = {0};
  file.read(header, 80);

  // ASCII STL files typically start with "solid " (note the space)
  // Binary STL files can have anything in the header, but usually don't
  // start with "solid "
  std::string headerStr(header);

  // If it starts with "solid ", might be ASCII (but not guaranteed)
  // Read the triangle count to validate
  if (headerStr.find("solid ") == 0)
  {
    // Could be ASCII, but some binary files also start with "solid"
    // Check file size to determine
    uint32_t triangleCount;
    file.read(reinterpret_cast<char*>(&triangleCount), sizeof(uint32_t));

    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.close();

    // If file size matches binary format exactly, it's binary
    if (validateBinarySTLSize(fileSize, triangleCount))
    {
      return true;
    }

    // Otherwise, assume ASCII
    return false;
  }

  file.close();

  // Doesn't start with "solid ", so it's binary
  return true;
}

Geometry STLLoader::trianglesToGeometry(const std::vector<STLTriangle>& triangles)
{
  std::vector<msd_sim::Coordinate> vertices;
  vertices.reserve(triangles.size() * 3);

  // Convert each triangle to 3 vertices
  // Note: STL stores separate vertices for each triangle (no shared vertices)
  for (const auto& triangle : triangles)
  {
    // Add the three vertices of this triangle
    vertices.emplace_back(
      triangle.vertex1.x(), triangle.vertex1.y(), triangle.vertex1.z());
    vertices.emplace_back(
      triangle.vertex2.x(), triangle.vertex2.y(), triangle.vertex2.z());
    vertices.emplace_back(
      triangle.vertex3.x(), triangle.vertex3.y(), triangle.vertex3.z());
  }

  return Geometry{vertices};
}

bool STLLoader::validateBinarySTLSize(size_t fileSize, uint32_t triangleCount)
{
  // Binary STL file size = 80 (header) + 4 (count) + count * 50 (triangles)
  size_t expectedSize = 80 + 4 + (triangleCount * 50);
  return fileSize == expectedSize;
}

}  // namespace msd_assets
