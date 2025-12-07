#ifndef STLLOADER_HPP
#define STLLOADER_HPP

#include <memory>
#include <string>
#include <vector>
#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"
#include <Eigen/Dense>

namespace msd_assets
{

/**
 * @brief Data structure representing a single triangle from an STL file
 */
struct STLTriangle
{
  Eigen::Vector3f normal;   ///< Triangle normal vector
  Eigen::Vector3f vertex1;  ///< First vertex position
  Eigen::Vector3f vertex2;  ///< Second vertex position
  Eigen::Vector3f vertex3;  ///< Third vertex position
};

/**
 * @brief Loader for STL (STereoLithography) 3D model files
 *
 * Supports both binary and ASCII STL formats. Binary format is recommended
 * for performance and file size.
 *
 * Binary STL format:
 * - 80-byte header (usually ignored)
 * - 4-byte unsigned int: triangle count
 * - For each triangle (50 bytes total):
 *   - 12 bytes: normal vector (3 floats)
 *   - 12 bytes: vertex 1 (3 floats)
 *   - 12 bytes: vertex 2 (3 floats)
 *   - 12 bytes: vertex 3 (3 floats)
 *   - 2 bytes: attribute byte count (usually 0)
 *
 * ASCII STL format:
 * - Text-based format starting with "solid <name>"
 * - Each triangle defined with "facet normal" and three "vertex" lines
 * - Ends with "endsolid <name>"
 */
class STLLoader
{
public:
  /**
   * @brief Load an STL file and create a Geometry
   *
   * Automatically detects whether file is binary or ASCII format.
   *
   * @param filename Path to the STL file
   * @return Unique pointer to Geometry if successful, nullptr on error
   */
  static std::unique_ptr<Geometry> loadSTL(const std::string& filename);

  /**
   * @brief Load binary STL file
   *
   * Binary format is more compact and faster to parse than ASCII.
   *
   * @param filename Path to the binary STL file
   * @return Unique pointer to Geometry if successful, nullptr on error
   */
  static std::unique_ptr<Geometry> loadBinarySTL(const std::string& filename);

  /**
   * @brief Load ASCII STL file
   *
   * ASCII format is human-readable but larger and slower to parse.
   *
   * @param filename Path to the ASCII STL file
   * @return Unique pointer to Geometry if successful, nullptr on error
   */
  static std::unique_ptr<Geometry> loadASCIISTL(const std::string& filename);

  /**
   * @brief Read raw triangle data from binary STL file
   *
   * @param filename Path to the binary STL file
   * @return Vector of triangles if successful, empty vector on error
   */
  static std::vector<STLTriangle> readBinarySTLTriangles(
    const std::string& filename);

  /**
   * @brief Read raw triangle data from ASCII STL file
   *
   * @param filename Path to the ASCII STL file
   * @return Vector of triangles if successful, empty vector on error
   */
  static std::vector<STLTriangle> readASCIISTLTriangles(
    const std::string& filename);

  /**
   * @brief Check if an STL file is binary format
   *
   * Heuristic: checks if file starts with "solid " (ASCII) or not (binary).
   * Also validates file size matches expected binary format.
   *
   * @param filename Path to the STL file
   * @return true if binary format, false if ASCII or error
   */
  static bool isBinarySTL(const std::string& filename);

  /**
   * @brief Convert triangle data to Geometry
   *
   * Creates a Geometry with vertices in the order needed for rendering.
   * Each triangle contributes 3 vertices.
   *
   * @param triangles Vector of STL triangles
   * @return Geometry object with all triangle vertices
   */
  static Geometry trianglesToGeometry(const std::vector<STLTriangle>& triangles);

private:
  /**
   * @brief Validate binary STL file size
   *
   * File size should be: 80 (header) + 4 (count) + count * 50 (triangles)
   *
   * @param fileSize Size of file in bytes
   * @param triangleCount Number of triangles from header
   * @return true if file size is valid
   */
  static bool validateBinarySTLSize(size_t fileSize, uint32_t triangleCount);
};

}  // namespace msd_assets

#endif  // STLLOADER_HPP
