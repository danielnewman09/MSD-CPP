// Ticket: 0056c_python_bindings
// Python bindings for geometry BLOB deserialization

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <tuple>
#include <vector>

namespace py = pybind11;

/**
 * @brief Deserialize collision vertex BLOB into Python list of (x, y, z) tuples
 *
 * Collision geometry format: array of doubles (3 per vertex: x, y, z)
 * Corresponds to msd_sim::Vector3D serialized as raw doubles.
 *
 * @param vertex_data Raw BLOB from MeshRecord.vertex_data
 * @return List of (x, y, z) tuples (doubles)
 */
std::vector<std::tuple<double, double, double>>
deserializeCollisionVertices(const std::vector<uint8_t>& vertex_data)
{
  std::vector<std::tuple<double, double, double>> vertices;

  // Each collision vertex is 3 doubles (24 bytes)
  const size_t vertex_size = 3 * sizeof(double);
  const size_t num_vertices = vertex_data.size() / vertex_size;

  vertices.reserve(num_vertices);

  for (size_t i = 0; i < num_vertices; ++i)
  {
    double coords[3];
    std::memcpy(coords, vertex_data.data() + i * vertex_size, vertex_size);
    vertices.emplace_back(coords[0], coords[1], coords[2]);
  }

  return vertices;
}

/**
 * @brief Deserialize visual vertex BLOB into Python list of 9-element tuples
 *
 * Visual geometry format (msd_assets::Vertex):
 * - float position[3]  (px, py, pz)
 * - float color[3]     (r, g, b)
 * - float normal[3]    (nx, ny, nz)
 *
 * @param vertex_data Raw BLOB from MeshRecord.vertex_data
 * @return List of (px, py, pz, r, g, b, nx, ny, nz) tuples (floats)
 */
std::vector<std::tuple<float, float, float, float, float, float, float, float,
                       float>>
deserializeVisualVertices(const std::vector<uint8_t>& vertex_data)
{
  std::vector<std::tuple<float, float, float, float, float, float, float, float,
                         float>>
    vertices;

  // Each visual vertex is 9 floats (36 bytes):
  // position[3], color[3], normal[3]
  const size_t vertex_size = 9 * sizeof(float);
  const size_t num_vertices = vertex_data.size() / vertex_size;

  vertices.reserve(num_vertices);

  for (size_t i = 0; i < num_vertices; ++i)
  {
    float components[9];
    std::memcpy(components, vertex_data.data() + i * vertex_size, vertex_size);

    vertices.emplace_back(components[0],  // px
                          components[1],  // py
                          components[2],  // pz
                          components[3],  // r
                          components[4],  // g
                          components[5],  // b
                          components[6],  // nx
                          components[7],  // ny
                          components[8]); // nz
  }

  return vertices;
}

void bind_geometry(py::module_& m)
{
  m.def("deserialize_collision_vertices",
        &deserializeCollisionVertices,
        py::arg("vertex_data"),
        "Deserialize collision geometry BLOB into list of (x, y, z) tuples");

  m.def("deserialize_visual_vertices",
        &deserializeVisualVertices,
        py::arg("vertex_data"),
        "Deserialize visual geometry BLOB into list of (px, py, pz, r, g, b, nx, "
        "ny, nz) tuples");
}
