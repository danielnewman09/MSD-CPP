// Ticket: 0056c_python_bindings
// Main pybind11 module entry point for msd_reader

#include <pybind11/pybind11.h>

namespace py = pybind11;

// Forward declarations for submodule binding functions
void bind_records(py::module_& m);
void bind_database(py::module_& m);
void bind_geometry(py::module_& m);
void bind_asset_registry(py::module_& m);
void bind_engine(py::module_& m);

/**
 * @brief Python module: msd_reader
 *
 * Read-only access to MSD simulation recording databases.
 *
 * Provides:
 * - Record type classes (SimulationFrameRecord, InertialStateRecord, etc.)
 * - Database wrapper for read-only queries
 * - Geometry BLOB deserialization functions
 *
 * Usage:
 *   import msd_reader
 *   db = msd_reader.Database("recording.db")
 *   frames = db.select_all_frames()
 *   vertices = msd_reader.deserialize_collision_vertices(mesh.vertex_data)
 */
PYBIND11_MODULE(msd_reader, m)
{
  m.doc() = "MSD Simulation Recording Database Reader";

  // Bind all transfer record types
  bind_records(m);

  // Bind database query wrapper
  bind_database(m);

  // Bind geometry deserialization functions
  bind_geometry(m);

  // Bind AssetRegistry for geometry lookup by asset ID
  bind_asset_registry(m);

  // Bind Engine for live simulation control
  bind_engine(m);

  // Module metadata
  m.attr("__version__") = "1.0.0";
}
