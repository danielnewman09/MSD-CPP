// Ticket: 0056c_python_bindings
// Python bindings for database read-only query operations

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include "msd-transfer/src/Records.hpp"

namespace py = pybind11;

/**
 * @brief Python wrapper for cpp_sqlite::Database with read-only queries
 *
 * Provides generic select_all() and select_by_id() methods that work with
 * any record type exposed in record_bindings.cpp. For FK-based queries
 * (select_by_frame, select_by_body), uses raw SQL via Database::select<T>().
 */
class DatabaseWrapper
{
public:
  explicit DatabaseWrapper(const std::string& path)
    : db_{path, false}
  {
  }

  // Generic select_all for any record type
  template<typename RecordType>
  std::vector<RecordType> selectAll()
  {
    auto& dao = db_.getDAO<RecordType>();
    return dao.selectAll();
  }

  // Generic select_by_id for any record type
  template<typename RecordType>
  std::optional<RecordType> selectById(uint32_t id)
  {
    auto& dao = db_.getDAO<RecordType>();
    return dao.selectById(id);
  }

  // Select by FK column value using raw SQL + Database::select<T>()
  template<typename RecordType>
  std::vector<RecordType> selectWhere(const std::string& fkColumn, uint32_t value)
  {
    auto& dao = db_.getDAO<RecordType>();
    std::string query = "SELECT * FROM " + dao.getTableName() +
                        " WHERE " + fkColumn + " = ?;";

    sqlite3_stmt* rawPtr = nullptr;
    int result = sqlite3_prepare_v2(
      &db_.getRawDB(), query.c_str(), -1, &rawPtr, nullptr);

    if (result != SQLITE_OK)
    {
      return {};
    }

    sqlite3_bind_int64(rawPtr, 1, static_cast<sqlite3_int64>(value));

    cpp_sqlite::PreparedSQLStmt stmt{rawPtr, sqlite3_finalize};
    return db_.select<RecordType>(stmt);
  }

  // Select by frame FK (for per-frame records)
  template<typename RecordType>
  std::vector<RecordType> selectByFrame(uint32_t frame_id)
  {
    return selectWhere<RecordType>("frame_id", frame_id);
  }

  // Select by body FK (for per-body records)
  template<typename RecordType>
  std::vector<RecordType> selectByBody(uint32_t body_id)
  {
    return selectWhere<RecordType>("body_id", body_id);
  }

private:
  cpp_sqlite::Database db_;
};

void bind_database(py::module_& m)
{
  py::class_<DatabaseWrapper>(m, "Database")
    .def(py::init<const std::string&>(), py::arg("path"))

    // ========================================
    // select_all() for all record types
    // ========================================

    .def("select_all_frames",
         &DatabaseWrapper::selectAll<msd_transfer::SimulationFrameRecord>)
    .def("select_all_static_assets",
         &DatabaseWrapper::selectAll<msd_transfer::AssetInertialStaticRecord>)
    .def("select_all_inertial_states",
         &DatabaseWrapper::selectAll<msd_transfer::InertialStateRecord>)
    .def("select_all_energy",
         &DatabaseWrapper::selectAll<msd_transfer::EnergyRecord>)
    .def("select_all_system_energy",
         &DatabaseWrapper::selectAll<msd_transfer::SystemEnergyRecord>)
    .def("select_all_collisions",
         &DatabaseWrapper::selectAll<msd_transfer::CollisionResultRecord>)
    .def("select_all_solver_diagnostics",
         &DatabaseWrapper::selectAll<msd_transfer::SolverDiagnosticRecord>)
    .def("select_all_meshes",
         &DatabaseWrapper::selectAll<msd_transfer::MeshRecord>)
    .def("select_all_objects",
         &DatabaseWrapper::selectAll<msd_transfer::ObjectRecord>)
    .def("select_all_materials",
         &DatabaseWrapper::selectAll<msd_transfer::MaterialRecord>)
    .def("select_all_physics_templates",
         &DatabaseWrapper::selectAll<msd_transfer::PhysicsTemplateRecord>)

    // Tier 3: Extended records (forward compatibility)
    .def("select_all_dynamic_states",
         &DatabaseWrapper::selectAll<msd_transfer::AssetDynamicStateRecord>)

    // ========================================
    // select_by_id() for all record types
    // ========================================

    .def("select_frame_by_id",
         &DatabaseWrapper::selectById<msd_transfer::SimulationFrameRecord>,
         py::arg("id"))
    .def("select_static_asset_by_id",
         &DatabaseWrapper::selectById<msd_transfer::AssetInertialStaticRecord>,
         py::arg("id"))
    .def("select_inertial_state_by_id",
         &DatabaseWrapper::selectById<msd_transfer::InertialStateRecord>,
         py::arg("id"))
    .def("select_energy_by_id",
         &DatabaseWrapper::selectById<msd_transfer::EnergyRecord>,
         py::arg("id"))
    .def("select_system_energy_by_id",
         &DatabaseWrapper::selectById<msd_transfer::SystemEnergyRecord>,
         py::arg("id"))
    .def("select_collision_by_id",
         &DatabaseWrapper::selectById<msd_transfer::CollisionResultRecord>,
         py::arg("id"))
    .def("select_solver_diagnostic_by_id",
         &DatabaseWrapper::selectById<msd_transfer::SolverDiagnosticRecord>,
         py::arg("id"))
    .def("select_mesh_by_id",
         &DatabaseWrapper::selectById<msd_transfer::MeshRecord>,
         py::arg("id"))
    .def("select_object_by_id",
         &DatabaseWrapper::selectById<msd_transfer::ObjectRecord>,
         py::arg("id"))

    // ========================================
    // select_by_frame() for per-frame records
    // ========================================

    .def("select_inertial_states_by_frame",
         &DatabaseWrapper::selectByFrame<msd_transfer::InertialStateRecord>,
         py::arg("frame_id"))
    .def("select_energy_by_frame",
         &DatabaseWrapper::selectByFrame<msd_transfer::EnergyRecord>,
         py::arg("frame_id"))
    .def("select_system_energy_by_frame",
         &DatabaseWrapper::selectByFrame<msd_transfer::SystemEnergyRecord>,
         py::arg("frame_id"))
    .def("select_collisions_by_frame",
         &DatabaseWrapper::selectByFrame<msd_transfer::CollisionResultRecord>,
         py::arg("frame_id"))
    .def("select_solver_diagnostic_by_frame",
         &DatabaseWrapper::selectByFrame<msd_transfer::SolverDiagnosticRecord>,
         py::arg("frame_id"))
    .def("select_contact_constraints_by_frame",
         &DatabaseWrapper::selectByFrame<msd_transfer::ContactConstraintRecord>,
         py::arg("frame_id"))
    .def("select_friction_constraints_by_frame",
         &DatabaseWrapper::selectByFrame<msd_transfer::FrictionConstraintRecord>,
         py::arg("frame_id"))

    // ========================================
    // select_by_body() for per-body records
    // ========================================

    .def("select_inertial_states_by_body",
         &DatabaseWrapper::selectByBody<msd_transfer::InertialStateRecord>,
         py::arg("body_id"))
    .def("select_energy_by_body",
         &DatabaseWrapper::selectByBody<msd_transfer::EnergyRecord>,
         py::arg("body_id"));
}
