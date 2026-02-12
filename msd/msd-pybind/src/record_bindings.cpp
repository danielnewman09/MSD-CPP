// Ticket: 0056c_python_bindings
// Python bindings for all transfer record types

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "msd-transfer/src/Records.hpp"

namespace py = pybind11;

void bind_records(py::module_& m)
{
  // ========================================
  // Tier 2: Sub-Records (bind first since top-level records depend on them)
  // ========================================

  py::class_<msd_transfer::CoordinateRecord>(m, "CoordinateRecord")
    .def(py::init<>())
    .def_readonly("x", &msd_transfer::CoordinateRecord::x)
    .def_readonly("y", &msd_transfer::CoordinateRecord::y)
    .def_readonly("z", &msd_transfer::CoordinateRecord::z);

  py::class_<msd_transfer::VelocityRecord>(m, "VelocityRecord")
    .def(py::init<>())
    .def_readonly("x", &msd_transfer::VelocityRecord::x)
    .def_readonly("y", &msd_transfer::VelocityRecord::y)
    .def_readonly("z", &msd_transfer::VelocityRecord::z);

  py::class_<msd_transfer::AccelerationRecord>(m, "AccelerationRecord")
    .def(py::init<>())
    .def_readonly("x", &msd_transfer::AccelerationRecord::x)
    .def_readonly("y", &msd_transfer::AccelerationRecord::y)
    .def_readonly("z", &msd_transfer::AccelerationRecord::z);

  py::class_<msd_transfer::QuaternionDRecord>(m, "QuaternionDRecord")
    .def(py::init<>())
    .def_readonly("w", &msd_transfer::QuaternionDRecord::w)
    .def_readonly("x", &msd_transfer::QuaternionDRecord::x)
    .def_readonly("y", &msd_transfer::QuaternionDRecord::y)
    .def_readonly("z", &msd_transfer::QuaternionDRecord::z);

  py::class_<msd_transfer::Vector4DRecord>(m, "Vector4DRecord")
    .def(py::init<>())
    .def_readonly("w", &msd_transfer::Vector4DRecord::w)
    .def_readonly("x", &msd_transfer::Vector4DRecord::x)
    .def_readonly("y", &msd_transfer::Vector4DRecord::y)
    .def_readonly("z", &msd_transfer::Vector4DRecord::z);

  py::class_<msd_transfer::Vector3DRecord>(m, "Vector3DRecord")
    .def(py::init<>())
    .def_readonly("x", &msd_transfer::Vector3DRecord::x)
    .def_readonly("y", &msd_transfer::Vector3DRecord::y)
    .def_readonly("z", &msd_transfer::Vector3DRecord::z);

  py::class_<msd_transfer::AngularAccelerationRecord>(m,
                                                       "AngularAccelerationRecord")
    .def(py::init<>())
    .def_readonly("pitch", &msd_transfer::AngularAccelerationRecord::pitch)
    .def_readonly("roll", &msd_transfer::AngularAccelerationRecord::roll)
    .def_readonly("yaw", &msd_transfer::AngularAccelerationRecord::yaw);

  py::class_<msd_transfer::AngularVelocityRecord>(m, "AngularVelocityRecord")
    .def(py::init<>())
    .def_readonly("pitch", &msd_transfer::AngularVelocityRecord::pitch)
    .def_readonly("roll", &msd_transfer::AngularVelocityRecord::roll)
    .def_readonly("yaw", &msd_transfer::AngularVelocityRecord::yaw);

  py::class_<msd_transfer::AngularCoordinateRecord>(m, "AngularCoordinateRecord")
    .def(py::init<>())
    .def_readonly("pitch", &msd_transfer::AngularCoordinateRecord::pitch)
    .def_readonly("roll", &msd_transfer::AngularCoordinateRecord::roll)
    .def_readonly("yaw", &msd_transfer::AngularCoordinateRecord::yaw);

  py::class_<msd_transfer::ContactPointRecord>(m, "ContactPointRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::ContactPointRecord::id)
    .def_readonly("pointA", &msd_transfer::ContactPointRecord::pointA)
    .def_readonly("pointB", &msd_transfer::ContactPointRecord::pointB)
    .def_readonly("depth", &msd_transfer::ContactPointRecord::depth);

  py::class_<msd_transfer::ForceVectorRecord>(m, "ForceVectorRecord")
    .def(py::init<>())
    .def_readonly("x", &msd_transfer::ForceVectorRecord::x)
    .def_readonly("y", &msd_transfer::ForceVectorRecord::y)
    .def_readonly("z", &msd_transfer::ForceVectorRecord::z);

  py::class_<msd_transfer::TorqueVectorRecord>(m, "TorqueVectorRecord")
    .def(py::init<>())
    .def_readonly("x", &msd_transfer::TorqueVectorRecord::x)
    .def_readonly("y", &msd_transfer::TorqueVectorRecord::y)
    .def_readonly("z", &msd_transfer::TorqueVectorRecord::z);

  py::class_<msd_transfer::ExternalForceRecord>(m, "ExternalForceRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::ExternalForceRecord::id)
    .def_readonly("force", &msd_transfer::ExternalForceRecord::force)
    .def_readonly("torque", &msd_transfer::ExternalForceRecord::torque)
    .def_readonly("applicationPoint",
                  &msd_transfer::ExternalForceRecord::applicationPoint);

  // ========================================
  // Tier 1: Top-Level Records (own DB tables)
  // ========================================

  py::class_<msd_transfer::SimulationFrameRecord>(m, "SimulationFrameRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::SimulationFrameRecord::id)
    .def_readonly("simulation_time",
                  &msd_transfer::SimulationFrameRecord::simulation_time)
    .def_readonly("wall_clock_time",
                  &msd_transfer::SimulationFrameRecord::wall_clock_time);

  py::class_<msd_transfer::AssetInertialStaticRecord>(m,
                                                       "AssetInertialStaticRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::AssetInertialStaticRecord::id)
    .def_readonly("body_id", &msd_transfer::AssetInertialStaticRecord::body_id)
    .def_readonly("mass", &msd_transfer::AssetInertialStaticRecord::mass)
    .def_readonly("restitution",
                  &msd_transfer::AssetInertialStaticRecord::restitution)
    .def_readonly("friction", &msd_transfer::AssetInertialStaticRecord::friction);

  py::class_<msd_transfer::InertialStateRecord>(m, "InertialStateRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::InertialStateRecord::id)
    .def_readonly("position", &msd_transfer::InertialStateRecord::position)
    .def_readonly("velocity", &msd_transfer::InertialStateRecord::velocity)
    .def_readonly("acceleration", &msd_transfer::InertialStateRecord::acceleration)
    .def_readonly("orientation", &msd_transfer::InertialStateRecord::orientation)
    .def_readonly("quaternionRate",
                  &msd_transfer::InertialStateRecord::quaternionRate)
    .def_readonly("angularAcceleration",
                  &msd_transfer::InertialStateRecord::angularAcceleration);
  // Note: body and frame FKs are not directly exposed; use Database queries

  py::class_<msd_transfer::EnergyRecord>(m, "EnergyRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::EnergyRecord::id)
    .def_readonly("linear_ke", &msd_transfer::EnergyRecord::linear_ke)
    .def_readonly("rotational_ke", &msd_transfer::EnergyRecord::rotational_ke)
    .def_readonly("potential_e", &msd_transfer::EnergyRecord::potential_e)
    .def_readonly("total_e", &msd_transfer::EnergyRecord::total_e);
  // Note: body and frame FKs not exposed; use Database queries

  py::class_<msd_transfer::SystemEnergyRecord>(m, "SystemEnergyRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::SystemEnergyRecord::id)
    .def_readonly("total_linear_ke",
                  &msd_transfer::SystemEnergyRecord::total_linear_ke)
    .def_readonly("total_rotational_ke",
                  &msd_transfer::SystemEnergyRecord::total_rotational_ke)
    .def_readonly("total_potential_e",
                  &msd_transfer::SystemEnergyRecord::total_potential_e)
    .def_readonly("total_system_e",
                  &msd_transfer::SystemEnergyRecord::total_system_e)
    .def_readonly("delta_e", &msd_transfer::SystemEnergyRecord::delta_e)
    .def_readonly("energy_injection",
                  &msd_transfer::SystemEnergyRecord::energy_injection)
    .def_readonly("collision_active",
                  &msd_transfer::SystemEnergyRecord::collision_active);
  // Note: frame FK not exposed; use Database queries

  py::class_<msd_transfer::CollisionResultRecord>(m, "CollisionResultRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::CollisionResultRecord::id)
    .def_readonly("body_a_id", &msd_transfer::CollisionResultRecord::body_a_id)
    .def_readonly("body_b_id", &msd_transfer::CollisionResultRecord::body_b_id)
    .def_readonly("normal", &msd_transfer::CollisionResultRecord::normal)
    .def_readonly("penetrationDepth",
                  &msd_transfer::CollisionResultRecord::penetrationDepth);
  // Note: contacts (RepeatedField) and frame FK handled by Database queries

  py::class_<msd_transfer::SolverDiagnosticRecord>(m, "SolverDiagnosticRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::SolverDiagnosticRecord::id)
    .def_readonly("iterations", &msd_transfer::SolverDiagnosticRecord::iterations)
    .def_readonly("residual", &msd_transfer::SolverDiagnosticRecord::residual)
    .def_readonly("converged", &msd_transfer::SolverDiagnosticRecord::converged)
    .def_readonly("num_constraints",
                  &msd_transfer::SolverDiagnosticRecord::num_constraints)
    .def_readonly("num_contacts",
                  &msd_transfer::SolverDiagnosticRecord::num_contacts);
  // Note: frame FK not exposed; use Database queries

  py::class_<msd_transfer::MeshRecord>(m, "MeshRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::MeshRecord::id)
    .def_readonly("vertex_data", &msd_transfer::MeshRecord::vertex_data)
    .def_readonly("vertex_count", &msd_transfer::MeshRecord::vertex_count);

  py::class_<msd_transfer::ObjectRecord>(m, "ObjectRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::ObjectRecord::id)
    .def_readonly("name", &msd_transfer::ObjectRecord::name)
    .def_readonly("category", &msd_transfer::ObjectRecord::category);
  // Note: meshRecord and collisionMeshRecord FKs not exposed; use Database queries

  // ========================================
  // Tier 3: Extended Records (forward compatibility)
  // ========================================

  py::class_<msd_transfer::AssetDynamicStateRecord>(m, "AssetDynamicStateRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::AssetDynamicStateRecord::id)
    .def_readonly("body_id", &msd_transfer::AssetDynamicStateRecord::body_id)
    .def_readonly("kinematicState",
                  &msd_transfer::AssetDynamicStateRecord::kinematicState);
  // Note: externalForces (RepeatedField) and frame FK handled by Database queries

  py::class_<msd_transfer::AssetPhysicalStaticRecord>(m,
                                                       "AssetPhysicalStaticRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::AssetPhysicalStaticRecord::id)
    .def_readonly("body_id", &msd_transfer::AssetPhysicalStaticRecord::body_id)
    .def_readonly("asset_id",
                  &msd_transfer::AssetPhysicalStaticRecord::asset_id)
    .def_readonly("is_environment",
                  &msd_transfer::AssetPhysicalStaticRecord::is_environment);

  py::class_<msd_transfer::AssetPhysicalDynamicRecord>(
    m,
    "AssetPhysicalDynamicRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::AssetPhysicalDynamicRecord::id)
    .def_readonly("body_id",
                  &msd_transfer::AssetPhysicalDynamicRecord::body_id)
    .def_readonly("position",
                  &msd_transfer::AssetPhysicalDynamicRecord::position)
    .def_readonly("orientation",
                  &msd_transfer::AssetPhysicalDynamicRecord::orientation);

  py::class_<msd_transfer::MaterialRecord>(m, "MaterialRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::MaterialRecord::id)
    .def_readonly("name", &msd_transfer::MaterialRecord::name)
    .def_readonly("shader_vertex",
                  &msd_transfer::MaterialRecord::shader_vertex)
    .def_readonly("shader_fragment",
                  &msd_transfer::MaterialRecord::shader_fragment);

  py::class_<msd_transfer::PhysicsTemplateRecord>(m, "PhysicsTemplateRecord")
    .def(py::init<>())
    .def_readonly("id", &msd_transfer::PhysicsTemplateRecord::id)
    .def_readonly("name", &msd_transfer::PhysicsTemplateRecord::name)
    .def_readonly("mass", &msd_transfer::PhysicsTemplateRecord::mass)
    .def_readonly("friction", &msd_transfer::PhysicsTemplateRecord::friction)
    .def_readonly("restitution",
                  &msd_transfer::PhysicsTemplateRecord::restitution);
}
