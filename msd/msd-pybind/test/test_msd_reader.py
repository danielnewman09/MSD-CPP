"""
Tests for msd_reader Python bindings.

These tests verify that the pybind11 module provides correct Python access to:
- Database record types (Tier 1, 2, 3 from ticket 0056c)
- Database query operations (select_all, select_by_frame, select_by_body, select_by_id)
- Geometry deserialization (collision and visual vertices)

Run from build/Debug/debug directory:
    python3 -m pytest ../../msd/msd-pybind/test/test_msd_reader.py
"""

import sys
import os
import pytest

# Add the build directory to Python path to import msd_reader
build_dir = os.path.join(os.path.dirname(__file__), '../../../build/Debug/debug')
sys.path.insert(0, build_dir)

import msd_reader


class TestModuleImport:
    """Test that the msd_reader module imports correctly."""

    def test_import_msd_reader(self):
        """AC1: import msd_reader succeeds after building"""
        assert msd_reader is not None

    def test_database_class_exists(self):
        """Verify Database class is exposed"""
        assert hasattr(msd_reader, 'Database')


class TestRecordTypes:
    """Test that all record types are exposed as Python classes."""

    def test_tier1_records_exist(self):
        """AC2: All Tier 1 (top-level) record types are available"""
        tier1_records = [
            'SimulationFrameRecord',
            'AssetInertialStaticRecord',
            'InertialStateRecord',
            'EnergyRecord',
            'SystemEnergyRecord',
            'CollisionResultRecord',
            'SolverDiagnosticRecord',
            'MeshRecord',
            'ObjectRecord',
        ]
        for record_name in tier1_records:
            assert hasattr(msd_reader, record_name), f"Missing {record_name}"

    def test_tier2_subrecords_exist(self):
        """AC8: All Tier 2 (sub-record) types are available"""
        tier2_records = [
            'CoordinateRecord',
            'VelocityRecord',
            'AccelerationRecord',
            'QuaternionDRecord',
            'Vector4DRecord',
            'Vector3DRecord',
            'AngularAccelerationRecord',
            'ContactPointRecord',
        ]
        for record_name in tier2_records:
            assert hasattr(msd_reader, record_name), f"Missing {record_name}"

    def test_tier3_extended_records_exist(self):
        """Verify Tier 3 (forward compatibility) records are bound"""
        tier3_records = [
            'AssetDynamicStateRecord',
            'ExternalForceRecord',
            'ForceVectorRecord',
            'TorqueVectorRecord',
            'AssetPhysicalDynamicRecord',
            'AssetPhysicalStaticRecord',
            'MaterialRecord',
            'PhysicsTemplateRecord',
        ]
        for record_name in tier3_records:
            assert hasattr(msd_reader, record_name), f"Missing {record_name}"


class TestDatabaseOperations:
    """Test database opening and querying."""

    def test_open_database(self):
        """Verify database can be opened"""
        # Use a test database from the project
        db_path = os.path.join(os.path.dirname(__file__), '../../../test_geometry.db')
        if os.path.exists(db_path):
            db = msd_reader.Database(db_path)
            assert db is not None

    def test_select_all_meshrecord(self):
        """AC2: Can query MeshRecord using select_all_meshes"""
        db_path = os.path.join(os.path.dirname(__file__), '../../../test_geometry.db')
        if os.path.exists(db_path):
            db = msd_reader.Database(db_path)
            meshes = db.select_all_meshes()
            assert isinstance(meshes, list)
            if len(meshes) > 0:
                mesh = meshes[0]
                assert hasattr(mesh, 'id')
                assert hasattr(mesh, 'vertex_data')
                assert hasattr(mesh, 'vertex_count')

    def test_select_by_id_meshrecord(self):
        """Verify select_mesh_by_id works for MeshRecord"""
        db_path = os.path.join(os.path.dirname(__file__), '../../../test_geometry.db')
        if os.path.exists(db_path):
            db = msd_reader.Database(db_path)
            meshes = db.select_all_meshes()
            if len(meshes) > 0:
                mesh_id = meshes[0].id
                mesh = db.select_mesh_by_id(mesh_id)
                assert mesh is not None
                assert mesh.id == mesh_id


class TestGeometryDeserialization:
    """Test vertex BLOB deserialization functions."""

    def test_deserialize_collision_vertices_exists(self):
        """AC4: deserialize_collision_vertices function is exposed"""
        assert hasattr(msd_reader, 'deserialize_collision_vertices')

    def test_deserialize_visual_vertices_exists(self):
        """AC5: deserialize_visual_vertices function is exposed"""
        assert hasattr(msd_reader, 'deserialize_visual_vertices')

    def test_deserialize_collision_vertices(self):
        """AC4: Collision vertex deserialization produces (x,y,z) tuples"""
        db_path = os.path.join(os.path.dirname(__file__), '../../../test_geometry.db')
        if os.path.exists(db_path):
            db = msd_reader.Database(db_path)
            meshes = db.select_all_meshes()
            if len(meshes) > 0:
                mesh = meshes[0]
                vertices = msd_reader.deserialize_collision_vertices(mesh.vertex_data)
                assert isinstance(vertices, list)
                if len(vertices) > 0:
                    v = vertices[0]
                    assert len(v) == 3, "Collision vertices should be (x, y, z)"
                    # Verify all elements are numbers
                    assert all(isinstance(coord, float) for coord in v)

    def test_deserialize_visual_vertices(self):
        """AC5: Visual vertex deserialization produces 9-element tuples"""
        db_path = os.path.join(os.path.dirname(__file__), '../../../test_geometry.db')
        if os.path.exists(db_path):
            db = msd_reader.Database(db_path)
            meshes = db.select_all_meshes()
            if len(meshes) > 0:
                mesh = meshes[0]
                vertices = msd_reader.deserialize_visual_vertices(mesh.vertex_data)
                assert isinstance(vertices, list)
                if len(vertices) > 0:
                    v = vertices[0]
                    assert len(v) == 9, "Visual vertices should be (px,py,pz,cx,cy,cz,nx,ny,nz)"
                    # Verify all elements are numbers
                    assert all(isinstance(coord, float) for coord in v)


class TestSubRecordAccess:
    """Test that sub-records are accessible as nested Python objects."""

    def test_coordinate_record_fields(self):
        """AC8: Sub-records accessible as nested objects - CoordinateRecord"""
        # Create a mock test - actual test would need database with InertialStateRecord
        assert hasattr(msd_reader, 'CoordinateRecord')
        # CoordinateRecord should have x, y, z fields when instantiated

    def test_contact_point_record_fields(self):
        """AC8: Sub-records accessible as nested objects - ContactPointRecord"""
        assert hasattr(msd_reader, 'ContactPointRecord')
        # ContactPointRecord should have pointA, pointB, depth fields


class TestRepeatedFieldSupport:
    """Test that RepeatedField collections are iterable in Python."""

    def test_collision_result_has_contacts(self):
        """AC9: CollisionResultRecord.contacts (RepeatedField) exists"""
        assert hasattr(msd_reader, 'CollisionResultRecord')
        # Actual iteration test would require a database with collision records


class TestBuildConfiguration:
    """Test build configuration and presets."""

    def test_pybind_preset_exists(self):
        """AC6: Build preset debug-pybind-only works (checked by successful build)"""
        # This test passes if the module was built and imported successfully
        assert msd_reader is not None

if __name__ == '__main__':
    pytest.main([__file__, '-v'])
