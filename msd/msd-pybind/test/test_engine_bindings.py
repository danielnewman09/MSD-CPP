"""
Tests for msd_reader.Engine Python bindings.

Ticket: 0072a_engine_pybind_bindings

Verifies that EngineWrapper exposes the msd_sim::Engine to Python correctly:
- Construction from assets.db path
- Asset listing
- Spawning inertial objects (dynamic)
- Spawning environment objects (static)
- Advancing simulation time
- Extracting full frame state (position, velocity, orientation, angular_velocity)
- Getting collision vertices for geometry lookup

Run from build/Debug/debug directory:
    python3 -m pytest ../../../../msd/msd-pybind/test/test_engine_bindings.py -v

Or from the project root with a pre-built module:
    python3 -m pytest msd/msd-pybind/test/test_engine_bindings.py -v
"""

import sys
import os
import math
import pytest

# Add the build directory to Python path to import msd_reader
build_dir = os.path.join(os.path.dirname(__file__), '../../../build/Debug/debug')
sys.path.insert(0, build_dir)

import msd_reader

# Path to the test assets database built by generate_assets
ASSETS_DB = os.path.join(
    os.path.dirname(__file__), '../../../build/Debug/debug/assets.db')


def assets_db_available():
    """Return True if the assets database is available for testing."""
    return os.path.exists(ASSETS_DB)


# ──────────────────────────────────────────────────────────────────────────────
# Module-level checks
# ──────────────────────────────────────────────────────────────────────────────

class TestEngineClassExists:
    """Verify the Engine class is exposed in the msd_reader module."""

    def test_engine_class_exists(self):
        """AC: msd_reader.Engine is available."""
        assert hasattr(msd_reader, 'Engine'), "msd_reader.Engine not found"

    def test_engine_is_a_class(self):
        """msd_reader.Engine is a type (class)."""
        assert isinstance(msd_reader.Engine, type)


# ──────────────────────────────────────────────────────────────────────────────
# Construction
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not assets_db_available(),
                    reason="assets.db not found — build generate_assets first")
class TestEngineConstruction:
    """Test Engine construction from a database path."""

    def test_construct_from_valid_path(self):
        """AC: Engine can be constructed from a valid assets.db path."""
        engine = msd_reader.Engine(ASSETS_DB)
        assert engine is not None

    def test_construct_from_invalid_path_raises(self):
        """Engine raises when given a non-existent path."""
        with pytest.raises(Exception):
            msd_reader.Engine("/non/existent/path/assets.db")


# ──────────────────────────────────────────────────────────────────────────────
# Asset listing
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not assets_db_available(),
                    reason="assets.db not found — build generate_assets first")
class TestListAssets:
    """Test Engine.list_assets() returns available assets."""

    def setup_method(self):
        self.engine = msd_reader.Engine(ASSETS_DB)

    def test_list_assets_returns_list(self):
        """AC: list_assets() returns a list."""
        assets = self.engine.list_assets()
        assert isinstance(assets, list)

    def test_list_assets_non_empty(self):
        """AC: list_assets() returns at least one asset."""
        assets = self.engine.list_assets()
        assert len(assets) > 0

    def test_list_assets_tuples(self):
        """Each element is a (asset_id, name) tuple."""
        assets = self.engine.list_assets()
        for entry in assets:
            asset_id, name = entry
            assert isinstance(asset_id, int)
            assert isinstance(name, str)
            assert len(name) > 0

    def test_list_assets_contains_cube(self):
        """The standard 'cube' asset should be present."""
        assets = self.engine.list_assets()
        names = [name for _, name in assets]
        assert any('cube' in name.lower() for name in names), \
            f"No 'cube' asset found. Available: {names}"


# ──────────────────────────────────────────────────────────────────────────────
# Spawn inertial object
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not assets_db_available(),
                    reason="assets.db not found — build generate_assets first")
class TestSpawnInertialObject:
    """Test Engine.spawn_inertial_object() spawns a dynamic body."""

    def setup_method(self):
        self.engine = msd_reader.Engine(ASSETS_DB)
        # Determine first available asset name
        assets = self.engine.list_assets()
        assert assets, "No assets available for testing"
        self.asset_id, self.asset_name = assets[0]

    def test_spawn_returns_dict(self):
        """AC: spawn_inertial_object() returns a dict."""
        result = self.engine.spawn_inertial_object(
            self.asset_name, 0.0, 0.0, 5.0)
        assert isinstance(result, dict)

    def test_spawn_result_has_instance_id(self):
        """AC: returned dict has 'instance_id' key."""
        result = self.engine.spawn_inertial_object(
            self.asset_name, 0.0, 0.0, 5.0)
        assert 'instance_id' in result
        assert isinstance(result['instance_id'], int)

    def test_spawn_result_has_asset_id(self):
        """AC: returned dict has 'asset_id' key."""
        result = self.engine.spawn_inertial_object(
            self.asset_name, 0.0, 0.0, 5.0)
        assert 'asset_id' in result
        assert isinstance(result['asset_id'], int)

    def test_spawn_with_explicit_mass_restitution_friction(self):
        """AC: spawn_inertial_object() accepts mass, restitution, friction."""
        result = self.engine.spawn_inertial_object(
            self.asset_name, 1.0, 2.0, 5.0,
            mass=20.0, restitution=0.8, friction=0.3)
        assert isinstance(result, dict)
        assert 'instance_id' in result

    def test_spawn_with_orientation(self):
        """Spawning with custom pitch/roll/yaw does not raise."""
        result = self.engine.spawn_inertial_object(
            self.asset_name, 0.0, 0.0, 10.0,
            pitch=0.1, roll=0.0, yaw=math.pi / 4)
        assert 'instance_id' in result

    def test_spawn_unknown_asset_raises(self):
        """Spawning an unknown asset name raises an exception."""
        with pytest.raises(Exception):
            self.engine.spawn_inertial_object(
                "__nonexistent_asset__", 0.0, 0.0, 5.0)


# ──────────────────────────────────────────────────────────────────────────────
# Spawn environment object
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not assets_db_available(),
                    reason="assets.db not found — build generate_assets first")
class TestSpawnEnvironmentObject:
    """Test Engine.spawn_environment_object() spawns a static body."""

    def setup_method(self):
        self.engine = msd_reader.Engine(ASSETS_DB)
        assets = self.engine.list_assets()
        assert assets, "No assets available for testing"
        self.asset_id, self.asset_name = assets[0]

    def test_spawn_env_returns_dict(self):
        """AC: spawn_environment_object() returns a dict."""
        result = self.engine.spawn_environment_object(
            self.asset_name, 0.0, 0.0, 0.0)
        assert isinstance(result, dict)

    def test_spawn_env_result_has_instance_id(self):
        """AC: returned dict has 'instance_id'."""
        result = self.engine.spawn_environment_object(
            self.asset_name, 0.0, 0.0, 0.0)
        assert 'instance_id' in result
        assert isinstance(result['instance_id'], int)

    def test_spawn_env_result_has_asset_id(self):
        """AC: returned dict has 'asset_id'."""
        result = self.engine.spawn_environment_object(
            self.asset_name, 0.0, 0.0, 0.0)
        assert 'asset_id' in result
        assert isinstance(result['asset_id'], int)


# ──────────────────────────────────────────────────────────────────────────────
# Simulation update
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not assets_db_available(),
                    reason="assets.db not found — build generate_assets first")
class TestSimulationUpdate:
    """Test Engine.update() advances simulation time."""

    def setup_method(self):
        self.engine = msd_reader.Engine(ASSETS_DB)
        assets = self.engine.list_assets()
        assert assets, "No assets available for testing"
        _, self.asset_name = assets[0]

    def test_update_does_not_raise(self):
        """AC: engine.update(16) does not raise."""
        self.engine.update(16)

    def test_update_advances_simulation_time(self):
        """AC: get_frame_state().simulation_time increases after update()."""
        initial = self.engine.get_frame_state()['simulation_time']
        self.engine.update(16)
        after = self.engine.get_frame_state()['simulation_time']
        assert after > initial, \
            f"Simulation time did not advance: {initial} -> {after}"

    def test_update_multiple_steps(self):
        """Multiple update() calls with increasing time advance simulation.

        Note: update(simTime) receives absolute simulation time (not delta).
        The caller is responsible for tracking cumulative time.
        """
        for i in range(1, 11):
            self.engine.update(i * 16)  # Pass cumulative time: 16, 32, ..., 160ms
        frame = self.engine.get_frame_state()
        # Final cumulative time: 10 × 16ms = 160ms = 0.160s
        assert frame['simulation_time'] == pytest.approx(0.160, abs=1e-6)


# ──────────────────────────────────────────────────────────────────────────────
# Frame state
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not assets_db_available(),
                    reason="assets.db not found — build generate_assets first")
class TestGetFrameState:
    """Test Engine.get_frame_state() returns complete simulation state."""

    def setup_method(self):
        self.engine = msd_reader.Engine(ASSETS_DB)
        assets = self.engine.list_assets()
        assert assets, "No assets available for testing"
        _, self.asset_name = assets[0]

    def test_frame_state_is_dict(self):
        """AC: get_frame_state() returns a dict."""
        frame = self.engine.get_frame_state()
        assert isinstance(frame, dict)

    def test_frame_state_has_simulation_time(self):
        """AC: frame has 'simulation_time' key (float, in seconds)."""
        frame = self.engine.get_frame_state()
        assert 'simulation_time' in frame
        assert isinstance(frame['simulation_time'], float)

    def test_frame_state_has_states_list(self):
        """AC: frame has 'states' key (a list)."""
        frame = self.engine.get_frame_state()
        assert 'states' in frame
        assert isinstance(frame['states'], list)

    def test_frame_state_empty_before_spawn(self):
        """states list is empty before any objects are spawned."""
        frame = self.engine.get_frame_state()
        assert frame['states'] == []

    def test_frame_state_has_body_after_spawn(self):
        """AC: states list contains an entry after spawning an inertial object."""
        self.engine.spawn_inertial_object(self.asset_name, 0.0, 0.0, 5.0)
        frame = self.engine.get_frame_state()
        assert len(frame['states']) >= 1

    def test_body_state_has_required_keys(self):
        """AC: each state dict has body_id, asset_id, position, velocity,
        orientation, angular_velocity."""
        self.engine.spawn_inertial_object(self.asset_name, 0.0, 0.0, 5.0)
        frame = self.engine.get_frame_state()
        body = frame['states'][0]
        for key in ('body_id', 'asset_id', 'position', 'velocity',
                    'orientation', 'angular_velocity'):
            assert key in body, f"Missing key '{key}' in body state"

    def test_position_is_dict_with_xyz(self):
        """AC: position is a dict with x, y, z floats."""
        self.engine.spawn_inertial_object(self.asset_name, 1.0, 2.0, 5.0)
        frame = self.engine.get_frame_state()
        pos = frame['states'][0]['position']
        assert isinstance(pos, dict)
        for axis in ('x', 'y', 'z'):
            assert axis in pos, f"Missing '{axis}' in position"
            assert isinstance(pos[axis], float)

    def test_velocity_is_dict_with_xyz(self):
        """AC: velocity is a dict with x, y, z floats."""
        self.engine.spawn_inertial_object(self.asset_name, 0.0, 0.0, 5.0)
        frame = self.engine.get_frame_state()
        vel = frame['states'][0]['velocity']
        assert isinstance(vel, dict)
        for axis in ('x', 'y', 'z'):
            assert axis in vel

    def test_orientation_is_dict_with_wxyz(self):
        """AC: orientation is a dict with w, x, y, z floats."""
        self.engine.spawn_inertial_object(self.asset_name, 0.0, 0.0, 5.0)
        frame = self.engine.get_frame_state()
        q = frame['states'][0]['orientation']
        assert isinstance(q, dict)
        for comp in ('w', 'x', 'y', 'z'):
            assert comp in q, f"Missing '{comp}' in orientation"
            assert isinstance(q[comp], float)

    def test_angular_velocity_is_dict_with_xyz(self):
        """AC: angular_velocity is a dict with x, y, z floats."""
        self.engine.spawn_inertial_object(self.asset_name, 0.0, 0.0, 5.0)
        frame = self.engine.get_frame_state()
        av = frame['states'][0]['angular_velocity']
        assert isinstance(av, dict)
        for axis in ('x', 'y', 'z'):
            assert axis in av

    def test_spawn_position_reflected_in_state(self):
        """Position at spawn time is reflected in frame state before update."""
        self.engine.spawn_inertial_object(self.asset_name, 3.0, 4.0, 10.0)
        frame = self.engine.get_frame_state()
        pos = frame['states'][0]['position']
        assert pos['x'] == pytest.approx(3.0, abs=1e-6)
        assert pos['y'] == pytest.approx(4.0, abs=1e-6)
        assert pos['z'] == pytest.approx(10.0, abs=1e-6)

    def test_body_falls_under_gravity(self):
        """Object spawned high above the floor should fall after several steps."""
        self.engine.spawn_inertial_object(self.asset_name, 0.0, 0.0, 100.0)
        initial_z = self.engine.get_frame_state()['states'][0]['position']['z']

        # Advance 500ms using cumulative absolute time (update receives sim time, not delta)
        for i in range(1, 31):
            self.engine.update(i * 16)

        after_z = self.engine.get_frame_state()['states'][0]['position']['z']
        assert after_z < initial_z, \
            f"Object did not fall: z {initial_z} -> {after_z}"


# ──────────────────────────────────────────────────────────────────────────────
# Collision vertices
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not assets_db_available(),
                    reason="assets.db not found — build generate_assets first")
class TestGetCollisionVertices:
    """Test Engine.get_collision_vertices() returns geometry for Three.js."""

    def setup_method(self):
        self.engine = msd_reader.Engine(ASSETS_DB)
        assets = self.engine.list_assets()
        assert assets, "No assets available for testing"
        self.asset_id, self.asset_name = assets[0]

    def test_get_vertices_returns_list(self):
        """AC: get_collision_vertices(asset_id) returns a list."""
        vertices = self.engine.get_collision_vertices(self.asset_id)
        assert isinstance(vertices, list)

    def test_get_vertices_non_empty(self):
        """AC: collision vertices list is non-empty for a valid asset."""
        vertices = self.engine.get_collision_vertices(self.asset_id)
        assert len(vertices) > 0

    def test_get_vertices_are_tuples(self):
        """Each vertex is a (x, y, z) tuple of floats."""
        vertices = self.engine.get_collision_vertices(self.asset_id)
        for v in vertices:
            x, y, z = v
            assert isinstance(x, float)
            assert isinstance(y, float)
            assert isinstance(z, float)

    def test_get_vertices_unknown_id_returns_empty(self):
        """AC: get_collision_vertices(999999) returns empty list."""
        vertices = self.engine.get_collision_vertices(999999)
        assert vertices == []
