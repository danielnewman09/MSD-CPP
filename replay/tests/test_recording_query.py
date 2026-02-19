"""
RecordingQuery unit tests

Ticket: 0060b_recording_query_api

These tests verify that RecordingQuery correctly reads recording databases
and provides accurate query results for position, velocity, energy, and
contact event data.
"""

import math
import pytest
from pathlib import Path

import msd_reader
from replay.testing import RecordingQuery


@pytest.fixture
def sample_recording() -> Path:
    """Get path to a sample recording database.

    Uses an existing recording from the test suite if available.
    Tests that depend on specific data should use more targeted fixtures.
    """
    # Look for any existing recording from the replay tests
    recordings_dir = Path(__file__).parent.parent / "recordings"
    if recordings_dir.exists():
        recordings = list(recordings_dir.glob("*.db"))
        if recordings:
            return recordings[0]

    pytest.skip("No sample recording database found")


@pytest.fixture
def step_recording() -> Path:
    """Get path to the Step test recording.

    This recording should have multiple frames from simulation steps.
    """
    recording_path = (
        Path(__file__).parent.parent
        / "recordings"
        / "ReplayEnabledTest_Step_AdvancesSimulationTime.db"
    )
    if not recording_path.exists():
        pytest.skip(f"Step recording not found: {recording_path}")
    return recording_path


class TestRecordingQueryInit:
    """Test RecordingQuery initialization."""

    def test_init_with_valid_path_succeeds(self, sample_recording):
        """RecordingQuery opens valid database without error."""
        query = RecordingQuery(sample_recording)
        assert query is not None

    def test_init_with_str_path_succeeds(self, sample_recording):
        """RecordingQuery accepts string paths."""
        query = RecordingQuery(str(sample_recording))
        assert query is not None

    def test_init_with_pathlib_path_succeeds(self, sample_recording):
        """RecordingQuery accepts pathlib.Path objects."""
        query = RecordingQuery(Path(sample_recording))
        assert query is not None


class TestFrameQueries:
    """Test frame count and timing queries."""

    def test_frame_count_returns_non_negative(self, sample_recording):
        """frame_count() returns a non-negative integer."""
        query = RecordingQuery(sample_recording)
        count = query.frame_count()
        assert isinstance(count, int)
        assert count >= 0

    def test_total_simulation_time_returns_non_negative(self, sample_recording):
        """total_simulation_time() returns a non-negative float."""
        query = RecordingQuery(sample_recording)
        time = query.total_simulation_time()
        assert isinstance(time, float)
        assert time >= 0.0

    def test_step_recording_has_multiple_frames(self, step_recording):
        """Step recording should have multiple frames."""
        query = RecordingQuery(step_recording)
        count = query.frame_count()
        assert count > 1  # Should have at least a few frames from stepping


class TestPositionVelocityQueries:
    """Test position and velocity timeseries queries."""

    def test_position_history_returns_list_of_tuples(self, step_recording):
        """position_history() returns list of (x, y, z) tuples."""
        query = RecordingQuery(step_recording)

        # Find a body_id that exists in the recording
        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id
        positions = query.position_history(body_id)

        assert isinstance(positions, list)
        if positions:
            assert len(positions[0]) == 3  # (x, y, z)
            assert all(isinstance(val, float) for val in positions[0])

    def test_velocity_history_returns_list_of_tuples(self, step_recording):
        """velocity_history() returns list of (vx, vy, vz) tuples."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id
        velocities = query.velocity_history(body_id)

        assert isinstance(velocities, list)
        if velocities:
            assert len(velocities[0]) == 3  # (vx, vy, vz)
            assert all(isinstance(val, float) for val in velocities[0])

    def test_speed_history_returns_non_negative_values(self, step_recording):
        """speed_history() returns non-negative speed values."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id
        speeds = query.speed_history(body_id)

        assert isinstance(speeds, list)
        assert all(speed >= 0.0 for speed in speeds)

    def test_speed_history_matches_velocity_magnitude(self, step_recording):
        """speed_history() correctly computes velocity magnitude."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id
        velocities = query.velocity_history(body_id)
        speeds = query.speed_history(body_id)

        assert len(velocities) == len(speeds)

        # Verify magnitude calculation for each frame
        for (vx, vy, vz), speed in zip(velocities, speeds):
            expected_speed = math.sqrt(vx**2 + vy**2 + vz**2)
            assert abs(speed - expected_speed) < 1e-9

    def test_position_history_is_ordered_by_frame_id(self, step_recording):
        """position_history() returns positions ordered by frame ID."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id

        # Get positions through query
        positions_from_query = query.position_history(body_id)

        # Get states directly and manually filter/sort
        all_states = query._db.select_all_states()
        body_states = [s for s in all_states if s.body_id == body_id]
        body_states.sort(key=lambda s: s.frame_id)
        positions_expected = [
            (s.position.x, s.position.y, s.position.z) for s in body_states
        ]

        assert positions_from_query == positions_expected


class TestPositionAggregates:
    """Test per-body position aggregate queries."""

    def test_position_at_frame_returns_correct_position(self, step_recording):
        """position_at_frame() returns the correct position for a specific frame."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        # Pick a state and verify we can retrieve it
        test_state = states[0]
        position = query.position_at_frame(test_state.body_id, test_state.frame_id)

        assert position == (
            test_state.position.x,
            test_state.position.y,
            test_state.position.z,
        )

    def test_position_at_frame_raises_on_invalid_body(self, step_recording):
        """position_at_frame() raises ValueError for non-existent body."""
        query = RecordingQuery(step_recording)

        with pytest.raises(ValueError, match="No state found"):
            query.position_at_frame(body_id=999999, frame_id=0)

    def test_min_z_returns_minimum_z_coordinate(self, step_recording):
        """min_z() returns the lowest z-coordinate across all frames."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id
        min_z_value = query.min_z(body_id)

        # Verify by manually checking positions
        positions = query.position_history(body_id)
        expected_min = min(z for _, _, z in positions)

        assert min_z_value == expected_min

    def test_max_z_returns_maximum_z_coordinate(self, step_recording):
        """max_z() returns the highest z-coordinate across all frames."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id
        max_z_value = query.max_z(body_id)

        # Verify by manually checking positions
        positions = query.position_history(body_id)
        expected_max = max(z for _, _, z in positions)

        assert max_z_value == expected_max

    def test_max_speed_returns_maximum_speed(self, step_recording):
        """max_speed() returns the highest speed across all frames."""
        query = RecordingQuery(step_recording)

        states = query._db.select_all_states()
        if not states:
            pytest.skip("Recording has no states")

        body_id = states[0].body_id
        max_speed_value = query.max_speed(body_id)

        # Verify by manually checking speeds
        speeds = query.speed_history(body_id)
        expected_max = max(speeds)

        assert max_speed_value == expected_max
        assert max_speed_value >= 0.0

    def test_min_z_raises_on_empty_history(self, step_recording):
        """min_z() raises ValueError when no positions exist for body."""
        query = RecordingQuery(step_recording)

        with pytest.raises(ValueError, match="No positions found"):
            query.min_z(body_id=999999)


class TestEnergyQueries:
    """Test system energy queries."""

    def test_system_energy_history_returns_list_of_floats(self, step_recording):
        """system_energy_history() returns a list of energy values."""
        query = RecordingQuery(step_recording)
        energies = query.system_energy_history()

        assert isinstance(energies, list)
        # May be empty if energy tracking not enabled in this recording
        if energies:
            assert all(isinstance(e, float) for e in energies)

    def test_max_energy_drift_returns_non_negative(self, step_recording):
        """max_energy_drift() returns a non-negative drift value."""
        query = RecordingQuery(step_recording)
        drift = query.max_energy_drift()

        assert isinstance(drift, float)
        assert drift >= 0.0

    def test_max_energy_drift_is_zero_when_no_energy_records(self, sample_recording):
        """max_energy_drift() returns 0.0 when no energy records exist."""
        query = RecordingQuery(sample_recording)

        # Check if there are energy records
        energies = query.system_energy_history()
        if not energies:
            drift = query.max_energy_drift()
            assert drift == 0.0

    def test_max_energy_drift_computes_relative_error(self, step_recording):
        """max_energy_drift() computes correct relative energy error."""
        query = RecordingQuery(step_recording)
        energies = query.system_energy_history()

        if not energies or abs(energies[0]) < 1e-12:
            pytest.skip("No energy data or initial energy near zero")

        drift = query.max_energy_drift()
        e0 = energies[0]

        # Manually compute expected drift
        expected_drift = max(abs(e - e0) / abs(e0) for e in energies)

        assert abs(drift - expected_drift) < 1e-12


class TestContactEventQueries:
    """Test contact/collision event queries."""

    def test_total_contact_frames_returns_non_negative(self, step_recording):
        """total_contact_frames() returns a non-negative integer."""
        query = RecordingQuery(step_recording)
        contact_frames = query.total_contact_frames()

        assert isinstance(contact_frames, int)
        assert contact_frames >= 0

    def test_total_contact_frames_counts_distinct_frames(self, step_recording):
        """total_contact_frames() counts distinct frames with collisions."""
        query = RecordingQuery(step_recording)

        # Get collisions directly
        collisions = query._db.select_all_collisions()
        if not collisions:
            # No collisions - count should be zero
            assert query.total_contact_frames() == 0
        else:
            # Manually count unique frames
            unique_frames = set(c.frame_id for c in collisions)
            expected_count = len(unique_frames)

            assert query.total_contact_frames() == expected_count

    def test_contact_frames_between_returns_non_negative(self, step_recording):
        """contact_frames_between() returns a non-negative integer."""
        query = RecordingQuery(step_recording)
        contact_frames = query.contact_frames_between(body_a_id=0, body_b_id=1)

        assert isinstance(contact_frames, int)
        assert contact_frames >= 0

    def test_contact_frames_between_is_order_independent(self, step_recording):
        """contact_frames_between() returns same result regardless of argument order."""
        query = RecordingQuery(step_recording)

        collisions = query._db.select_all_collisions()
        if not collisions:
            pytest.skip("No collisions in recording")

        # Pick a collision pair from the recording
        body_a = collisions[0].body_a_id
        body_b = collisions[0].body_b_id

        count_ab = query.contact_frames_between(body_a, body_b)
        count_ba = query.contact_frames_between(body_b, body_a)

        assert count_ab == count_ba

    def test_contact_frames_between_filters_by_body_pair(self, step_recording):
        """contact_frames_between() only counts frames with the specific body pair."""
        query = RecordingQuery(step_recording)

        collisions = query._db.select_all_collisions()
        if not collisions:
            pytest.skip("No collisions in recording")

        # Pick a collision pair
        body_a = collisions[0].body_a_id
        body_b = collisions[0].body_b_id

        # Manually count frames where this pair collides
        contact_frames = set()
        for c in collisions:
            if (c.body_a_id == body_a and c.body_b_id == body_b) or (
                c.body_a_id == body_b and c.body_b_id == body_a
            ):
                contact_frames.add(c.frame_id)

        expected_count = len(contact_frames)
        actual_count = query.contact_frames_between(body_a, body_b)

        assert actual_count == expected_count


class TestEmptyRecording:
    """Test RecordingQuery behavior with empty or minimal recordings."""

    def test_empty_recording_frame_count_is_zero(self, sample_recording):
        """Empty recording returns zero frame count."""
        query = RecordingQuery(sample_recording)

        # If the recording truly has no frames
        if query.frame_count() == 0:
            assert query.total_simulation_time() == 0.0

    def test_no_collisions_returns_zero(self, sample_recording):
        """Recording with no collisions returns zero contact frames."""
        query = RecordingQuery(sample_recording)

        collisions = query._db.select_all_collisions()
        if not collisions:
            assert query.total_contact_frames() == 0
            assert query.contact_frames_between(0, 1) == 0
