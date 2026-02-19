"""
Unit tests for physics-invariant assertion functions.

Ticket: 0060c_replay_matchers
"""

import pytest
import sqlite3
from pathlib import Path
import msd_reader

from replay.testing import (
        assert_energy_conserved,
        assert_never_penetrates_below,
        assert_body_comes_to_rest,
    )

from replay.testing.conftest import recording_for


# Helper to create a minimal recording database for testing
def create_test_recording(
    db_path: Path,
    *,
    energy_values: list[float] | None = None,
    body_positions: dict[int, list[tuple[float, float, float]]] | None = None,
    body_velocities: dict[int, list[tuple[float, float, float]]] | None = None,
) -> None:
    """Create a minimal recording database for testing assertions.

    Args:
        db_path: Path where database should be created
        energy_values: List of total system energy values (one per frame)
        body_positions: Dict mapping body_id to list of (x, y, z) positions
        body_velocities: Dict mapping body_id to list of (vx, vy, vz) velocities
    """
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Create tables
    cursor.execute("""
        CREATE TABLE SimulationFrameRecord (
            frame_id INTEGER PRIMARY KEY,
            simulation_time REAL
        )
    """)

    cursor.execute("""
        CREATE TABLE InertialStateRecord (
            frame_id INTEGER,
            body_id INTEGER,
            position_x REAL,
            position_y REAL,
            position_z REAL,
            velocity_x REAL,
            velocity_y REAL,
            velocity_z REAL
        )
    """)

    cursor.execute("""
        CREATE TABLE SystemEnergyRecord (
            frame_id INTEGER PRIMARY KEY,
            total_system_e REAL
        )
    """)

    # Insert frame records
    frame_count = 0
    if energy_values:
        frame_count = max(frame_count, len(energy_values))
    if body_positions:
        for positions in body_positions.values():
            frame_count = max(frame_count, len(positions))

    for frame_id in range(frame_count):
        cursor.execute(
            "INSERT INTO SimulationFrameRecord (frame_id, simulation_time) VALUES (?, ?)",
            (frame_id, frame_id * 0.01),
        )

    # Insert energy records
    if energy_values:
        for frame_id, energy in enumerate(energy_values):
            cursor.execute(
                "INSERT INTO SystemEnergyRecord (frame_id, total_system_e) VALUES (?, ?)",
                (frame_id, energy),
            )

    # Insert position/velocity records
    if body_positions:
        for body_id, positions in body_positions.items():
            velocities = body_velocities.get(body_id, [(0, 0, 0)] * len(positions)) if body_velocities else [(0, 0, 0)] * len(positions)
            for frame_id, (pos, vel) in enumerate(zip(positions, velocities)):
                cursor.execute(
                    """
                    INSERT INTO InertialStateRecord
                    (frame_id, body_id, position_x, position_y, position_z,
                     velocity_x, velocity_y, velocity_z)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (frame_id, body_id, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]),
                )

    conn.commit()
    conn.close()

class TestAssertEnergyConserved:
    """Tests for assert_energy_conserved assertion function."""

    def test_passes_when_drift_below_tolerance(self, tmp_path):
        """Verify no assertion raised when energy drift is within tolerance."""
        db_path = tmp_path / "recording.db"
        # Energy drift of 2% (100 -> 102)
        create_test_recording(db_path, energy_values=[100.0, 101.0, 102.0])

        # Should pass with 5% tolerance
        assert_energy_conserved(db_path, tolerance=0.05)

    def test_fails_when_drift_exceeds_tolerance(self, tmp_path):
        """Verify AssertionError raised with actual drift in message."""
        db_path = tmp_path / "recording.db"
        # Energy drift of 10% (100 -> 110)
        create_test_recording(db_path, energy_values=[100.0, 105.0, 110.0])

        # Should fail with 5% tolerance
        with pytest.raises(AssertionError) as exc_info:
            assert_energy_conserved(db_path, tolerance=0.05)

        # Verify error message contains actual drift
        assert "max energy drift was 0.1" in str(exc_info.value)
        assert "expected < 0.05" in str(exc_info.value)

    def test_handles_zero_drift(self, tmp_path):
        """Verify assertion passes when energy is perfectly conserved."""
        db_path = tmp_path / "recording.db"
        create_test_recording(db_path, energy_values=[100.0, 100.0, 100.0])

        assert_energy_conserved(db_path, tolerance=0.01)

class TestAssertNeverPenetratesBelow:
    """Tests for assert_never_penetrates_below assertion function."""

    def test_passes_when_above_threshold(self, tmp_path):
        """Verify no assertion raised when body stays above z threshold."""
        db_path = tmp_path / "recording.db"
        # Body stays between z=0 and z=5
        positions = [(0, 0, 0), (0, 0, 2.5), (0, 0, 5.0)]
        create_test_recording(db_path, body_positions={1: positions})

        # Should pass with threshold of -1.0
        assert_never_penetrates_below(db_path, body_id=1, z_min=-1.0)

    def test_fails_when_below_threshold(self, tmp_path):
        """Verify AssertionError raised with actual min z in message."""
        db_path = tmp_path / "recording.db"
        # Body drops to z=-2.0
        positions = [(0, 0, 5), (0, 0, 0), (0, 0, -2.0)]
        create_test_recording(db_path, body_positions={1: positions})

        # Should fail with threshold of -1.0
        with pytest.raises(AssertionError) as exc_info:
            assert_never_penetrates_below(db_path, body_id=1, z_min=-1.0)

        # Verify error message contains actual min z
        assert "body 1 min z was -2.0" in str(exc_info.value)
        assert "expected >= -1.0" in str(exc_info.value)

    def test_passes_when_exactly_at_threshold(self, tmp_path):
        """Verify assertion passes when body reaches exactly z_min."""
        db_path = tmp_path / "recording.db"
        positions = [(0, 0, 5), (0, 0, 0), (0, 0, -1.0)]
        create_test_recording(db_path, body_positions={1: positions})

        # Should pass (>= allows equality)
        assert_never_penetrates_below(db_path, body_id=1, z_min=-1.0)

class TestAssertBodyComesToRest:
    """Tests for assert_body_comes_to_rest assertion function."""

    def test_passes_when_final_speed_below_threshold(self, tmp_path):
        """Verify no assertion raised when body has settled."""
        db_path = tmp_path / "recording.db"
        # Body slows down from 5.0 to 0.05
        velocities = [(5, 0, 0), (2, 0, 0), (0.05, 0, 0)]
        positions = [(0, 0, 0)] * len(velocities)
        create_test_recording(
            db_path, body_positions={1: positions}, body_velocities={1: velocities}
        )

        # Should pass with threshold of 0.1
        assert_body_comes_to_rest(db_path, body_id=1, speed_threshold=0.1)

    def test_fails_when_still_moving(self, tmp_path):
        """Verify AssertionError raised with actual final speed in message."""
        db_path = tmp_path / "recording.db"
        # Body slows down but final speed is 2.0
        velocities = [(5, 0, 0), (3, 0, 0), (2, 0, 0)]
        positions = [(0, 0, 0)] * len(velocities)
        create_test_recording(
            db_path, body_positions={1: positions}, body_velocities={1: velocities}
        )

        # Should fail with threshold of 0.1
        with pytest.raises(AssertionError) as exc_info:
            assert_body_comes_to_rest(db_path, body_id=1, speed_threshold=0.1)

        # Verify error message contains actual final speed
        assert "body 1 final speed was 2.0" in str(exc_info.value)
        assert "expected < 0.1" in str(exc_info.value)

    def test_handles_3d_velocity(self, tmp_path):
        """Verify speed calculation handles 3D velocity vectors correctly."""
        db_path = tmp_path / "recording.db"
        # Final velocity is (0.03, 0.04, 0.0) -> speed = 0.05
        velocities = [(5, 0, 0), (1, 1, 1), (0.03, 0.04, 0.0)]
        positions = [(0, 0, 0)] * len(velocities)
        create_test_recording(
            db_path, body_positions={1: positions}, body_velocities={1: velocities}
        )

        # Should pass with threshold of 0.06
        assert_body_comes_to_rest(db_path, body_id=1, speed_threshold=0.06)

        # Should fail with threshold of 0.04
        with pytest.raises(AssertionError):
            assert_body_comes_to_rest(db_path, body_id=1, speed_threshold=0.04)

    def test_handles_empty_recording(self, tmp_path):
        """Verify assertion handles empty recording gracefully."""
        db_path = tmp_path / "recording.db"
        create_test_recording(db_path)

        # Should pass (empty -> final speed is 0.0)
        assert_body_comes_to_rest(db_path, body_id=1, speed_threshold=0.1)


class TestRecordingForHelper:
    """Tests for recording_for() helper function."""

    def test_raises_when_file_missing(self, tmp_path):
        """Verify helpful error message when recording not found."""
        # Temporarily override RECORDINGS_DIR
        import replay.testing.conftest as conftest_module

        original_dir = conftest_module.RECORDINGS_DIR
        conftest_module.RECORDINGS_DIR = tmp_path

        try:
            with pytest.raises(FileNotFoundError) as exc_info:
                recording_for("PhysicsTest", "BallDrop")

            # Verify helpful error message
            error_msg = str(exc_info.value)
            assert "Recording not found" in error_msg
            assert "PhysicsTest_BallDrop.db" in error_msg
            assert "Run C++ tests first" in error_msg
            assert "gtest_filter" in error_msg
        finally:
            conftest_module.RECORDINGS_DIR = original_dir

    def test_returns_path_when_file_exists(self, tmp_path):
        """Verify recording_for returns correct path when file exists."""
        import replay.testing.conftest as conftest_module

        original_dir = conftest_module.RECORDINGS_DIR
        conftest_module.RECORDINGS_DIR = tmp_path

        try:
            # Create a dummy recording file
            recording_path = tmp_path / "PhysicsTest_BallDrop.db"
            recording_path.touch()

            # Should return the path without raising
            result = recording_for("PhysicsTest", "BallDrop")
            assert result == recording_path
            assert result.exists()
        finally:
            conftest_module.RECORDINGS_DIR = original_dir
