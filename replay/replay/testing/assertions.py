"""
Physics-invariant assertion functions for pytest tests.

Ticket: 0060c_replay_matchers
"""

from pathlib import Path
from .recording_query import RecordingQuery


def assert_energy_conserved(db_path: str | Path, tolerance: float) -> None:
    """Assert that system energy is conserved within the given relative tolerance.

    Args:
        db_path: Path to recording database file
        tolerance: Maximum acceptable relative energy drift (e.g., 0.05 for 5%)

    Raises:
        AssertionError: If max energy drift exceeds tolerance, with actual drift value

    Example:
        >>> assert_energy_conserved("recording.db", tolerance=0.05)
    """
    q = RecordingQuery(db_path)
    drift = q.max_energy_drift()
    assert drift < tolerance, f"max energy drift was {drift:.6f}, expected < {tolerance}"


def assert_never_penetrates_below(
    db_path: str | Path, body_id: int, z_min: float
) -> None:
    """Assert that a body's z-position never drops below the given threshold.

    Args:
        db_path: Path to recording database file
        body_id: ID of the body to check
        z_min: Minimum acceptable z-position

    Raises:
        AssertionError: If body penetrates below z_min, with actual min z value

    Example:
        >>> assert_never_penetrates_below("recording.db", body_id=1, z_min=-0.6)
    """
    q = RecordingQuery(db_path)
    actual_min_z = q.min_z(body_id)
    assert actual_min_z >= z_min, (
        f"body {body_id} min z was {actual_min_z:.6f}, expected >= {z_min}"
    )


def assert_body_comes_to_rest(
    db_path: str | Path, body_id: int, speed_threshold: float
) -> None:
    """Assert that a body's final speed is below the given threshold.

    Args:
        db_path: Path to recording database file
        body_id: ID of the body to check
        speed_threshold: Maximum acceptable final speed

    Raises:
        AssertionError: If final speed exceeds threshold, with actual final speed

    Example:
        >>> assert_body_comes_to_rest("recording.db", body_id=1, speed_threshold=0.1)
    """
    q = RecordingQuery(db_path)
    speeds = q.speed_history(body_id)
    final_speed = speeds[-1] if speeds else 0.0
    assert final_speed < speed_threshold, (
        f"body {body_id} final speed was {final_speed:.6f}, expected < {speed_threshold}"
    )
