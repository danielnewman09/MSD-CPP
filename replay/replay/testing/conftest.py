"""
Pytest fixtures for recording path discovery.

Ticket: 0060c_replay_matchers
"""

import pytest
from pathlib import Path

RECORDINGS_DIR = Path(__file__).parent.parent.parent / "recordings"


@pytest.fixture
def recordings_dir() -> Path:
    """Path to the recordings directory."""
    return RECORDINGS_DIR


def recording_for(test_suite: str, test_name: str) -> Path:
    """Resolve path to a specific recording produced by C++ GTest.

    Args:
        test_suite: Name of the C++ GTest suite (e.g., "PhysicsTest")
        test_name: Name of the specific test (e.g., "BallDrop")

    Returns:
        Path to the recording database file

    Raises:
        FileNotFoundError: If recording does not exist, with helpful message

    Example:
        >>> path = recording_for("PhysicsTest", "BallDrop")
        >>> assert path.exists()
    """
    path = RECORDINGS_DIR / f"{test_suite}_{test_name}.db"
    if not path.exists():
        raise FileNotFoundError(
            f"Recording not found: {path}\n"
            f"Run C++ tests first: ./build/Debug/debug/msd_sim_test "
            f'--gtest_filter="{test_suite}*"'
        )
    return path
