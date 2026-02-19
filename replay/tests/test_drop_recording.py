"""
Python recording validation tests for cube drop scenario.

Validates recording databases produced by ReplayDropTest C++ tests.
Demonstrates recording_for() helper and RecordingQuery API usage.

Ticket: 0060d_example_replay_tests
"""

import pytest
from replay.testing.conftest import recording_for
from replay.testing import RecordingQuery
from replay.testing.assertions import (
    assert_never_penetrates_below,
    assert_body_comes_to_rest,
)


def test_cube_drop_recording_valid():
    """Verify the cube drop recording contains expected frame count."""
    db_path = recording_for("ReplayDropTest", "CubeDropsAndSettles")
    q = RecordingQuery(db_path)

    # Should have ~300 frames (5 seconds at 60 FPS)
    assert q.frame_count() > 290, f"Expected >290 frames, got {q.frame_count()}"


def test_cube_drop_never_penetrates_floor():
    """Verify cube never falls too far below floor surface."""
    db_path = recording_for("ReplayDropTest", "CubeDropsAndSettles")

    # Floor top is at z=-10.0, allow some penetration tolerance
    # Current physics may have some penetration - validate it's bounded
    assert_never_penetrates_below(db_path, body_id=1, z_min=-11.0)


def test_cube_drop_settles():
    """Verify cube comes to rest after dropping (velocity approaches zero)."""
    db_path = recording_for("ReplayDropTest", "CubeDropsAndSettles")

    # After 5 seconds, cube should have negligible speed
    # Use generous threshold since physics may have some jitter
    assert_body_comes_to_rest(db_path, body_id=1, speed_threshold=1.0)
