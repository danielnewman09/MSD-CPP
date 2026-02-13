"""
RecordingQuery — Python query API over recording databases

Ticket: 0060b_recording_query_api
"""

import math
from pathlib import Path

try:
    import msd_reader
except ImportError:
    msd_reader = None


class RecordingQuery:
    """Query wrapper over a recording database for test assertions.

    Opens a recording database (read-only) via the msd_reader pybind11 module
    and provides typed queries for position/velocity timeseries, energy tracking,
    and contact event analysis.

    Example:
        >>> query = RecordingQuery("path/to/recording.db")
        >>> assert query.frame_count() == 100
        >>> assert query.min_z(body_id=1) > 0.0  # Never fell through floor
        >>> assert query.max_energy_drift() < 0.01  # <1% energy drift
    """

    def __init__(self, db_path: str | Path) -> None:
        """Initialize query wrapper with database connection.

        Args:
            db_path: Path to the recording database (.db file)

        Raises:
            RuntimeError: If msd_reader module is not available
        """
        if msd_reader is None:
            raise RuntimeError(
                "msd_reader module not available. "
                "Build with -o '&:enable_pybind=True' to enable Python bindings."
            )

        self._db = msd_reader.Database(str(db_path))

    # Frame queries

    def frame_count(self) -> int:
        """Get total number of recorded frames.

        Returns:
            Number of frames in the recording
        """
        return len(self._db.select_all_frames())

    def total_simulation_time(self) -> float:
        """Get total simulation time from last frame.

        Returns:
            Simulation time of the last frame, or 0.0 if no frames exist
        """
        frames = self._db.select_all_frames()
        return frames[-1].simulation_time if frames else 0.0

    # Per-body timeseries

    def position_history(self, body_id: int) -> list[tuple[float, float, float]]:
        """Get position timeseries for a body.

        Args:
            body_id: ID of the body to query

        Returns:
            List of (x, y, z) tuples, one per frame, ordered by frame ID
        """
        states = self._db.select_all_states()
        body_states = [s for s in states if s.body_id == body_id]
        body_states.sort(key=lambda s: s.frame_id)
        return [(s.position.x, s.position.y, s.position.z) for s in body_states]

    def velocity_history(self, body_id: int) -> list[tuple[float, float, float]]:
        """Get velocity timeseries for a body.

        Args:
            body_id: ID of the body to query

        Returns:
            List of (vx, vy, vz) tuples, one per frame, ordered by frame ID
        """
        states = self._db.select_all_states()
        body_states = [s for s in states if s.body_id == body_id]
        body_states.sort(key=lambda s: s.frame_id)
        return [(s.velocity.x, s.velocity.y, s.velocity.z) for s in body_states]

    def speed_history(self, body_id: int) -> list[float]:
        """Get speed (velocity magnitude) timeseries for a body.

        Args:
            body_id: ID of the body to query

        Returns:
            List of speed values, one per frame, ordered by frame ID
        """
        velocities = self.velocity_history(body_id)
        return [math.sqrt(vx**2 + vy**2 + vz**2) for vx, vy, vz in velocities]

    # Per-body aggregates

    def position_at_frame(
        self, body_id: int, frame_id: int
    ) -> tuple[float, float, float]:
        """Get position of a body at a specific frame.

        Args:
            body_id: ID of the body to query
            frame_id: Frame ID to query

        Returns:
            (x, y, z) position tuple

        Raises:
            ValueError: If no state exists for the given body_id and frame_id
        """
        states = self._db.select_all_states()
        for s in states:
            if s.body_id == body_id and s.frame_id == frame_id:
                return (s.position.x, s.position.y, s.position.z)
        raise ValueError(f"No state found for body_id={body_id}, frame_id={frame_id}")

    def min_z(self, body_id: int) -> float:
        """Get minimum z-coordinate across all frames.

        Useful for "object never fell through the floor" assertions.

        Args:
            body_id: ID of the body to query

        Returns:
            Minimum z-coordinate value

        Raises:
            ValueError: If no states exist for the given body_id
        """
        positions = self.position_history(body_id)
        if not positions:
            raise ValueError(f"No positions found for body_id={body_id}")
        return min(z for _, _, z in positions)

    def max_z(self, body_id: int) -> float:
        """Get maximum z-coordinate across all frames.

        Args:
            body_id: ID of the body to query

        Returns:
            Maximum z-coordinate value

        Raises:
            ValueError: If no states exist for the given body_id
        """
        positions = self.position_history(body_id)
        if not positions:
            raise ValueError(f"No positions found for body_id={body_id}")
        return max(z for _, _, z in positions)

    def max_speed(self, body_id: int) -> float:
        """Get maximum speed across all frames.

        Args:
            body_id: ID of the body to query

        Returns:
            Maximum speed value

        Raises:
            ValueError: If no states exist for the given body_id
        """
        speeds = self.speed_history(body_id)
        if not speeds:
            raise ValueError(f"No speeds found for body_id={body_id}")
        return max(speeds)

    # System energy

    def system_energy_history(self) -> list[float]:
        """Get system-level total energy timeseries.

        Returns:
            List of total system energy values, one per frame, ordered by frame ID
        """
        system_energies = self._db.select_all_system_energy()
        # Sort by frame_id to ensure correct ordering
        system_energies.sort(key=lambda e: e.frame_id)
        return [e.total_system_e for e in system_energies]

    def max_energy_drift(self) -> float:
        """Compute maximum relative energy drift.

        Calculates max |E(t) - E(0)| / |E(0)| across all frames, where E(0) is
        the initial system energy. This measures the relative energy error over
        the simulation.

        Returns:
            Maximum relative energy drift (dimensionless)

        Notes:
            Returns 0.0 if:
            - No energy records exist
            - Initial energy is near zero (|E(0)| < 1e-12)
        """
        energies = self.system_energy_history()
        if not energies or abs(energies[0]) < 1e-12:
            return 0.0

        e0 = energies[0]
        return max(abs(e - e0) / abs(e0) for e in energies)

    # Contact events

    def total_contact_frames(self) -> int:
        """Count distinct frames with at least one collision record.

        Returns:
            Number of distinct frames containing collision records
        """
        collisions = self._db.select_all_collisions()
        if not collisions:
            return 0
        unique_frames = set(c.frame_id for c in collisions)
        return len(unique_frames)

    def contact_frames_between(self, body_a_id: int, body_b_id: int) -> int:
        """Count frames where a specific body pair has collision records.

        Args:
            body_a_id: ID of the first body
            body_b_id: ID of the second body

        Returns:
            Number of distinct frames with collisions between the two bodies

        Notes:
            Order of body_a_id and body_b_id does not matter - the query
            checks both (A, B) and (B, A) pairs.
        """
        collisions = self._db.select_all_collisions()
        if not collisions:
            return 0

        # Collect frames where either (A, B) or (B, A) appears
        contact_frames = set()
        for c in collisions:
            if (c.body_a_id == body_a_id and c.body_b_id == body_b_id) or (
                c.body_a_id == body_b_id and c.body_b_id == body_a_id
            ):
                contact_frames.add(c.frame_id)

        return len(contact_frames)
