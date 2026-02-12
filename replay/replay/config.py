"""
Configuration for MSD Replay Backend

Ticket: 0056d_fastapi_backend
"""

import os
from pathlib import Path


class Config:
    """Application configuration from environment variables."""

    def __init__(self):
        """Initialize configuration from environment."""
        self.recordings_dir = Path(
            os.getenv("MSD_RECORDINGS_DIR", "./recordings")
        ).resolve()

        # Validate recordings directory
        if not self.recordings_dir.exists():
            raise FileNotFoundError(
                f"Recordings directory does not exist: {self.recordings_dir}"
            )

    def list_databases(self) -> list[Path]:
        """List all .db files in recordings directory."""
        return sorted(self.recordings_dir.glob("*.db"))

    def get_database_path(self, db_id: str) -> Path:
        """Get path to database by ID (filename without extension)."""
        db_path = self.recordings_dir / f"{db_id}.db"
        if not db_path.exists():
            raise FileNotFoundError(f"Database not found: {db_id}")
        return db_path


# Global config instance
config = Config()
