"""
Configuration for MSD Replay Backend

Ticket: 0056d_fastapi_backend
Ticket: 0056e_threejs_core_visualization (R0c)
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

        # Asset database path (for geometry lookup)
        # Ticket: 0056e - Two-database architecture
        self.assets_db_path = Path(
            os.getenv("MSD_ASSETS_DB", "./recordings/assets.db")
        ).resolve()

        # Validate recordings directory
        if not self.recordings_dir.exists():
            raise FileNotFoundError(
                f"Recordings directory does not exist: {self.recordings_dir}"
            )

        # Validate assets database
        if not self.assets_db_path.exists():
            raise FileNotFoundError(
                f"Assets database does not exist: {self.assets_db_path}"
            )

    def list_databases(self) -> list[Path]:
        """List recording .db files in recordings directory (excludes asset DB)."""
        return sorted(
            db for db in self.recordings_dir.glob("*.db")
            if db.resolve() != self.assets_db_path
        )

    def get_database_path(self, db_id: str) -> Path:
        """Get path to database by ID (filename without extension)."""
        db_path = self.recordings_dir / f"{db_id}.db"
        if not db_path.exists():
            raise FileNotFoundError(f"Database not found: {db_id}")
        return db_path


# Global config instance
config = Config()
