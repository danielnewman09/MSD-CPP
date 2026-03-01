"""
Configuration for MSD Replay Backend

Ticket: 0056d_fastapi_backend
Ticket: 0056e_threejs_core_visualization (R0c)
"""

import os
from pathlib import Path

# Separator used in database IDs to represent directory boundaries.
# Forward slashes can't be used in URL path segments, so subdirectory
# structure is encoded as "--" in the ID (e.g. "TiltedDrop--X_5deg").
ID_PATH_SEPARATOR = "--"


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
        """List recording .db files in recordings directory and subdirectories (excludes asset DB)."""
        return sorted(
            db for db in self.recordings_dir.rglob("*.db")
            if db.resolve() != self.assets_db_path
        )

    def make_database_id(self, db_path: Path) -> str:
        """Create a URL-safe database ID from a filesystem path.

        Converts the relative path (without .db extension) to an ID where
        directory separators are encoded as '--'.
        """
        relative = db_path.relative_to(self.recordings_dir).with_suffix("")
        return str(relative).replace(os.sep, ID_PATH_SEPARATOR)

    def make_database_name(self, db_path: Path) -> str:
        """Create a human-readable name from a filesystem path.

        Returns the relative path without .db extension, using '/' separators.
        For top-level files this is just the stem; for subdirectories it
        includes the folder path (e.g. "TiltedDrop/X_5deg").
        """
        relative = db_path.relative_to(self.recordings_dir).with_suffix("")
        return str(relative)

    def get_database_path(self, db_id: str) -> Path:
        """Get path to database by ID.

        The db_id uses '--' to encode directory separators, e.g.
        "TiltedDrop--TiltedDropSingleAxisTest_BounceIsolation--X_5deg".
        """
        relative = db_id.replace(ID_PATH_SEPARATOR, os.sep)
        db_path = (self.recordings_dir / f"{relative}.db").resolve()
        # Ensure the resolved path is still within recordings_dir
        if not str(db_path).startswith(str(self.recordings_dir)):
            raise FileNotFoundError(f"Database not found: {db_id}")
        if not db_path.exists():
            raise FileNotFoundError(f"Database not found: {db_id}")
        return db_path


# Global config instance
config = Config()
