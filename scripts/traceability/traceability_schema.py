#!/usr/bin/env python3
"""
Traceability Database Schema

Shared module that creates and manages the traceability database schema.
This database accumulates data over time (unlike codebase.db which is rebuilt).

Tables:
    snapshots          - One row per git commit
    file_changes       - Per-commit file diff summary
    symbol_snapshots   - Symbol locations at a commit
    symbol_changes     - Computed diffs between consecutive snapshots
    design_decisions   - Extracted design decisions from tickets/docs
    decision_symbols   - Links decisions to affected symbols
    decision_commits   - Links decisions to implementing commits
    design_decisions_fts - FTS5 index over decision text
"""

import sqlite3
from pathlib import Path

SCHEMA_VERSION = 2


def create_schema(db_path: str | Path) -> sqlite3.Connection:
    """Create the traceability database schema.

    Creates tables if they don't exist (safe to call repeatedly).

    Args:
        db_path: Path to the SQLite database file.

    Returns:
        Open connection with row_factory set to sqlite3.Row.
    """
    db_path = Path(db_path)
    db_path.parent.mkdir(parents=True, exist_ok=True)

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")

    conn.executescript("""
        -- Schema version tracking
        CREATE TABLE IF NOT EXISTS schema_info (
            key   TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );

        -- Git commit snapshots
        CREATE TABLE IF NOT EXISTS snapshots (
            id             INTEGER PRIMARY KEY AUTOINCREMENT,
            sha            TEXT    NOT NULL UNIQUE,
            date           TEXT    NOT NULL,
            author         TEXT    NOT NULL,
            message        TEXT    NOT NULL,
            prefix         TEXT,              -- e.g. 'impl', 'design', 'fix'
            ticket_number  TEXT,              -- e.g. '0045', '0039a'
            phase          TEXT               -- derived: 'design', 'review', 'impl', 'docs', etc.
        );
        CREATE INDEX IF NOT EXISTS idx_snapshots_ticket ON snapshots(ticket_number);
        CREATE INDEX IF NOT EXISTS idx_snapshots_date   ON snapshots(date);

        -- Per-commit file changes
        CREATE TABLE IF NOT EXISTS file_changes (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            snapshot_id INTEGER NOT NULL REFERENCES snapshots(id),
            file_path   TEXT    NOT NULL,
            change_type TEXT    NOT NULL,  -- 'A' added, 'M' modified, 'D' deleted, 'R' renamed
            insertions  INTEGER NOT NULL DEFAULT 0,
            deletions   INTEGER NOT NULL DEFAULT 0,
            old_path    TEXT               -- set for renames
        );
        CREATE INDEX IF NOT EXISTS idx_file_changes_snapshot ON file_changes(snapshot_id);
        CREATE INDEX IF NOT EXISTS idx_file_changes_path     ON file_changes(file_path);

        -- Symbol locations at a given commit
        CREATE TABLE IF NOT EXISTS symbol_snapshots (
            id             INTEGER PRIMARY KEY AUTOINCREMENT,
            snapshot_id    INTEGER NOT NULL REFERENCES snapshots(id),
            qualified_name TEXT    NOT NULL,
            kind           TEXT    NOT NULL,  -- 'class', 'struct', 'function', 'method'
            file_path      TEXT    NOT NULL,
            line_number    INTEGER NOT NULL,
            signature      TEXT,              -- for functions/methods
            class_scope    TEXT               -- owning class for methods
        );
        CREATE INDEX IF NOT EXISTS idx_symbol_snap_snapshot ON symbol_snapshots(snapshot_id);
        CREATE INDEX IF NOT EXISTS idx_symbol_snap_name     ON symbol_snapshots(qualified_name);

        -- Computed diffs between consecutive snapshots
        CREATE TABLE IF NOT EXISTS symbol_changes (
            id             INTEGER PRIMARY KEY AUTOINCREMENT,
            snapshot_id    INTEGER NOT NULL REFERENCES snapshots(id),
            qualified_name TEXT    NOT NULL,
            change_type    TEXT    NOT NULL,  -- 'added', 'removed', 'modified', 'moved'
            old_file_path  TEXT,
            new_file_path  TEXT,
            old_line       INTEGER,
            new_line       INTEGER,
            old_signature  TEXT,
            new_signature  TEXT
        );
        CREATE INDEX IF NOT EXISTS idx_symbol_changes_snapshot ON symbol_changes(snapshot_id);
        CREATE INDEX IF NOT EXISTS idx_symbol_changes_name     ON symbol_changes(qualified_name);

        -- Design decisions extracted from tickets and design docs
        CREATE TABLE IF NOT EXISTS design_decisions (
            id                INTEGER PRIMARY KEY AUTOINCREMENT,
            dd_id             TEXT    NOT NULL UNIQUE,  -- e.g. 'DD-0045-001' or 'DD-0039-H1'
            ticket            TEXT    NOT NULL,          -- ticket number e.g. '0045'
            title             TEXT    NOT NULL,
            rationale         TEXT,
            alternatives      TEXT,
            trade_offs        TEXT,
            status            TEXT    NOT NULL DEFAULT 'active',  -- 'active', 'superseded', 'deprecated'
            extraction_method TEXT    NOT NULL DEFAULT 'structured',  -- 'structured' or 'heuristic'
            source_file       TEXT    NOT NULL,           -- path to source document
            source_line       INTEGER                     -- approximate line in source
        );
        CREATE INDEX IF NOT EXISTS idx_dd_ticket ON design_decisions(ticket);

        -- Links decisions to affected symbols
        CREATE TABLE IF NOT EXISTS decision_symbols (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            decision_id INTEGER NOT NULL REFERENCES design_decisions(id),
            symbol_name TEXT    NOT NULL   -- qualified name of affected symbol
        );
        CREATE INDEX IF NOT EXISTS idx_ds_decision ON decision_symbols(decision_id);
        CREATE INDEX IF NOT EXISTS idx_ds_symbol   ON decision_symbols(symbol_name);

        -- Links decisions to implementing commits
        CREATE TABLE IF NOT EXISTS decision_commits (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            decision_id INTEGER NOT NULL REFERENCES design_decisions(id),
            snapshot_id INTEGER NOT NULL REFERENCES snapshots(id)
        );
        CREATE INDEX IF NOT EXISTS idx_dc_decision ON decision_commits(decision_id);
        CREATE INDEX IF NOT EXISTS idx_dc_snapshot ON decision_commits(snapshot_id);

        -- Record layer field mappings (Ticket: 0061_cross_layer_record_mapping)
        CREATE TABLE IF NOT EXISTS record_layer_fields (
            id             INTEGER PRIMARY KEY AUTOINCREMENT,
            record_name    TEXT NOT NULL,       -- e.g., "EnergyRecord"
            layer          TEXT NOT NULL,       -- "cpp", "sql", "pybind", "pydantic"
            field_name     TEXT NOT NULL,       -- field name in that layer
            field_type     TEXT,                -- type info where available
            source_field   TEXT,                -- corresponding C++ field (NULL for cpp layer)
            notes          TEXT                 -- e.g., "ForeignKey → _id suffix"
        );
        CREATE INDEX IF NOT EXISTS idx_rlf_record ON record_layer_fields(record_name);
        CREATE INDEX IF NOT EXISTS idx_rlf_layer ON record_layer_fields(layer);

        -- Record layer mapping (cross-layer associations)
        CREATE TABLE IF NOT EXISTS record_layer_mapping (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            record_name     TEXT NOT NULL UNIQUE, -- C++ record name
            pydantic_model  TEXT,                 -- corresponding Pydantic class (may be NULL)
            pybind_class    TEXT,                 -- corresponding pybind class (may be NULL)
            sql_table       TEXT                  -- corresponding SQL table name
        );
        CREATE INDEX IF NOT EXISTS idx_rlm_pydantic ON record_layer_mapping(pydantic_model);
    """)

    # FTS5 virtual tables (CREATE VIRTUAL TABLE IF NOT EXISTS is supported)
    conn.execute("""
        CREATE VIRTUAL TABLE IF NOT EXISTS design_decisions_fts USING fts5(
            dd_id,
            title,
            rationale,
            alternatives,
            trade_offs,
            content=design_decisions,
            content_rowid=id,
            tokenize='porter unicode61'
        )
    """)

    conn.execute("""
        CREATE VIRTUAL TABLE IF NOT EXISTS record_layer_fields_fts USING fts5(
            field_name,
            field_type,
            notes,
            content=record_layer_fields,
            content_rowid=id,
            tokenize='porter unicode61'
        )
    """)

    # Set schema version
    conn.execute(
        "INSERT OR REPLACE INTO schema_info (key, value) VALUES ('version', ?)",
        (str(SCHEMA_VERSION),)
    )
    conn.commit()

    return conn


def rebuild_fts(conn: sqlite3.Connection) -> None:
    """Rebuild the FTS5 index from the design_decisions table."""
    conn.execute("INSERT INTO design_decisions_fts(design_decisions_fts) VALUES('rebuild')")
    conn.commit()


def get_schema_version(conn: sqlite3.Connection) -> int:
    """Get the current schema version."""
    try:
        row = conn.execute(
            "SELECT value FROM schema_info WHERE key = 'version'"
        ).fetchone()
        return int(row["value"]) if row else 0
    except sqlite3.OperationalError:
        return 0
