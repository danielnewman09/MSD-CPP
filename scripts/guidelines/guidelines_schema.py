#!/usr/bin/env python3
# Ticket: 0078_cpp_guidelines_mcp_server
# Design: docs/designs/0078_cpp_guidelines_mcp_server/design.md
"""
Guidelines Database Schema

Defines the SQLite schema for the C++ coding guidelines database as Python
constants (CREATE TABLE DDL + FTS5 virtual table). Imported by both the seed
script and the MCP server so the schema is the single source of truth.
"""

import sqlite3

# ---------------------------------------------------------------------------
# DDL statements — ordered by dependency (no FK violations on fresh create)
# ---------------------------------------------------------------------------

_CREATE_CATEGORIES = """
CREATE TABLE IF NOT EXISTS categories (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT NOT NULL UNIQUE,
    description TEXT
)
"""

_CREATE_RULES = """
CREATE TABLE IF NOT EXISTS rules (
    rule_id            TEXT PRIMARY KEY,
    category_id        INTEGER NOT NULL REFERENCES categories(id),
    source             TEXT NOT NULL CHECK(source IN ('project', 'cpp_core_guidelines', 'misra', 'clang_tidy')),
    severity           TEXT NOT NULL CHECK(severity IN ('required', 'recommended', 'advisory')),
    status             TEXT NOT NULL DEFAULT 'active'
                           CHECK(status IN ('proposed', 'active', 'deprecated')),
    title              TEXT NOT NULL,
    rationale          TEXT NOT NULL,
    enforcement_notes  TEXT NOT NULL,
    enforcement_check  TEXT,
    good_example       TEXT,
    bad_example        TEXT
)
"""

_CREATE_RULE_CROSS_REFS = """
CREATE TABLE IF NOT EXISTS rule_cross_refs (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    from_rule_id    TEXT NOT NULL REFERENCES rules(rule_id),
    to_rule_id      TEXT NOT NULL REFERENCES rules(rule_id),
    relationship    TEXT NOT NULL CHECK(relationship IN (
                        'derived_from', 'related', 'supersedes', 'conflicts_with'
                    ))
)
"""

_CREATE_TAGS = """
CREATE TABLE IF NOT EXISTS tags (
    id   INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE
)
"""

_CREATE_RULE_TAGS = """
CREATE TABLE IF NOT EXISTS rule_tags (
    rule_id TEXT NOT NULL REFERENCES rules(rule_id),
    tag_id  INTEGER NOT NULL REFERENCES tags(id),
    PRIMARY KEY (rule_id, tag_id)
)
"""

# FTS5 virtual table — porter tokenizer for stemming (e.g., "initialize" matches
# "initialization"). BM25 ranking used via bm25(rules_fts) in queries.
# DD-0078-005: Default get_category to summary mode to avoid context window overflow.
_CREATE_RULES_FTS = """
CREATE VIRTUAL TABLE IF NOT EXISTS rules_fts USING fts5(
    rule_id,
    title,
    rationale,
    enforcement_notes,
    good_example,
    bad_example,
    content='rules',
    content_rowid='rowid',
    tokenize='porter ascii'
)
"""

# Triggers to keep rules_fts in sync with the rules table
_CREATE_FTS_INSERT_TRIGGER = """
CREATE TRIGGER IF NOT EXISTS rules_fts_insert AFTER INSERT ON rules BEGIN
    INSERT INTO rules_fts(rowid, rule_id, title, rationale, enforcement_notes,
                           good_example, bad_example)
    VALUES (new.rowid, new.rule_id, new.title, new.rationale,
            new.enforcement_notes, new.good_example, new.bad_example);
END
"""

_CREATE_FTS_DELETE_TRIGGER = """
CREATE TRIGGER IF NOT EXISTS rules_fts_delete AFTER DELETE ON rules BEGIN
    INSERT INTO rules_fts(rules_fts, rowid, rule_id, title, rationale,
                           enforcement_notes, good_example, bad_example)
    VALUES ('delete', old.rowid, old.rule_id, old.title, old.rationale,
            old.enforcement_notes, old.good_example, old.bad_example);
END
"""

_CREATE_FTS_UPDATE_TRIGGER = """
CREATE TRIGGER IF NOT EXISTS rules_fts_update AFTER UPDATE ON rules BEGIN
    INSERT INTO rules_fts(rules_fts, rowid, rule_id, title, rationale,
                           enforcement_notes, good_example, bad_example)
    VALUES ('delete', old.rowid, old.rule_id, old.title, old.rationale,
            old.enforcement_notes, old.good_example, old.bad_example);
    INSERT INTO rules_fts(rowid, rule_id, title, rationale, enforcement_notes,
                           good_example, bad_example)
    VALUES (new.rowid, new.rule_id, new.title, new.rationale,
            new.enforcement_notes, new.good_example, new.bad_example);
END
"""

# All DDL statements in dependency order
SCHEMA_STATEMENTS: list[str] = [
    _CREATE_CATEGORIES,
    _CREATE_RULES,
    _CREATE_RULE_CROSS_REFS,
    _CREATE_TAGS,
    _CREATE_RULE_TAGS,
    _CREATE_RULES_FTS,
    _CREATE_FTS_INSERT_TRIGGER,
    _CREATE_FTS_DELETE_TRIGGER,
    _CREATE_FTS_UPDATE_TRIGGER,
]

# Drop statements in reverse dependency order (for idempotent reseed)
DROP_STATEMENTS: list[str] = [
    "DROP TRIGGER IF EXISTS rules_fts_update",
    "DROP TRIGGER IF EXISTS rules_fts_delete",
    "DROP TRIGGER IF EXISTS rules_fts_insert",
    "DROP TABLE IF EXISTS rules_fts",
    "DROP TABLE IF EXISTS rule_tags",
    "DROP TABLE IF EXISTS tags",
    "DROP TABLE IF EXISTS rule_cross_refs",
    "DROP TABLE IF EXISTS rules",
    "DROP TABLE IF EXISTS categories",
]


def create_schema(conn: sqlite3.Connection) -> None:
    """Apply all DDL statements to the given connection."""
    conn.execute("PRAGMA foreign_keys = ON")
    for stmt in SCHEMA_STATEMENTS:
        conn.execute(stmt)
    conn.commit()


def drop_schema(conn: sqlite3.Connection) -> None:
    """Drop all tables (for idempotent reseed)."""
    for stmt in DROP_STATEMENTS:
        conn.execute(stmt)
    conn.commit()
