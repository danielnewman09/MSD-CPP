#!/usr/bin/env python3
# Ticket: 0078_cpp_guidelines_mcp_server
# Design: docs/designs/0078_cpp_guidelines_mcp_server/design.md
"""
Guidelines Database Seeder

Reads YAML seed files, validates them with Pydantic models, and populates
guidelines.db. Fully idempotent (DROP + CREATE on each run).

Usage:
    python seed_guidelines.py [--db PATH] [--data-dir PATH]

Example:
    python seed_guidelines.py --db build/Debug/docs/guidelines.db
    python seed_guidelines.py --db guidelines.db --data-dir scripts/guidelines/data
"""

import argparse
import sqlite3
import sys
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ValidationError, field_validator

# Allow importing guidelines_schema when running from any cwd
_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

from guidelines_schema import create_schema, drop_schema  # noqa: E402


# ---------------------------------------------------------------------------
# Pydantic validation models
# ---------------------------------------------------------------------------

class CrossRefModel(BaseModel):
    """A cross-reference from one rule to another."""
    to_rule_id: str
    relationship: Literal["derived_from", "related", "supersedes", "conflicts_with"]


class RuleModel(BaseModel):
    """A single coding guideline rule."""
    rule_id: str
    category: str
    source: Literal["project", "cpp_core_guidelines", "misra"]
    severity: Literal["required", "recommended", "advisory"]
    status: Literal["proposed", "active", "deprecated"] = "active"
    title: str
    rationale: str
    enforcement_notes: str
    # N4: enforcement_check is optional — populated incrementally in 0078b/0078c
    # via clang-tidy / cppcheck check IDs (e.g., "cppcoreguidelines-special-member-functions")
    enforcement_check: str | None = None
    good_example: str | None = None
    bad_example: str | None = None
    tags: list[str] = []
    cross_refs: list[CrossRefModel] = []

    @field_validator("rule_id")
    @classmethod
    def validate_rule_id_format(cls, v: str) -> str:
        """Loosely validate rule_id format."""
        valid_prefixes = ("MSD-", "CPP-", "MISRA-")
        if not any(v.startswith(p) for p in valid_prefixes):
            raise ValueError(
                f"rule_id '{v}' must start with MSD-, CPP-, or MISRA-"
            )
        return v


class SeedFileModel(BaseModel):
    """Top-level structure of a YAML seed file."""
    rules: list[RuleModel]


# ---------------------------------------------------------------------------
# Seed logic
# ---------------------------------------------------------------------------

def load_yaml_file(path: Path) -> SeedFileModel:
    """Load and validate a single YAML seed file."""
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    if raw is None or not isinstance(raw, dict):
        raise ValueError(f"Invalid YAML structure in {path}: expected a mapping")

    # Normalize missing 'rules' key to empty list (valid for stub files)
    if "rules" not in raw:
        raw["rules"] = []

    try:
        return SeedFileModel.model_validate(raw)
    except ValidationError as exc:
        raise ValueError(f"Validation error in {path}:\n{exc}") from exc


def collect_all_rules(data_dir: Path) -> list[RuleModel]:
    """Load and validate all YAML files in data_dir. Aborts on any error."""
    yaml_files = sorted(data_dir.glob("*.yaml"))
    if not yaml_files:
        raise FileNotFoundError(f"No YAML seed files found in {data_dir}")

    all_rules: list[RuleModel] = []
    errors: list[str] = []

    for yaml_path in yaml_files:
        try:
            seed = load_yaml_file(yaml_path)
            all_rules.extend(seed.rules)
            print(f"  Loaded {len(seed.rules):3d} rule(s) from {yaml_path.name}")
        except (ValueError, OSError) as exc:
            errors.append(str(exc))

    if errors:
        print("\nValidation errors (aborting before any writes):", file=sys.stderr)
        for err in errors:
            print(f"  {err}", file=sys.stderr)
        sys.exit(1)

    # Check for duplicate rule_ids across files
    seen_ids: dict[str, str] = {}
    duplicates: list[str] = []
    for rule in all_rules:
        if rule.rule_id in seen_ids:
            duplicates.append(
                f"  '{rule.rule_id}' appears in multiple files"
            )
        else:
            seen_ids[rule.rule_id] = rule.category

    if duplicates:
        print("\nDuplicate rule_ids found (aborting):", file=sys.stderr)
        for dup in duplicates:
            print(dup, file=sys.stderr)
        sys.exit(1)

    return all_rules


def insert_rules(conn: sqlite3.Connection, rules: list[RuleModel]) -> None:
    """Insert all rules (and related categories/tags/cross_refs) atomically."""
    conn.execute("PRAGMA foreign_keys = ON")

    # Collect and insert categories first
    categories: dict[str, int] = {}  # name → id
    for rule in rules:
        if rule.category not in categories:
            cursor = conn.execute(
                "INSERT INTO categories(name) VALUES (?)", (rule.category,)
            )
            categories[rule.category] = cursor.lastrowid  # type: ignore[assignment]

    # Collect and insert tags
    tags: dict[str, int] = {}  # name → id
    for rule in rules:
        for tag in rule.tags:
            if tag not in tags:
                cursor = conn.execute(
                    "INSERT INTO tags(name) VALUES (?)", (tag,)
                )
                tags[tag] = cursor.lastrowid  # type: ignore[assignment]

    # Insert rules
    for rule in rules:
        conn.execute(
            """
            INSERT INTO rules(
                rule_id, category_id, source, severity, status,
                title, rationale, enforcement_notes, enforcement_check,
                good_example, bad_example
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                rule.rule_id,
                categories[rule.category],
                rule.source,
                rule.severity,
                rule.status,
                rule.title,
                rule.rationale,
                rule.enforcement_notes,
                rule.enforcement_check,
                rule.good_example,
                rule.bad_example,
            ),
        )

    # Insert rule_tags
    for rule in rules:
        for tag in rule.tags:
            conn.execute(
                "INSERT INTO rule_tags(rule_id, tag_id) VALUES (?, ?)",
                (rule.rule_id, tags[tag]),
            )

    # Insert cross_refs (after all rules are inserted)
    for rule in rules:
        for xref in rule.cross_refs:
            conn.execute(
                """
                INSERT INTO rule_cross_refs(from_rule_id, to_rule_id, relationship)
                VALUES (?, ?, ?)
                """,
                (rule.rule_id, xref.to_rule_id, xref.relationship),
            )

    conn.commit()


def seed_database(db_path: Path, data_dir: Path) -> None:
    """Main entry point: load YAML, validate, and populate the database."""
    print(f"Seeding guidelines database: {db_path}")
    print(f"Data directory: {data_dir}")
    print()

    # Validate all YAML before touching the database (all-or-nothing)
    print("Loading and validating YAML seed files...")
    rules = collect_all_rules(data_dir)
    print(f"\nTotal: {len(rules)} rule(s) validated — proceeding to seed")

    # Ensure output directory exists
    db_path.parent.mkdir(parents=True, exist_ok=True)

    conn = sqlite3.connect(str(db_path))
    try:
        # Idempotent: drop and recreate schema
        print("\nDropping existing schema (idempotent reseed)...")
        drop_schema(conn)
        print("Creating schema...")
        create_schema(conn)
        print(f"Inserting {len(rules)} rule(s)...")
        insert_rules(conn, rules)
        print(f"\nDone. Database written to: {db_path}")
    except Exception:
        conn.close()
        raise
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Seed the C++ guidelines SQLite database from YAML files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Default paths (from project root)
    python scripts/guidelines/seed_guidelines.py

    # Custom paths
    python scripts/guidelines/seed_guidelines.py \\
        --db build/Debug/docs/guidelines.db \\
        --data-dir scripts/guidelines/data
        """,
    )
    parser.add_argument(
        "--db",
        default="build/Debug/docs/guidelines.db",
        help="Path to the output SQLite database (default: build/Debug/docs/guidelines.db)",
    )
    parser.add_argument(
        "--data-dir",
        default=str(_SCRIPT_DIR / "data"),
        help="Directory containing YAML seed files (default: scripts/guidelines/data)",
    )

    args = parser.parse_args()
    seed_database(Path(args.db), Path(args.data_dir))


if __name__ == "__main__":
    main()
