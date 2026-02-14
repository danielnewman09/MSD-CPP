#!/usr/bin/env python3
"""
Record Layer Mapping Indexer (Composite Pydantic Models)

Indexes hand-written composite Pydantic models (BodyState, FrameData, etc.)
into the traceability database. These models aggregate generated leaf models
and cannot be auto-generated.

For the four generator-managed layers (cpp, sql, pybind, leaf-pydantic),
use: python scripts/generate_record_layers.py --update-traceability <db_path>

Ticket: 0061_cross_layer_record_mapping

Usage:
    python index_record_mappings.py <traceability_db_path> [--repo <repo_root>]
"""

import argparse
import ast
import re
import sys
from pathlib import Path
from typing import Any

from traceability_schema import create_schema, rebuild_fts


# ===== Pydantic Parsing =====


def parse_pydantic_models(models_content: str) -> dict[str, dict[str, Any]]:
    """Extract field definitions from hand-written Pydantic BaseModel classes.

    Only returns models that are NOT auto-generated (no "Maps-to:" in docstring).

    Args:
        models_content: Full text of models.py.

    Returns:
        Dict mapping Pydantic class name to:
        - fields: List of dicts with field_name and field_type
        - cpp_record: Inferred C++ record name (or None)
    """
    tree = ast.parse(models_content)
    models = {}

    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue

        # Check if inherits from BaseModel
        is_base_model = any(
            isinstance(base, ast.Name) and base.id == "BaseModel"
            for base in node.bases
        )

        if not is_base_model:
            continue

        # Skip generated leaf models (they have "Maps-to:" in docstring)
        docstring = ast.get_docstring(node)
        if docstring and "Maps-to:" in docstring:
            continue

        class_name = node.name
        fields = []

        # Extract fields from class body (ast.AnnAssign nodes)
        for item in node.body:
            if isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
                field_name = item.target.id
                field_type = ast.unparse(item.annotation) if item.annotation else "Any"
                fields.append({"field_name": field_name, "field_type": field_type})

        # Infer C++ record name from docstring
        cpp_record = None
        if docstring:
            doc_match = re.search(r"From\s+(\w+Record)", docstring)
            if doc_match:
                cpp_record = doc_match.group(1)

        models[class_name] = {"fields": fields, "cpp_record": cpp_record}

    return models


# ===== Database Population =====


def populate_composite_models(
    conn,
    pydantic_models: dict[str, dict[str, Any]],
) -> None:
    """Populate composite Pydantic model fields into record_layer_fields.

    Only touches composite models. Does NOT clear generator-managed layers.
    Generator handles: cpp, sql, pybind, leaf-pydantic.

    Args:
        conn: SQLite connection.
        pydantic_models: Dict[class_name, {fields, cpp_record}] from Pydantic.
    """
    # Clear only composite pydantic entries (notes = 'composite')
    conn.execute(
        "DELETE FROM record_layer_fields WHERE layer = 'pydantic' AND notes = 'composite'"
    )

    for class_name, model_info in pydantic_models.items():
        fields = model_info["fields"]
        cpp_record = model_info["cpp_record"]

        # Insert composite Pydantic fields
        for field_info in fields:
            field_name = field_info["field_name"]
            field_type = field_info["field_type"]

            conn.execute(
                """
                INSERT INTO record_layer_fields
                (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, 'pydantic', ?, ?, ?, 'composite')
                """,
                (
                    cpp_record or class_name,  # Use C++ record name if known, else class name
                    field_name,
                    field_type,
                    None,
                ),
            )

    conn.commit()


# ===== Main =====


def main() -> int:
    """Index composite Pydantic models into the traceability database."""
    parser = argparse.ArgumentParser(
        description="Index hand-written composite Pydantic models into traceability database"
    )
    parser.add_argument("db_path", help="Path to traceability.db")
    parser.add_argument(
        "--repo", default=".", help="Path to repository root (default: current dir)"
    )
    args = parser.parse_args()

    repo_root = Path(args.repo).resolve()
    db_path = Path(args.db_path).resolve()

    # Ensure schema exists
    conn = create_schema(db_path)

    print(f"Indexing composite Pydantic models from {repo_root}...")

    # === Parse hand-written Pydantic models ===
    pydantic_models = {}
    models_path = repo_root / "replay" / "replay" / "models.py"
    if models_path.exists():
        models_content = models_path.read_text()
        pydantic_models = parse_pydantic_models(models_content)
        print(f"  Pydantic composites: {len(pydantic_models)} models parsed")
    else:
        print(f"WARNING: Pydantic models not found: {models_path}", file=sys.stderr)

    # Also check generated_models.py for any that might be imported
    gen_models_path = repo_root / "replay" / "replay" / "generated_models.py"
    if gen_models_path.exists():
        gen_content = gen_models_path.read_text()
        gen_models = {}
        tree = ast.parse(gen_content)
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                gen_models[node.name] = True
        print(f"  Generated leaf models: {len(gen_models)} (handled by generator)")

    # === Populate database ===
    populate_composite_models(conn, pydantic_models)

    # === Rebuild FTS index ===
    rebuild_fts(conn)

    print(f"✓ Indexed {len(pydantic_models)} composite Pydantic models")
    print(f"✓ Database updated: {db_path}")

    conn.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
