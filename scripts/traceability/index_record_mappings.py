#!/usr/bin/env python3
"""
Record Layer Mapping Indexer

Parses C++ transfer records (BOOST_DESCRIBE_STRUCT), pybind11 bindings, and
Pydantic models to extract field lists and create cross-layer mappings. Stores
the mappings in the traceability database for drift detection and visibility.

Ticket: 0061_cross_layer_record_mapping
Design: docs/designs/0061_cross_layer_record_mapping/design.md

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


# ===== BOOST_DESCRIBE Parsing =====

# BOOST_DESCRIBE_STRUCT(RecordName, (BaseClass), (field1, field2, field3));
BOOST_DESCRIBE_RE = re.compile(
    r"BOOST_DESCRIBE_STRUCT\s*\(\s*(\w+)\s*,\s*\([^)]*\)\s*,\s*\(([^)]+)\)\s*\)",
    re.MULTILINE | re.DOTALL,
)

# Field declaration patterns
FOREIGN_KEY_RE = re.compile(r"ForeignKey<(\w+)>")
REPEATED_FIELD_RE = re.compile(r"RepeatedField<(\w+)>")


def parse_boost_describe_fields(
    header_content: str, header_path: str
) -> list[dict[str, Any]]:
    """Parse BOOST_DESCRIBE_STRUCT macro to extract field information.

    Args:
        header_content: Full text of a .hpp file.
        header_path: Relative path to the header (for error reporting).

    Returns:
        List of dicts with:
        - field_name: Field identifier
        - field_type: C++ type (or 'unknown' if not parseable)
        - is_foreign_key: True if ForeignKey<T>
        - is_repeated: True if RepeatedField<T>
        - target_type: Target type for FK/Repeated (else None)
    """
    fields = []
    match = BOOST_DESCRIBE_RE.search(header_content)

    if not match:
        return fields  # No BOOST_DESCRIBE_STRUCT in this file

    field_list_str = match.group(2)
    field_names = [f.strip() for f in field_list_str.split(",")]

    # For each field, try to determine its type by scanning backward
    # from the macro to find field declarations
    lines = header_content[: match.start()].split("\n")

    for field_name in field_names:
        if not field_name:
            continue

        field_info = {
            "field_name": field_name,
            "field_type": "unknown",
            "is_foreign_key": False,
            "is_repeated": False,
            "target_type": None,
        }

        # Scan backward through lines to find declaration
        for line in reversed(lines):
            # Match: {type} {field_name}{...}
            decl_pattern = re.compile(rf"\b(\S+(?:<\w+>)?)\s+{re.escape(field_name)}\b")
            decl_match = decl_pattern.search(line)

            if decl_match:
                raw_type = decl_match.group(1)
                field_info["field_type"] = raw_type

                # Check for ForeignKey
                fk_match = FOREIGN_KEY_RE.search(raw_type)
                if fk_match:
                    field_info["is_foreign_key"] = True
                    field_info["target_type"] = fk_match.group(1)

                # Check for RepeatedField
                rf_match = REPEATED_FIELD_RE.search(raw_type)
                if rf_match:
                    field_info["is_repeated"] = True
                    field_info["target_type"] = rf_match.group(1)

                break

        fields.append(field_info)

    return fields


# ===== pybind11 Parsing =====

# py::class_<msd_transfer::RecordName>(m, "RecordName")
PYBIND_CLASS_RE = re.compile(
    r'py::class_<msd_transfer::(\w+)>\(m,\s*"(\w+)"\)(.*?)(?=py::class_|void\s+bind_|$)',
    re.DOTALL,
)

# .def_readonly("field", &msd_transfer::RecordType::field)
DEF_READONLY_RE = re.compile(
    r'\.def_readonly\(\s*"(\w+)",\s*&[\w:]+::(\w+)\)', re.DOTALL
)

# .def_property_readonly("field_id", [](const Record& r) { return r.field.id; })
DEF_PROPERTY_READONLY_RE = re.compile(
    r'\.def_property_readonly\(\s*"(\w+)"', re.DOTALL
)


def parse_pybind_bindings(bindings_content: str) -> dict[str, list[dict[str, Any]]]:
    """Extract field bindings from record_bindings.cpp.

    Args:
        bindings_content: Full text of record_bindings.cpp.

    Returns:
        Dict mapping C++ record name to list of exposed fields:
        - field_name: Python attribute name
        - source_field: C++ field name (may differ for ForeignKey transformations)
        - is_property: True if def_property_readonly, False if def_readonly
    """
    bindings = {}

    for class_match in PYBIND_CLASS_RE.finditer(bindings_content):
        cpp_record_name = class_match.group(1)
        pybind_class_name = class_match.group(2)
        class_body = class_match.group(3)

        fields = []

        # Extract .def_readonly
        for def_match in DEF_READONLY_RE.finditer(class_body):
            python_name = def_match.group(1)
            cpp_field = def_match.group(2)
            fields.append(
                {
                    "field_name": python_name,
                    "source_field": cpp_field,
                    "is_property": False,
                }
            )

        # Extract .def_property_readonly (e.g., FK lambda: field.id -> field_id)
        for prop_match in DEF_PROPERTY_READONLY_RE.finditer(class_body):
            python_name = prop_match.group(1)
            # Infer C++ source from pattern: {field}_id likely comes from FK field
            if python_name.endswith("_id"):
                source_field = python_name[:-3] + ".id"
            else:
                source_field = python_name  # Fallback

            fields.append(
                {
                    "field_name": python_name,
                    "source_field": source_field,
                    "is_property": True,
                }
            )

        bindings[cpp_record_name] = fields

    return bindings


# ===== Pydantic Parsing =====


def camel_to_snake(name: str) -> str:
    """Convert camelCase to snake_case, handling single-letter suffixes.

    Examples:
        penetrationDepth -> penetration_depth
        pointA -> point_a
        pointB -> point_b
    """
    # Insert underscore before uppercase letters (except at start)
    s1 = re.sub(r"(?<!^)(?=[A-Z])", "_", name)
    return s1.lower()


def parse_pydantic_models(models_content: str) -> dict[str, dict[str, Any]]:
    """Extract field definitions from Pydantic BaseModel classes.

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

        class_name = node.name
        fields = []

        # Extract fields from class body (ast.AnnAssign nodes)
        for item in node.body:
            if isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
                field_name = item.target.id
                field_type = ast.unparse(item.annotation) if item.annotation else "Any"
                fields.append({"field_name": field_name, "field_type": field_type})

        # Infer C++ record name from docstring or class name
        cpp_record = None
        docstring = ast.get_docstring(node)
        if docstring:
            # Look for "From {RecordName}" pattern
            doc_match = re.search(r"From\s+(\w+Record)", docstring)
            if doc_match:
                cpp_record = doc_match.group(1)

        # Fallback: if class name ends with expected suffix, infer from class name
        # (e.g., EnergyPoint might map to EnergyRecord, but this is unreliable)
        # Design review noted this is unreliable — accept NULL mappings

        models[class_name] = {"fields": fields, "cpp_record": cpp_record}

    return models


# ===== Database Population =====


def infer_sql_type(cpp_type: str) -> str:
    """Infer SQL type from C++ type (deterministic mapping per design)."""
    if "double" in cpp_type or "float" in cpp_type:
        return "REAL"
    if "int" in cpp_type or "uint" in cpp_type:
        return "INTEGER"
    if "string" in cpp_type or "std::string" in cpp_type:
        return "TEXT"
    if "ForeignKey" in cpp_type:
        return "INTEGER"  # FK fields become INTEGER columns
    if "RepeatedField" in cpp_type:
        return "junction table"
    return "BLOB"  # Default for unknown types


def populate_record_mappings(
    conn,
    cpp_records: dict[str, list[dict[str, Any]]],
    pybind_bindings: dict[str, list[dict[str, Any]]],
    pydantic_models: dict[str, dict[str, Any]],
) -> None:
    """Populate record_layer_fields and record_layer_mapping tables.

    Args:
        conn: SQLite connection.
        cpp_records: Dict[record_name, field_list] from C++ headers.
        pybind_bindings: Dict[record_name, field_list] from pybind.
        pydantic_models: Dict[class_name, {fields, cpp_record}] from Pydantic.
    """
    # Clear existing data
    conn.execute("DELETE FROM record_layer_fields")
    conn.execute("DELETE FROM record_layer_mapping")

    # Populate C++ layer and build mapping table
    for record_name, fields in cpp_records.items():
        # Insert into record_layer_mapping
        conn.execute(
            """
            INSERT INTO record_layer_mapping (record_name, sql_table, pybind_class)
            VALUES (?, ?, ?)
            """,
            (record_name, record_name, record_name),  # SQL table = C++ record name
        )

        # Insert C++ fields
        for field_info in fields:
            field_name = field_info["field_name"]
            field_type = field_info["field_type"]
            is_fk = field_info["is_foreign_key"]
            is_repeated = field_info["is_repeated"]

            conn.execute(
                """
                INSERT INTO record_layer_fields
                (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, ?, ?, ?, ?, ?)
                """,
                (record_name, "cpp", field_name, field_type, None, None),
            )

            # Generate SQL layer entry
            if is_fk:
                sql_field_name = f"{field_name}_id"
                sql_type = "INTEGER"
                notes = "ForeignKey → _id suffix"
            elif is_repeated:
                sql_field_name = f"{field_name}_junction"
                sql_type = "junction table"
                notes = "RepeatedField → junction table"
            else:
                sql_field_name = field_name
                sql_type = infer_sql_type(field_type)
                notes = None

            conn.execute(
                """
                INSERT INTO record_layer_fields
                (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, ?, ?, ?, ?, ?)
                """,
                (record_name, "sql", sql_field_name, sql_type, field_name, notes),
            )

    # Populate pybind layer
    for record_name, fields in pybind_bindings.items():
        for field_info in fields:
            field_name = field_info["field_name"]
            source_field = field_info["source_field"]
            is_property = field_info["is_property"]
            notes = "def_property_readonly" if is_property else "def_readonly"

            conn.execute(
                """
                INSERT INTO record_layer_fields
                (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, ?, ?, ?, ?, ?)
                """,
                (record_name, "pybind", field_name, None, source_field, notes),
            )

    # Populate Pydantic layer
    for class_name, model_info in pydantic_models.items():
        cpp_record = model_info["cpp_record"]
        fields = model_info["fields"]

        # Update record_layer_mapping if C++ record is known
        if cpp_record and cpp_record in cpp_records:
            conn.execute(
                """
                UPDATE record_layer_mapping
                SET pydantic_model = ?
                WHERE record_name = ?
                """,
                (class_name, cpp_record),
            )

            # Insert Pydantic fields
            for field_info in fields:
                field_name = field_info["field_name"]
                field_type = field_info["field_type"]

                # Try to infer source C++ field (reverse snake_case transformation)
                # This is heuristic — may not always match
                source_field = None
                for cpp_field_info in cpp_records[cpp_record]:
                    cpp_field_name = cpp_field_info["field_name"]
                    if camel_to_snake(cpp_field_name) == field_name:
                        source_field = cpp_field_name
                        break

                conn.execute(
                    """
                    INSERT INTO record_layer_fields
                    (record_name, layer, field_name, field_type, source_field, notes)
                    VALUES (?, ?, ?, ?, ?, ?)
                    """,
                    (cpp_record, "pydantic", field_name, field_type, source_field, None),
                )

    conn.commit()


# ===== Main =====


def main() -> int:
    """Index record field mappings from all four layers."""
    parser = argparse.ArgumentParser(
        description="Index cross-layer record field mappings"
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

    print(f"Indexing record mappings from {repo_root}...")

    # === Parse C++ transfer records ===
    cpp_records = {}
    transfer_dir = repo_root / "msd" / "msd-transfer" / "src"
    if not transfer_dir.exists():
        print(f"ERROR: Transfer directory not found: {transfer_dir}", file=sys.stderr)
        return 1

    for header_path in transfer_dir.glob("*.hpp"):
        if header_path.name == "Records.hpp":
            continue  # Skip convenience header

        header_content = header_path.read_text()
        record_name = header_path.stem  # e.g., EnergyRecord from EnergyRecord.hpp

        fields = parse_boost_describe_fields(header_content, str(header_path.relative_to(repo_root)))

        if fields:
            cpp_records[record_name] = fields
            print(f"  C++: {record_name} ({len(fields)} fields)")

    # === Parse pybind11 bindings ===
    pybind_bindings = {}
    bindings_path = repo_root / "msd" / "msd-pybind" / "src" / "record_bindings.cpp"
    if bindings_path.exists():
        bindings_content = bindings_path.read_text()
        pybind_bindings = parse_pybind_bindings(bindings_content)
        print(f"  pybind: {len(pybind_bindings)} record classes parsed")
    else:
        print(
            f"WARNING: pybind bindings not found: {bindings_path}", file=sys.stderr
        )

    # === Parse Pydantic models ===
    pydantic_models = {}
    models_path = repo_root / "replay" / "replay" / "models.py"
    if models_path.exists():
        models_content = models_path.read_text()
        pydantic_models = parse_pydantic_models(models_content)
        print(f"  Pydantic: {len(pydantic_models)} model classes parsed")
    else:
        print(f"WARNING: Pydantic models not found: {models_path}", file=sys.stderr)

    # === Populate database ===
    populate_record_mappings(conn, cpp_records, pybind_bindings, pydantic_models)

    # === Rebuild FTS index ===
    rebuild_fts(conn)

    print(f"✓ Indexed {len(cpp_records)} C++ records")
    print(f"✓ Database updated: {db_path}")

    conn.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
