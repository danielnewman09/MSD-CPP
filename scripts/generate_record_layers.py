#!/usr/bin/env python3
"""
Record Layer Code Generator

AUTO-GENERATES pybind11 bindings and Pydantic leaf models from msd-transfer
C++ record headers. Parses BOOST_DESCRIBE_STRUCT macros using tree-sitter
to extract record metadata, then emits mechanical code following deterministic
patterns based on C++ field types.

Ticket: 0062_pybind_codegen_from_boost_describe
Design: docs/designs/0062_pybind_codegen_from_boost_describe/design.md

Usage:
    python scripts/generate_record_layers.py [--check-only] [--update-traceability DB_PATH]

Options:
    --check-only                Check if generated files match committed versions (for CI)
    --update-traceability PATH  Write cpp/sql/pybind/pydantic layer data to traceability.db
"""

import argparse
import re
import sqlite3
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

try:
    from tree_sitter import Language, Parser
except ImportError:
    print("ERROR: tree-sitter not installed", file=sys.stderr)
    print("Install with: pip install tree-sitter", file=sys.stderr)
    sys.exit(1)


# ==============================================================================
# Configuration: Tier Classification and Name Mapping
# ==============================================================================

# Tier 2: Sub-records (no FKs, used as nested fields)
TIER2_SUBRECORDS = {
    "CoordinateRecord",
    "VelocityRecord",
    "AccelerationRecord",
    "QuaternionDRecord",
    "Vector4DRecord",
    "Vector3DRecord",
    "AngularAccelerationRecord",
    "AngularVelocityRecord",
    "AngularCoordinateRecord",
    "ContactPointRecord",
    "ForceVectorRecord",
    "TorqueVectorRecord",
    "ExternalForceRecord",
}

# Tier 1: Top-level records (own DB tables, may have FKs)
TIER1_TOPLEVEL = {
    "SimulationFrameRecord",
    "AssetInertialStaticRecord",
    "InertialStateRecord",
    "EnergyRecord",
    "SystemEnergyRecord",
    "CollisionResultRecord",
    "SolverDiagnosticRecord",
    "MeshRecord",
    "ObjectRecord",
    "MaterialRecord",
    "PhysicsTemplateRecord",
}

# C++ Record → Pydantic Class Name Mapping
# Multiple C++ records can map to the same Pydantic class (structural equivalence)
NAME_MAPPING = {
    # Vec3 family (all structurally equivalent: {x, y, z})
    "CoordinateRecord": "Vec3",
    "VelocityRecord": "Vec3",
    "AccelerationRecord": "Vec3",
    "Vector3DRecord": "Vec3",
    "ForceVectorRecord": "Vec3",
    "TorqueVectorRecord": "Vec3",
    # Quaternion family (all structurally equivalent: {w, x, y, z})
    "QuaternionDRecord": "Quaternion",
    "Vector4DRecord": "Quaternion",
    # Contact geometry
    "ContactPointRecord": "ContactPoint",
    # Diagnostics and energy
    "SolverDiagnosticRecord": "SolverDiagnostics",
    "EnergyRecord": "EnergyPoint",
    "SystemEnergyRecord": "SystemEnergyPoint",
    # Angular sub-records (if needed for future API endpoints)
    # NOTE: Not currently exposed in Pydantic, but mapping defined for future use
    # "AngularVelocityRecord": "EulerAngles",
    # "AngularAccelerationRecord": "EulerAngles",
    # "AngularCoordinateRecord": "EulerAngles",
}


# ==============================================================================
# Data Classes
# ==============================================================================

class FieldType(Enum):
    """Classification of C++ field types for code generation patterns."""

    PRIMITIVE = "primitive"
    NESTED_SUB_RECORD = "nested_sub_record"
    FOREIGN_KEY = "foreign_key"
    REPEATED_FIELD = "repeated_field"


@dataclass
class FieldInfo:
    """Parsed field metadata from C++ struct."""

    name: str  # Field name from BOOST_DESCRIBE
    type: str  # C++ type string (e.g., "double", "ForeignKey<T>")
    field_type: FieldType  # Classified type for pattern selection


@dataclass
class RecordInfo:
    """Parsed record metadata from C++ header."""

    record_name: str  # e.g., "CoordinateRecord"
    base_class: str  # e.g., "cpp_sqlite::BaseTransferObject"
    fields: list[FieldInfo]
    header_path: Path
    tier: int = 0  # 1=top-level, 2=sub-record, 3=extended


# ==============================================================================
# RecordParser: Extract metadata from C++ headers using tree-sitter
# ==============================================================================

class RecordParser:
    """Parse C++ record headers to extract record name, fields, and types."""

    def __init__(self):
        """Initialize tree-sitter C++ parser."""
        # Load C++ language from tree-sitter-cpp
        try:
            import tree_sitter_cpp
            cpp_lang = Language(tree_sitter_cpp.language())
        except Exception as e:
            print(f"ERROR: Failed to load tree-sitter-cpp: {e}", file=sys.stderr)
            print("Install with: pip install tree-sitter-cpp", file=sys.stderr)
            sys.exit(1)

        self.parser = Parser(language=cpp_lang)

    def parse_header(self, header_path: Path) -> Optional[RecordInfo]:
        """Extract record metadata from a header file.

        Returns:
            RecordInfo if header contains a BOOST_DESCRIBE_STRUCT macro, else None
        """
        source = header_path.read_bytes()
        tree = self.parser.parse(source)

        # Find BOOST_DESCRIBE_STRUCT macro invocation
        describe_macro = self._find_boost_describe(tree.root_node, source)
        if not describe_macro:
            return None

        record_name, base_class, field_names = self._parse_macro_args(
            describe_macro, source
        )

        # Find struct definition
        struct_node = self._find_struct(tree.root_node, record_name, source)
        if not struct_node:
            raise ParseError(
                f"BOOST_DESCRIBE found but no struct {record_name} in {header_path}"
            )

        # Extract field types from struct body
        fields = self._extract_fields(struct_node, field_names, source)

        # Classify tier
        tier = self._classify_tier(record_name, fields)

        return RecordInfo(
            record_name=record_name,
            base_class=base_class,
            fields=fields,
            header_path=header_path,
            tier=tier,
        )

    def _find_boost_describe(self, node, source: bytes):
        """Find BOOST_DESCRIBE_STRUCT macro invocation in AST."""
        # BOOST_DESCRIBE_STRUCT is parsed as expression_statement > call_expression
        if node.type == "call_expression":
            # Check if function name is BOOST_DESCRIBE_STRUCT
            function_node = node.child_by_field_name("function")
            if function_node and function_node.type == "identifier":
                function_name = source[function_node.start_byte : function_node.end_byte].decode("utf-8")
                if function_name == "BOOST_DESCRIBE_STRUCT":
                    return node

        for child in node.children:
            result = self._find_boost_describe(child, source)
            if result:
                return result
        return None

    def _parse_macro_args(self, macro_node, source: bytes) -> tuple[str, str, list[str]]:
        """Parse BOOST_DESCRIBE_STRUCT arguments.

        Format: BOOST_DESCRIBE_STRUCT(RecordName, (BaseClass), (field1, field2, ...))
        Parsed as: call_expression with argument_list

        Returns:
            (record_name, base_class, [field_names])
        """
        # Get argument_list node
        arg_list_node = macro_node.child_by_field_name("arguments")
        if not arg_list_node:
            raise ParseError("BOOST_DESCRIBE_STRUCT has no arguments")

        # Extract arguments: should be 3 (record_name, (base_class), (fields))
        args = [child for child in arg_list_node.children if child.type not in ("(", ")", ",")]

        if len(args) != 3:
            raise ParseError(f"BOOST_DESCRIBE_STRUCT expects 3 arguments, got {len(args)}")

        # Arg 0: record name (identifier)
        record_name_node = args[0]
        if record_name_node.type == "identifier":
            record_name = source[record_name_node.start_byte : record_name_node.end_byte].decode("utf-8")
        else:
            raise ParseError(f"Expected identifier for record name, got {record_name_node.type}")

        # Arg 1: base class wrapped in parentheses (parenthesized_expression)
        base_class_node = args[1]
        if base_class_node.type == "parenthesized_expression":
            # Extract the qualified_identifier inside
            base_qual = base_class_node.children[1]  # Skip opening '('
            base_class = source[base_qual.start_byte : base_qual.end_byte].decode("utf-8")
        else:
            raise ParseError(f"Expected parenthesized base class, got {base_class_node.type}")

        # Arg 2: field list wrapped in parentheses (parenthesized_expression containing comma_expression)
        fields_node = args[2]
        if fields_node.type == "parenthesized_expression":
            # Extract comma_expression or single identifier inside
            fields_expr = fields_node.children[1]  # Skip opening '('
            field_names = self._parse_field_list(fields_expr, source)
        else:
            raise ParseError(f"Expected parenthesized field list, got {fields_node.type}")

        return record_name, base_class, field_names

    def _parse_field_list(self, node, source: bytes) -> list[str]:
        """Parse comma-separated field list from AST node."""
        if node.type == "identifier":
            # Single field
            return [source[node.start_byte : node.end_byte].decode("utf-8")]
        elif node.type == "comma_expression":
            # Multiple fields: recursively extract from nested comma_expression
            fields = []
            # Left side: may be another comma_expression or identifier
            left = node.children[0]
            fields.extend(self._parse_field_list(left, source))
            # Right side: after comma, may be identifier or comma_expression
            # Find all identifiers after commas
            for child in node.children[1:]:
                if child.type == "identifier":
                    fields.append(source[child.start_byte : child.end_byte].decode("utf-8"))
                elif child.type == "comma_expression":
                    fields.extend(self._parse_field_list(child, source))
            return fields
        else:
            raise ParseError(f"Unexpected field list node type: {node.type}")

    def _find_struct(self, node, struct_name: str, source: bytes):
        """Find struct definition by name."""
        if node.type == "struct_specifier":
            # Extract struct name
            name_node = node.child_by_field_name("name")
            if name_node:
                name = source[name_node.start_byte : name_node.end_byte].decode("utf-8")
                if name == struct_name:
                    return node

        for child in node.children:
            result = self._find_struct(child, struct_name, source)
            if result:
                return result
        return None

    def _extract_fields(
        self, struct_node, field_names: list[str], source: bytes
    ) -> list[FieldInfo]:
        """Extract field types from struct body."""
        fields = []

        # Find field declaration list
        body = struct_node.child_by_field_name("body")
        if not body:
            raise ParseError(f"Struct {struct_node} has no body")

        # Build map of field_name → type
        field_types = {}
        for child in body.children:
            if child.type == "field_declaration":
                # Field declaration: <type> <declarator>;
                field_type = self._extract_field_type(child, source)
                declarator = child.child_by_field_name("declarator")
                if declarator:
                    field_name = self._extract_declarator_name(declarator, source)
                    if field_name:
                        field_types[field_name] = field_type

        # Match BOOST_DESCRIBE field names to types
        for field_name in field_names:
            if field_name not in field_types:
                raise ParseError(
                    f"Field {field_name} in BOOST_DESCRIBE but not in struct body"
                )

            field_type_str = field_types[field_name]
            field_type = self._classify_field_type(field_type_str)

            fields.append(
                FieldInfo(name=field_name, type=field_type_str, field_type=field_type)
            )

        return fields

    def _extract_field_type(self, field_decl_node, source: bytes) -> str:
        """Extract type from field_declaration node."""
        type_node = field_decl_node.child_by_field_name("type")
        if not type_node:
            raise ParseError(f"Field declaration has no type: {field_decl_node}")

        type_str = source[type_node.start_byte : type_node.end_byte].decode("utf-8")
        return type_str.strip()

    def _extract_declarator_name(self, declarator_node, source: bytes) -> Optional[str]:
        """Extract variable name from declarator node."""
        if declarator_node.type == "identifier":
            return source[declarator_node.start_byte : declarator_node.end_byte].decode(
                "utf-8"
            )
        elif declarator_node.type == "field_identifier":
            return source[declarator_node.start_byte : declarator_node.end_byte].decode(
                "utf-8"
            )
        elif declarator_node.type == "init_declarator":
            # Has initializer: type name{value}
            decl = declarator_node.child_by_field_name("declarator")
            if decl:
                return self._extract_declarator_name(decl, source)

        # Recursively check children
        for child in declarator_node.children:
            name = self._extract_declarator_name(child, source)
            if name:
                return name

        return None

    def _classify_field_type(self, type_str: str) -> FieldType:
        """Classify field into PRIMITIVE, NESTED_SUB_RECORD, FOREIGN_KEY, or REPEATED_FIELD."""
        # Normalize whitespace
        type_str = " ".join(type_str.split())

        if "ForeignKey<" in type_str or "cpp_sqlite::ForeignKey<" in type_str:
            return FieldType.FOREIGN_KEY
        elif (
            "RepeatedFieldTransferObject<" in type_str
            or "cpp_sqlite::RepeatedFieldTransferObject<" in type_str
        ):
            return FieldType.REPEATED_FIELD
        elif type_str.endswith("Record") or "Record>" in type_str:
            # Nested sub-record (e.g., CoordinateRecord, or ForeignKey<...Record>)
            # But ForeignKey already matched above, so this is a nested sub-record
            return FieldType.NESTED_SUB_RECORD
        elif type_str in (
            "double",
            "float",
            "uint32_t",
            "int",
            "bool",
            "std::string",
            "uint8_t",
            "int32_t",
            "uint64_t",
            "int64_t",
        ) or type_str.startswith("std::vector<"):
            # Primitives and BLOB types (std::vector<uint8_t>)
            return FieldType.PRIMITIVE
        else:
            raise ValueError(f"Unknown field type: {type_str}")

    def _classify_tier(self, record_name: str, fields: list[FieldInfo]) -> int:
        """Classify record into Tier 1, 2, or 3."""
        if record_name in TIER2_SUBRECORDS:
            return 2
        elif record_name in TIER1_TOPLEVEL:
            return 1
        else:
            # Tier 3: Extended/future records
            return 3


class ParseError(Exception):
    """Raised when header parsing fails."""

    pass


# ==============================================================================
# PybindCodegen: Generate record_bindings.cpp
# ==============================================================================

class PybindCodegen:
    """Generate pybind11 binding code from RecordInfo list."""

    # Binding patterns for each field type
    BINDING_PATTERNS = {
        FieldType.PRIMITIVE: lambda name, record: f'.def_readonly("{name}", &{record}::{name})',
        FieldType.NESTED_SUB_RECORD: lambda name, record: f'.def_readonly("{name}", &{record}::{name})',
        FieldType.FOREIGN_KEY: lambda name, record: f'.def_property_readonly("{name}_id", []({record} const& r) {{ return r.{name}.id; }})',
        FieldType.REPEATED_FIELD: lambda name, record: f'.def_property_readonly("{name}", []({record} const& r) {{ return r.{name}.data; }})',
    }

    def generate(self, records: list[RecordInfo]) -> str:
        """Generate full record_bindings.cpp content."""
        header = self._generate_header()

        # Group by tier: Tier 2 → Tier 1 → Tier 3
        tier2 = [r for r in records if r.tier == 2]
        tier1 = [r for r in records if r.tier == 1]
        tier3 = [r for r in records if r.tier == 3]

        body = []

        # Tier 2: Sub-Records
        body.append(self._generate_tier_comment("Tier 2: Sub-Records (bind first since top-level records depend on them)"))
        for record in tier2:
            body.append(self._generate_class_binding(record))
            body.append("")  # Blank line between records

        # Tier 1: Top-Level Records
        body.append(self._generate_tier_comment("Tier 1: Top-Level Records (own DB tables)"))
        for record in tier1:
            body.append(self._generate_class_binding(record))
            body.append("")

        # Tier 3: Extended Records
        if tier3:
            body.append(self._generate_tier_comment("Tier 3: Extended Records (forward compatibility)"))
            for record in tier3:
                body.append(self._generate_class_binding(record))
                body.append("")

        footer = self._generate_footer()

        return header + "\n".join(body) + footer

    def _generate_header(self) -> str:
        """Generate file header with includes and namespace."""
        from datetime import datetime

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return f"""// AUTO-GENERATED by scripts/generate_record_layers.py
// DO NOT EDIT MANUALLY
// Generated: {timestamp}
// Source: msd-transfer/src/Records.hpp
// Ticket: 0062_pybind_codegen_from_boost_describe

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "msd-transfer/src/Records.hpp"

namespace py = pybind11;

void bind_records(py::module_& m)
{{
"""

    def _generate_tier_comment(self, comment: str) -> str:
        """Generate tier section comment."""
        separator = "=" * 40
        return f"  // {separator}\n  // {comment}\n  // {separator}\n"

    def _generate_class_binding(self, record: RecordInfo) -> str:
        """Generate py::class_<> block for a single record."""
        record_fqn = f"msd_transfer::{record.record_name}"

        lines = [f'  py::class_<{record_fqn}>(m, "{record.record_name}")']
        lines.append("    .def(py::init<>())")
        lines.append(f'    .def_readonly("id", &{record_fqn}::id)')

        for field in record.fields:
            pattern = self.BINDING_PATTERNS[field.field_type]
            binding_line = pattern(field.name, record_fqn)
            lines.append(f"    {binding_line}")

        lines.append("    ;")
        return "\n".join(lines)

    def _generate_footer(self) -> str:
        """Generate function closing brace."""
        return "}\n"


# ==============================================================================
# PydanticCodegen: Generate generated_models.py
# ==============================================================================

class PydanticCodegen:
    """Generate Pydantic leaf models from RecordInfo list."""

    # C++ type → Python type mapping
    TYPE_MAPPING = {
        "double": "float",
        "float": "float",
        "uint32_t": "int",
        "int": "int",
        "int32_t": "int",
        "int64_t": "int",
        "uint64_t": "int",
        "uint8_t": "int",
        "bool": "bool",
        "std::string": "str",
    }

    def __init__(self, name_mapping: dict[str, str]):
        """Initialize with C++ → Pydantic name mapping."""
        self.name_mapping = name_mapping

    def generate(self, records: list[RecordInfo]) -> str:
        """Generate full generated_models.py content."""
        header = self._generate_header()
        imports = "from pydantic import BaseModel\n\n\n"

        # Only generate models for records in the mapping
        models = []
        generated_names = set()  # Track generated model names to avoid duplicates

        for record in records:
            if record.record_name in self.name_mapping:
                pydantic_name = self.name_mapping[record.record_name]

                # Skip if we've already generated this model name
                # (multiple C++ records can map to same Pydantic class)
                if pydantic_name in generated_names:
                    continue

                model_code = self._generate_model(record)
                models.append(model_code)
                generated_names.add(pydantic_name)

        return header + imports + "\n\n".join(models) + "\n"

    def _generate_header(self) -> str:
        """Generate file header with auto-generated warning."""
        from datetime import datetime

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return f"""# AUTO-GENERATED by scripts/generate_record_layers.py
# DO NOT EDIT MANUALLY
# Generated: {timestamp}
# Source: msd-transfer C++ records
# Ticket: 0062_pybind_codegen_from_boost_describe

\"\"\"
Leaf Pydantic models generated from msd-transfer C++ record definitions.

These models are structurally equivalent to their C++ counterparts and provide
deterministic linkage via Maps-to: annotations for the 0061 mapping indexer.

Hand-written composite models (FrameData, BodyState, etc.) should import from
this module rather than redefining leaf models.
\"\"\"

"""

    def _generate_model(self, record: RecordInfo) -> str:
        """Generate a single Pydantic model."""
        pydantic_name = self.name_mapping[record.record_name]

        lines = [
            f"class {pydantic_name}(BaseModel):",
            f'    """Generated from {record.record_name}. Maps-to: {record.record_name}"""',
            "",
        ]

        for field in record.fields:
            # For ForeignKeys, append _id suffix to field name
            if field.field_type == FieldType.FOREIGN_KEY:
                py_field_name = self._camel_to_snake(field.name) + "_id"
            else:
                py_field_name = self._camel_to_snake(field.name)

            py_type = self._map_field_type(field)
            lines.append(f"    {py_field_name}: {py_type}")

        return "\n".join(lines)

    def _map_field_type(self, field: FieldInfo) -> str:
        """Map C++ field type to Python type annotation."""
        if field.field_type == FieldType.PRIMITIVE:
            # Handle std::vector<uint8_t> BLOB fields
            if field.type.startswith("std::vector<"):
                return "list[int]"  # BLOB as list of bytes
            return self.TYPE_MAPPING.get(field.type, "Any")
        elif field.field_type == FieldType.FOREIGN_KEY:
            # ForeignKey<T> → int (expose the ID)
            return "int"
        elif field.field_type == FieldType.NESTED_SUB_RECORD:
            # Look up the nested record's Pydantic name
            nested_record_name = field.type
            if nested_record_name in self.name_mapping:
                return self.name_mapping[nested_record_name]
            else:
                # Fallback: use RecordName → RecordNameModel
                return f"{nested_record_name.replace('Record', '')}Model"
        elif field.field_type == FieldType.REPEATED_FIELD:
            # Extract template argument from RepeatedFieldTransferObject<T>
            element_type = self._extract_template_arg(field.type)
            if element_type in self.name_mapping:
                py_element_type = self.name_mapping[element_type]
            else:
                py_element_type = f"{element_type.replace('Record', '')}Model"
            return f"list[{py_element_type}]"

    def _extract_template_arg(self, type_str: str) -> str:
        """Extract template argument from RepeatedFieldTransferObject<T>."""
        match = re.search(r"<([^>]+)>", type_str)
        if match:
            return match.group(1).strip()
        raise ValueError(f"Failed to extract template arg from {type_str}")

    def _camel_to_snake(self, name: str) -> str:
        """Transform camelCase to snake_case."""
        # Insert underscore before uppercase letters (except at start)
        s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
        s2 = re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1)
        return s2.lower()


# ==============================================================================
# TraceabilityWriter: Populate traceability DB from in-memory RecordInfo
# ==============================================================================

class TraceabilityWriter:
    """Write record layer data directly to the traceability database.

    Replaces redundant re-parsing by the 0061 indexer for layers that the
    generator already has authoritative data for: cpp, sql, pybind, and
    leaf-pydantic. The indexer retains responsibility for hand-written
    composite Pydantic models only.
    """

    # C++ type → SQL type (deterministic mapping)
    SQL_TYPE_MAP = {
        "double": "REAL",
        "float": "REAL",
        "uint32_t": "INTEGER",
        "int": "INTEGER",
        "int32_t": "INTEGER",
        "int64_t": "INTEGER",
        "uint64_t": "INTEGER",
        "uint8_t": "INTEGER",
        "bool": "INTEGER",
        "std::string": "TEXT",
    }

    # C++ type → Python type (for pydantic layer field_type column)
    PYTHON_TYPE_MAP = {
        "double": "float",
        "float": "float",
        "uint32_t": "int",
        "int": "int",
        "int32_t": "int",
        "int64_t": "int",
        "uint64_t": "int",
        "uint8_t": "int",
        "bool": "bool",
        "std::string": "str",
    }

    def __init__(self, db_path: Path, name_mapping: dict[str, str]):
        """Open the traceability database and ensure schema exists."""
        # Import traceability schema from sibling directory
        trace_dir = Path(__file__).parent / "traceability"
        sys.path.insert(0, str(trace_dir))
        from traceability_schema import create_schema, rebuild_fts

        self.conn = create_schema(db_path)
        self.rebuild_fts = rebuild_fts
        self.name_mapping = name_mapping

    def populate(self, records: list["RecordInfo"]) -> None:
        """Write all four generator-authoritative layers to the database."""
        # Clear existing generator-managed layers (preserve hand-written pydantic composites)
        self.conn.execute(
            "DELETE FROM record_layer_fields WHERE layer IN ('cpp', 'sql', 'pybind', 'pydantic')"
        )
        self.conn.execute("DELETE FROM record_layer_mapping")

        for record in records:
            self._write_mapping_row(record)
            self._write_cpp_layer(record)
            self._write_sql_layer(record)
            self._write_pybind_layer(record)
            self._write_pydantic_layer(record)

        self.conn.commit()
        self.rebuild_fts(self.conn)

    def close(self):
        """Close database connection."""
        self.conn.close()

    def _write_mapping_row(self, record: "RecordInfo") -> None:
        """Insert record_layer_mapping row."""
        pydantic_model = self.name_mapping.get(record.record_name)
        self.conn.execute(
            """
            INSERT INTO record_layer_mapping
                (record_name, sql_table, pybind_class, pydantic_model)
            VALUES (?, ?, ?, ?)
            """,
            (record.record_name, record.record_name, record.record_name, pydantic_model),
        )

    def _write_cpp_layer(self, record: "RecordInfo") -> None:
        """Write C++ layer fields from RecordInfo."""
        for field in record.fields:
            self.conn.execute(
                """
                INSERT INTO record_layer_fields
                    (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, 'cpp', ?, ?, NULL, ?)
                """,
                (
                    record.record_name,
                    field.name,
                    field.type,
                    f"tier {record.tier}" if record.tier else None,
                ),
            )

    def _write_sql_layer(self, record: "RecordInfo") -> None:
        """Write SQL layer fields (deterministic from C++ types)."""
        for field in record.fields:
            if field.field_type == FieldType.FOREIGN_KEY:
                sql_name = f"{field.name}_id"
                sql_type = "INTEGER"
                notes = "ForeignKey → _id suffix"
            elif field.field_type == FieldType.REPEATED_FIELD:
                sql_name = f"{field.name}_junction"
                sql_type = "junction table"
                notes = "RepeatedField → junction table"
            else:
                sql_name = field.name
                sql_type = self.SQL_TYPE_MAP.get(field.type, "BLOB")
                if field.field_type == FieldType.NESTED_SUB_RECORD:
                    sql_type = "BLOB"  # Sub-records serialized as BLOB
                notes = None

            self.conn.execute(
                """
                INSERT INTO record_layer_fields
                    (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, 'sql', ?, ?, ?, ?)
                """,
                (record.record_name, sql_name, sql_type, field.name, notes),
            )

    def _write_pybind_layer(self, record: "RecordInfo") -> None:
        """Write pybind layer fields (deterministic from codegen patterns)."""
        # The 'id' field is always bound as def_readonly
        self.conn.execute(
            """
            INSERT INTO record_layer_fields
                (record_name, layer, field_name, field_type, source_field, notes)
            VALUES (?, 'pybind', 'id', NULL, 'id', 'def_readonly')
            """,
            (record.record_name,),
        )

        for field in record.fields:
            if field.field_type == FieldType.FOREIGN_KEY:
                python_name = f"{field.name}_id"
                source_field = f"{field.name}.id"
                notes = "def_property_readonly"
            elif field.field_type == FieldType.REPEATED_FIELD:
                python_name = field.name
                source_field = f"{field.name}.data"
                notes = "def_property_readonly"
            else:
                python_name = field.name
                source_field = field.name
                notes = "def_readonly"

            self.conn.execute(
                """
                INSERT INTO record_layer_fields
                    (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, 'pybind', ?, NULL, ?, ?)
                """,
                (record.record_name, python_name, source_field, notes),
            )

    def _write_pydantic_layer(self, record: "RecordInfo") -> None:
        """Write leaf-pydantic layer fields for records in NAME_MAPPING."""
        pydantic_name = self.name_mapping.get(record.record_name)
        if not pydantic_name:
            return  # No Pydantic model for this record

        for field in record.fields:
            # Mirror the PydanticCodegen camel_to_snake transformation
            if field.field_type == FieldType.FOREIGN_KEY:
                py_field_name = self._camel_to_snake(field.name) + "_id"
                py_type = "int"
            elif field.field_type == FieldType.PRIMITIVE:
                py_field_name = self._camel_to_snake(field.name)
                py_type = self.PYTHON_TYPE_MAP.get(field.type, "Any")
            elif field.field_type == FieldType.NESTED_SUB_RECORD:
                py_field_name = self._camel_to_snake(field.name)
                py_type = self.name_mapping.get(field.type, f"{field.type.replace('Record', '')}Model")
            elif field.field_type == FieldType.REPEATED_FIELD:
                py_field_name = self._camel_to_snake(field.name)
                element_type = re.search(r"<([^>]+)>", field.type)
                if element_type:
                    et = element_type.group(1).strip()
                    py_element = self.name_mapping.get(et, f"{et.replace('Record', '')}Model")
                else:
                    py_element = "Any"
                py_type = f"list[{py_element}]"
            else:
                py_field_name = self._camel_to_snake(field.name)
                py_type = "Any"

            self.conn.execute(
                """
                INSERT INTO record_layer_fields
                    (record_name, layer, field_name, field_type, source_field, notes)
                VALUES (?, 'pydantic', ?, ?, ?, ?)
                """,
                (
                    record.record_name,
                    py_field_name,
                    py_type,
                    field.name,
                    f"Maps-to: {pydantic_name}",
                ),
            )

    @staticmethod
    def _camel_to_snake(name: str) -> str:
        """Transform camelCase to snake_case (mirrors PydanticCodegen)."""
        s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
        s2 = re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1)
        return s2.lower()


# ==============================================================================
# Main Generator Orchestration
# ==============================================================================

def parse_record_headers(headers_dir: Path) -> list[RecordInfo]:
    """Parse all record headers in msd-transfer/src/."""
    parser = RecordParser()
    records = []

    # Parse all .hpp files in headers_dir
    for header_file in sorted(headers_dir.glob("*.hpp")):
        # Skip Records.hpp (umbrella header)
        if header_file.name == "Records.hpp":
            continue

        try:
            record_info = parser.parse_header(header_file)
            if record_info:
                records.append(record_info)
                print(f"  Parsed {record_info.record_name} (Tier {record_info.tier})")
        except ParseError as e:
            print(f"  WARNING: Failed to parse {header_file.name}: {e}", file=sys.stderr)
        except Exception as e:
            print(f"  ERROR: Unexpected error parsing {header_file.name}: {e}", file=sys.stderr)
            raise

    return records


def write_file(path: Path, content: str):
    """Write generated content to file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    print(f"  Wrote {path}")


def main():
    """Entry point for record layer generator."""
    parser = argparse.ArgumentParser(
        description="Generate pybind11 bindings and Pydantic models from msd-transfer records"
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Check if generated files match committed versions (for CI)",
    )
    parser.add_argument(
        "--update-traceability",
        metavar="DB_PATH",
        help="Write record layer data to the traceability database",
    )
    args = parser.parse_args()

    # Determine project root
    script_dir = Path(__file__).parent
    project_root = script_dir.parent

    # Paths
    headers_dir = project_root / "msd" / "msd-transfer" / "src"
    pybind_output = project_root / "msd" / "msd-pybind" / "src" / "record_bindings.cpp"
    pydantic_output = project_root / "replay" / "replay" / "generated_models.py"

    print("=" * 60)
    print("Record Layer Generator")
    print("=" * 60)

    # Phase 1: Parse C++ headers
    print("\n[1/3] Parsing msd-transfer headers...")
    records = parse_record_headers(headers_dir)
    print(f"  Found {len(records)} records")

    # Phase 2: Generate pybind bindings
    print("\n[2/3] Generating pybind11 bindings...")
    pybind_codegen = PybindCodegen()
    pybind_code = pybind_codegen.generate(records)

    # Phase 3: Generate Pydantic models
    print("\n[3/3] Generating Pydantic leaf models...")
    pydantic_codegen = PydanticCodegen(NAME_MAPPING)
    pydantic_code = pydantic_codegen.generate(records)

    # Count generated models (unique Pydantic class names)
    generated_model_count = len(set(NAME_MAPPING.values()))
    print(f"  Generating {generated_model_count} unique models from {len(NAME_MAPPING)} records")

    check_failed = False
    if args.check_only:
        # Check mode: compare against committed files
        errors = []

        if not pybind_output.exists():
            errors.append(f"{pybind_output} does not exist")
        elif pybind_output.read_text() != pybind_code:
            errors.append(f"Generated pybind code differs from {pybind_output}")

        if not pydantic_output.exists():
            errors.append(f"{pydantic_output} does not exist")
        elif pydantic_output.read_text() != pydantic_code:
            errors.append(f"Generated Pydantic code differs from {pydantic_output}")

        if errors:
            for error in errors:
                print(f"  ERROR: {error}", file=sys.stderr)
            print("  Run: python scripts/generate_record_layers.py", file=sys.stderr)
            check_failed = True
        else:
            print(f"  OK: All generated files match committed versions")
    else:
        # Write mode: overwrite files
        write_file(pybind_output, pybind_code)
        write_file(pydantic_output, pydantic_code)
        print(f"\n✓ Generated {len(records)} record bindings")
        print(f"✓ Generated {generated_model_count} Pydantic leaf models")

    # Phase 4 (optional): Update traceability database
    # Runs even when --check-only fails — traceability data depends on parsed
    # records (always correct), not on whether committed files are stale.
    if args.update_traceability:
        db_path = Path(args.update_traceability)
        print(f"\n[4/4] Updating traceability database: {db_path}...")
        writer = TraceabilityWriter(db_path, NAME_MAPPING)
        writer.populate(records)
        writer.close()
        print(f"  ✓ Wrote {len(records)} records across 4 layers (cpp, sql, pybind, pydantic)")

    print("\n" + "=" * 60)
    print("Generation complete")
    print("=" * 60)

    if check_failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
