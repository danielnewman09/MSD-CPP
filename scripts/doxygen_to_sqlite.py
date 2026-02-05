#!/usr/bin/env python3
"""
Doxygen XML to SQLite Database Converter

Parses Doxygen XML output and creates a SQLite database for MCP server integration.
This enables fast code search, symbol lookup, and documentation queries.

Usage:
    python doxygen_to_sqlite.py <xml_dir> <output_db>

Example:
    python doxygen_to_sqlite.py build/Debug/docs/xml build/Debug/docs/codebase.db
"""

import argparse
import os
import re
import sqlite3
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional


def create_schema(conn: sqlite3.Connection) -> None:
    """Create the database schema for code indexing."""
    conn.executescript("""
        -- Files table: source files in the codebase
        CREATE TABLE IF NOT EXISTS files (
            id INTEGER PRIMARY KEY,
            refid TEXT UNIQUE NOT NULL,
            name TEXT NOT NULL,
            path TEXT,
            language TEXT
        );
        CREATE INDEX IF NOT EXISTS idx_files_name ON files(name);
        CREATE INDEX IF NOT EXISTS idx_files_path ON files(path);

        -- Namespaces table
        CREATE TABLE IF NOT EXISTS namespaces (
            id INTEGER PRIMARY KEY,
            refid TEXT UNIQUE NOT NULL,
            name TEXT NOT NULL,
            qualified_name TEXT NOT NULL
        );
        CREATE INDEX IF NOT EXISTS idx_namespaces_name ON namespaces(name);

        -- Compounds table: classes, structs, unions, enums
        CREATE TABLE IF NOT EXISTS compounds (
            id INTEGER PRIMARY KEY,
            refid TEXT UNIQUE NOT NULL,
            kind TEXT NOT NULL,  -- class, struct, union, enum, namespace
            name TEXT NOT NULL,
            qualified_name TEXT NOT NULL,
            file_id INTEGER REFERENCES files(id),
            line_number INTEGER,
            brief_description TEXT,
            detailed_description TEXT,
            base_classes TEXT,  -- JSON array of base class names
            is_final INTEGER DEFAULT 0,
            is_abstract INTEGER DEFAULT 0
        );
        CREATE INDEX IF NOT EXISTS idx_compounds_name ON compounds(name);
        CREATE INDEX IF NOT EXISTS idx_compounds_qualified_name ON compounds(qualified_name);
        CREATE INDEX IF NOT EXISTS idx_compounds_kind ON compounds(kind);
        CREATE INDEX IF NOT EXISTS idx_compounds_file_id ON compounds(file_id);

        -- Members table: functions, variables, typedefs, enums
        CREATE TABLE IF NOT EXISTS members (
            id INTEGER PRIMARY KEY,
            refid TEXT UNIQUE NOT NULL,
            compound_id INTEGER REFERENCES compounds(id),
            kind TEXT NOT NULL,  -- function, variable, typedef, enum, define
            name TEXT NOT NULL,
            qualified_name TEXT NOT NULL,
            type TEXT,
            definition TEXT,
            argsstring TEXT,
            file_id INTEGER REFERENCES files(id),
            line_number INTEGER,
            brief_description TEXT,
            detailed_description TEXT,
            protection TEXT,  -- public, protected, private
            is_static INTEGER DEFAULT 0,
            is_const INTEGER DEFAULT 0,
            is_constexpr INTEGER DEFAULT 0,
            is_virtual INTEGER DEFAULT 0,
            is_inline INTEGER DEFAULT 0,
            is_explicit INTEGER DEFAULT 0
        );
        CREATE INDEX IF NOT EXISTS idx_members_name ON members(name);
        CREATE INDEX IF NOT EXISTS idx_members_qualified_name ON members(qualified_name);
        CREATE INDEX IF NOT EXISTS idx_members_kind ON members(kind);
        CREATE INDEX IF NOT EXISTS idx_members_compound_id ON members(compound_id);
        CREATE INDEX IF NOT EXISTS idx_members_file_id ON members(file_id);

        -- Parameters table: function parameters
        CREATE TABLE IF NOT EXISTS parameters (
            id INTEGER PRIMARY KEY,
            member_id INTEGER REFERENCES members(id),
            position INTEGER NOT NULL,
            name TEXT,
            type TEXT NOT NULL,
            default_value TEXT,
            description TEXT
        );
        CREATE INDEX IF NOT EXISTS idx_parameters_member_id ON parameters(member_id);

        -- Symbol references table: call graph and reference relationships
        CREATE TABLE IF NOT EXISTS symbol_refs (
            id INTEGER PRIMARY KEY,
            from_member_id INTEGER,
            to_member_refid TEXT NOT NULL,
            to_member_name TEXT NOT NULL,
            relationship TEXT NOT NULL,  -- 'calls', 'called_by'
            FOREIGN KEY (from_member_id) REFERENCES members(id)
        );
        CREATE INDEX IF NOT EXISTS idx_symbol_refs_from ON symbol_refs(from_member_id);
        CREATE INDEX IF NOT EXISTS idx_symbol_refs_to ON symbol_refs(to_member_refid);

        -- Include dependencies
        CREATE TABLE IF NOT EXISTS includes (
            id INTEGER PRIMARY KEY,
            file_id INTEGER REFERENCES files(id),
            included_file TEXT NOT NULL,
            included_refid TEXT,
            is_local INTEGER DEFAULT 0
        );
        CREATE INDEX IF NOT EXISTS idx_includes_file_id ON includes(file_id);

        -- Full-text search virtual table for documentation
        CREATE VIRTUAL TABLE IF NOT EXISTS fts_docs USING fts5(
            name,
            qualified_name,
            description,
            tokenize='porter'
        );

        -- Metadata table
        CREATE TABLE IF NOT EXISTS metadata (
            key TEXT PRIMARY KEY,
            value TEXT
        );
    """)
    conn.commit()


def get_text(element: Optional[ET.Element], default: str = "") -> str:
    """Extract text content from an element, handling nested elements."""
    if element is None:
        return default

    # Recursively get all text
    text_parts = []
    if element.text:
        text_parts.append(element.text)
    for child in element:
        text_parts.append(get_text(child))
        if child.tail:
            text_parts.append(child.tail)

    result = " ".join(text_parts)
    # Clean up whitespace
    result = re.sub(r'\s+', ' ', result).strip()
    return result


def parse_description(desc_elem: Optional[ET.Element]) -> str:
    """Parse a brief or detailed description element."""
    if desc_elem is None:
        return ""
    return get_text(desc_elem)


def parse_location(loc_elem: Optional[ET.Element]) -> tuple[Optional[str], Optional[int]]:
    """Extract file path and line number from location element."""
    if loc_elem is None:
        return None, None
    file_path = loc_elem.get("file")
    line = loc_elem.get("line")
    return file_path, int(line) if line else None


def parse_compound_file(conn: sqlite3.Connection, xml_path: Path, file_cache: dict, compound_cache: dict) -> None:
    """Parse a compound (class/struct/file) XML file."""
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"Warning: Could not parse {xml_path}: {e}", file=sys.stderr)
        return

    for compounddef in root.findall(".//compounddef"):
        refid = compounddef.get("id", "")
        kind = compounddef.get("kind", "")
        language = compounddef.get("language", "")

        compoundname = compounddef.findtext("compoundname", "")

        # Handle files
        if kind == "file":
            loc = compounddef.find("location")
            file_path = loc.get("file") if loc is not None else None

            cursor = conn.execute(
                "INSERT OR IGNORE INTO files (refid, name, path, language) VALUES (?, ?, ?, ?)",
                (refid, compoundname, file_path, language)
            )
            if cursor.lastrowid:
                file_cache[refid] = cursor.lastrowid
            else:
                row = conn.execute("SELECT id FROM files WHERE refid = ?", (refid,)).fetchone()
                if row:
                    file_cache[refid] = row[0]

            # Parse includes
            file_id = file_cache.get(refid)
            if file_id:
                for inc in compounddef.findall("includes"):
                    included_file = inc.text or ""
                    included_refid = inc.get("refid")
                    is_local = 1 if inc.get("local") == "yes" else 0
                    conn.execute(
                        "INSERT INTO includes (file_id, included_file, included_refid, is_local) VALUES (?, ?, ?, ?)",
                        (file_id, included_file, included_refid, is_local)
                    )
            continue

        # Handle namespaces
        if kind == "namespace":
            name = compoundname.split("::")[-1] if "::" in compoundname else compoundname
            conn.execute(
                "INSERT OR IGNORE INTO namespaces (refid, name, qualified_name) VALUES (?, ?, ?)",
                (refid, name, compoundname)
            )
            continue

        # Handle classes, structs, unions
        if kind in ("class", "struct", "union"):
            name = compoundname.split("::")[-1] if "::" in compoundname else compoundname

            # Get location
            loc = compounddef.find("location")
            file_path, line_number = parse_location(loc)

            # Find file_id from path
            file_id = None
            if file_path:
                row = conn.execute("SELECT id FROM files WHERE path = ?", (file_path,)).fetchone()
                if row:
                    file_id = row[0]

            # Get descriptions
            brief = parse_description(compounddef.find("briefdescription"))
            detailed = parse_description(compounddef.find("detaileddescription"))

            # Get base classes
            base_classes = []
            for baseref in compounddef.findall("basecompoundref"):
                base_classes.append(baseref.text or "")
            base_classes_json = str(base_classes) if base_classes else None

            # Check modifiers
            is_final = 1 if compounddef.get("final") == "yes" else 0
            is_abstract = 1 if compounddef.get("abstract") == "yes" else 0

            cursor = conn.execute(
                """INSERT OR REPLACE INTO compounds
                   (refid, kind, name, qualified_name, file_id, line_number,
                    brief_description, detailed_description, base_classes, is_final, is_abstract)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (refid, kind, name, compoundname, file_id, line_number,
                 brief, detailed, base_classes_json, is_final, is_abstract)
            )
            compound_id = cursor.lastrowid
            compound_cache[refid] = compound_id

            # Add to FTS
            description = f"{brief} {detailed}".strip()
            if description:
                conn.execute(
                    "INSERT INTO fts_docs (name, qualified_name, description) VALUES (?, ?, ?)",
                    (name, compoundname, description)
                )

            # Parse members
            for sectiondef in compounddef.findall("sectiondef"):
                for memberdef in sectiondef.findall("memberdef"):
                    parse_member(conn, memberdef, compound_id, file_cache)


def parse_member(conn: sqlite3.Connection, memberdef: ET.Element, compound_id: Optional[int], file_cache: dict) -> None:
    """Parse a member definition element."""
    refid = memberdef.get("id", "")
    kind = memberdef.get("kind", "")
    prot = memberdef.get("prot", "public")

    name = memberdef.findtext("name", "")
    qualified_name = memberdef.findtext("qualifiedname", name)
    type_str = get_text(memberdef.find("type"))
    definition = memberdef.findtext("definition", "")
    argsstring = memberdef.findtext("argsstring", "")

    # Get location
    loc = memberdef.find("location")
    file_path, line_number = parse_location(loc)

    # Find file_id
    file_id = None
    if file_path:
        row = conn.execute("SELECT id FROM files WHERE path = ?", (file_path,)).fetchone()
        if row:
            file_id = row[0]

    # Get descriptions
    brief = parse_description(memberdef.find("briefdescription"))
    detailed = parse_description(memberdef.find("detaileddescription"))

    # Get modifiers
    is_static = 1 if memberdef.get("static") == "yes" else 0
    is_const = 1 if memberdef.get("const") == "yes" else 0
    is_constexpr = 1 if memberdef.get("constexpr") == "yes" else 0
    is_virtual = 1 if memberdef.get("virt") in ("virtual", "pure-virtual") else 0
    is_inline = 1 if memberdef.get("inline") == "yes" else 0
    is_explicit = 1 if memberdef.get("explicit") == "yes" else 0

    cursor = conn.execute(
        """INSERT OR REPLACE INTO members
           (refid, compound_id, kind, name, qualified_name, type, definition, argsstring,
            file_id, line_number, brief_description, detailed_description, protection,
            is_static, is_const, is_constexpr, is_virtual, is_inline, is_explicit)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
        (refid, compound_id, kind, name, qualified_name, type_str, definition, argsstring,
         file_id, line_number, brief, detailed, prot,
         is_static, is_const, is_constexpr, is_virtual, is_inline, is_explicit)
    )
    member_id = cursor.lastrowid

    # Add to FTS
    description = f"{brief} {detailed}".strip()
    if description:
        conn.execute(
            "INSERT INTO fts_docs (name, qualified_name, description) VALUES (?, ?, ?)",
            (name, qualified_name, description)
        )

    # Parse parameters
    for i, param in enumerate(memberdef.findall("param")):
        param_name = param.findtext("declname", "")
        param_type = get_text(param.find("type"))
        default_value = param.findtext("defval")
        conn.execute(
            "INSERT INTO parameters (member_id, position, name, type, default_value) VALUES (?, ?, ?, ?, ?)",
            (member_id, i, param_name, param_type, default_value)
        )

    # Parse references (calls)
    for ref in memberdef.findall("references"):
        to_refid = ref.get("refid", "")
        to_name = ref.text or ""
        conn.execute(
            "INSERT INTO symbol_refs (from_member_id, to_member_refid, to_member_name, relationship) VALUES (?, ?, ?, ?)",
            (member_id, to_refid, to_name, "calls")
        )

    # Parse referenced by (called by)
    for ref in memberdef.findall("referencedby"):
        to_refid = ref.get("refid", "")
        to_name = ref.text or ""
        conn.execute(
            "INSERT INTO symbol_refs (from_member_id, to_member_refid, to_member_name, relationship) VALUES (?, ?, ?, ?)",
            (member_id, to_refid, to_name, "called_by")
        )


def parse_index(conn: sqlite3.Connection, index_path: Path) -> list[tuple[str, str]]:
    """Parse the index.xml file to get list of all compounds."""
    compounds = []
    try:
        tree = ET.parse(index_path)
        root = tree.getroot()
        for compound in root.findall("compound"):
            refid = compound.get("refid", "")
            kind = compound.get("kind", "")
            compounds.append((refid, kind))
    except ET.ParseError as e:
        print(f"Warning: Could not parse index.xml: {e}", file=sys.stderr)
    return compounds


def main():
    parser = argparse.ArgumentParser(description="Convert Doxygen XML to SQLite database")
    parser.add_argument("xml_dir", help="Directory containing Doxygen XML output")
    parser.add_argument("output_db", help="Output SQLite database path")
    parser.add_argument("--project-root", help="Project root path to make paths relative", default=None)
    args = parser.parse_args()

    xml_dir = Path(args.xml_dir)
    output_db = Path(args.output_db)

    if not xml_dir.is_dir():
        print(f"Error: XML directory not found: {xml_dir}", file=sys.stderr)
        sys.exit(1)

    index_path = xml_dir / "index.xml"
    if not index_path.exists():
        print(f"Error: index.xml not found in {xml_dir}", file=sys.stderr)
        sys.exit(1)

    # Remove existing database
    if output_db.exists():
        output_db.unlink()

    # Create database
    conn = sqlite3.connect(str(output_db))
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=NORMAL")

    print(f"Creating database schema...")
    create_schema(conn)

    # Store metadata
    conn.execute("INSERT INTO metadata (key, value) VALUES (?, ?)",
                 ("xml_dir", str(xml_dir.absolute())))
    if args.project_root:
        conn.execute("INSERT INTO metadata (key, value) VALUES (?, ?)",
                     ("project_root", args.project_root))

    # Parse index to get all compounds
    print(f"Parsing index.xml...")
    compounds = parse_index(conn, index_path)
    print(f"Found {len(compounds)} compounds")

    # Parse all compound files
    file_cache = {}
    compound_cache = {}

    for i, (refid, kind) in enumerate(compounds):
        xml_file = xml_dir / f"{refid}.xml"
        if xml_file.exists():
            parse_compound_file(conn, xml_file, file_cache, compound_cache)

        if (i + 1) % 50 == 0:
            print(f"Processed {i + 1}/{len(compounds)} compounds...")
            conn.commit()

    conn.commit()

    # Print statistics
    stats = {
        "files": conn.execute("SELECT COUNT(*) FROM files").fetchone()[0],
        "namespaces": conn.execute("SELECT COUNT(*) FROM namespaces").fetchone()[0],
        "compounds": conn.execute("SELECT COUNT(*) FROM compounds").fetchone()[0],
        "members": conn.execute("SELECT COUNT(*) FROM members").fetchone()[0],
        "parameters": conn.execute("SELECT COUNT(*) FROM parameters").fetchone()[0],
        "symbol_refs": conn.execute("SELECT COUNT(*) FROM symbol_refs").fetchone()[0],
        "includes": conn.execute("SELECT COUNT(*) FROM includes").fetchone()[0],
    }

    print(f"\nDatabase created: {output_db}")
    print(f"Statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value}")

    conn.close()


if __name__ == "__main__":
    main()
