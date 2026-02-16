#!/usr/bin/env python3
"""
Python AST to SQLite Database Converter

Parses Python source files using the ast module and inserts symbols into the
existing codebase.db schema for MCP server integration. This enables Python
code to be searchable alongside C++ symbols through existing MCP tools.

Usage:
    python python_to_sqlite.py <python_source_dir> <existing_db_path> [--project-root PATH]

Example:
    python python_to_sqlite.py replay/replay build/Debug/docs/codebase.db --project-root .

Ticket: 0064_python_codebase_documentation_index
"""

import argparse
import ast
import os
import sqlite3
import sys
from pathlib import Path
from typing import Optional


def create_schema(conn: sqlite3.Connection) -> None:
    """
    Defensively create schema if it doesn't exist.

    The schema is designed for both C++ (Doxygen) and Python (AST) symbols.
    All tables use CREATE TABLE IF NOT EXISTS, so this is safe to call
    even if the C++ indexer has already created the schema.
    """
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

        -- Namespaces table (Python packages/modules)
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
            kind TEXT NOT NULL,
            name TEXT NOT NULL,
            qualified_name TEXT NOT NULL,
            file_id INTEGER REFERENCES files(id),
            line_number INTEGER,
            brief_description TEXT,
            detailed_description TEXT,
            base_classes TEXT,
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
            kind TEXT NOT NULL,
            name TEXT NOT NULL,
            qualified_name TEXT NOT NULL,
            type TEXT,
            definition TEXT,
            argsstring TEXT,
            file_id INTEGER REFERENCES files(id),
            line_number INTEGER,
            brief_description TEXT,
            detailed_description TEXT,
            protection TEXT,
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

        -- Include dependencies (Python imports)
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


def sanitize_for_refid(name: str) -> str:
    """
    Convert a Python qualified name to a refid-safe string.

    Examples:
        "replay.models" -> "replay_models"
        "replay.services.SimulationService" -> "replay_services_SimulationService"
    """
    return name.replace(".", "_").replace("/", "_")


def make_refid(prefix: str, qualified_name: str) -> str:
    """
    Generate a Python refid with py_ prefix to avoid collision with C++ refids.

    Args:
        prefix: Type prefix (file, class, func, var)
        qualified_name: Dot-notation qualified name

    Returns:
        Refid like "py_class_replay_models_Vec3"
    """
    return f"py_{prefix}_{sanitize_for_refid(qualified_name)}"


def get_protection(name: str) -> str:
    """
    Map Python naming conventions to protection levels.

    Args:
        name: Symbol name

    Returns:
        "private" if name starts with _ or __, "public" otherwise
    """
    if name.startswith("_"):
        return "private"
    return "public"


def extract_docstring(node: ast.AST) -> tuple[str, str]:
    """
    Extract brief and detailed descriptions from a docstring.

    Args:
        node: AST node that may have a docstring

    Returns:
        Tuple of (brief, detailed) where brief is first line, detailed is rest
    """
    docstring = ast.get_docstring(node)
    if not docstring:
        return "", ""

    lines = docstring.strip().split("\n", 1)
    brief = lines[0].strip()
    detailed = lines[1].strip() if len(lines) > 1 else ""

    return brief, detailed


def format_type_annotation(annotation: Optional[ast.AST]) -> str:
    """
    Convert an AST type annotation to a string.

    Args:
        annotation: Type annotation AST node

    Returns:
        String representation of the type, or empty string if None
    """
    if annotation is None:
        return ""

    try:
        return ast.unparse(annotation)
    except Exception:
        return ""


def format_decorator_list(decorators: list[ast.expr]) -> str:
    """
    Convert a list of decorator AST nodes to strings.

    Args:
        decorators: List of decorator AST nodes

    Returns:
        Space-separated decorator strings (e.g., "@router.get('/health') @staticmethod")
    """
    result = []
    for dec in decorators:
        try:
            result.append(f"@{ast.unparse(dec)}")
        except Exception:
            result.append("@<unknown>")
    return " ".join(result)


def index_file(
    conn: sqlite3.Connection,
    file_path: Path,
    module_prefix: str,
    project_root: Path
) -> None:
    """
    Index a single Python file into the database.

    Args:
        conn: Database connection
        file_path: Path to .py file
        module_prefix: Dot-notation module prefix (e.g., "replay.replay")
        project_root: Project root directory for relative path computation
    """
    try:
        source = file_path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(file_path))
    except (SyntaxError, UnicodeDecodeError) as e:
        print(f"Warning: Could not parse {file_path}: {e}", file=sys.stderr)
        return

    # Compute relative path from project root
    try:
        rel_path = file_path.relative_to(project_root)
    except ValueError:
        rel_path = file_path

    # Insert file
    file_refid = make_refid("file", str(rel_path).replace("/", "_").replace(".py", "_py"))
    cursor = conn.execute(
        "INSERT OR IGNORE INTO files (refid, name, path, language) VALUES (?, ?, ?, ?)",
        (file_refid, file_path.name, str(rel_path), "Python")
    )
    file_id = cursor.lastrowid or conn.execute(
        "SELECT id FROM files WHERE refid = ?", (file_refid,)
    ).fetchone()[0]

    # Module name from file path (e.g., replay/replay/models.py -> replay.replay.models)
    module_name = module_prefix

    # Track current class context for methods
    current_class_id: Optional[int] = None
    current_class_qualified_name: Optional[str] = None

    # Process imports
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported_module = alias.name
                is_local = 1 if imported_module.startswith(module_prefix.split(".")[0]) else 0
                conn.execute(
                    "INSERT OR IGNORE INTO includes (file_id, included_file, is_local) VALUES (?, ?, ?)",
                    (file_id, imported_module, is_local)
                )

        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported_module = node.module
                is_local = 1 if imported_module.startswith(module_prefix.split(".")[0]) else 0
                conn.execute(
                    "INSERT OR IGNORE INTO includes (file_id, included_file, is_local) VALUES (?, ?, ?)",
                    (file_id, imported_module, is_local)
                )

    # Process top-level classes and functions
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            # Index class
            class_qualified_name = f"{module_name}.{node.name}"
            class_refid = make_refid("class", class_qualified_name)

            # Extract base classes
            base_classes = []
            for base in node.bases:
                try:
                    base_classes.append(ast.unparse(base))
                except Exception:
                    base_classes.append("<unknown>")
            base_classes_str = ", ".join(base_classes) if base_classes else None

            # Extract docstring
            brief, detailed = extract_docstring(node)

            # Add decorator info to description if present
            if node.decorator_list:
                decorator_str = format_decorator_list(node.decorator_list)
                if detailed:
                    detailed = f"{decorator_str}\n\n{detailed}"
                elif brief:
                    detailed = decorator_str
                else:
                    brief = decorator_str

            cursor = conn.execute(
                """INSERT OR IGNORE INTO compounds
                   (refid, kind, name, qualified_name, file_id, line_number,
                    brief_description, detailed_description, base_classes)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (class_refid, "class", node.name, class_qualified_name, file_id,
                 node.lineno, brief, detailed, base_classes_str)
            )
            class_id = cursor.lastrowid or conn.execute(
                "SELECT id FROM compounds WHERE refid = ?", (class_refid,)
            ).fetchone()[0]

            # Index FTS
            full_desc = f"{brief} {detailed}".strip()
            if full_desc:
                conn.execute(
                    "INSERT OR IGNORE INTO fts_docs (name, qualified_name, description) VALUES (?, ?, ?)",
                    (node.name, class_qualified_name, full_desc)
                )

            # Index class members (methods and fields)
            for item in node.body:
                if isinstance(item, ast.FunctionDef):
                    # Method
                    method_qualified_name = f"{class_qualified_name}.{item.name}"
                    method_refid = make_refid("func", method_qualified_name)

                    # Format arguments
                    args = item.args
                    arg_strs = []
                    for i, arg in enumerate(args.args):
                        if i == 0 and arg.arg == "self":
                            continue
                        type_str = format_type_annotation(arg.annotation)
                        if type_str:
                            arg_strs.append(f"{arg.arg}: {type_str}")
                        else:
                            arg_strs.append(arg.arg)

                    argsstring = f"({', '.join(arg_strs)})"
                    return_type = format_type_annotation(item.returns)

                    brief_m, detailed_m = extract_docstring(item)

                    # Add decorator info
                    if item.decorator_list:
                        decorator_str = format_decorator_list(item.decorator_list)
                        if detailed_m:
                            detailed_m = f"{decorator_str}\n\n{detailed_m}"
                        elif brief_m:
                            detailed_m = decorator_str
                        else:
                            brief_m = decorator_str

                    protection = get_protection(item.name)

                    cursor_m = conn.execute(
                        """INSERT OR IGNORE INTO members
                           (refid, compound_id, kind, name, qualified_name, type, argsstring,
                            file_id, line_number, brief_description, detailed_description, protection)
                           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                        (method_refid, class_id, "function", item.name, method_qualified_name,
                         return_type, argsstring, file_id, item.lineno, brief_m, detailed_m, protection)
                    )
                    member_id = cursor_m.lastrowid or conn.execute(
                        "SELECT id FROM members WHERE refid = ?", (method_refid,)
                    ).fetchone()[0]

                    # Index parameters
                    param_offset = 1 if args.args and args.args[0].arg == "self" else 0
                    for i, arg in enumerate(args.args[param_offset:], start=1):
                        type_str = format_type_annotation(arg.annotation)
                        default_val = None
                        # Check if this parameter has a default
                        default_offset = len(args.args) - len(args.defaults)
                        arg_idx = i - 1 + param_offset
                        if arg_idx >= default_offset:
                            default_node = args.defaults[arg_idx - default_offset]
                            try:
                                default_val = ast.unparse(default_node)
                            except Exception:
                                default_val = "<unknown>"

                        conn.execute(
                            """INSERT OR IGNORE INTO parameters
                               (member_id, position, name, type, default_value)
                               VALUES (?, ?, ?, ?, ?)""",
                            (member_id, i, arg.arg, type_str or "Any", default_val)
                        )

                    # Index FTS
                    full_desc_m = f"{brief_m} {detailed_m}".strip()
                    if full_desc_m:
                        conn.execute(
                            "INSERT OR IGNORE INTO fts_docs (name, qualified_name, description) VALUES (?, ?, ?)",
                            (item.name, method_qualified_name, full_desc_m)
                        )

                elif isinstance(item, ast.AnnAssign):
                    # Class field with type annotation
                    if isinstance(item.target, ast.Name):
                        field_name = item.target.id
                        field_qualified_name = f"{class_qualified_name}.{field_name}"
                        field_refid = make_refid("var", field_qualified_name)
                        field_type = format_type_annotation(item.annotation)
                        protection = get_protection(field_name)

                        conn.execute(
                            """INSERT OR IGNORE INTO members
                               (refid, compound_id, kind, name, qualified_name, type,
                                file_id, line_number, protection)
                               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                            (field_refid, class_id, "variable", field_name, field_qualified_name,
                             field_type, file_id, item.lineno, protection)
                        )

        elif isinstance(node, ast.FunctionDef):
            # Top-level function (module-level)
            func_qualified_name = f"{module_name}.{node.name}"
            func_refid = make_refid("func", func_qualified_name)

            # Format arguments
            args = node.args
            arg_strs = []
            for arg in args.args:
                type_str = format_type_annotation(arg.annotation)
                if type_str:
                    arg_strs.append(f"{arg.arg}: {type_str}")
                else:
                    arg_strs.append(arg.arg)

            argsstring = f"({', '.join(arg_strs)})"
            return_type = format_type_annotation(node.returns)

            brief_f, detailed_f = extract_docstring(node)

            # Add decorator info
            if node.decorator_list:
                decorator_str = format_decorator_list(node.decorator_list)
                if detailed_f:
                    detailed_f = f"{decorator_str}\n\n{detailed_f}"
                elif brief_f:
                    detailed_f = decorator_str
                else:
                    brief_f = decorator_str

            protection = get_protection(node.name)

            cursor_f = conn.execute(
                """INSERT OR IGNORE INTO members
                   (refid, kind, name, qualified_name, type, argsstring,
                    file_id, line_number, brief_description, detailed_description, protection)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (func_refid, "function", node.name, func_qualified_name,
                 return_type, argsstring, file_id, node.lineno, brief_f, detailed_f, protection)
            )
            member_id = cursor_f.lastrowid or conn.execute(
                "SELECT id FROM members WHERE refid = ?", (func_refid,)
            ).fetchone()[0]

            # Index parameters
            for i, arg in enumerate(args.args, start=1):
                type_str = format_type_annotation(arg.annotation)
                default_val = None
                # Check if this parameter has a default
                default_offset = len(args.args) - len(args.defaults)
                if i - 1 >= default_offset:
                    default_node = args.defaults[i - 1 - default_offset]
                    try:
                        default_val = ast.unparse(default_node)
                    except Exception:
                        default_val = "<unknown>"

                conn.execute(
                    """INSERT OR IGNORE INTO parameters
                       (member_id, position, name, type, default_value)
                       VALUES (?, ?, ?, ?, ?)""",
                    (member_id, i, arg.arg, type_str or "Any", default_val)
                )

            # Index FTS
            full_desc_f = f"{brief_f} {detailed_f}".strip()
            if full_desc_f:
                conn.execute(
                    "INSERT OR IGNORE INTO fts_docs (name, qualified_name, description) VALUES (?, ?, ?)",
                    (node.name, func_qualified_name, full_desc_f)
                )

        elif isinstance(node, ast.AnnAssign):
            # Module-level variable with type annotation
            if isinstance(node.target, ast.Name):
                var_name = node.target.id
                var_qualified_name = f"{module_name}.{var_name}"
                var_refid = make_refid("var", var_qualified_name)
                var_type = format_type_annotation(node.annotation)
                protection = get_protection(var_name)

                conn.execute(
                    """INSERT OR IGNORE INTO members
                       (refid, kind, name, qualified_name, type,
                        file_id, line_number, protection)
                       VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
                    (var_refid, "variable", var_name, var_qualified_name,
                     var_type, file_id, node.lineno, protection)
                )

    conn.commit()


def walk_python_files(source_dir: Path) -> list[Path]:
    """
    Recursively walk a directory and collect all .py files.

    Skips:
        - __pycache__ directories
        - .venv directories
        - .pyc files

    Args:
        source_dir: Root directory to walk

    Returns:
        List of Python file paths
    """
    python_files = []
    for root, dirs, files in os.walk(source_dir):
        # Skip virtual environments and cache directories
        dirs[:] = [d for d in dirs if d not in ("__pycache__", ".venv", "venv")]

        for file in files:
            if file.endswith(".py") and not file.endswith(".pyc"):
                python_files.append(Path(root) / file)

    return python_files


def compute_module_prefix(file_path: Path, source_dir: Path) -> str:
    """
    Compute the Python module prefix for a file.

    Example:
        file_path = Path("replay/replay/models.py")
        source_dir = Path("replay/replay")
        returns "replay.replay.models"

    Args:
        file_path: Path to the Python file
        source_dir: Root source directory

    Returns:
        Dot-notation module name
    """
    try:
        rel_path = file_path.relative_to(source_dir.parent)
    except ValueError:
        rel_path = file_path

    module_parts = list(rel_path.parts)
    # Remove .py extension from last part
    if module_parts[-1].endswith(".py"):
        module_parts[-1] = module_parts[-1][:-3]

    # Remove __init__ if present
    if module_parts[-1] == "__init__":
        module_parts.pop()

    return ".".join(module_parts)


def main():
    parser = argparse.ArgumentParser(
        description="Index Python source files into an existing codebase.db"
    )
    parser.add_argument(
        "python_source_dir",
        type=Path,
        help="Directory containing Python source files to index"
    )
    parser.add_argument(
        "existing_db_path",
        type=Path,
        help="Path to existing codebase.db (created by doxygen-db target)"
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        default=Path.cwd(),
        help="Project root directory for computing relative paths (default: current directory)"
    )

    args = parser.parse_args()

    source_dir = args.python_source_dir.resolve()
    db_path = args.existing_db_path.resolve()
    project_root = args.project_root.resolve()

    if not source_dir.exists():
        print(f"Error: Source directory does not exist: {source_dir}", file=sys.stderr)
        sys.exit(1)

    if not db_path.exists():
        print(f"Error: Database does not exist: {db_path}", file=sys.stderr)
        print("Hint: Run 'cmake --build --preset doxygen-db' first to create the database", file=sys.stderr)
        sys.exit(1)

    # Open existing database
    conn = sqlite3.connect(db_path)

    # Defensively create schema (no-op if already exists)
    create_schema(conn)

    # Index all Python files
    python_files = walk_python_files(source_dir)
    print(f"Found {len(python_files)} Python files to index")

    for file_path in python_files:
        module_prefix = compute_module_prefix(file_path, source_dir)
        print(f"Indexing {file_path} as {module_prefix}")
        index_file(conn, file_path, module_prefix, project_root)

    # Update metadata
    conn.execute(
        "INSERT OR REPLACE INTO metadata (key, value) VALUES (?, ?)",
        ("python_indexed", "true")
    )
    conn.commit()

    # Report statistics
    file_count = conn.execute("SELECT COUNT(*) FROM files WHERE language='Python'").fetchone()[0]
    class_count = conn.execute("SELECT COUNT(*) FROM compounds WHERE kind='class'").fetchone()[0]
    func_count = conn.execute("SELECT COUNT(*) FROM members WHERE kind='function'").fetchone()[0]

    print(f"\nIndexing complete:")
    print(f"  Python files: {file_count}")
    print(f"  Classes: {class_count}")
    print(f"  Functions: {func_count}")

    conn.close()


if __name__ == "__main__":
    main()
