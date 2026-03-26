#!/usr/bin/env python3
"""
Python AST to Neo4j Graph Database Ingester

Parses Python source files using the ast module and inserts symbols into
the existing Neo4j codebase graph. Python symbols use a py_ refid prefix
to avoid collision with C++ (Doxygen) refids.

Designed to run after doxygen_to_neo4j.py — adds Python symbols to the
same graph so both languages are searchable together.

Usage:
    python python_to_neo4j.py <python_source_dir> [options]

Example:
    python python_to_neo4j.py replay/replay
    python python_to_neo4j.py replay/replay --uri bolt://localhost:7687

Ticket: 0064_python_codebase_documentation_index
"""

import argparse
import ast
import os
import sys
from pathlib import Path
from typing import Optional

from neo4j import GraphDatabase


# ---------------------------------------------------------------------------
# Refid and naming helpers (same logic as python_to_sqlite.py)
# ---------------------------------------------------------------------------

def sanitize_for_refid(name: str) -> str:
    """Convert a Python qualified name to a refid-safe string."""
    return name.replace(".", "_").replace("/", "_")


def make_refid(prefix: str, qualified_name: str) -> str:
    """Generate a Python refid with py_ prefix to avoid collision with C++ refids."""
    return f"py_{prefix}_{sanitize_for_refid(qualified_name)}"


def get_protection(name: str) -> str:
    """Map Python naming conventions to protection levels."""
    if name.startswith("_"):
        return "private"
    return "public"


def extract_docstring(node: ast.AST) -> tuple[str, str]:
    """Extract brief and detailed descriptions from a docstring."""
    docstring = ast.get_docstring(node)
    if not docstring:
        return "", ""
    lines = docstring.strip().split("\n", 1)
    brief = lines[0].strip()
    detailed = lines[1].strip() if len(lines) > 1 else ""
    return brief, detailed


def format_type_annotation(annotation: Optional[ast.AST]) -> str:
    """Convert an AST type annotation to a string."""
    if annotation is None:
        return ""
    try:
        return ast.unparse(annotation)
    except Exception:
        return ""


def format_decorator_list(decorators: list[ast.expr]) -> str:
    """Convert a list of decorator AST nodes to strings."""
    result = []
    for dec in decorators:
        try:
            result.append(f"@{ast.unparse(dec)}")
        except Exception:
            result.append("@<unknown>")
    return " ".join(result)


# ---------------------------------------------------------------------------
# Batch collector for Python symbols
# ---------------------------------------------------------------------------

class PythonBatchWriter:
    """Collects parsed Python AST data and writes to Neo4j in batches."""

    def __init__(self, driver, database: str = "neo4j"):
        self.driver = driver
        self.database = database

        self.files: list[dict] = []
        self.namespaces: list[dict] = []
        self.compounds: list[dict] = []
        self.members: list[dict] = []
        self.parameters: list[dict] = []
        self.imports: list[dict] = []

    def clear_python_data(self):
        """Remove existing Python nodes (refid starts with py_)."""
        with self.driver.session(database=self.database) as session:
            session.run("MATCH (p:Parameter)<-[:HAS_PARAMETER]-(m:Member) WHERE m.refid STARTS WITH 'py_' DETACH DELETE p")
            session.run("MATCH (m:Member) WHERE m.refid STARTS WITH 'py_' DETACH DELETE m")
            session.run("MATCH (c:Compound) WHERE c.refid STARTS WITH 'py_' DETACH DELETE c")
            session.run("MATCH (n:Namespace) WHERE n.refid STARTS WITH 'py_' DETACH DELETE n")
            session.run("MATCH (f:File) WHERE f.refid STARTS WITH 'py_' DETACH DELETE f")
        print("Cleared existing Python data from Neo4j.")

    def add_file(self, refid: str, name: str, path: str):
        self.files.append({
            "refid": refid,
            "name": name,
            "path": path,
            "language": "Python",
        })

    def add_namespace(self, refid: str, name: str, qualified_name: str):
        self.namespaces.append({
            "refid": refid,
            "name": name,
            "qualified_name": qualified_name,
        })

    def add_compound(self, refid: str, name: str, qualified_name: str,
                     file_refid: str, line_number: int, brief: str,
                     detailed: str, base_classes: list[str]):
        self.compounds.append({
            "refid": refid,
            "kind": "class",
            "name": name,
            "qualified_name": qualified_name,
            "file_refid": file_refid,
            "file_path": "",  # set via relationship
            "line_number": line_number,
            "brief_description": brief,
            "detailed_description": detailed,
            "base_classes": base_classes,
            "is_final": False,
            "is_abstract": False,
        })

    def add_member(self, refid: str, compound_refid: str, kind: str,
                   name: str, qualified_name: str, type_str: str,
                   argsstring: str, file_refid: str, line_number: int,
                   brief: str, detailed: str, protection: str):
        self.members.append({
            "refid": refid,
            "compound_refid": compound_refid,
            "kind": kind,
            "name": name,
            "qualified_name": qualified_name,
            "type": type_str,
            "definition": "",
            "argsstring": argsstring,
            "file_path": "",
            "line_number": line_number,
            "brief_description": brief,
            "detailed_description": detailed,
            "protection": protection,
            "is_static": False,
            "is_const": False,
            "is_constexpr": False,
            "is_virtual": False,
            "is_inline": False,
            "is_explicit": False,
            "file_refid": file_refid,
        })

    def add_parameter(self, member_refid: str, position: int, name: str,
                      type_str: str, default_value: str):
        self.parameters.append({
            "member_refid": member_refid,
            "position": position,
            "name": name,
            "type": type_str,
            "default_value": default_value,
        })

    def add_import(self, file_refid: str, imported_module: str, is_local: bool):
        self.imports.append({
            "file_refid": file_refid,
            "included_file": imported_module,
            "is_local": is_local,
        })

    def flush(self):
        """Write all collected data to Neo4j."""
        with self.driver.session(database=self.database) as session:
            self._write_files(session)
            self._write_namespaces(session)
            self._write_compounds(session)
            self._write_members(session)
            self._write_parameters(session)
            self._write_relationships(session)

    def _write_files(self, session):
        if not self.files:
            return
        session.run(
            """
            UNWIND $batch AS row
            CREATE (f:File {
                refid: row.refid,
                name: row.name,
                path: row.path,
                language: row.language
            })
            """,
            batch=self.files,
        )
        print(f"  Files: {len(self.files)}")

    def _write_namespaces(self, session):
        if not self.namespaces:
            return
        session.run(
            """
            UNWIND $batch AS row
            MERGE (n:Namespace {refid: row.refid})
            SET n.name = row.name, n.qualified_name = row.qualified_name
            """,
            batch=self.namespaces,
        )
        print(f"  Namespaces: {len(self.namespaces)}")

    def _write_compounds(self, session):
        if not self.compounds:
            return
        session.run(
            """
            UNWIND $batch AS row
            CREATE (c:Compound {
                refid: row.refid,
                kind: row.kind,
                name: row.name,
                qualified_name: row.qualified_name,
                file_path: row.file_path,
                line_number: row.line_number,
                brief_description: row.brief_description,
                detailed_description: row.detailed_description,
                base_classes: row.base_classes,
                is_final: row.is_final,
                is_abstract: row.is_abstract
            })
            """,
            batch=self.compounds,
        )
        print(f"  Compounds: {len(self.compounds)}")

    def _write_members(self, session):
        if not self.members:
            return
        batch_size = 1000
        for i in range(0, len(self.members), batch_size):
            batch = self.members[i:i + batch_size]
            session.run(
                """
                UNWIND $batch AS row
                CREATE (m:Member {
                    refid: row.refid,
                    compound_refid: row.compound_refid,
                    kind: row.kind,
                    name: row.name,
                    qualified_name: row.qualified_name,
                    type: row.type,
                    definition: row.definition,
                    argsstring: row.argsstring,
                    file_path: row.file_path,
                    line_number: row.line_number,
                    brief_description: row.brief_description,
                    detailed_description: row.detailed_description,
                    protection: row.protection,
                    is_static: row.is_static,
                    is_const: row.is_const,
                    is_constexpr: row.is_constexpr,
                    is_virtual: row.is_virtual,
                    is_inline: row.is_inline,
                    is_explicit: row.is_explicit
                })
                """,
                batch=batch,
            )
        print(f"  Members: {len(self.members)}")

    def _write_parameters(self, session):
        if not self.parameters:
            return
        batch_size = 1000
        for i in range(0, len(self.parameters), batch_size):
            batch = self.parameters[i:i + batch_size]
            session.run(
                """
                UNWIND $batch AS row
                MATCH (m:Member {refid: row.member_refid})
                CREATE (p:Parameter {
                    position: row.position,
                    name: row.name,
                    type: row.type,
                    default_value: row.default_value
                })
                CREATE (m)-[:HAS_PARAMETER]->(p)
                """,
                batch=batch,
            )
        print(f"  Parameters: {len(self.parameters)}")

    def _write_relationships(self, session):
        """Create DEFINED_IN and CONTAINS relationships."""
        # Compound -> File (via file_refid stored in compounds)
        if self.compounds:
            compound_file_links = [
                {"compound_refid": c["refid"], "file_refid": c["file_refid"]}
                for c in self.compounds if c.get("file_refid")
            ]
            if compound_file_links:
                session.run(
                    """
                    UNWIND $batch AS row
                    MATCH (c:Compound {refid: row.compound_refid})
                    MATCH (f:File {refid: row.file_refid})
                    CREATE (c)-[:DEFINED_IN]->(f)
                    """,
                    batch=compound_file_links,
                )

        # Member -> Compound (via compound_refid)
        session.run(
            """
            MATCH (m:Member) WHERE m.refid STARTS WITH 'py_' AND m.compound_refid <> ''
            MATCH (c:Compound {refid: m.compound_refid})
            CREATE (c)-[:CONTAINS]->(m)
            """
        )

        # Member -> File (via file_refid)
        if self.members:
            member_file_links = [
                {"member_refid": m["refid"], "file_refid": m["file_refid"]}
                for m in self.members if m.get("file_refid")
            ]
            if member_file_links:
                batch_size = 1000
                for i in range(0, len(member_file_links), batch_size):
                    batch = member_file_links[i:i + batch_size]
                    session.run(
                        """
                        UNWIND $batch AS row
                        MATCH (m:Member {refid: row.member_refid})
                        MATCH (f:File {refid: row.file_refid})
                        CREATE (m)-[:DEFINED_IN]->(f)
                        """,
                        batch=batch,
                    )

        print("  Relationships: DEFINED_IN, CONTAINS")


# ---------------------------------------------------------------------------
# AST indexing — walks Python files and feeds the batch writer
# ---------------------------------------------------------------------------

def index_file(writer: PythonBatchWriter, file_path: Path,
               module_prefix: str, project_root: Path) -> None:
    """Index a single Python file."""
    try:
        source = file_path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(file_path))
    except (SyntaxError, UnicodeDecodeError) as e:
        print(f"Warning: Could not parse {file_path}: {e}", file=sys.stderr)
        return

    try:
        rel_path = str(file_path.relative_to(project_root))
    except ValueError:
        rel_path = str(file_path)

    # File node
    file_refid = make_refid("file", str(rel_path).replace("/", "_").replace(".py", "_py"))
    writer.add_file(file_refid, file_path.name, rel_path)

    module_name = module_prefix

    # Namespace for the package
    package_parts = module_name.rsplit(".", 1)
    if len(package_parts) > 1:
        package_name = package_parts[0]
        ns_refid = make_refid("ns", package_name)
        writer.add_namespace(ns_refid, package_parts[0].split(".")[-1], package_name)

    # Process imports
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                is_local = alias.name.startswith(module_prefix.split(".")[0])
                writer.add_import(file_refid, alias.name, is_local)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                is_local = node.module.startswith(module_prefix.split(".")[0])
                writer.add_import(file_refid, node.module, is_local)

    # Process top-level definitions
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            _index_class(writer, node, module_name, file_refid)
        elif isinstance(node, ast.FunctionDef):
            _index_function(writer, node, module_name, file_refid, compound_refid="")
        elif isinstance(node, ast.AnnAssign):
            _index_variable(writer, node, module_name, file_refid, compound_refid="")


def _index_class(writer: PythonBatchWriter, node: ast.ClassDef,
                 module_name: str, file_refid: str) -> None:
    """Index a class definition."""
    class_qualified_name = f"{module_name}.{node.name}"
    class_refid = make_refid("class", class_qualified_name)

    base_classes = []
    for base in node.bases:
        try:
            base_classes.append(ast.unparse(base))
        except Exception:
            base_classes.append("<unknown>")

    brief, detailed = extract_docstring(node)
    if node.decorator_list:
        decorator_str = format_decorator_list(node.decorator_list)
        if detailed:
            detailed = f"{decorator_str}\n\n{detailed}"
        elif brief:
            detailed = decorator_str
        else:
            brief = decorator_str

    writer.add_compound(
        class_refid, node.name, class_qualified_name,
        file_refid, node.lineno, brief, detailed, base_classes,
    )

    # Index class body
    for item in node.body:
        if isinstance(item, ast.FunctionDef):
            _index_method(writer, item, class_qualified_name, class_refid, file_refid)
        elif isinstance(item, ast.AnnAssign):
            _index_variable(writer, item, class_qualified_name, file_refid, class_refid)


def _index_method(writer: PythonBatchWriter, node: ast.FunctionDef,
                  class_qualified_name: str, class_refid: str,
                  file_refid: str) -> None:
    """Index a method definition."""
    method_qualified_name = f"{class_qualified_name}.{node.name}"
    method_refid = make_refid("func", method_qualified_name)

    args = node.args
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
    return_type = format_type_annotation(node.returns)

    brief, detailed = extract_docstring(node)
    if node.decorator_list:
        decorator_str = format_decorator_list(node.decorator_list)
        if detailed:
            detailed = f"{decorator_str}\n\n{detailed}"
        elif brief:
            detailed = decorator_str
        else:
            brief = decorator_str

    protection = get_protection(node.name)

    writer.add_member(
        method_refid, class_refid, "function", node.name, method_qualified_name,
        return_type, argsstring, file_refid, node.lineno, brief, detailed, protection,
    )

    # Parameters
    param_offset = 1 if args.args and args.args[0].arg == "self" else 0
    for i, arg in enumerate(args.args[param_offset:], start=1):
        type_str = format_type_annotation(arg.annotation)
        default_val = ""
        default_offset = len(args.args) - len(args.defaults)
        arg_idx = i - 1 + param_offset
        if arg_idx >= default_offset:
            default_node = args.defaults[arg_idx - default_offset]
            try:
                default_val = ast.unparse(default_node)
            except Exception:
                default_val = "<unknown>"

        writer.add_parameter(method_refid, i, arg.arg, type_str or "Any", default_val)


def _index_function(writer: PythonBatchWriter, node: ast.FunctionDef,
                    module_name: str, file_refid: str,
                    compound_refid: str) -> None:
    """Index a top-level function."""
    func_qualified_name = f"{module_name}.{node.name}"
    func_refid = make_refid("func", func_qualified_name)

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

    brief, detailed = extract_docstring(node)
    if node.decorator_list:
        decorator_str = format_decorator_list(node.decorator_list)
        if detailed:
            detailed = f"{decorator_str}\n\n{detailed}"
        elif brief:
            detailed = decorator_str
        else:
            brief = decorator_str

    protection = get_protection(node.name)

    writer.add_member(
        func_refid, compound_refid, "function", node.name, func_qualified_name,
        return_type, argsstring, file_refid, node.lineno, brief, detailed, protection,
    )

    # Parameters
    for i, arg in enumerate(args.args, start=1):
        type_str = format_type_annotation(arg.annotation)
        default_val = ""
        default_offset = len(args.args) - len(args.defaults)
        if i - 1 >= default_offset:
            default_node = args.defaults[i - 1 - default_offset]
            try:
                default_val = ast.unparse(default_node)
            except Exception:
                default_val = "<unknown>"

        writer.add_parameter(func_refid, i, arg.arg, type_str or "Any", default_val)


def _index_variable(writer: PythonBatchWriter, node: ast.AnnAssign,
                    parent_qualified_name: str, file_refid: str,
                    compound_refid: str) -> None:
    """Index a variable with type annotation."""
    if not isinstance(node.target, ast.Name):
        return

    var_name = node.target.id
    var_qualified_name = f"{parent_qualified_name}.{var_name}"
    var_refid = make_refid("var", var_qualified_name)
    var_type = format_type_annotation(node.annotation)
    protection = get_protection(var_name)

    writer.add_member(
        var_refid, compound_refid, "variable", var_name, var_qualified_name,
        var_type, "", file_refid, node.lineno, "", "", protection,
    )


# ---------------------------------------------------------------------------
# File walking
# ---------------------------------------------------------------------------

def walk_python_files(source_dir: Path) -> list[Path]:
    """Recursively collect all .py files, skipping __pycache__ and venvs."""
    python_files = []
    for root, dirs, files in os.walk(source_dir):
        dirs[:] = [d for d in dirs if d not in ("__pycache__", ".venv", "venv")]
        for file in files:
            if file.endswith(".py") and not file.endswith(".pyc"):
                python_files.append(Path(root) / file)
    return python_files


def compute_module_prefix(file_path: Path, source_dir: Path) -> str:
    """Compute dot-notation module name from file path."""
    try:
        rel_path = file_path.relative_to(source_dir.parent)
    except ValueError:
        rel_path = file_path

    module_parts = list(rel_path.parts)
    if module_parts[-1].endswith(".py"):
        module_parts[-1] = module_parts[-1][:-3]
    if module_parts[-1] == "__init__":
        module_parts.pop()

    return ".".join(module_parts)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Ingest Python source files into Neo4j graph database"
    )
    parser.add_argument("python_source_dir", type=Path,
                        help="Directory containing Python source files to index")
    parser.add_argument("--uri", default=os.environ.get("NEO4J_URI", "bolt://localhost:7687"),
                        help="Neo4j Bolt URI (default: bolt://localhost:7687)")
    parser.add_argument("--user", default=os.environ.get("NEO4J_USER", "neo4j"),
                        help="Neo4j username (default: neo4j)")
    parser.add_argument("--password", default=os.environ.get("NEO4J_PASSWORD", "msd-local-dev"),
                        help="Neo4j password (default: msd-local-dev)")
    parser.add_argument("--database", default="neo4j",
                        help="Neo4j database name (default: neo4j)")
    parser.add_argument("--project-root", type=Path, default=Path.cwd(),
                        help="Project root for relative paths (default: cwd)")
    parser.add_argument("--no-clear", action="store_true",
                        help="Don't clear existing Python data before ingesting")

    args = parser.parse_args()

    source_dir = args.python_source_dir.resolve()
    project_root = args.project_root.resolve()

    if not source_dir.exists():
        print(f"Error: Source directory does not exist: {source_dir}", file=sys.stderr)
        sys.exit(1)

    # Connect to Neo4j
    print(f"Connecting to Neo4j at {args.uri}...")
    driver = GraphDatabase.driver(args.uri, auth=(args.user, args.password))
    try:
        driver.verify_connectivity()
    except Exception as e:
        print(f"Error: Could not connect to Neo4j: {e}", file=sys.stderr)
        sys.exit(1)

    writer = PythonBatchWriter(driver, database=args.database)

    if not args.no_clear:
        writer.clear_python_data()

    # Index all Python files
    python_files = walk_python_files(source_dir)
    print(f"Found {len(python_files)} Python files to index")

    for file_path in python_files:
        module_prefix = compute_module_prefix(file_path, source_dir)
        print(f"  Indexing {file_path.name} as {module_prefix}")
        index_file(writer, file_path, module_prefix, project_root)

    # Flush to Neo4j
    print("\nWriting to Neo4j...")
    writer.flush()

    # Summary
    print("\nPython ingestion complete.")
    with driver.session(database=args.database) as session:
        result = session.run(
            """
            MATCH (n) WHERE n.refid STARTS WITH 'py_'
            WITH labels(n)[0] AS label
            RETURN label, count(*) AS cnt
            ORDER BY cnt DESC
            """
        )
        print("Python node counts:")
        for record in result:
            print(f"  {record['label']}: {record['cnt']}")

    driver.close()


if __name__ == "__main__":
    main()
