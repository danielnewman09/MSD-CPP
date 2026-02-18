#!/usr/bin/env python3
"""
Symbol Snapshot Indexer

For each snapshot in the traceability database, checks out the commit using
git worktree, parses all C++ headers/sources under msd/ with tree-sitter,
and records symbol locations. Computes symbol diffs between consecutive
snapshots.

Populates `symbol_snapshots` and `symbol_changes` tables.

Dependencies:
    pip install tree-sitter tree-sitter-cpp

Usage:
    python index_symbols.py <traceability_db_path> [--repo <repo_root>]
"""

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from traceability_schema import create_schema

try:
    import tree_sitter_cpp as tscpp
    from tree_sitter import Language, Parser
    HAS_TREE_SITTER = True
except ImportError:
    HAS_TREE_SITTER = False


def get_parser() -> "Parser":
    """Create and return a tree-sitter C++ parser."""
    cpp_language = Language(tscpp.language(), "cpp")
    parser = Parser()
    parser.set_language(cpp_language)
    return parser


def extract_symbols(parser: "Parser", file_path: Path, relative_path: str) -> list[dict]:
    """Extract class/struct/function symbols from a C++ file using tree-sitter.

    Args:
        parser: tree-sitter Parser instance.
        file_path: Absolute path to the file.
        relative_path: Relative path for storage.

    Returns:
        List of symbol dicts with keys: qualified_name, kind, file_path,
        line_number, signature, class_scope.
    """
    try:
        source = file_path.read_bytes()
    except (OSError, UnicodeDecodeError):
        return []

    tree = parser.parse(source)
    symbols = []

    _walk_node(tree.root_node, symbols, relative_path, namespace_stack=[], class_stack=[])

    return symbols


def _walk_node(node, symbols, file_path, namespace_stack, class_stack):
    """Recursively walk the AST and extract symbols."""
    for child in node.children:
        if child.type == "namespace_definition":
            ns_name = _get_namespace_name(child)
            if ns_name:
                namespace_stack.append(ns_name)
                body = _find_child(child, "declaration_list")
                if body:
                    _walk_node(body, symbols, file_path, namespace_stack, class_stack)
                namespace_stack.pop()
            else:
                # Anonymous namespace
                body = _find_child(child, "declaration_list")
                if body:
                    _walk_node(body, symbols, file_path, namespace_stack, class_stack)

        elif child.type in ("class_specifier", "struct_specifier"):
            name = _get_class_name(child)
            if name:
                kind = "class" if child.type == "class_specifier" else "struct"
                qualified = _qualify(namespace_stack, class_stack, name)
                symbols.append({
                    "qualified_name": qualified,
                    "kind": kind,
                    "file_path": file_path,
                    "line_number": child.start_point[0] + 1,
                    "signature": None,
                    "class_scope": "::".join(class_stack) if class_stack else None,
                })

                # Walk class body for methods
                body = _find_child(child, "field_declaration_list")
                if body:
                    class_stack.append(name)
                    _walk_node(body, symbols, file_path, namespace_stack, class_stack)
                    class_stack.pop()

        elif child.type == "function_definition":
            name, signature = _get_function_info(child)
            if name:
                # Check if this is a method definition (e.g., ClassName::methodName)
                class_scope, method_name = _split_qualified_function(name)
                if class_scope:
                    qualified = _qualify(namespace_stack, [], f"{class_scope}::{method_name}")
                else:
                    qualified = _qualify(namespace_stack, class_stack, name)

                symbols.append({
                    "qualified_name": qualified,
                    "kind": "method" if (class_stack or class_scope) else "function",
                    "file_path": file_path,
                    "line_number": child.start_point[0] + 1,
                    "signature": signature,
                    "class_scope": class_scope or ("::".join(class_stack) if class_stack else None),
                })

        elif child.type == "declaration":
            # Could be a function declaration inside a class
            decl = _find_child(child, "function_declarator")
            if decl and class_stack:
                name = _get_declarator_name(decl)
                signature = _get_text(child)
                if name:
                    qualified = _qualify(namespace_stack, class_stack, name)
                    symbols.append({
                        "qualified_name": qualified,
                        "kind": "method",
                        "file_path": file_path,
                        "line_number": child.start_point[0] + 1,
                        "signature": signature.strip() if signature else None,
                        "class_scope": "::".join(class_stack),
                    })
            else:
                # Recurse for nested declarations
                _walk_node(child, symbols, file_path, namespace_stack, class_stack)

        elif child.type == "template_declaration":
            _walk_node(child, symbols, file_path, namespace_stack, class_stack)

        else:
            _walk_node(child, symbols, file_path, namespace_stack, class_stack)


def _qualify(namespace_stack, class_stack, name):
    """Build qualified name from namespace and class stacks."""
    parts = list(namespace_stack) + list(class_stack) + [name]
    return "::".join(parts)


def _get_namespace_name(node):
    """Get the name from a namespace_definition node."""
    for child in node.children:
        if child.type in ("identifier", "namespace_identifier"):
            return _get_text(child)
    return None


def _get_class_name(node):
    """Get the name from a class/struct specifier."""
    for child in node.children:
        if child.type == "type_identifier":
            return _get_text(child)
    return None


def _get_function_info(node):
    """Get function name and signature from a function_definition."""
    declarator = _find_child(node, "function_declarator")
    if not declarator:
        declarator = _find_child(node, "qualified_identifier")
        if not declarator:
            return None, None

    name = _get_declarator_name(declarator)

    # Build signature from the full declaration minus the body
    sig_parts = []
    for child in node.children:
        if child.type == "compound_statement":
            break
        sig_parts.append(_get_text(child))
    signature = " ".join(sig_parts).strip()

    return name, signature


def _get_declarator_name(node):
    """Extract the simple name from a declarator node."""
    if node.type == "identifier":
        return _get_text(node)
    if node.type == "qualified_identifier":
        # Return the full qualified name
        return _get_text(node)
    if node.type == "field_identifier":
        return _get_text(node)
    if node.type == "destructor_name":
        return _get_text(node)

    for child in node.children:
        result = _get_declarator_name(child)
        if result:
            return result
    return None


def _split_qualified_function(name):
    """Split 'ClassName::methodName' into (ClassName, methodName).

    Returns (None, name) if not qualified.
    """
    if "::" in name:
        parts = name.rsplit("::", 1)
        return parts[0], parts[1]
    return None, name


def _find_child(node, child_type):
    """Find first child of a given type."""
    for child in node.children:
        if child.type == child_type:
            return child
    return None


def _get_text(node):
    """Get the text content of a node."""
    return node.text.decode("utf-8") if node.text else ""


def index_snapshot(
    parser, snapshot_id: int, sha: str, repo_root: str, conn
) -> dict[str, dict]:
    """Index symbols for a single commit snapshot.

    Uses git worktree to checkout the commit, then parses all C++ files.

    Returns:
        Dict mapping qualified_name -> symbol dict for diff computation.
    """
    worktree_dir = os.path.join(tempfile.gettempdir(), f"msd-trace-{sha[:8]}")

    try:
        # Create a detached worktree
        subprocess.run(
            ["git", "worktree", "add", worktree_dir, sha, "--detach"],
            capture_output=True, text=True, cwd=repo_root, check=True,
        )

        msd_dir = Path(worktree_dir) / "msd"
        if not msd_dir.exists():
            return {}

        # Find all C++ files under msd/
        cpp_files = list(msd_dir.rglob("*.hpp")) + list(msd_dir.rglob("*.cpp"))

        symbol_map = {}
        for cpp_file in cpp_files:
            relative = str(cpp_file.relative_to(worktree_dir))
            symbols = extract_symbols(parser, cpp_file, relative)

            for sym in symbols:
                conn.execute(
                    """INSERT INTO symbol_snapshots
                       (snapshot_id, qualified_name, kind, file_path, line_number, signature, class_scope)
                       VALUES (?, ?, ?, ?, ?, ?, ?)""",
                    (snapshot_id, sym["qualified_name"], sym["kind"],
                     sym["file_path"], sym["line_number"], sym["signature"],
                     sym["class_scope"]),
                )

                # For diff computation, store the latest location per name
                symbol_map[sym["qualified_name"]] = sym

        return symbol_map

    finally:
        # Clean up worktree
        subprocess.run(
            ["git", "worktree", "remove", worktree_dir, "--force"],
            capture_output=True, text=True, cwd=repo_root,
        )


def compute_symbol_changes(
    snapshot_id: int, prev_symbols: dict, curr_symbols: dict, conn
) -> None:
    """Compute and store symbol changes between two consecutive snapshots."""
    prev_names = set(prev_symbols.keys())
    curr_names = set(curr_symbols.keys())

    # Added symbols
    for name in curr_names - prev_names:
        sym = curr_symbols[name]
        conn.execute(
            """INSERT INTO symbol_changes
               (snapshot_id, qualified_name, change_type, new_file_path, new_line, new_signature)
               VALUES (?, ?, 'added', ?, ?, ?)""",
            (snapshot_id, name, sym["file_path"], sym["line_number"], sym["signature"]),
        )

    # Removed symbols
    for name in prev_names - curr_names:
        sym = prev_symbols[name]
        conn.execute(
            """INSERT INTO symbol_changes
               (snapshot_id, qualified_name, change_type, old_file_path, old_line, old_signature)
               VALUES (?, ?, 'removed', ?, ?, ?)""",
            (snapshot_id, name, sym["file_path"], sym["line_number"], sym["signature"]),
        )

    # Modified or moved symbols
    for name in prev_names & curr_names:
        prev = prev_symbols[name]
        curr = curr_symbols[name]

        moved = prev["file_path"] != curr["file_path"]
        modified = prev.get("signature") != curr.get("signature")
        line_changed = prev["line_number"] != curr["line_number"]

        if moved:
            change_type = "moved"
        elif modified:
            change_type = "modified"
        elif line_changed:
            # Only line number changed (e.g., code above was added/removed)
            # Skip these to reduce noise
            continue
        else:
            continue

        conn.execute(
            """INSERT INTO symbol_changes
               (snapshot_id, qualified_name, change_type,
                old_file_path, new_file_path, old_line, new_line,
                old_signature, new_signature)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (snapshot_id, name, change_type,
             prev["file_path"], curr["file_path"],
             prev["line_number"], curr["line_number"],
             prev.get("signature"), curr.get("signature")),
        )


def index_symbols(db_path: str, repo_root: str) -> None:
    """Index symbols for all snapshots in the traceability database.

    Args:
        db_path: Path to the traceability SQLite database.
        repo_root: Path to the git repository root.
    """
    if not HAS_TREE_SITTER:
        print("Error: tree-sitter and tree-sitter-cpp are required.", file=sys.stderr)
        print("Install with: pip install tree-sitter tree-sitter-cpp", file=sys.stderr)
        sys.exit(1)

    conn = create_schema(db_path)
    parser = get_parser()

    # Get already-indexed snapshot IDs
    indexed_ids = set()
    for row in conn.execute("SELECT DISTINCT snapshot_id FROM symbol_snapshots"):
        indexed_ids.add(row[0])

    # Get all snapshots in chronological order
    snapshots = conn.execute(
        "SELECT id, sha FROM snapshots ORDER BY date"
    ).fetchall()

    if not snapshots:
        print("No snapshots found. Run index_git_history.py first.")
        return

    prev_symbols: dict = {}
    new_count = 0

    for snapshot in snapshots:
        sid = snapshot["id"]
        sha = snapshot["sha"]

        if sid in indexed_ids:
            # Load existing symbols for diff computation against next snapshot
            rows = conn.execute(
                """SELECT qualified_name, kind, file_path, line_number, signature, class_scope
                   FROM symbol_snapshots WHERE snapshot_id = ?""",
                (sid,),
            ).fetchall()
            prev_symbols = {r["qualified_name"]: dict(r) for r in rows}
            continue

        print(f"  Indexing {sha[:8]}...", end="", flush=True)
        curr_symbols = index_snapshot(parser, sid, sha, repo_root, conn)

        if prev_symbols:
            compute_symbol_changes(sid, prev_symbols, curr_symbols, conn)

        conn.commit()
        prev_symbols = curr_symbols
        new_count += 1
        print(f" {len(curr_symbols)} symbols")

    total = conn.execute("SELECT COUNT(DISTINCT snapshot_id) FROM symbol_snapshots").fetchone()[0]
    changes = conn.execute("SELECT COUNT(*) FROM symbol_changes").fetchone()[0]
    print(f"Indexed {new_count} new snapshots ({total} total), {changes} total symbol changes")
    conn.close()


def main():
    parser = argparse.ArgumentParser(description="Index symbol snapshots into traceability database")
    parser.add_argument("database", help="Path to the traceability SQLite database")
    parser.add_argument("--repo", default=".", help="Path to the git repository root")

    args = parser.parse_args()

    if not Path(args.repo).joinpath(".git").exists():
        print(f"Error: {args.repo} is not a git repository", file=sys.stderr)
        sys.exit(1)

    index_symbols(args.database, args.repo)


if __name__ == "__main__":
    main()
