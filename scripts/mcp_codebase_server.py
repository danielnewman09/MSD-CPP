#!/usr/bin/env python3
"""
MCP Codebase Server

A Model Context Protocol (MCP) server that provides efficient codebase navigation
using the SQLite database generated from Doxygen XML output.

Usage:
    python mcp_codebase_server.py <database_path>

Example:
    python mcp_codebase_server.py build/Debug/docs/codebase.db

Tools provided:
    - search_symbols: Full-text search across all symbols
    - find_class: Find a class/struct by name
    - find_function: Find a function by name
    - get_class_hierarchy: Get inheritance hierarchy for a class
    - get_callers: Find all functions that call a given function
    - get_callees: Find all functions called by a given function
    - get_file_symbols: List all symbols defined in a file
    - get_includes: Get include dependencies for a file
    - get_class_members: Get all members of a class
    - search_documentation: Full-text search in documentation
"""

import argparse
import json
import sqlite3
import sys
from pathlib import Path
from typing import Any

try:
    from mcp.server.fastmcp import FastMCP
    HAS_MCP = True
except ImportError:
    HAS_MCP = False


class CodebaseServer:
    """MCP server for codebase navigation queries."""

    def __init__(self, db_path: str):
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row

    def close(self):
        """Close database connection."""
        self.conn.close()

    def _rows_to_dicts(self, rows: list[sqlite3.Row]) -> list[dict[str, Any]]:
        """Convert SQLite Row objects to dictionaries."""
        return [dict(row) for row in rows]

    # =========================================================================
    # Symbol Search Tools
    # =========================================================================

    def search_symbols(self, query: str, kind: str | None = None, limit: int = 20) -> list[dict]:
        """
        Full-text search across all symbols (classes, functions, variables).

        Args:
            query: Search term (supports wildcards with *)
            kind: Optional filter by kind (class, struct, function, variable, etc.)
            limit: Maximum number of results (default: 20)

        Returns:
            List of matching symbols with file location and description
        """
        # Search in FTS index
        fts_sql = """
            SELECT name, qualified_name, description
            FROM fts_docs
            WHERE fts_docs MATCH ?
            LIMIT ?
        """

        # Also search by name prefix for non-FTS matches
        prefix_sql = """
            SELECT DISTINCT
                COALESCE(m.name, c.name) as name,
                COALESCE(m.qualified_name, c.qualified_name) as qualified_name,
                COALESCE(m.kind, c.kind) as kind,
                COALESCE(m.brief_description, c.brief_description) as description,
                f.path as file_path,
                COALESCE(m.line_number, c.line_number) as line_number
            FROM (
                SELECT name, qualified_name, kind, brief_description, file_id, line_number
                FROM members WHERE name LIKE ? OR qualified_name LIKE ?
                UNION ALL
                SELECT name, qualified_name, kind, brief_description, file_id, line_number
                FROM compounds WHERE name LIKE ? OR qualified_name LIKE ?
            ) AS combined
            LEFT JOIN members m ON m.name = combined.name AND m.kind = combined.kind
            LEFT JOIN compounds c ON c.name = combined.name AND c.kind = combined.kind
            LEFT JOIN files f ON f.id = COALESCE(m.file_id, c.file_id)
        """

        # Simplified approach: search members and compounds directly
        if kind:
            sql = """
                SELECT 'member' as source, m.name, m.qualified_name, m.kind,
                       m.brief_description as description, f.path as file_path,
                       m.line_number, m.definition
                FROM members m
                LEFT JOIN files f ON f.id = m.file_id
                WHERE (m.name LIKE ? OR m.qualified_name LIKE ?) AND m.kind = ?
                UNION ALL
                SELECT 'compound' as source, c.name, c.qualified_name, c.kind,
                       c.brief_description as description, f.path as file_path,
                       c.line_number, NULL as definition
                FROM compounds c
                LEFT JOIN files f ON f.id = c.file_id
                WHERE (c.name LIKE ? OR c.qualified_name LIKE ?) AND c.kind = ?
                LIMIT ?
            """
            pattern = f"%{query}%"
            cursor = self.conn.execute(sql, (pattern, pattern, kind, pattern, pattern, kind, limit))
        else:
            sql = """
                SELECT 'member' as source, m.name, m.qualified_name, m.kind,
                       m.brief_description as description, f.path as file_path,
                       m.line_number, m.definition
                FROM members m
                LEFT JOIN files f ON f.id = m.file_id
                WHERE m.name LIKE ? OR m.qualified_name LIKE ?
                UNION ALL
                SELECT 'compound' as source, c.name, c.qualified_name, c.kind,
                       c.brief_description as description, f.path as file_path,
                       c.line_number, NULL as definition
                FROM compounds c
                LEFT JOIN files f ON f.id = c.file_id
                WHERE c.name LIKE ? OR c.qualified_name LIKE ?
                LIMIT ?
            """
            pattern = f"%{query}%"
            cursor = self.conn.execute(sql, (pattern, pattern, pattern, pattern, limit))

        return self._rows_to_dicts(cursor.fetchall())

    def find_class(self, name: str, exact: bool = False) -> list[dict]:
        """
        Find a class or struct by name.

        Args:
            name: Class/struct name to search for
            exact: If True, match exactly; if False, use pattern matching

        Returns:
            List of matching classes with their details
        """
        if exact:
            sql = """
                SELECT c.name, c.qualified_name, c.kind, c.brief_description,
                       c.detailed_description, c.base_classes, c.is_final, c.is_abstract,
                       f.path as file_path, c.line_number
                FROM compounds c
                LEFT JOIN files f ON f.id = c.file_id
                WHERE c.name = ? AND c.kind IN ('class', 'struct')
            """
            cursor = self.conn.execute(sql, (name,))
        else:
            sql = """
                SELECT c.name, c.qualified_name, c.kind, c.brief_description,
                       c.detailed_description, c.base_classes, c.is_final, c.is_abstract,
                       f.path as file_path, c.line_number
                FROM compounds c
                LEFT JOIN files f ON f.id = c.file_id
                WHERE (c.name LIKE ? OR c.qualified_name LIKE ?)
                      AND c.kind IN ('class', 'struct')
            """
            pattern = f"%{name}%"
            cursor = self.conn.execute(sql, (pattern, pattern))

        return self._rows_to_dicts(cursor.fetchall())

    def find_function(self, name: str, class_name: str | None = None, exact: bool = False) -> list[dict]:
        """
        Find a function by name.

        Args:
            name: Function name to search for
            class_name: Optional class name to scope the search
            exact: If True, match exactly; if False, use pattern matching

        Returns:
            List of matching functions with their signatures and locations
        """
        if class_name:
            if exact:
                sql = """
                    SELECT m.name, m.qualified_name, m.type, m.definition, m.argsstring,
                           m.brief_description, m.protection, m.is_static, m.is_const,
                           m.is_virtual, m.is_constexpr, f.path as file_path, m.line_number,
                           c.name as class_name
                    FROM members m
                    LEFT JOIN compounds c ON c.id = m.compound_id
                    LEFT JOIN files f ON f.id = m.file_id
                    WHERE m.name = ? AND c.name = ? AND m.kind = 'function'
                """
                cursor = self.conn.execute(sql, (name, class_name))
            else:
                sql = """
                    SELECT m.name, m.qualified_name, m.type, m.definition, m.argsstring,
                           m.brief_description, m.protection, m.is_static, m.is_const,
                           m.is_virtual, m.is_constexpr, f.path as file_path, m.line_number,
                           c.name as class_name
                    FROM members m
                    LEFT JOIN compounds c ON c.id = m.compound_id
                    LEFT JOIN files f ON f.id = m.file_id
                    WHERE m.name LIKE ? AND c.name LIKE ? AND m.kind = 'function'
                """
                cursor = self.conn.execute(sql, (f"%{name}%", f"%{class_name}%"))
        else:
            if exact:
                sql = """
                    SELECT m.name, m.qualified_name, m.type, m.definition, m.argsstring,
                           m.brief_description, m.protection, m.is_static, m.is_const,
                           m.is_virtual, m.is_constexpr, f.path as file_path, m.line_number,
                           c.name as class_name
                    FROM members m
                    LEFT JOIN compounds c ON c.id = m.compound_id
                    LEFT JOIN files f ON f.id = m.file_id
                    WHERE m.name = ? AND m.kind = 'function'
                """
                cursor = self.conn.execute(sql, (name,))
            else:
                sql = """
                    SELECT m.name, m.qualified_name, m.type, m.definition, m.argsstring,
                           m.brief_description, m.protection, m.is_static, m.is_const,
                           m.is_virtual, m.is_constexpr, f.path as file_path, m.line_number,
                           c.name as class_name
                    FROM members m
                    LEFT JOIN compounds c ON c.id = m.compound_id
                    LEFT JOIN files f ON f.id = m.file_id
                    WHERE (m.name LIKE ? OR m.qualified_name LIKE ?) AND m.kind = 'function'
                """
                pattern = f"%{name}%"
                cursor = self.conn.execute(sql, (pattern, pattern))

        return self._rows_to_dicts(cursor.fetchall())

    # =========================================================================
    # Hierarchy and Relationship Tools
    # =========================================================================

    def get_class_hierarchy(self, class_name: str) -> dict:
        """
        Get the inheritance hierarchy for a class.

        Args:
            class_name: Name of the class to get hierarchy for

        Returns:
            Dictionary with base classes and derived classes
        """
        # Get the class and its base classes
        sql = """
            SELECT c.name, c.qualified_name, c.base_classes, c.kind,
                   f.path as file_path, c.line_number
            FROM compounds c
            LEFT JOIN files f ON f.id = c.file_id
            WHERE c.name = ? AND c.kind IN ('class', 'struct')
        """
        cursor = self.conn.execute(sql, (class_name,))
        class_info = cursor.fetchone()

        if not class_info:
            return {"error": f"Class '{class_name}' not found"}

        result = dict(class_info)

        # Parse base classes from JSON string
        if result.get("base_classes"):
            try:
                result["base_classes"] = eval(result["base_classes"])  # Stored as Python list repr
            except (ValueError, SyntaxError):
                pass

        # Find derived classes (classes that have this class as a base)
        # Use qualified_name for matching since base_classes stores qualified names
        # e.g., base_classes = "['msd_sim::Constraint']" requires matching 'msd_sim::Constraint'
        qualified_name = result["qualified_name"]
        sql = """
            SELECT c.name, c.qualified_name, c.kind, f.path as file_path, c.line_number
            FROM compounds c
            LEFT JOIN files f ON f.id = c.file_id
            WHERE c.base_classes LIKE ? AND c.kind IN ('class', 'struct')
        """
        cursor = self.conn.execute(sql, (f"%'{qualified_name}'%",))
        result["derived_classes"] = self._rows_to_dicts(cursor.fetchall())

        return result

    def get_callers(self, function_name: str, class_name: str | None = None) -> list[dict]:
        """
        Find all functions that call a given function.

        Args:
            function_name: Name of the function to find callers for
            class_name: Optional class name to scope the search

        Returns:
            List of calling functions with their locations
        """
        if class_name:
            # Find the specific function first
            sql = """
                SELECT m.refid
                FROM members m
                LEFT JOIN compounds c ON c.id = m.compound_id
                WHERE m.name = ? AND c.name = ? AND m.kind = 'function'
            """
            cursor = self.conn.execute(sql, (function_name, class_name))
        else:
            sql = """
                SELECT refid FROM members WHERE name = ? AND kind = 'function'
            """
            cursor = self.conn.execute(sql, (function_name,))

        target_refids = [row[0] for row in cursor.fetchall()]

        if not target_refids:
            return []

        # Find callers using symbol_refs
        placeholders = ",".join("?" * len(target_refids))
        sql = f"""
            SELECT DISTINCT
                caller.name as caller_name,
                caller.qualified_name as caller_qualified_name,
                caller.definition,
                f.path as file_path,
                caller.line_number,
                c.name as class_name
            FROM symbol_refs sr
            JOIN members caller ON caller.id = sr.from_member_id
            LEFT JOIN compounds c ON c.id = caller.compound_id
            LEFT JOIN files f ON f.id = caller.file_id
            WHERE sr.to_member_refid IN ({placeholders}) AND sr.relationship = 'called_by'
        """
        cursor = self.conn.execute(sql, target_refids)
        return self._rows_to_dicts(cursor.fetchall())

    def get_callees(self, function_name: str, class_name: str | None = None) -> list[dict]:
        """
        Find all functions called by a given function.

        Args:
            function_name: Name of the function to find callees for
            class_name: Optional class name to scope the search

        Returns:
            List of called functions
        """
        if class_name:
            sql = """
                SELECT DISTINCT
                    sr.to_member_name as callee_name,
                    sr.to_member_refid as callee_refid,
                    m2.qualified_name as callee_qualified_name,
                    m2.definition as callee_definition,
                    f2.path as callee_file_path,
                    m2.line_number as callee_line_number
                FROM symbol_refs sr
                JOIN members m ON m.id = sr.from_member_id
                JOIN compounds c ON c.id = m.compound_id
                LEFT JOIN members m2 ON m2.refid = sr.to_member_refid
                LEFT JOIN files f2 ON f2.id = m2.file_id
                WHERE m.name = ? AND c.name = ? AND sr.relationship = 'calls'
            """
            cursor = self.conn.execute(sql, (function_name, class_name))
        else:
            sql = """
                SELECT DISTINCT
                    sr.to_member_name as callee_name,
                    sr.to_member_refid as callee_refid,
                    m2.qualified_name as callee_qualified_name,
                    m2.definition as callee_definition,
                    f2.path as callee_file_path,
                    m2.line_number as callee_line_number
                FROM symbol_refs sr
                JOIN members m ON m.id = sr.from_member_id
                LEFT JOIN members m2 ON m2.refid = sr.to_member_refid
                LEFT JOIN files f2 ON f2.id = m2.file_id
                WHERE m.name = ? AND sr.relationship = 'calls'
            """
            cursor = self.conn.execute(sql, (function_name,))

        return self._rows_to_dicts(cursor.fetchall())

    # =========================================================================
    # File Navigation Tools
    # =========================================================================

    def get_file_symbols(self, file_path: str) -> dict:
        """
        List all symbols defined in a file.

        Args:
            file_path: Path to the file (can be partial match)

        Returns:
            Dictionary with classes, functions, and variables in the file
        """
        # Find the file
        sql = """
            SELECT id, name, path FROM files WHERE path LIKE ?
        """
        cursor = self.conn.execute(sql, (f"%{file_path}%",))
        files = self._rows_to_dicts(cursor.fetchall())

        if not files:
            return {"error": f"File matching '{file_path}' not found"}

        file_info = files[0]
        file_id = file_info["id"]

        result = {
            "file": file_info,
            "classes": [],
            "functions": [],
            "variables": [],
            "typedefs": [],
        }

        # Get classes/structs defined in this file
        sql = """
            SELECT name, qualified_name, kind, brief_description, line_number, is_final, is_abstract
            FROM compounds
            WHERE file_id = ? AND kind IN ('class', 'struct')
            ORDER BY line_number
        """
        cursor = self.conn.execute(sql, (file_id,))
        result["classes"] = self._rows_to_dicts(cursor.fetchall())

        # Get functions defined in this file
        sql = """
            SELECT m.name, m.qualified_name, m.definition, m.argsstring,
                   m.brief_description, m.line_number, m.protection,
                   m.is_static, m.is_const, m.is_virtual,
                   c.name as class_name
            FROM members m
            LEFT JOIN compounds c ON c.id = m.compound_id
            WHERE m.file_id = ? AND m.kind = 'function'
            ORDER BY m.line_number
        """
        cursor = self.conn.execute(sql, (file_id,))
        result["functions"] = self._rows_to_dicts(cursor.fetchall())

        # Get variables defined in this file
        sql = """
            SELECT m.name, m.qualified_name, m.type, m.definition,
                   m.brief_description, m.line_number, m.protection,
                   m.is_static, m.is_const,
                   c.name as class_name
            FROM members m
            LEFT JOIN compounds c ON c.id = m.compound_id
            WHERE m.file_id = ? AND m.kind = 'variable'
            ORDER BY m.line_number
        """
        cursor = self.conn.execute(sql, (file_id,))
        result["variables"] = self._rows_to_dicts(cursor.fetchall())

        # Get typedefs
        sql = """
            SELECT m.name, m.qualified_name, m.type, m.definition,
                   m.brief_description, m.line_number
            FROM members m
            WHERE m.file_id = ? AND m.kind = 'typedef'
            ORDER BY m.line_number
        """
        cursor = self.conn.execute(sql, (file_id,))
        result["typedefs"] = self._rows_to_dicts(cursor.fetchall())

        return result

    def get_includes(self, file_path: str) -> dict:
        """
        Get include dependencies for a file.

        Args:
            file_path: Path to the file (can be partial match)

        Returns:
            Dictionary with includes and included_by relationships
        """
        # Find the file
        sql = """
            SELECT id, name, path, refid FROM files WHERE path LIKE ?
        """
        cursor = self.conn.execute(sql, (f"%{file_path}%",))
        files = self._rows_to_dicts(cursor.fetchall())

        if not files:
            return {"error": f"File matching '{file_path}' not found"}

        file_info = files[0]
        file_id = file_info["id"]
        file_refid = file_info["refid"]

        result = {
            "file": file_info,
            "includes": [],
            "included_by": [],
        }

        # Get files this file includes
        sql = """
            SELECT included_file, included_refid, is_local
            FROM includes
            WHERE file_id = ?
            ORDER BY included_file
        """
        cursor = self.conn.execute(sql, (file_id,))
        result["includes"] = self._rows_to_dicts(cursor.fetchall())

        # Get files that include this file
        sql = """
            SELECT f.name, f.path
            FROM includes i
            JOIN files f ON f.id = i.file_id
            WHERE i.included_refid = ?
            ORDER BY f.name
        """
        cursor = self.conn.execute(sql, (file_refid,))
        result["included_by"] = self._rows_to_dicts(cursor.fetchall())

        return result

    # =========================================================================
    # Class Member Tools
    # =========================================================================

    def get_class_members(
        self,
        class_name: str,
        include_private: bool = True,
        kind: str | None = None
    ) -> dict:
        """
        Get all members of a class.

        Args:
            class_name: Name of the class
            include_private: Include private members (default: True)
            kind: Filter by kind (function, variable, typedef, etc.)

        Returns:
            Dictionary with class info and categorized members
        """
        # Find the class
        sql = """
            SELECT id, name, qualified_name, kind, brief_description,
                   detailed_description, base_classes, is_final, is_abstract
            FROM compounds
            WHERE name = ? AND kind IN ('class', 'struct')
        """
        cursor = self.conn.execute(sql, (class_name,))
        class_row = cursor.fetchone()

        if not class_row:
            return {"error": f"Class '{class_name}' not found"}

        class_info = dict(class_row)
        compound_id = class_info["id"]

        result = {
            "class": class_info,
            "constructors": [],
            "destructor": None,
            "methods": [],
            "static_methods": [],
            "variables": [],
            "static_variables": [],
            "typedefs": [],
            "enums": [],
        }

        # Build protection filter
        protection_filter = ""
        if not include_private:
            protection_filter = "AND m.protection IN ('public', 'protected')"

        kind_filter = ""
        if kind:
            kind_filter = f"AND m.kind = '{kind}'"

        # Get all members
        sql = f"""
            SELECT m.name, m.qualified_name, m.kind, m.type, m.definition,
                   m.argsstring, m.brief_description, m.protection,
                   m.is_static, m.is_const, m.is_constexpr, m.is_virtual,
                   m.is_inline, m.is_explicit, m.line_number
            FROM members m
            WHERE m.compound_id = ? {protection_filter} {kind_filter}
            ORDER BY m.line_number
        """
        cursor = self.conn.execute(sql, (compound_id,))
        members = self._rows_to_dicts(cursor.fetchall())

        # Categorize members
        for member in members:
            if member["kind"] == "function":
                if member["name"] == class_name:
                    result["constructors"].append(member)
                elif member["name"] == f"~{class_name}":
                    result["destructor"] = member
                elif member["is_static"]:
                    result["static_methods"].append(member)
                else:
                    result["methods"].append(member)
            elif member["kind"] == "variable":
                if member["is_static"]:
                    result["static_variables"].append(member)
                else:
                    result["variables"].append(member)
            elif member["kind"] == "typedef":
                result["typedefs"].append(member)
            elif member["kind"] == "enum":
                result["enums"].append(member)

        return result

    def get_function_parameters(self, function_name: str, class_name: str | None = None) -> list[dict]:
        """
        Get parameters for a function.

        Args:
            function_name: Name of the function
            class_name: Optional class name to scope the search

        Returns:
            List of functions with their parameters
        """
        if class_name:
            sql = """
                SELECT m.id, m.name, m.qualified_name, m.definition, m.argsstring,
                       c.name as class_name
                FROM members m
                LEFT JOIN compounds c ON c.id = m.compound_id
                WHERE m.name = ? AND c.name = ? AND m.kind = 'function'
            """
            cursor = self.conn.execute(sql, (function_name, class_name))
        else:
            sql = """
                SELECT m.id, m.name, m.qualified_name, m.definition, m.argsstring,
                       c.name as class_name
                FROM members m
                LEFT JOIN compounds c ON c.id = m.compound_id
                WHERE m.name = ? AND m.kind = 'function'
            """
            cursor = self.conn.execute(sql, (function_name,))

        functions = self._rows_to_dicts(cursor.fetchall())

        for func in functions:
            sql = """
                SELECT position, name, type, default_value, description
                FROM parameters
                WHERE member_id = ?
                ORDER BY position
            """
            cursor = self.conn.execute(sql, (func["id"],))
            func["parameters"] = self._rows_to_dicts(cursor.fetchall())

        return functions

    # =========================================================================
    # Documentation Search Tools
    # =========================================================================

    def search_documentation(self, query: str, limit: int = 20) -> list[dict]:
        """
        Full-text search in documentation.

        Args:
            query: Search term (uses FTS5 match syntax)
            limit: Maximum number of results

        Returns:
            List of matching symbols with their documentation
        """
        # Use FTS5 for full-text search
        sql = """
            SELECT name, qualified_name, description,
                   bm25(fts_docs) as rank
            FROM fts_docs
            WHERE fts_docs MATCH ?
            ORDER BY rank
            LIMIT ?
        """
        try:
            cursor = self.conn.execute(sql, (query, limit))
            return self._rows_to_dicts(cursor.fetchall())
        except sqlite3.OperationalError:
            # Fallback to LIKE if FTS query syntax is invalid
            sql = """
                SELECT name, qualified_name, description
                FROM fts_docs
                WHERE name LIKE ? OR qualified_name LIKE ? OR description LIKE ?
                LIMIT ?
            """
            pattern = f"%{query}%"
            cursor = self.conn.execute(sql, (pattern, pattern, pattern, limit))
            return self._rows_to_dicts(cursor.fetchall())

    # =========================================================================
    # Statistics and Overview Tools
    # =========================================================================

    def get_statistics(self) -> dict:
        """
        Get database statistics.

        Returns:
            Dictionary with counts of various symbol types
        """
        stats = {}

        tables = ["files", "namespaces", "compounds", "members", "parameters", "symbol_refs", "includes"]
        for table in tables:
            cursor = self.conn.execute(f"SELECT COUNT(*) FROM {table}")
            stats[table] = cursor.fetchone()[0]

        # Get breakdown by kind
        cursor = self.conn.execute("SELECT kind, COUNT(*) FROM compounds GROUP BY kind")
        stats["compounds_by_kind"] = dict(cursor.fetchall())

        cursor = self.conn.execute("SELECT kind, COUNT(*) FROM members GROUP BY kind")
        stats["members_by_kind"] = dict(cursor.fetchall())

        return stats

    def list_namespaces(self) -> list[dict]:
        """
        List all namespaces in the codebase.

        Returns:
            List of namespaces with their qualified names
        """
        sql = """
            SELECT name, qualified_name FROM namespaces ORDER BY qualified_name
        """
        cursor = self.conn.execute(sql)
        return self._rows_to_dicts(cursor.fetchall())

    def list_classes(self, namespace: str | None = None) -> list[dict]:
        """
        List all classes in the codebase.

        Args:
            namespace: Optional namespace to filter by

        Returns:
            List of classes with their basic info
        """
        if namespace:
            sql = """
                SELECT c.name, c.qualified_name, c.kind, c.brief_description,
                       f.path as file_path, c.line_number
                FROM compounds c
                LEFT JOIN files f ON f.id = c.file_id
                WHERE c.kind IN ('class', 'struct') AND c.qualified_name LIKE ?
                ORDER BY c.qualified_name
            """
            cursor = self.conn.execute(sql, (f"{namespace}::%",))
        else:
            sql = """
                SELECT c.name, c.qualified_name, c.kind, c.brief_description,
                       f.path as file_path, c.line_number
                FROM compounds c
                LEFT JOIN files f ON f.id = c.file_id
                WHERE c.kind IN ('class', 'struct')
                ORDER BY c.qualified_name
            """
            cursor = self.conn.execute(sql)

        return self._rows_to_dicts(cursor.fetchall())


def create_mcp_server(db_path: str) -> "FastMCP":
    """Create a FastMCP server wrapping the CodebaseServer."""
    server = CodebaseServer(db_path)
    mcp = FastMCP("codebase")

    @mcp.tool()
    def search_symbols(query: str, kind: str | None = None, limit: int = 20) -> str:
        """Full-text search across all symbols (classes, functions, variables)."""
        return json.dumps(server.search_symbols(query, kind, limit), indent=2, default=str)

    @mcp.tool()
    def find_class(name: str, exact: bool = False) -> str:
        """Find a class or struct by name."""
        return json.dumps(server.find_class(name, exact), indent=2, default=str)

    @mcp.tool()
    def find_function(name: str, class_name: str | None = None, exact: bool = False) -> str:
        """Find a function by name, optionally scoped to a class."""
        return json.dumps(server.find_function(name, class_name, exact), indent=2, default=str)

    @mcp.tool()
    def get_class_hierarchy(class_name: str) -> str:
        """Get the inheritance hierarchy (base and derived classes) for a class."""
        return json.dumps(server.get_class_hierarchy(class_name), indent=2, default=str)

    @mcp.tool()
    def get_callers(function_name: str, class_name: str | None = None) -> str:
        """Find all functions that call a given function."""
        return json.dumps(server.get_callers(function_name, class_name), indent=2, default=str)

    @mcp.tool()
    def get_callees(function_name: str, class_name: str | None = None) -> str:
        """Find all functions called by a given function."""
        return json.dumps(server.get_callees(function_name, class_name), indent=2, default=str)

    @mcp.tool()
    def get_file_symbols(file_path: str) -> str:
        """List all symbols (classes, functions, variables) defined in a file."""
        return json.dumps(server.get_file_symbols(file_path), indent=2, default=str)

    @mcp.tool()
    def get_includes(file_path: str) -> str:
        """Get include dependencies for a file (what it includes and what includes it)."""
        return json.dumps(server.get_includes(file_path), indent=2, default=str)

    @mcp.tool()
    def get_class_members(class_name: str, include_private: bool = True, kind: str | None = None) -> str:
        """Get all members of a class, categorized by type."""
        return json.dumps(server.get_class_members(class_name, include_private, kind), indent=2, default=str)

    @mcp.tool()
    def get_function_parameters(function_name: str, class_name: str | None = None) -> str:
        """Get parameters for a function."""
        return json.dumps(server.get_function_parameters(function_name, class_name), indent=2, default=str)

    @mcp.tool()
    def search_documentation(query: str, limit: int = 20) -> str:
        """Full-text search in documentation (uses FTS5 match syntax)."""
        return json.dumps(server.search_documentation(query, limit), indent=2, default=str)

    @mcp.tool()
    def get_statistics() -> str:
        """Get database statistics (counts of files, classes, functions, etc.)."""
        return json.dumps(server.get_statistics(), indent=2, default=str)

    @mcp.tool()
    def list_namespaces() -> str:
        """List all namespaces in the codebase."""
        return json.dumps(server.list_namespaces(), indent=2, default=str)

    @mcp.tool()
    def list_classes(namespace: str | None = None) -> str:
        """List all classes in the codebase, optionally filtered by namespace."""
        return json.dumps(server.list_classes(namespace), indent=2, default=str)

    return mcp


def format_output(data: Any) -> str:
    """Format output as pretty-printed JSON."""
    return json.dumps(data, indent=2, default=str)


def main():
    parser = argparse.ArgumentParser(
        description="MCP Codebase Server - Navigate your codebase efficiently",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Search for symbols containing "Convex"
    python mcp_codebase_server.py codebase.db search_symbols Convex

    # Find a class by name
    python mcp_codebase_server.py codebase.db find_class ConvexHull

    # Get class hierarchy
    python mcp_codebase_server.py codebase.db get_class_hierarchy ConvexHull

    # Find callers of a function
    python mcp_codebase_server.py codebase.db get_callers compute

    # List all symbols in a file
    python mcp_codebase_server.py codebase.db get_file_symbols ConvexHull.hpp

    # Get class members
    python mcp_codebase_server.py codebase.db get_class_members ConvexHull

    # Get statistics
    python mcp_codebase_server.py codebase.db get_statistics
        """
    )
    parser.add_argument("database", help="Path to the SQLite database")
    parser.add_argument("command", nargs="?", help="Command to execute (omit for MCP server mode)")
    parser.add_argument("args", nargs="*", help="Command arguments")
    parser.add_argument("--json", action="store_true", help="Output as JSON")
    parser.add_argument("--cli", action="store_true", help="Force CLI mode (show help)")

    args = parser.parse_args()

    if not Path(args.database).exists():
        print(f"Error: Database not found: {args.database}", file=sys.stderr)
        sys.exit(1)

    if not args.command and not args.cli:
        # MCP server mode (default when no command given)
        if not HAS_MCP:
            print("Error: mcp package not installed. Run: pip install mcp", file=sys.stderr)
            sys.exit(1)
        mcp_server = create_mcp_server(args.database)
        mcp_server.run(transport="stdio")
        return

    server = CodebaseServer(args.database)

    try:
        if args.cli or not args.command:
            # Show help
            print("Available commands:")
            commands = [
                ("search_symbols <query> [kind]", "Search for symbols by name"),
                ("find_class <name> [--exact]", "Find a class by name"),
                ("find_function <name> [class_name]", "Find a function by name"),
                ("get_class_hierarchy <class_name>", "Get inheritance hierarchy"),
                ("get_callers <function_name> [class_name]", "Find callers of a function"),
                ("get_callees <function_name> [class_name]", "Find functions called by a function"),
                ("get_file_symbols <file_path>", "List symbols in a file"),
                ("get_includes <file_path>", "Get include dependencies"),
                ("get_class_members <class_name>", "Get all members of a class"),
                ("get_function_parameters <function_name> [class_name]", "Get function parameters"),
                ("search_documentation <query>", "Full-text search in docs"),
                ("get_statistics", "Get database statistics"),
                ("list_namespaces", "List all namespaces"),
                ("list_classes [namespace]", "List all classes"),
            ]
            for cmd, desc in commands:
                print(f"  {cmd:50} {desc}")
            sys.exit(0)

        # Execute command
        command = args.command
        cmd_args = args.args

        if command == "search_symbols":
            query = cmd_args[0] if cmd_args else ""
            kind = cmd_args[1] if len(cmd_args) > 1 else None
            result = server.search_symbols(query, kind)
        elif command == "find_class":
            name = cmd_args[0] if cmd_args else ""
            exact = "--exact" in cmd_args
            result = server.find_class(name, exact)
        elif command == "find_function":
            name = cmd_args[0] if cmd_args else ""
            class_name = cmd_args[1] if len(cmd_args) > 1 and not cmd_args[1].startswith("-") else None
            exact = "--exact" in cmd_args
            result = server.find_function(name, class_name, exact)
        elif command == "get_class_hierarchy":
            class_name = cmd_args[0] if cmd_args else ""
            result = server.get_class_hierarchy(class_name)
        elif command == "get_callers":
            function_name = cmd_args[0] if cmd_args else ""
            class_name = cmd_args[1] if len(cmd_args) > 1 else None
            result = server.get_callers(function_name, class_name)
        elif command == "get_callees":
            function_name = cmd_args[0] if cmd_args else ""
            class_name = cmd_args[1] if len(cmd_args) > 1 else None
            result = server.get_callees(function_name, class_name)
        elif command == "get_file_symbols":
            file_path = cmd_args[0] if cmd_args else ""
            result = server.get_file_symbols(file_path)
        elif command == "get_includes":
            file_path = cmd_args[0] if cmd_args else ""
            result = server.get_includes(file_path)
        elif command == "get_class_members":
            class_name = cmd_args[0] if cmd_args else ""
            include_private = "--public-only" not in cmd_args
            kind = None
            for i, arg in enumerate(cmd_args):
                if arg == "--kind" and i + 1 < len(cmd_args):
                    kind = cmd_args[i + 1]
            result = server.get_class_members(class_name, include_private, kind)
        elif command == "get_function_parameters":
            function_name = cmd_args[0] if cmd_args else ""
            class_name = cmd_args[1] if len(cmd_args) > 1 else None
            result = server.get_function_parameters(function_name, class_name)
        elif command == "search_documentation":
            query = cmd_args[0] if cmd_args else ""
            result = server.search_documentation(query)
        elif command == "get_statistics":
            result = server.get_statistics()
        elif command == "list_namespaces":
            result = server.list_namespaces()
        elif command == "list_classes":
            namespace = cmd_args[0] if cmd_args else None
            result = server.list_classes(namespace)
        else:
            print(f"Unknown command: {command}", file=sys.stderr)
            sys.exit(1)

        print(format_output(result))

    finally:
        server.close()


if __name__ == "__main__":
    main()
