#!/usr/bin/env python3
# Ticket: 0078_cpp_guidelines_mcp_server
# Design: docs/designs/0078_cpp_guidelines_mcp_server/design.md
"""
Guidelines MCP Server

A Model Context Protocol (MCP) server backed by the SQLite + FTS5 guidelines
database. Provides 5 tools for AI agents to query C++ coding guidelines with
rationale-backed recommendations and traceable rule IDs.

Usage (MCP server mode — default when invoked by Claude):
    python guidelines_server.py <database_path>

Usage (CLI smoke-test mode):
    python guidelines_server.py <database_path> <command> [args...]

Commands:
    search_guidelines <query> [--source <s>] [--category <c>] [--severity <s>] [--limit <n>]
    get_rule <rule_id>
    list_categories
    get_category <name> [--detailed]
    get_rules_by_tag <tag>

Example:
    python guidelines_server.py build/Debug/docs/guidelines.db search_guidelines "brace initialization"
    python guidelines_server.py build/Debug/docs/guidelines.db get_rule MSD-RES-001
    python guidelines_server.py build/Debug/docs/guidelines.db list_categories
    python guidelines_server.py build/Debug/docs/guidelines.db get_category Initialization
    python guidelines_server.py build/Debug/docs/guidelines.db get_category Initialization --detailed
    python guidelines_server.py build/Debug/docs/guidelines.db get_rules_by_tag safety
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


class GuidelinesServer:
    """
    MCP server for C++ coding guidelines queries.

    Wraps the SQLite + FTS5 guidelines database with 5 query methods:
    - search_guidelines: FTS full-text search with optional filters
    - get_rule: Full rule details + cross-refs + tags
    - list_categories: All categories with rule counts
    - get_category: Rules in a category (summary or detailed)
    - get_rules_by_tag: All rules with a given tag
    """

    def __init__(self, db_path: str):
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row
        self.conn.execute("PRAGMA foreign_keys = ON")

    def close(self) -> None:
        """Close the database connection."""
        self.conn.close()

    def _rows_to_dicts(self, rows: list[sqlite3.Row]) -> list[dict[str, Any]]:
        """Convert SQLite Row objects to plain dicts."""
        return [dict(row) for row in rows]

    # =========================================================================
    # Tool: search_guidelines
    # =========================================================================

    def search_guidelines(
        self,
        query: str,
        source: str | None = None,
        category: str | None = None,
        severity: str | None = None,
        limit: int = 20,
    ) -> list[dict]:
        """
        FTS full-text search over rule title, rationale, enforcement_notes, and examples.

        Uses porter stemmer tokenization so "initialize" matches "initialization".
        Results are ranked by BM25 relevance. Only 'active' rules are returned by
        default (deprecated rules are excluded).

        Args:
            query: Search terms (FTS5 match syntax — supports quoted phrases, AND, OR, NOT)
            source: Optional filter — 'project', 'cpp_core_guidelines', or 'misra'
            category: Optional filter by category name (substring match)
            severity: Optional filter — 'required', 'recommended', or 'advisory'
            limit: Maximum results (default: 20)

        Returns:
            List of matching rules with rule_id, title, source, severity, category,
            rationale, and BM25 rank score.
        """
        # Build filter clause fragments
        filters: list[str] = ["r.status = 'active'"]
        params: list[Any] = []

        if source:
            filters.append("r.source = ?")
            params.append(source)
        if severity:
            filters.append("r.severity = ?")
            params.append(severity)
        if category:
            filters.append("c.name LIKE ?")
            params.append(f"%{category}%")

        filter_clause = " AND ".join(filters)

        # FTS5 search with BM25 ranking
        fts_sql = f"""
            SELECT
                r.rule_id,
                r.title,
                r.source,
                r.severity,
                r.status,
                c.name AS category,
                r.rationale,
                r.enforcement_notes,
                r.enforcement_check,
                bm25(rules_fts) AS rank
            FROM rules_fts
            JOIN rules r ON r.rule_id = rules_fts.rule_id
            JOIN categories c ON c.id = r.category_id
            WHERE rules_fts MATCH ? AND {filter_clause}
            ORDER BY rank
            LIMIT ?
        """
        try:
            cursor = self.conn.execute(fts_sql, [query] + params + [limit])
            return self._rows_to_dicts(cursor.fetchall())
        except sqlite3.OperationalError:
            # Fall back to LIKE search if FTS query syntax is invalid
            like_sql = f"""
                SELECT
                    r.rule_id,
                    r.title,
                    r.source,
                    r.severity,
                    r.status,
                    c.name AS category,
                    r.rationale,
                    r.enforcement_notes,
                    r.enforcement_check,
                    0.0 AS rank
                FROM rules r
                JOIN categories c ON c.id = r.category_id
                WHERE (r.title LIKE ? OR r.rationale LIKE ? OR r.enforcement_notes LIKE ?)
                  AND {filter_clause}
                LIMIT ?
            """
            pattern = f"%{query}%"
            cursor = self.conn.execute(
                like_sql, [pattern, pattern, pattern] + params + [limit]
            )
            return self._rows_to_dicts(cursor.fetchall())

    # =========================================================================
    # Tool: get_rule
    # =========================================================================

    def get_rule(self, rule_id: str) -> dict:
        """
        Retrieve full details for a single rule by its rule_id.

        Returns all columns from the rules table plus:
        - category: Category name
        - tags: List of tag names
        - cross_refs: List of {rule_id, relationship, title} for related rules

        Not-found contract (N1): Returns {"error": "Rule not found", "rule_id": "<id>"}
        when the rule_id does not exist in the database.

        Args:
            rule_id: Rule identifier (e.g., "MSD-INIT-002", "CPP-R.11")

        Returns:
            Full rule dict with category, tags, and cross_refs, or error dict.
        """
        cursor = self.conn.execute(
            """
            SELECT
                r.rule_id,
                c.name AS category,
                r.source,
                r.severity,
                r.status,
                r.title,
                r.rationale,
                r.enforcement_notes,
                r.enforcement_check,
                r.good_example,
                r.bad_example
            FROM rules r
            JOIN categories c ON c.id = r.category_id
            WHERE r.rule_id = ?
            """,
            (rule_id,),
        )
        row = cursor.fetchone()
        if row is None:
            return {"error": "Rule not found", "rule_id": rule_id}

        result = dict(row)

        # Fetch tags
        cursor = self.conn.execute(
            """
            SELECT t.name
            FROM rule_tags rt
            JOIN tags t ON t.id = rt.tag_id
            WHERE rt.rule_id = ?
            ORDER BY t.name
            """,
            (rule_id,),
        )
        result["tags"] = [r["name"] for r in cursor.fetchall()]

        # Fetch cross-refs (both directions)
        cursor = self.conn.execute(
            """
            SELECT
                rc.to_rule_id AS related_rule_id,
                rc.relationship,
                r2.title AS related_title,
                'outgoing' AS direction
            FROM rule_cross_refs rc
            JOIN rules r2 ON r2.rule_id = rc.to_rule_id
            WHERE rc.from_rule_id = ?
            UNION ALL
            SELECT
                rc.from_rule_id AS related_rule_id,
                rc.relationship,
                r2.title AS related_title,
                'incoming' AS direction
            FROM rule_cross_refs rc
            JOIN rules r2 ON r2.rule_id = rc.from_rule_id
            WHERE rc.to_rule_id = ?
            ORDER BY related_rule_id
            """,
            (rule_id, rule_id),
        )
        result["cross_refs"] = self._rows_to_dicts(cursor.fetchall())

        return result

    # =========================================================================
    # Tool: list_categories
    # =========================================================================

    def list_categories(self) -> list[dict]:
        """
        List all categories with their rule counts.

        Returns:
            List of {category_name, total_rules, active_rules} dicts,
            sorted alphabetically by category name.
        """
        cursor = self.conn.execute(
            """
            SELECT
                c.name AS category,
                COUNT(r.rule_id) AS total_rules,
                SUM(CASE WHEN r.status = 'active' THEN 1 ELSE 0 END) AS active_rules
            FROM categories c
            LEFT JOIN rules r ON r.category_id = c.id
            GROUP BY c.id, c.name
            ORDER BY c.name
            """
        )
        return self._rows_to_dicts(cursor.fetchall())

    # =========================================================================
    # Tool: get_category
    # =========================================================================

    def get_category(self, name: str, detailed: bool = False) -> dict:
        """
        Retrieve rules in a category.

        DD-0078-005: Defaults to summary mode to avoid context window overflow
        for large categories (CppCoreGuidelines can have 100+ rules per section).

        Args:
            name: Category name (exact match, case-sensitive)
            detailed: If False (default), return {rule_id, title, severity, status} only.
                      If True, return full rationale, enforcement_notes, and examples.

        Returns:
            {category, rules: [...]} dict, or error dict if category not found.
        """
        # Look up category
        cursor = self.conn.execute(
            "SELECT id, name FROM categories WHERE name = ?", (name,)
        )
        cat_row = cursor.fetchone()
        if cat_row is None:
            return {"error": "Category not found", "category": name}

        cat_id = cat_row["id"]

        if detailed:
            columns = """
                r.rule_id,
                r.title,
                r.severity,
                r.status,
                r.source,
                r.rationale,
                r.enforcement_notes,
                r.enforcement_check,
                r.good_example,
                r.bad_example
            """
        else:
            # Summary mode — rule_id + title + severity + status only (DD-0078-005)
            columns = "r.rule_id, r.title, r.severity, r.status, r.source"

        cursor = self.conn.execute(
            f"""
            SELECT {columns}
            FROM rules r
            WHERE r.category_id = ?
            ORDER BY r.rule_id
            """,
            (cat_id,),
        )
        rules = self._rows_to_dicts(cursor.fetchall())

        return {
            "category": cat_row["name"],
            "rule_count": len(rules),
            "detailed": detailed,
            "rules": rules,
        }

    # =========================================================================
    # Tool: get_rules_by_tag
    # =========================================================================

    def get_rules_by_tag(self, tag: str) -> list[dict]:
        """
        Retrieve all rules associated with a given tag.

        Tags provide cross-cutting queries that span categories (e.g., all rules
        tagged "safety" across Initialization, Resource Management, etc.).

        Args:
            tag: Tag name (e.g., "safety", "ownership", "memory", "initialization")

        Returns:
            List of {rule_id, title, category, severity, status} dicts,
            or empty list if no rules have the tag.
        """
        cursor = self.conn.execute(
            """
            SELECT
                r.rule_id,
                r.title,
                c.name AS category,
                r.severity,
                r.status,
                r.source
            FROM rule_tags rt
            JOIN tags t ON t.id = rt.tag_id
            JOIN rules r ON r.rule_id = rt.rule_id
            JOIN categories c ON c.id = r.category_id
            WHERE t.name = ?
            ORDER BY r.rule_id
            """,
            (tag,),
        )
        return self._rows_to_dicts(cursor.fetchall())


# ---------------------------------------------------------------------------
# FastMCP server factory
# ---------------------------------------------------------------------------

def create_mcp_server(db_path: str) -> "FastMCP":
    """
    Create a FastMCP server wrapping GuidelinesServer.

    Follows the same class + factory pattern as mcp_codebase_server.py.
    """
    server = GuidelinesServer(db_path)
    mcp = FastMCP("guidelines")

    @mcp.tool()
    def search_guidelines(
        query: str,
        source: str | None = None,
        category: str | None = None,
        severity: str | None = None,
        limit: int = 20,
    ) -> str:
        """
        Search C++ coding guidelines using full-text search (FTS5 with porter stemming).

        Returns rules matching the query ranked by BM25 relevance. Only active rules
        are returned. Supports FTS5 match syntax: quoted phrases, AND, OR, NOT operators.

        Porter stemming means "initialize" matches "initialization", "ownership" matches
        "owns", etc.

        Args:
            query: Search terms (e.g., "brace initialization", "unique_ptr ownership")
            source: Filter by source — 'project', 'cpp_core_guidelines', or 'misra'
            category: Filter by category name (substring match)
            severity: Filter by severity — 'required', 'recommended', or 'advisory'
            limit: Maximum results to return (default: 20)
        """
        return json.dumps(
            server.search_guidelines(query, source, category, severity, limit),
            indent=2,
            default=str,
        )

    @mcp.tool()
    def get_rule(rule_id: str) -> str:
        """
        Get full details for a C++ coding guideline rule by its rule_id.

        Returns the complete rule including: category, source, severity, status,
        title, rationale, enforcement_notes, enforcement_check (clang-tidy/cppcheck
        ID if available), good_example, bad_example, tags, and cross-references to
        related rules.

        If the rule_id does not exist, returns: {"error": "Rule not found", "rule_id": "..."}

        Args:
            rule_id: Rule identifier (e.g., "MSD-INIT-002", "MSD-RES-001", "CPP-R.11")
        """
        return json.dumps(server.get_rule(rule_id), indent=2, default=str)

    @mcp.tool()
    def list_categories() -> str:
        """
        List all guideline categories with rule counts.

        Returns each category with total_rules and active_rules counts,
        sorted alphabetically. Use this to discover what categories exist
        before calling get_category.
        """
        return json.dumps(server.list_categories(), indent=2, default=str)

    @mcp.tool()
    def get_category(name: str, detailed: bool = False) -> str:
        """
        Get rules in a specific guideline category.

        By default (detailed=False), returns summary mode: {rule_id, title, severity,
        status} for each rule. This avoids context window overflow for large categories
        (C++ Core Guidelines sections can have 100+ rules).

        Use detailed=True to include full rationale, enforcement_notes, and examples.
        When using detailed=True for large categories, prefer calling get_rule on
        specific rules of interest instead.

        Args:
            name: Exact category name (use list_categories to discover names)
            detailed: If True, include full rationale and examples (default: False)
        """
        return json.dumps(
            server.get_category(name, detailed), indent=2, default=str
        )

    @mcp.tool()
    def get_rules_by_tag(tag: str) -> str:
        """
        Get all C++ coding guideline rules associated with a tag.

        Tags provide cross-cutting queries spanning multiple categories. Common tags:
        - safety: Rules that prevent undefined behavior or memory errors
        - ownership: Rules about object lifetime and resource ownership
        - memory: Rules about memory allocation and deallocation
        - initialization: Rules about variable and member initialization
        - naming: Rules about identifier naming conventions
        - style: Rules about code formatting and style consistency
        - documentation: Rules about API documentation
        - organization: Rules about file and code organization

        Args:
            tag: Tag name (e.g., "safety", "ownership", "memory")
        """
        return json.dumps(server.get_rules_by_tag(tag), indent=2, default=str)

    return mcp


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def format_output(data: Any) -> str:
    """Pretty-print data as JSON."""
    return json.dumps(data, indent=2, default=str)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="C++ Guidelines MCP Server",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # MCP server mode (used by Claude)
    python guidelines_server.py build/Debug/docs/guidelines.db

    # CLI smoke tests
    python guidelines_server.py build/Debug/docs/guidelines.db search_guidelines "brace initialization"
    python guidelines_server.py build/Debug/docs/guidelines.db get_rule MSD-RES-001
    python guidelines_server.py build/Debug/docs/guidelines.db list_categories
    python guidelines_server.py build/Debug/docs/guidelines.db get_category Initialization
    python guidelines_server.py build/Debug/docs/guidelines.db get_category Initialization --detailed
    python guidelines_server.py build/Debug/docs/guidelines.db get_rules_by_tag safety
        """,
    )
    parser.add_argument("database", help="Path to the guidelines SQLite database")
    parser.add_argument(
        "command",
        nargs="?",
        help="Command to execute (omit for MCP server mode)",
    )
    parser.add_argument("args", nargs="*", help="Command arguments")
    parser.add_argument("--detailed", action="store_true", help="Detailed output for get_category")
    parser.add_argument("--source", help="Filter by source")
    parser.add_argument("--category", help="Filter by category")
    parser.add_argument("--severity", help="Filter by severity")
    parser.add_argument("--limit", type=int, default=20, help="Limit results")

    args = parser.parse_args()

    db_path = Path(args.database)
    if not db_path.exists():
        print(f"Error: Database not found: {db_path}", file=sys.stderr)
        print(
            "Run: cmake --build --preset debug-guidelines  (or seed_guidelines.py)",
            file=sys.stderr,
        )
        sys.exit(1)

    if not args.command:
        # MCP server mode
        if not HAS_MCP:
            print(
                "Error: mcp package not installed. Run: pip install mcp",
                file=sys.stderr,
            )
            sys.exit(1)
        mcp_server = create_mcp_server(str(db_path))
        mcp_server.run(transport="stdio")
        return

    # CLI mode
    server = GuidelinesServer(str(db_path))
    try:
        command = args.command
        cmd_args = args.args

        if command == "search_guidelines":
            query = cmd_args[0] if cmd_args else ""
            result = server.search_guidelines(
                query,
                source=args.source,
                category=args.category,
                severity=args.severity,
                limit=args.limit,
            )

        elif command == "get_rule":
            rule_id = cmd_args[0] if cmd_args else ""
            result = server.get_rule(rule_id)

        elif command == "list_categories":
            result = server.list_categories()

        elif command == "get_category":
            name = cmd_args[0] if cmd_args else ""
            result = server.get_category(name, detailed=args.detailed)

        elif command == "get_rules_by_tag":
            tag = cmd_args[0] if cmd_args else ""
            result = server.get_rules_by_tag(tag)

        else:
            print(f"Unknown command: {command}", file=sys.stderr)
            print(
                "Commands: search_guidelines, get_rule, list_categories, "
                "get_category, get_rules_by_tag",
                file=sys.stderr,
            )
            sys.exit(1)

        print(format_output(result))

    finally:
        server.close()


if __name__ == "__main__":
    main()
