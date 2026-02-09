#!/usr/bin/env python3
"""
Traceability MCP Server

A Model Context Protocol (MCP) server that provides design decision traceability,
symbol history, and ticket impact analysis. Works alongside the existing codebase
MCP server, joining via ATTACH DATABASE for cross-referencing.

Usage:
    # MCP server mode (default)
    python traceability_server.py <traceability_db_path> [--codebase-db <codebase_db_path>]

    # CLI mode
    python traceability_server.py <traceability_db_path> <command> [args...]

Tools provided:
    - search_decisions: FTS search across design decision rationale
    - get_decision: Full details of a decision with linked symbols and commits
    - get_symbol_history: Timeline of changes to a symbol across commits
    - get_ticket_impact: All commits, file changes, symbol changes, and decisions for a ticket
    - get_commit_context: Context for a commit: ticket, phase, decisions, symbol changes
    - why_symbol: Design decision(s) that created/modified a symbol, with rationale
    - get_snapshot_symbols: All symbols at a specific point in time
"""

import argparse
import json
import re
import sqlite3
import sys
from pathlib import Path
from typing import Any

try:
    from mcp.server.fastmcp import FastMCP
    HAS_MCP = True
except ImportError:
    HAS_MCP = False


class TraceabilityServer:
    """Server for design decision traceability queries."""

    def __init__(self, db_path: str, codebase_db_path: str | None = None):
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row

        if codebase_db_path and Path(codebase_db_path).exists():
            self.conn.execute(f"ATTACH DATABASE ? AS codebase", (codebase_db_path,))
            self.has_codebase = True
        else:
            self.has_codebase = False

    def close(self):
        """Close database connection."""
        self.conn.close()

    def _rows_to_dicts(self, rows: list[sqlite3.Row]) -> list[dict[str, Any]]:
        """Convert SQLite Row objects to dictionaries."""
        return [dict(row) for row in rows]

    @staticmethod
    def _split_pascal_case(text: str) -> list[str]:
        """Split PascalCase/camelCase identifiers into individual words.

        E.g. "CollisionPipeline" -> ["Collision", "Pipeline"]
             "collision pipeline" -> ["collision", "pipeline"]
        """
        # First split on whitespace
        tokens = text.split()
        words = []
        for token in tokens:
            # Split PascalCase: insert boundary before each uppercase letter
            # that follows a lowercase letter or precedes a lowercase letter
            parts = re.sub(r'([a-z])([A-Z])', r'\1 \2', token)
            parts = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1 \2', parts)
            words.extend(parts.split())
        return words

    # =========================================================================
    # Design Decision Tools
    # =========================================================================

    def search_decisions(
        self, query: str, ticket: str | None = None, status: str | None = None
    ) -> list[dict]:
        """FTS search across design decision rationale.

        Args:
            query: Search term (FTS5 syntax supported).
            ticket: Optional ticket number filter.
            status: Optional status filter ('active', 'superseded', 'deprecated').

        Returns:
            List of matching design decisions.
        """
        results = []

        # Try FTS first
        try:
            sql = """
                SELECT dd.dd_id, dd.ticket, dd.title, dd.rationale,
                       dd.status, dd.extraction_method, dd.source_file,
                       bm25(design_decisions_fts) as rank
                FROM design_decisions_fts fts
                JOIN design_decisions dd ON dd.id = fts.rowid
                WHERE design_decisions_fts MATCH ?
            """
            params: list = [query]

            if ticket:
                sql += " AND dd.ticket = ?"
                params.append(ticket)
            if status:
                sql += " AND dd.status = ?"
                params.append(status)

            sql += " ORDER BY rank LIMIT 20"
            cursor = self.conn.execute(sql, params)
            results = self._rows_to_dicts(cursor.fetchall())
        except sqlite3.OperationalError:
            pass  # Fall through to LIKE search

        if results:
            return results

        # Fallback: LIKE search with PascalCase-aware word splitting.
        # Splits query into individual words so "collision pipeline" matches
        # "CollisionPipeline" via case-insensitive substring matching.
        words = self._split_pascal_case(query)
        if not words:
            return []

        # Build WHERE clause: each word must appear in at least one text column
        word_clauses = []
        params = []
        for word in words:
            pattern = f"%{word}%"
            word_clauses.append(
                "(title LIKE ? OR rationale LIKE ? OR alternatives LIKE ? OR trade_offs LIKE ?)"
            )
            params.extend([pattern] * 4)

        sql = f"""
            SELECT dd_id, ticket, title, rationale, status,
                   extraction_method, source_file
            FROM design_decisions
            WHERE {' AND '.join(word_clauses)}
        """

        if ticket:
            sql += " AND ticket = ?"
            params.append(ticket)
        if status:
            sql += " AND status = ?"
            params.append(status)

        sql += " LIMIT 20"
        cursor = self.conn.execute(sql, params)
        return self._rows_to_dicts(cursor.fetchall())

    def get_decision(self, dd_id: str) -> dict:
        """Get full details of a design decision with linked symbols and commits.

        Args:
            dd_id: Design decision ID (e.g. 'DD-0045-001').

        Returns:
            Full decision details with linked symbols and commits.
        """
        cursor = self.conn.execute(
            "SELECT * FROM design_decisions WHERE dd_id = ?", (dd_id,)
        )
        row = cursor.fetchone()
        if not row:
            return {"error": f"Decision '{dd_id}' not found"}

        result = dict(row)

        # Get linked symbols
        cursor = self.conn.execute(
            "SELECT symbol_name FROM decision_symbols WHERE decision_id = ?",
            (result["id"],),
        )
        result["symbols"] = [r["symbol_name"] for r in cursor.fetchall()]

        # Get linked commits
        cursor = self.conn.execute(
            """SELECT s.sha, s.date, s.message, s.prefix, s.phase
               FROM decision_commits dc
               JOIN snapshots s ON s.id = dc.snapshot_id
               WHERE dc.decision_id = ?
               ORDER BY s.date""",
            (result["id"],),
        )
        result["commits"] = self._rows_to_dicts(cursor.fetchall())

        # Remove internal id
        result.pop("id", None)
        return result

    # =========================================================================
    # Symbol History Tools
    # =========================================================================

    def get_symbol_history(self, qualified_name: str) -> list[dict]:
        """Timeline of changes to a symbol across commits.

        Args:
            qualified_name: Qualified name of the symbol (supports LIKE patterns with %).

        Returns:
            List of symbol changes with commit context, ordered by date.
        """
        pattern = qualified_name if "%" in qualified_name else f"%{qualified_name}%"

        cursor = self.conn.execute(
            """SELECT sc.change_type, sc.qualified_name,
                      sc.old_file_path, sc.new_file_path,
                      sc.old_line, sc.new_line,
                      sc.old_signature, sc.new_signature,
                      s.sha, s.date, s.message, s.ticket_number, s.phase
               FROM symbol_changes sc
               JOIN snapshots s ON s.id = sc.snapshot_id
               WHERE sc.qualified_name LIKE ?
               ORDER BY s.date""",
            (pattern,),
        )
        return self._rows_to_dicts(cursor.fetchall())

    # =========================================================================
    # Ticket Impact Tools
    # =========================================================================

    def get_ticket_impact(self, ticket_number: str) -> dict:
        """All commits, file changes, symbol changes, and decisions for a ticket.

        Args:
            ticket_number: Ticket number (e.g. '0045', '0039a').

        Returns:
            Dictionary with commits, file_changes, symbol_changes, and decisions.
        """
        # Get all commits for this ticket
        cursor = self.conn.execute(
            """SELECT id, sha, date, author, message, prefix, phase
               FROM snapshots
               WHERE ticket_number = ?
               ORDER BY date""",
            (ticket_number,),
        )
        commits = self._rows_to_dicts(cursor.fetchall())

        snapshot_ids = [c["id"] for c in commits]
        file_changes = []
        symbol_changes = []

        if snapshot_ids:
            placeholders = ",".join("?" * len(snapshot_ids))

            # Get aggregated file changes
            cursor = self.conn.execute(
                f"""SELECT file_path, change_type,
                           SUM(insertions) as total_insertions,
                           SUM(deletions) as total_deletions,
                           COUNT(*) as touch_count
                    FROM file_changes
                    WHERE snapshot_id IN ({placeholders})
                    GROUP BY file_path
                    ORDER BY touch_count DESC""",
                snapshot_ids,
            )
            file_changes = self._rows_to_dicts(cursor.fetchall())

            # Get symbol changes
            cursor = self.conn.execute(
                f"""SELECT sc.qualified_name, sc.change_type,
                           sc.old_file_path, sc.new_file_path,
                           s.sha, s.date, s.message
                    FROM symbol_changes sc
                    JOIN snapshots s ON s.id = sc.snapshot_id
                    WHERE sc.snapshot_id IN ({placeholders})
                    ORDER BY s.date""",
                snapshot_ids,
            )
            symbol_changes = self._rows_to_dicts(cursor.fetchall())

        # Get design decisions (always, even if no commits matched)
        cursor = self.conn.execute(
            """SELECT dd_id, title, rationale, status, extraction_method
               FROM design_decisions
               WHERE ticket = ?""",
            (ticket_number,),
        )
        decisions = self._rows_to_dicts(cursor.fetchall())

        if not commits and not decisions:
            return {"error": f"No data found for ticket '{ticket_number}'"}

        # Remove internal ids from commits
        for c in commits:
            c.pop("id", None)

        return {
            "ticket": ticket_number,
            "commits": commits,
            "file_changes": file_changes,
            "symbol_changes": symbol_changes,
            "decisions": decisions,
        }

    # =========================================================================
    # Commit Context Tools
    # =========================================================================

    def get_commit_context(self, commit_sha: str) -> dict:
        """Context for a commit: ticket, phase, decisions, symbol changes.

        Args:
            commit_sha: Full or partial (7+ char) commit SHA.

        Returns:
            Dictionary with commit info, file changes, symbol changes, and related decisions.
        """
        # Find the snapshot (support partial SHA)
        cursor = self.conn.execute(
            "SELECT * FROM snapshots WHERE sha LIKE ?",
            (f"{commit_sha}%",),
        )
        row = cursor.fetchone()
        if not row:
            return {"error": f"Commit '{commit_sha}' not found"}

        result = dict(row)
        snapshot_id = result["id"]

        # Get file changes
        cursor = self.conn.execute(
            """SELECT file_path, change_type, insertions, deletions, old_path
               FROM file_changes WHERE snapshot_id = ?""",
            (snapshot_id,),
        )
        result["file_changes"] = self._rows_to_dicts(cursor.fetchall())

        # Get symbol changes
        cursor = self.conn.execute(
            """SELECT qualified_name, change_type,
                      old_file_path, new_file_path,
                      old_line, new_line,
                      old_signature, new_signature
               FROM symbol_changes WHERE snapshot_id = ?""",
            (snapshot_id,),
        )
        result["symbol_changes"] = self._rows_to_dicts(cursor.fetchall())

        # Get related decisions (via ticket number or direct commit link)
        decisions = []
        if result.get("ticket_number"):
            cursor = self.conn.execute(
                """SELECT dd_id, title, rationale, status
                   FROM design_decisions WHERE ticket = ?""",
                (result["ticket_number"],),
            )
            decisions = self._rows_to_dicts(cursor.fetchall())

        # Also check decision_commits
        cursor = self.conn.execute(
            """SELECT dd.dd_id, dd.title, dd.rationale, dd.status
               FROM decision_commits dc
               JOIN design_decisions dd ON dd.id = dc.decision_id
               WHERE dc.snapshot_id = ?""",
            (snapshot_id,),
        )
        direct_links = self._rows_to_dicts(cursor.fetchall())
        seen_ids = {d["dd_id"] for d in decisions}
        for d in direct_links:
            if d["dd_id"] not in seen_ids:
                decisions.append(d)

        result["decisions"] = decisions
        result.pop("id", None)
        return result

    # =========================================================================
    # Why-Symbol Tool
    # =========================================================================

    def why_symbol(self, qualified_name: str) -> dict:
        """Design decision(s) that created/modified a symbol, with rationale.

        Args:
            qualified_name: Qualified name of the symbol.

        Returns:
            Dictionary with decisions and commit history for the symbol.
        """
        pattern = qualified_name if "%" in qualified_name else f"%{qualified_name}%"

        # Check decision_symbols for direct links
        cursor = self.conn.execute(
            """SELECT dd.dd_id, dd.ticket, dd.title, dd.rationale,
                      dd.alternatives, dd.trade_offs, dd.status,
                      dd.extraction_method, dd.source_file
               FROM decision_symbols ds
               JOIN design_decisions dd ON dd.id = ds.decision_id
               WHERE ds.symbol_name LIKE ?""",
            (pattern,),
        )
        direct_decisions = self._rows_to_dicts(cursor.fetchall())

        # Get symbol change history to find creating and modifying commits
        cursor = self.conn.execute(
            """SELECT sc.change_type, sc.qualified_name,
                      s.sha, s.date, s.message, s.ticket_number, s.phase
               FROM symbol_changes sc
               JOIN snapshots s ON s.id = sc.snapshot_id
               WHERE sc.qualified_name LIKE ?
               ORDER BY s.date""",
            (pattern,),
        )
        changes = self._rows_to_dicts(cursor.fetchall())

        # Find decisions via ticket numbers from the commits that touched this symbol
        ticket_numbers = {c["ticket_number"] for c in changes if c.get("ticket_number")}
        indirect_decisions = []
        if ticket_numbers:
            placeholders = ",".join("?" * len(ticket_numbers))
            cursor = self.conn.execute(
                f"""SELECT dd_id, ticket, title, rationale, status, extraction_method
                    FROM design_decisions
                    WHERE ticket IN ({placeholders})""",
                list(ticket_numbers),
            )
            indirect_decisions = self._rows_to_dicts(cursor.fetchall())

        # Merge, preferring direct links
        seen_ids = {d["dd_id"] for d in direct_decisions}
        for d in indirect_decisions:
            if d["dd_id"] not in seen_ids:
                d["link_type"] = "indirect (via ticket)"
                direct_decisions.append(d)

        return {
            "symbol": qualified_name,
            "decisions": direct_decisions,
            "change_history": changes,
        }

    # =========================================================================
    # Snapshot Symbols Tool
    # =========================================================================

    def get_snapshot_symbols(
        self, commit_sha: str, file_path: str | None = None
    ) -> list[dict]:
        """All symbols at a specific point in time.

        Args:
            commit_sha: Full or partial commit SHA.
            file_path: Optional file path filter (LIKE pattern).

        Returns:
            List of symbols at that commit.
        """
        cursor = self.conn.execute(
            "SELECT id FROM snapshots WHERE sha LIKE ?",
            (f"{commit_sha}%",),
        )
        row = cursor.fetchone()
        if not row:
            return [{"error": f"Commit '{commit_sha}' not found"}]

        snapshot_id = row["id"]

        if file_path:
            cursor = self.conn.execute(
                """SELECT qualified_name, kind, file_path, line_number, signature, class_scope
                   FROM symbol_snapshots
                   WHERE snapshot_id = ? AND file_path LIKE ?
                   ORDER BY file_path, line_number""",
                (snapshot_id, f"%{file_path}%"),
            )
        else:
            cursor = self.conn.execute(
                """SELECT qualified_name, kind, file_path, line_number, signature, class_scope
                   FROM symbol_snapshots
                   WHERE snapshot_id = ?
                   ORDER BY file_path, line_number""",
                (snapshot_id,),
            )

        return self._rows_to_dicts(cursor.fetchall())


def create_mcp_server(db_path: str, codebase_db_path: str | None = None) -> "FastMCP":
    """Create a FastMCP server wrapping the TraceabilityServer."""
    server = TraceabilityServer(db_path, codebase_db_path)
    mcp = FastMCP("traceability")

    @mcp.tool()
    def search_decisions(
        query: str, ticket: str | None = None, status: str | None = None
    ) -> str:
        """FTS search across design decision rationale, alternatives, and trade-offs."""
        return json.dumps(
            server.search_decisions(query, ticket, status), indent=2, default=str
        )

    @mcp.tool()
    def get_decision(dd_id: str) -> str:
        """Get full details of a design decision with linked symbols and commits."""
        return json.dumps(server.get_decision(dd_id), indent=2, default=str)

    @mcp.tool()
    def get_symbol_history(qualified_name: str) -> str:
        """Timeline of changes to a symbol across commits (supports % wildcards)."""
        return json.dumps(
            server.get_symbol_history(qualified_name), indent=2, default=str
        )

    @mcp.tool()
    def get_ticket_impact(ticket_number: str) -> str:
        """All commits, file changes, symbol changes, and decisions for a ticket."""
        return json.dumps(
            server.get_ticket_impact(ticket_number), indent=2, default=str
        )

    @mcp.tool()
    def get_commit_context(commit_sha: str) -> str:
        """Context for a commit: ticket, phase, file changes, symbol changes, decisions."""
        return json.dumps(
            server.get_commit_context(commit_sha), indent=2, default=str
        )

    @mcp.tool()
    def why_symbol(qualified_name: str) -> str:
        """Design decision(s) that created or modified a symbol, with rationale."""
        return json.dumps(server.why_symbol(qualified_name), indent=2, default=str)

    @mcp.tool()
    def get_snapshot_symbols(
        commit_sha: str, file_path: str | None = None
    ) -> str:
        """All symbols at a specific point in time, optionally filtered by file."""
        return json.dumps(
            server.get_snapshot_symbols(commit_sha, file_path), indent=2, default=str
        )

    return mcp


def format_output(data: Any) -> str:
    """Format output as pretty-printed JSON."""
    return json.dumps(data, indent=2, default=str)


def main():
    parser = argparse.ArgumentParser(
        description="Traceability MCP Server - Design decision and symbol history queries",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Search for decisions about energy conservation
    python traceability_server.py traceability.db search_decisions energy

    # Get a specific decision
    python traceability_server.py traceability.db get_decision DD-0045-001

    # Get symbol change history
    python traceability_server.py traceability.db get_symbol_history ConstraintSolver

    # Get ticket impact
    python traceability_server.py traceability.db get_ticket_impact 0045

    # Get commit context
    python traceability_server.py traceability.db get_commit_context 3ca903f

    # Why does a symbol exist?
    python traceability_server.py traceability.db why_symbol PositionCorrector

    # Symbols at a point in time
    python traceability_server.py traceability.db get_snapshot_symbols 3ca903f ConstraintSolver.hpp
        """,
    )
    parser.add_argument("database", help="Path to the traceability SQLite database")
    parser.add_argument("command", nargs="?", help="Command to execute (omit for MCP server mode)")
    parser.add_argument("args", nargs="*", help="Command arguments")
    parser.add_argument(
        "--codebase-db", default=None,
        help="Path to codebase.db for cross-referencing",
    )
    parser.add_argument("--cli", action="store_true", help="Force CLI mode (show help)")

    args = parser.parse_args()

    if not Path(args.database).exists() and args.command:
        print(f"Error: Database not found: {args.database}", file=sys.stderr)
        sys.exit(1)

    if not args.command and not args.cli:
        # MCP server mode
        if not HAS_MCP:
            print("Error: mcp package not installed. Run: pip install mcp", file=sys.stderr)
            sys.exit(1)
        if not Path(args.database).exists():
            print(f"Warning: Database not found: {args.database}. Server will start but queries will fail.", file=sys.stderr)
        mcp_server = create_mcp_server(args.database, args.codebase_db)
        mcp_server.run(transport="stdio")
        return

    server = TraceabilityServer(args.database, args.codebase_db)

    try:
        if args.cli or not args.command:
            print("Available commands:")
            commands = [
                ("search_decisions <query> [ticket] [status]", "FTS search across design decisions"),
                ("get_decision <dd_id>", "Get full details of a design decision"),
                ("get_symbol_history <qualified_name>", "Timeline of changes to a symbol"),
                ("get_ticket_impact <ticket_number>", "All impact data for a ticket"),
                ("get_commit_context <commit_sha>", "Context for a commit"),
                ("why_symbol <qualified_name>", "Why does this symbol exist?"),
                ("get_snapshot_symbols <commit_sha> [file_path]", "Symbols at a point in time"),
            ]
            for cmd, desc in commands:
                print(f"  {cmd:55} {desc}")
            sys.exit(0)

        command = args.command
        cmd_args = args.args

        if command == "search_decisions":
            query = cmd_args[0] if cmd_args else ""
            ticket = cmd_args[1] if len(cmd_args) > 1 else None
            status = cmd_args[2] if len(cmd_args) > 2 else None
            result = server.search_decisions(query, ticket, status)
        elif command == "get_decision":
            dd_id = cmd_args[0] if cmd_args else ""
            result = server.get_decision(dd_id)
        elif command == "get_symbol_history":
            name = cmd_args[0] if cmd_args else ""
            result = server.get_symbol_history(name)
        elif command == "get_ticket_impact":
            ticket = cmd_args[0] if cmd_args else ""
            result = server.get_ticket_impact(ticket)
        elif command == "get_commit_context":
            sha = cmd_args[0] if cmd_args else ""
            result = server.get_commit_context(sha)
        elif command == "why_symbol":
            name = cmd_args[0] if cmd_args else ""
            result = server.why_symbol(name)
        elif command == "get_snapshot_symbols":
            sha = cmd_args[0] if cmd_args else ""
            file_path = cmd_args[1] if len(cmd_args) > 1 else None
            result = server.get_snapshot_symbols(sha, file_path)
        else:
            print(f"Unknown command: {command}", file=sys.stderr)
            sys.exit(1)

        print(format_output(result))

    finally:
        server.close()


if __name__ == "__main__":
    main()
