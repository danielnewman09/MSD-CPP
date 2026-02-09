#!/usr/bin/env python3
"""
Design Decision Indexer

Parses ticket files and design documents to extract design decisions.
Supports two extraction modes:

  - Structured: Parses explicit DD-NNNN-NNN blocks in design docs.
  - Heuristic: Scans for decision-indicating patterns in existing docs.

Populates `design_decisions`, `decision_symbols`, and `decision_commits`
tables, and rebuilds the FTS5 index.

Usage:
    python index_decisions.py <traceability_db_path> [--repo <repo_root>]
"""

import argparse
import re
import sys
from pathlib import Path

from traceability_schema import create_schema, rebuild_fts

# Structured DD block pattern: ### DD-NNNN-NNN: Title
DD_BLOCK_RE = re.compile(
    r"^###\s+(DD-\d{4}[a-e]?-\d{3}):\s*(.+)$", re.MULTILINE
)

# Field patterns within a DD block
FIELD_AFFECTS = re.compile(r"^\s*[-*]\s*\*\*Affects\*\*:\s*(.+)$", re.MULTILINE)
FIELD_RATIONALE = re.compile(r"^\s*[-*]\s*\*\*Rationale\*\*:\s*(.+)$", re.MULTILINE)
FIELD_ALTERNATIVES = re.compile(
    r"^\s*[-*]\s*\*\*Alternatives\s+Considered\*\*:", re.MULTILINE
)
FIELD_TRADEOFFS = re.compile(r"^\s*[-*]\s*\*\*Trade-offs?\*\*:\s*(.+)$", re.MULTILINE)
FIELD_STATUS = re.compile(r"^\s*[-*]\s*\*\*Status\*\*:\s*(\w+)", re.MULTILINE)

# Heuristic patterns for extracting implicit decisions
HEURISTIC_PATTERNS = [
    # "Design rationale:" paragraphs
    (re.compile(r"\*\*Design [Rr]ationale\*\*:\s*(.+?)(?=\n\n|\n\*\*|\Z)", re.DOTALL),
     "design_rationale"),
    # "Design decision:" blocks
    (re.compile(r"\*\*Design [Dd]ecision[s]?\*\*:\s*(.+?)(?=\n\n|\n\*\*|\Z)", re.DOTALL),
     "design_decision"),
    # "Recommendation:" blocks
    (re.compile(r"\*\*Recommendation\*\*:\s*(.+?)(?=\n\n|\n\*\*|\Z)", re.DOTALL),
     "recommendation"),
    # "RESOLVED" open questions
    (re.compile(r"(?:RESOLVED|Resolved)(?:\s+by\s+\w+)?[:\s]+(.+?)(?=\n\n|\n-|\Z)", re.DOTALL),
     "resolved_question"),
    # "Why" explanations in comments
    (re.compile(r"### Why\s+(.+?)(?=\n###|\n##|\Z)", re.DOTALL),
     "why_explanation"),
    # Problem statement sections that explain decisions
    (re.compile(r"### (?:Root Cause|Problem)[:\s]+(.+?)(?=\n###|\n##|\Z)", re.DOTALL),
     "root_cause"),
]

# PascalCase identifier pattern for symbol linking
PASCAL_CASE_RE = re.compile(r"\b([A-Z][a-z]+(?:[A-Z][a-z]+)+)\b")

# Ticket number extraction from filenames
TICKET_FROM_PATH = re.compile(r"(\d{4}[a-e]?)")


def extract_ticket_number(path: str) -> str | None:
    """Extract ticket number from a file path."""
    match = TICKET_FROM_PATH.search(path)
    return match.group(1) if match else None


def parse_structured_decisions(content: str, file_path: str, ticket: str) -> list[dict]:
    """Parse explicit DD-NNNN-NNN blocks from a design document.

    Args:
        content: Full text of the design document.
        file_path: Relative path to the file.
        ticket: Ticket number.

    Returns:
        List of decision dicts.
    """
    decisions = []

    # Find all DD block headers
    matches = list(DD_BLOCK_RE.finditer(content))

    for i, match in enumerate(matches):
        dd_id = match.group(1)
        title = match.group(2).strip()
        start = match.end()
        # Block extends to the next DD header or next ## heading
        end = matches[i + 1].start() if i + 1 < len(matches) else len(content)
        block = content[start:end]

        # Stop at next ## heading
        next_heading = re.search(r"^##\s", block, re.MULTILINE)
        if next_heading:
            block = block[:next_heading.start()]

        # Extract fields
        affects_match = FIELD_AFFECTS.search(block)
        rationale_match = FIELD_RATIONALE.search(block)
        tradeoffs_match = FIELD_TRADEOFFS.search(block)
        status_match = FIELD_STATUS.search(block)

        # Extract alternatives (multi-line list)
        alternatives = None
        alt_match = FIELD_ALTERNATIVES.search(block)
        if alt_match:
            alt_start = alt_match.end()
            alt_lines = []
            for line in block[alt_start:].split("\n"):
                line = line.strip()
                if line.startswith("-") or line.startswith("*"):
                    alt_lines.append(line.lstrip("-* "))
                elif alt_lines and line:
                    alt_lines[-1] += " " + line
                elif not line and alt_lines:
                    break
            alternatives = "; ".join(alt_lines) if alt_lines else None

        affects = [s.strip() for s in affects_match.group(1).split(",") if s.strip()] if affects_match else []

        decisions.append({
            "dd_id": dd_id,
            "ticket": ticket,
            "title": title,
            "rationale": rationale_match.group(1).strip() if rationale_match else None,
            "alternatives": alternatives,
            "trade_offs": tradeoffs_match.group(1).strip() if tradeoffs_match else None,
            "status": status_match.group(1).lower() if status_match else "active",
            "extraction_method": "structured",
            "source_file": file_path,
            "source_line": content[:match.start()].count("\n") + 1,
            "affects": affects,
        })

    return decisions


def parse_heuristic_decisions(content: str, file_path: str, ticket: str) -> list[dict]:
    """Extract implicit design decisions using heuristic patterns.

    Args:
        content: Full text of the document.
        file_path: Relative path to the file.
        ticket: Ticket number.

    Returns:
        List of decision dicts.
    """
    decisions = []
    seq = 1

    for pattern, pattern_type in HEURISTIC_PATTERNS:
        for match in pattern.finditer(content):
            text = match.group(1).strip()

            # Skip very short matches (likely false positives)
            if len(text) < 30:
                continue

            # Truncate very long matches
            if len(text) > 500:
                text = text[:500] + "..."

            dd_id = f"DD-{ticket}-H{seq}"
            title = _make_title(text, pattern_type)

            # Extract PascalCase identifiers as potential affected symbols
            affects = list(set(PASCAL_CASE_RE.findall(text)))

            decisions.append({
                "dd_id": dd_id,
                "ticket": ticket,
                "title": title,
                "rationale": text,
                "alternatives": None,
                "trade_offs": None,
                "status": "active",
                "extraction_method": "heuristic",
                "source_file": file_path,
                "source_line": content[:match.start()].count("\n") + 1,
                "affects": affects,
            })
            seq += 1

    return decisions


def _make_title(text: str, pattern_type: str) -> str:
    """Generate a short title from a heuristic match."""
    # Take first sentence or first 80 chars
    first_sentence = text.split(".")[0].strip()
    if len(first_sentence) > 80:
        first_sentence = first_sentence[:77] + "..."

    # Clean up markdown formatting
    first_sentence = re.sub(r"\*\*(.+?)\*\*", r"\1", first_sentence)
    first_sentence = re.sub(r"`(.+?)`", r"\1", first_sentence)

    prefix = {
        "design_rationale": "Rationale",
        "design_decision": "Decision",
        "recommendation": "Recommendation",
        "resolved_question": "Resolved",
        "why_explanation": "Why",
        "root_cause": "Root Cause",
    }.get(pattern_type, "Decision")

    return f"{prefix}: {first_sentence}"


def find_documents(repo_root: str) -> list[tuple[str, str]]:
    """Find all ticket and design documents.

    Returns:
        List of (absolute_path, relative_path) tuples.
    """
    root = Path(repo_root)
    docs = []

    # Design documents
    for md_file in (root / "docs" / "designs").rglob("*.md"):
        docs.append((str(md_file), str(md_file.relative_to(root))))

    # Ticket files
    for md_file in (root / "tickets").glob("*.md"):
        docs.append((str(md_file), str(md_file.relative_to(root))))

    return docs


def index_decisions(db_path: str, repo_root: str) -> None:
    """Index design decisions from tickets and design documents.

    Args:
        db_path: Path to the traceability SQLite database.
        repo_root: Path to the git repository root.
    """
    conn = create_schema(db_path)

    # Get existing DD IDs to support incremental indexing
    existing_ids = set()
    for row in conn.execute("SELECT dd_id FROM design_decisions"):
        existing_ids.add(row["dd_id"])

    documents = find_documents(repo_root)
    new_count = 0

    # Track heuristic sequence numbers per ticket to avoid DD ID collisions
    # when multiple files for the same ticket generate heuristic DDs
    heuristic_seq: dict[str, int] = {}

    for abs_path, rel_path in documents:
        ticket = extract_ticket_number(rel_path)
        if not ticket:
            continue

        try:
            content = Path(abs_path).read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue

        # Try structured extraction first (design docs)
        decisions = parse_structured_decisions(content, rel_path, ticket)

        # If no structured decisions found, try heuristic extraction
        if not decisions:
            start_seq = heuristic_seq.get(ticket, 0)
            decisions = parse_heuristic_decisions(content, rel_path, ticket)
            # Renumber heuristic IDs to avoid collisions across files
            for dd in decisions:
                start_seq += 1
                dd["dd_id"] = f"DD-{ticket}-H{start_seq}"
            heuristic_seq[ticket] = start_seq

        for dd in decisions:
            if dd["dd_id"] in existing_ids:
                continue

            cursor = conn.execute(
                """INSERT INTO design_decisions
                   (dd_id, ticket, title, rationale, alternatives, trade_offs,
                    status, extraction_method, source_file, source_line)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (dd["dd_id"], dd["ticket"], dd["title"], dd["rationale"],
                 dd["alternatives"], dd["trade_offs"], dd["status"],
                 dd["extraction_method"], dd["source_file"], dd.get("source_line")),
            )
            decision_id = cursor.lastrowid

            # Link to affected symbols
            for symbol in dd.get("affects", []):
                conn.execute(
                    "INSERT INTO decision_symbols (decision_id, symbol_name) VALUES (?, ?)",
                    (decision_id, symbol),
                )

            # Link to commits via ticket number
            snapshots = conn.execute(
                "SELECT id FROM snapshots WHERE ticket_number = ?",
                (dd["ticket"],),
            ).fetchall()
            for snap in snapshots:
                conn.execute(
                    "INSERT INTO decision_commits (decision_id, snapshot_id) VALUES (?, ?)",
                    (decision_id, snap["id"]),
                )

            new_count += 1

    conn.commit()

    # Rebuild FTS index
    rebuild_fts(conn)

    total = conn.execute("SELECT COUNT(*) FROM design_decisions").fetchone()[0]
    symbols = conn.execute("SELECT COUNT(*) FROM decision_symbols").fetchone()[0]
    print(f"Indexed {new_count} new decisions ({total} total), {symbols} symbol links")
    conn.close()


def main():
    parser = argparse.ArgumentParser(
        description="Index design decisions into traceability database"
    )
    parser.add_argument("database", help="Path to the traceability SQLite database")
    parser.add_argument("--repo", default=".", help="Path to the git repository root")

    args = parser.parse_args()

    root = Path(args.repo)
    if not (root / "tickets").exists() and not (root / "docs" / "designs").exists():
        print(f"Warning: No tickets/ or docs/designs/ found in {args.repo}", file=sys.stderr)

    index_decisions(args.database, args.repo)


if __name__ == "__main__":
    main()
