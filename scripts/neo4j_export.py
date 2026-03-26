#!/usr/bin/env python3
"""
Export Neo4j codebase graph as a Cypher script.

Generates a .cypher file containing CREATE statements for all nodes and
relationships, suitable for importing into another Neo4j instance or
reading as a schema + data reference.

Usage:
    python neo4j_export.py                          # stdout
    python neo4j_export.py -o codebase_graph.cypher # to file
"""

import argparse
import os
import sys
from typing import TextIO

from neo4j import GraphDatabase


def escape_cypher(value) -> str:
    """Escape a value for Cypher string literal."""
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    if isinstance(value, list):
        items = ", ".join(escape_cypher(v) for v in value)
        return f"[{items}]"
    # String — escape backslashes and single quotes
    s = str(value).replace("\\", "\\\\").replace("'", "\\'")
    return f"'{s}'"


def format_props(props: dict) -> str:
    """Format a dict as a Cypher property map."""
    parts = []
    for k, v in sorted(props.items()):
        if k == "element_id":
            continue
        parts.append(f"{k}: {escape_cypher(v)}")
    return "{" + ", ".join(parts) + "}"


def export_schema(session, out: TextIO):
    """Export constraints and indexes as Cypher."""
    out.write("// =============================================================\n")
    out.write("// Schema: constraints and indexes\n")
    out.write("// =============================================================\n\n")

    result = session.run("SHOW CONSTRAINTS")
    for record in result:
        name = record.get("name", "")
        constraint_type = record.get("type", "")
        entity_type = record.get("entityType", "")
        labels = record.get("labelsOrTypes", [])
        properties = record.get("properties", [])

        if constraint_type == "UNIQUENESS" and entity_type == "NODE" and labels and properties:
            label = labels[0]
            prop = properties[0]
            out.write(f"CREATE CONSTRAINT {name} IF NOT EXISTS FOR (n:{label}) REQUIRE n.{prop} IS UNIQUE;\n")

    result = session.run("SHOW INDEXES")
    for record in result:
        name = record.get("name", "")
        index_type = record.get("type", "")
        entity_type = record.get("entityType", "")
        labels = record.get("labelsOrTypes", [])
        properties = record.get("properties", [])
        owning_constraint = record.get("owningConstraint", None)

        # Skip indexes that back constraints (already emitted above)
        if owning_constraint:
            continue

        if index_type == "FULLTEXT" and labels and properties:
            labels_str = "|".join(labels)
            props_str = ", ".join(f"n.{p}" for p in properties)
            out.write(f"CREATE FULLTEXT INDEX {name} IF NOT EXISTS FOR (n:{labels_str}) ON EACH [{props_str}];\n")
        elif index_type == "RANGE" and entity_type == "NODE" and labels and properties:
            label = labels[0]
            prop = properties[0]
            out.write(f"CREATE INDEX {name} IF NOT EXISTS FOR (n:{label}) ON (n.{prop});\n")

    out.write("\n")


def export_nodes(session, out: TextIO):
    """Export all nodes as CREATE statements, grouped by label."""
    # Get all labels
    result = session.run("CALL db.labels() YIELD label RETURN label ORDER BY label")
    labels = [record["label"] for record in result]

    for label in labels:
        result = session.run(f"MATCH (n:{label}) RETURN n ORDER BY n.refid")
        nodes = list(result)
        if not nodes:
            continue

        out.write(f"// =============================================================\n")
        out.write(f"// {label} nodes ({len(nodes)})\n")
        out.write(f"// =============================================================\n\n")

        for record in nodes:
            node = record["n"]
            props = dict(node)
            out.write(f"CREATE (:{label} {format_props(props)});\n")

        out.write("\n")


def export_relationships(session, out: TextIO):
    """Export all relationships as MATCH/CREATE statements."""
    # Get all relationship types
    result = session.run("CALL db.relationshipTypes() YIELD relationshipType RETURN relationshipType ORDER BY relationshipType")
    rel_types = [record["relationshipType"] for record in result]

    for rel_type in rel_types:
        result = session.run(
            f"""
            MATCH (a)-[r:{rel_type}]->(b)
            RETURN labels(a)[0] AS src_label, a.refid AS src_refid,
                   labels(b)[0] AS dst_label, b.refid AS dst_refid,
                   properties(r) AS rel_props
            """
        )
        rels = list(result)
        if not rels:
            continue

        out.write(f"// =============================================================\n")
        out.write(f"// {rel_type} relationships ({len(rels)})\n")
        out.write(f"// =============================================================\n\n")

        for record in rels:
            src_label = record["src_label"]
            src_refid = record["src_refid"]
            dst_label = record["dst_label"]
            dst_refid = record["dst_refid"]
            rel_props = record["rel_props"] or {}

            # Parameters without refids (e.g. Parameter nodes) — use position-based match
            if dst_refid is None and dst_label == "Parameter":
                continue  # HAS_PARAMETER already implied by node creation
            if src_refid is None:
                continue

            props_str = f" {format_props(rel_props)}" if rel_props else ""
            out.write(
                f"MATCH (a:{src_label} {{refid: {escape_cypher(src_refid)}}}), "
                f"(b:{dst_label} {{refid: {escape_cypher(dst_refid)}}})\n"
                f"CREATE (a)-[:{rel_type}{props_str}]->(b);\n"
            )

        out.write("\n")


def main():
    parser = argparse.ArgumentParser(description="Export Neo4j codebase graph as Cypher")
    parser.add_argument("-o", "--output", help="Output file (default: stdout)")
    parser.add_argument("--uri", default=os.environ.get("NEO4J_URI", "bolt://localhost:7687"))
    parser.add_argument("--user", default=os.environ.get("NEO4J_USER", "neo4j"))
    parser.add_argument("--password", default=os.environ.get("NEO4J_PASSWORD", "msd-local-dev"))
    parser.add_argument("--database", default="neo4j")
    parser.add_argument("--schema-only", action="store_true",
                        help="Export only schema (constraints/indexes), no data")
    args = parser.parse_args()

    driver = GraphDatabase.driver(args.uri, auth=(args.user, args.password))
    try:
        driver.verify_connectivity()
    except Exception as e:
        print(f"Error: Could not connect to Neo4j: {e}", file=sys.stderr)
        sys.exit(1)

    out = open(args.output, "w") if args.output else sys.stdout

    try:
        with driver.session(database=args.database) as session:
            out.write("// Neo4j Codebase Graph Export\n")
            out.write("// Generated by neo4j_export.py\n\n")

            export_schema(session, out)

            if not args.schema_only:
                export_nodes(session, out)
                export_relationships(session, out)

            # Summary comment
            result = session.run(
                "MATCH (n) WITH labels(n)[0] AS l RETURN l, count(*) AS c ORDER BY c DESC"
            )
            out.write("// Summary:\n")
            for record in result:
                out.write(f"//   {record['l']}: {record['c']} nodes\n")

            result = session.run(
                "MATCH ()-[r]->() WITH type(r) AS t RETURN t, count(*) AS c ORDER BY c DESC"
            )
            for record in result:
                out.write(f"//   {record['t']}: {record['c']} relationships\n")

    finally:
        if args.output:
            out.close()
        driver.close()

    if args.output:
        print(f"Exported to {args.output}")


if __name__ == "__main__":
    main()
