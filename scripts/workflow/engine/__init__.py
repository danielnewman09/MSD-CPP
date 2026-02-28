"""
Workflow Engine — SQLite-backed multi-agent work queue.

Ticket: 0083_database_agent_orchestration
Design: docs/designs/0083_database_agent_orchestration/design.md

This package is the engine core. It is project-agnostic: all project-specific
configuration (phase definitions, agent types, timeouts) comes from the consuming
repository's .workflow/ directory.
"""
