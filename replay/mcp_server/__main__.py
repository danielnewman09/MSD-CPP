"""
MCP Server Entry Point

Ticket: 0063_replay_mcp_server
Design: docs/designs/0063_replay_mcp_server/design.md

Allows running the server via: python -m replay.mcp_server
"""

from .server import main

if __name__ == "__main__":
    main()
