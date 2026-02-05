#!/usr/bin/env python3
"""
MCP Codebase Server - Model Context Protocol Implementation

A standards-compliant MCP server that provides codebase navigation tools
using the SQLite database generated from Doxygen XML output.

This server implements the MCP specification for tool-based interaction,
allowing AI assistants to efficiently query and navigate the codebase.

Usage:
    python mcp_server.py <database_path>

Environment:
    MCP_CODEBASE_DB: Path to the SQLite database (alternative to CLI arg)

Tools:
    - search_symbols: Full-text search across all symbols
    - find_class: Find a class/struct by name
    - find_function: Find a function by name
    - get_class_hierarchy: Get inheritance hierarchy
    - get_callers: Find all callers of a function
    - get_callees: Find all functions called by a function
    - get_file_symbols: List symbols defined in a file
    - get_includes: Get include dependencies
    - get_class_members: Get all members of a class
    - search_documentation: Full-text search in docs
    - get_statistics: Get database statistics
    - list_classes: List all classes in the codebase
"""

import json
import os
import sqlite3
import sys
from pathlib import Path
from typing import Any

# Import the CodebaseServer from the query module
from mcp_codebase_server import CodebaseServer


class MCPServer:
    """MCP-compliant server wrapper for CodebaseServer."""

    def __init__(self, db_path: str):
        self.server = CodebaseServer(db_path)
        self.tools = self._define_tools()

    def _define_tools(self) -> list[dict]:
        """Define available tools with their schemas."""
        return [
            {
                "name": "search_symbols",
                "description": "Search for symbols (classes, functions, variables) by name. Returns matching symbols with file locations and descriptions.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Search term (supports partial matching)"
                        },
                        "kind": {
                            "type": "string",
                            "description": "Filter by kind: class, struct, function, variable, typedef, enum",
                            "enum": ["class", "struct", "function", "variable", "typedef", "enum"]
                        },
                        "limit": {
                            "type": "integer",
                            "description": "Maximum number of results (default: 20)",
                            "default": 20
                        }
                    },
                    "required": ["query"]
                }
            },
            {
                "name": "find_class",
                "description": "Find a class or struct by name. Returns class details including base classes, description, and file location.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "name": {
                            "type": "string",
                            "description": "Class or struct name to search for"
                        },
                        "exact": {
                            "type": "boolean",
                            "description": "If true, match name exactly; if false, use pattern matching",
                            "default": False
                        }
                    },
                    "required": ["name"]
                }
            },
            {
                "name": "find_function",
                "description": "Find a function by name. Returns function signature, parameters, and location.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "name": {
                            "type": "string",
                            "description": "Function name to search for"
                        },
                        "class_name": {
                            "type": "string",
                            "description": "Optional class name to scope the search"
                        },
                        "exact": {
                            "type": "boolean",
                            "description": "If true, match name exactly",
                            "default": False
                        }
                    },
                    "required": ["name"]
                }
            },
            {
                "name": "get_class_hierarchy",
                "description": "Get the inheritance hierarchy for a class, including base classes and derived classes.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "class_name": {
                            "type": "string",
                            "description": "Name of the class"
                        }
                    },
                    "required": ["class_name"]
                }
            },
            {
                "name": "get_callers",
                "description": "Find all functions that call a given function (who calls this?).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "function_name": {
                            "type": "string",
                            "description": "Name of the function to find callers for"
                        },
                        "class_name": {
                            "type": "string",
                            "description": "Optional class name to scope the search"
                        }
                    },
                    "required": ["function_name"]
                }
            },
            {
                "name": "get_callees",
                "description": "Find all functions called by a given function (what does this call?).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "function_name": {
                            "type": "string",
                            "description": "Name of the function to find callees for"
                        },
                        "class_name": {
                            "type": "string",
                            "description": "Optional class name to scope the search"
                        }
                    },
                    "required": ["function_name"]
                }
            },
            {
                "name": "get_file_symbols",
                "description": "List all symbols (classes, functions, variables) defined in a file.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "file_path": {
                            "type": "string",
                            "description": "File path (can be partial match, e.g., 'ConvexHull.hpp')"
                        }
                    },
                    "required": ["file_path"]
                }
            },
            {
                "name": "get_includes",
                "description": "Get include dependencies for a file (what it includes and what includes it).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "file_path": {
                            "type": "string",
                            "description": "File path (can be partial match)"
                        }
                    },
                    "required": ["file_path"]
                }
            },
            {
                "name": "get_class_members",
                "description": "Get all members of a class, categorized by type (constructors, methods, variables, etc.).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "class_name": {
                            "type": "string",
                            "description": "Name of the class"
                        },
                        "include_private": {
                            "type": "boolean",
                            "description": "Include private members (default: true)",
                            "default": True
                        },
                        "kind": {
                            "type": "string",
                            "description": "Filter by member kind",
                            "enum": ["function", "variable", "typedef", "enum"]
                        }
                    },
                    "required": ["class_name"]
                }
            },
            {
                "name": "get_function_parameters",
                "description": "Get detailed parameter information for a function.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "function_name": {
                            "type": "string",
                            "description": "Name of the function"
                        },
                        "class_name": {
                            "type": "string",
                            "description": "Optional class name to scope the search"
                        }
                    },
                    "required": ["function_name"]
                }
            },
            {
                "name": "search_documentation",
                "description": "Full-text search in documentation. Uses FTS5 for efficient searching.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Search term (FTS5 syntax supported)"
                        },
                        "limit": {
                            "type": "integer",
                            "description": "Maximum results (default: 20)",
                            "default": 20
                        }
                    },
                    "required": ["query"]
                }
            },
            {
                "name": "get_statistics",
                "description": "Get database statistics: counts of files, classes, functions, etc.",
                "inputSchema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "list_namespaces",
                "description": "List all namespaces in the codebase.",
                "inputSchema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "list_classes",
                "description": "List all classes in the codebase, optionally filtered by namespace.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "namespace": {
                            "type": "string",
                            "description": "Optional namespace to filter by"
                        }
                    }
                }
            }
        ]

    def handle_request(self, request: dict) -> dict:
        """Handle an MCP request and return response."""
        method = request.get("method", "")
        params = request.get("params", {})
        request_id = request.get("id")

        try:
            if method == "initialize":
                return self._handle_initialize(request_id, params)
            elif method == "tools/list":
                return self._handle_tools_list(request_id)
            elif method == "tools/call":
                return self._handle_tools_call(request_id, params)
            else:
                return self._error_response(request_id, -32601, f"Method not found: {method}")
        except Exception as e:
            return self._error_response(request_id, -32603, str(e))

    def _handle_initialize(self, request_id: Any, params: dict) -> dict:
        """Handle initialize request."""
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "protocolVersion": "2024-11-05",
                "capabilities": {
                    "tools": {}
                },
                "serverInfo": {
                    "name": "msd-codebase-server",
                    "version": "1.0.0"
                }
            }
        }

    def _handle_tools_list(self, request_id: Any) -> dict:
        """Handle tools/list request."""
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "tools": self.tools
            }
        }

    def _handle_tools_call(self, request_id: Any, params: dict) -> dict:
        """Handle tools/call request."""
        tool_name = params.get("name", "")
        arguments = params.get("arguments", {})

        # Dispatch to appropriate method
        method_map = {
            "search_symbols": lambda: self.server.search_symbols(
                arguments.get("query", ""),
                arguments.get("kind"),
                arguments.get("limit", 20)
            ),
            "find_class": lambda: self.server.find_class(
                arguments.get("name", ""),
                arguments.get("exact", False)
            ),
            "find_function": lambda: self.server.find_function(
                arguments.get("name", ""),
                arguments.get("class_name"),
                arguments.get("exact", False)
            ),
            "get_class_hierarchy": lambda: self.server.get_class_hierarchy(
                arguments.get("class_name", "")
            ),
            "get_callers": lambda: self.server.get_callers(
                arguments.get("function_name", ""),
                arguments.get("class_name")
            ),
            "get_callees": lambda: self.server.get_callees(
                arguments.get("function_name", ""),
                arguments.get("class_name")
            ),
            "get_file_symbols": lambda: self.server.get_file_symbols(
                arguments.get("file_path", "")
            ),
            "get_includes": lambda: self.server.get_includes(
                arguments.get("file_path", "")
            ),
            "get_class_members": lambda: self.server.get_class_members(
                arguments.get("class_name", ""),
                arguments.get("include_private", True),
                arguments.get("kind")
            ),
            "get_function_parameters": lambda: self.server.get_function_parameters(
                arguments.get("function_name", ""),
                arguments.get("class_name")
            ),
            "search_documentation": lambda: self.server.search_documentation(
                arguments.get("query", ""),
                arguments.get("limit", 20)
            ),
            "get_statistics": lambda: self.server.get_statistics(),
            "list_namespaces": lambda: self.server.list_namespaces(),
            "list_classes": lambda: self.server.list_classes(
                arguments.get("namespace")
            ),
        }

        if tool_name not in method_map:
            return self._error_response(request_id, -32602, f"Unknown tool: {tool_name}")

        result = method_map[tool_name]()

        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "content": [
                    {
                        "type": "text",
                        "text": json.dumps(result, indent=2, default=str)
                    }
                ]
            }
        }

    def _error_response(self, request_id: Any, code: int, message: str) -> dict:
        """Create an error response."""
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "error": {
                "code": code,
                "message": message
            }
        }

    def run_stdio(self):
        """Run the server using stdio transport."""
        while True:
            try:
                # Read Content-Length header
                line = sys.stdin.readline()
                if not line:
                    break

                # Skip empty lines
                line = line.strip()
                if not line:
                    continue

                # Parse Content-Length
                if line.startswith("Content-Length:"):
                    content_length = int(line.split(":")[1].strip())

                    # Read empty line after headers
                    sys.stdin.readline()

                    # Read the JSON content
                    content = sys.stdin.read(content_length)
                    request = json.loads(content)

                    # Handle the request
                    response = self.handle_request(request)

                    # Send the response
                    response_json = json.dumps(response)
                    response_bytes = response_json.encode('utf-8')

                    sys.stdout.write(f"Content-Length: {len(response_bytes)}\r\n\r\n")
                    sys.stdout.write(response_json)
                    sys.stdout.flush()

            except json.JSONDecodeError as e:
                sys.stderr.write(f"JSON decode error: {e}\n")
            except Exception as e:
                sys.stderr.write(f"Error: {e}\n")

    def close(self):
        """Close the server."""
        self.server.close()


def main():
    # Get database path from argument or environment
    db_path = None
    if len(sys.argv) > 1:
        db_path = sys.argv[1]
    else:
        db_path = os.environ.get("MCP_CODEBASE_DB")

    if not db_path:
        print("Usage: python mcp_server.py <database_path>", file=sys.stderr)
        print("   or: MCP_CODEBASE_DB=<path> python mcp_server.py", file=sys.stderr)
        sys.exit(1)

    if not Path(db_path).exists():
        print(f"Error: Database not found: {db_path}", file=sys.stderr)
        sys.exit(1)

    server = MCPServer(db_path)
    try:
        server.run_stdio()
    finally:
        server.close()


if __name__ == "__main__":
    main()
