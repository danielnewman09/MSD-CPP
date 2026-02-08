# Scripts

## MCP Codebase Server

An MCP (Model Context Protocol) server that provides codebase navigation tools using a SQLite database generated from Doxygen XML output.

### Setup

1. Generate the database:
   ```bash
   cmake --build --preset doxygen-db
   ```
   Database is generated at `build/{build_type}/docs/codebase.db`.

2. Add to your Claude configuration (see `scripts/mcp_config.json`).

### CLI Usage

The server can also be used directly from the command line:

```bash
# Show available commands
python3 scripts/mcp_codebase_server.py build/Debug/docs/codebase.db

# Search for symbols
python3 scripts/mcp_codebase_server.py build/Debug/docs/codebase.db search_symbols Convex

# Find a class
python3 scripts/mcp_codebase_server.py build/Debug/docs/codebase.db find_class ConvexHull

# Get class members
python3 scripts/mcp_codebase_server.py build/Debug/docs/codebase.db get_class_members ConvexHull

# Find callers of a function
python3 scripts/mcp_codebase_server.py build/Debug/docs/codebase.db get_callers contains

# Search documentation
python3 scripts/mcp_codebase_server.py build/Debug/docs/codebase.db search_documentation collision
```

### Available Commands

| Command | Description |
|---------|-------------|
| `search_symbols` | Full-text search across all symbols |
| `find_class` | Find class/struct by name with details |
| `find_function` | Find function by name with signature |
| `get_class_hierarchy` | Get inheritance hierarchy (base + derived) |
| `get_callers` | Find all functions that call a given function |
| `get_callees` | Find all functions called by a given function |
| `get_file_symbols` | List all symbols defined in a file |
| `get_includes` | Get include dependencies for a file |
| `get_class_members` | Get all members of a class (categorized) |
| `search_documentation` | Full-text search in documentation |
| `get_statistics` | Get database statistics |
| `list_classes` | List all classes in the codebase |
