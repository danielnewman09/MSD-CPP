#!/usr/bin/env python3
"""
MCP API Contract Server

A Model Context Protocol (MCP) server that provides tools for querying and
validating the authoritative API contract defined in contracts.yaml.

Usage:
    python mcp_api_contract_server.py <contracts_yaml_path>

Example:
    python mcp_api_contract_server.py docs/api-contracts/contracts.yaml

Tools provided:
    - list_endpoints: List all endpoints (method, path, summary)
    - get_endpoint: Full spec for one endpoint (params, request/response schemas)
    - search_endpoints: Search by path pattern or description
    - get_schema: Get a component schema definition
    - list_websocket_messages: List WS message types for an endpoint
    - get_websocket_message: Full schema for a WS message type
    - validate_contract_drift: Compare contracts.yaml against live server's /openapi.json
"""

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    from mcp.server.fastmcp import FastMCP
    HAS_MCP = True
except ImportError:
    HAS_MCP = False


class ApiContractServer:
    """Server for querying OpenAPI contract definitions."""

    def __init__(self, contracts_path: str):
        self.contracts_path = contracts_path
        with open(contracts_path, "r") as f:
            self.spec = yaml.safe_load(f)

    def list_endpoints(self) -> list[dict[str, str]]:
        """List all endpoints with method, path, and summary."""
        endpoints = []
        for path, methods in self.spec.get("paths", {}).items():
            for method, details in methods.items():
                if method.startswith("x-"):
                    continue
                endpoints.append({
                    "method": method.upper(),
                    "path": path,
                    "summary": details.get("summary", ""),
                    "tags": details.get("tags", []),
                    "operationId": details.get("operationId", ""),
                })
        return endpoints

    def get_endpoint(self, method: str, path: str) -> dict[str, Any]:
        """Get full specification for one endpoint."""
        path_spec = self.spec.get("paths", {}).get(path)
        if not path_spec:
            return {"error": f"Path '{path}' not found"}

        method_lower = method.lower()
        endpoint_spec = path_spec.get(method_lower)
        if not endpoint_spec:
            return {"error": f"Method '{method}' not found for path '{path}'"}

        # Resolve parameter references
        params = []
        for param in endpoint_spec.get("parameters", []):
            if "$ref" in param:
                resolved = self._resolve_ref(param["$ref"])
                if resolved:
                    params.append(resolved)
            else:
                params.append(param)

        # Resolve response schema references
        responses = {}
        for status, resp in endpoint_spec.get("responses", {}).items():
            resolved_resp = {"description": resp.get("description", "")}
            content = resp.get("content", {})
            for media_type, media_spec in content.items():
                schema = media_spec.get("schema", {})
                resolved_resp["schema"] = self._resolve_schema_deep(schema)
            responses[status] = resolved_resp

        return {
            "method": method.upper(),
            "path": path,
            "operationId": endpoint_spec.get("operationId", ""),
            "summary": endpoint_spec.get("summary", ""),
            "description": endpoint_spec.get("description", ""),
            "tags": endpoint_spec.get("tags", []),
            "parameters": params,
            "responses": responses,
        }

    def search_endpoints(self, query: str) -> list[dict[str, str]]:
        """Search endpoints by path pattern or description text."""
        query_lower = query.lower()
        results = []
        for path, methods in self.spec.get("paths", {}).items():
            for method, details in methods.items():
                if method.startswith("x-"):
                    continue
                searchable = f"{path} {details.get('summary', '')} {details.get('description', '')}".lower()
                if query_lower in searchable or re.search(query_lower, path):
                    results.append({
                        "method": method.upper(),
                        "path": path,
                        "summary": details.get("summary", ""),
                        "operationId": details.get("operationId", ""),
                    })
        return results

    def get_schema(self, schema_name: str) -> dict[str, Any]:
        """Get a component schema definition by name."""
        schemas = self.spec.get("components", {}).get("schemas", {})
        schema = schemas.get(schema_name)
        if not schema:
            # Try case-insensitive search
            for name, s in schemas.items():
                if name.lower() == schema_name.lower():
                    return {"name": name, "schema": self._resolve_schema_deep(s)}
            return {"error": f"Schema '{schema_name}' not found",
                    "available": sorted(schemas.keys())}
        return {"name": schema_name, "schema": self._resolve_schema_deep(schema)}

    def list_websocket_messages(self) -> dict[str, list[dict[str, str]]]:
        """List all WebSocket message types organized by direction."""
        ws_messages = self.spec.get("x-websocket-messages", {})
        result = {}
        for direction, messages in ws_messages.items():
            result[direction] = []
            for msg_type, msg_spec in messages.items():
                result[direction].append({
                    "type": msg_type,
                    "description": msg_spec.get("description", ""),
                })
        return result

    def get_websocket_message(self, direction: str, message_type: str) -> dict[str, Any]:
        """Get full schema for a WebSocket message type.

        Args:
            direction: 'client-to-server' or 'server-to-client'
            message_type: Message type name (e.g., 'configure', 'frame')
        """
        ws_messages = self.spec.get("x-websocket-messages", {})
        dir_messages = ws_messages.get(direction)
        if not dir_messages:
            return {"error": f"Direction '{direction}' not found",
                    "available": list(ws_messages.keys())}

        msg_spec = dir_messages.get(message_type)
        if not msg_spec:
            return {"error": f"Message type '{message_type}' not found in '{direction}'",
                    "available": list(dir_messages.keys())}

        return {
            "direction": direction,
            "type": message_type,
            "description": msg_spec.get("description", ""),
            "schema": self._resolve_schema_deep(msg_spec.get("schema", {})),
        }

    def validate_contract_drift(self, openapi_url: str = "http://localhost:8000/openapi.json") -> dict[str, Any]:
        """Compare contracts.yaml against a live server's /openapi.json.

        Args:
            openapi_url: URL to fetch the live OpenAPI spec from.

        Returns:
            Drift report with added, removed, and changed endpoints.
        """
        try:
            import urllib.request
            with urllib.request.urlopen(openapi_url, timeout=5) as resp:
                live_spec = json.loads(resp.read())
        except Exception as e:
            return {"error": f"Could not fetch live spec from {openapi_url}: {e}"}

        contract_paths = set()
        for path, methods in self.spec.get("paths", {}).items():
            for method in methods:
                if not method.startswith("x-"):
                    contract_paths.add((method.upper(), path))

        live_paths = set()
        for path, methods in live_spec.get("paths", {}).items():
            # Normalize path prefix: live spec may use /api/v1 prefix
            normalized = path
            for method in methods:
                if not method.startswith("x-"):
                    live_paths.add((method.upper(), normalized))

        # Compare
        in_contract_only = sorted(contract_paths - live_paths)
        in_live_only = sorted(live_paths - contract_paths)
        in_both = sorted(contract_paths & live_paths)

        # Check schema differences for shared endpoints
        schema_diffs = []
        for method, path in in_both:
            contract_ep = self.spec["paths"].get(path, {}).get(method.lower(), {})
            live_ep = live_spec.get("paths", {}).get(path, {}).get(method.lower(), {})

            contract_params = len(contract_ep.get("parameters", []))
            live_params = len(live_ep.get("parameters", []))
            if contract_params != live_params:
                schema_diffs.append({
                    "endpoint": f"{method} {path}",
                    "issue": f"Parameter count differs: contract={contract_params}, live={live_params}",
                })

        return {
            "status": "CLEAN" if not in_contract_only and not in_live_only and not schema_diffs else "DRIFT_DETECTED",
            "in_contract_only": [f"{m} {p}" for m, p in in_contract_only],
            "in_live_only": [f"{m} {p}" for m, p in in_live_only],
            "schema_differences": schema_diffs,
            "shared_endpoints": len(in_both),
        }

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------

    def _resolve_ref(self, ref: str) -> dict[str, Any] | None:
        """Resolve a $ref pointer within the spec."""
        parts = ref.lstrip("#/").split("/")
        obj = self.spec
        for part in parts:
            obj = obj.get(part, {})
            if not obj:
                return None
        return obj

    def _resolve_schema_deep(self, schema: dict[str, Any]) -> dict[str, Any]:
        """Recursively resolve $ref in a schema."""
        if "$ref" in schema:
            resolved = self._resolve_ref(schema["$ref"])
            if resolved:
                return self._resolve_schema_deep(resolved)
            return schema

        result = {}
        for key, value in schema.items():
            if isinstance(value, dict):
                result[key] = self._resolve_schema_deep(value)
            elif isinstance(value, list):
                result[key] = [
                    self._resolve_schema_deep(item) if isinstance(item, dict) else item
                    for item in value
                ]
            else:
                result[key] = value
        return result


def create_mcp_server(contracts_path: str) -> "FastMCP":
    """Create a FastMCP server wrapping the ApiContractServer."""
    server = ApiContractServer(contracts_path)
    mcp = FastMCP("api-contracts")

    @mcp.tool()
    def list_endpoints() -> str:
        """List all endpoints (method, path, summary)."""
        return json.dumps(server.list_endpoints(), indent=2)

    @mcp.tool()
    def get_endpoint(method: str, path: str) -> str:
        """Full spec for one endpoint (params, request/response schemas)."""
        return json.dumps(server.get_endpoint(method, path), indent=2, default=str)

    @mcp.tool()
    def search_endpoints(query: str) -> str:
        """Search by path pattern or description."""
        return json.dumps(server.search_endpoints(query), indent=2)

    @mcp.tool()
    def get_schema(schema_name: str) -> str:
        """Get a component schema definition."""
        return json.dumps(server.get_schema(schema_name), indent=2, default=str)

    @mcp.tool()
    def list_websocket_messages() -> str:
        """List WS message types for the live simulation endpoint."""
        return json.dumps(server.list_websocket_messages(), indent=2)

    @mcp.tool()
    def get_websocket_message(direction: str, message_type: str) -> str:
        """Full schema for a WS message type. Direction: 'client-to-server' or 'server-to-client'."""
        return json.dumps(server.get_websocket_message(direction, message_type), indent=2, default=str)

    @mcp.tool()
    def validate_contract_drift(openapi_url: str = "http://localhost:8000/openapi.json") -> str:
        """Compare contracts.yaml against live server's /openapi.json."""
        return json.dumps(server.validate_contract_drift(openapi_url), indent=2)

    return mcp


def main():
    parser = argparse.ArgumentParser(
        description="MCP API Contract Server - Query and validate API contracts",
    )
    parser.add_argument("contracts", help="Path to contracts.yaml")
    parser.add_argument("command", nargs="?", help="Command (omit for MCP server mode)")
    parser.add_argument("args", nargs="*", help="Command arguments")

    args = parser.parse_args()

    if not Path(args.contracts).exists():
        print(f"Error: Contracts file not found: {args.contracts}", file=sys.stderr)
        sys.exit(1)

    if not HAS_YAML:
        print("Error: PyYAML not installed. Run: pip install pyyaml", file=sys.stderr)
        sys.exit(1)

    if not args.command:
        if not HAS_MCP:
            print("Error: mcp package not installed. Run: pip install mcp", file=sys.stderr)
            sys.exit(1)
        mcp_server = create_mcp_server(args.contracts)
        mcp_server.run(transport="stdio")
        return

    server = ApiContractServer(args.contracts)

    if args.command == "list_endpoints":
        print(json.dumps(server.list_endpoints(), indent=2))
    elif args.command == "get_endpoint":
        method = args.args[0] if args.args else "GET"
        path = args.args[1] if len(args.args) > 1 else "/"
        print(json.dumps(server.get_endpoint(method, path), indent=2, default=str))
    elif args.command == "search_endpoints":
        query = args.args[0] if args.args else ""
        print(json.dumps(server.search_endpoints(query), indent=2))
    elif args.command == "get_schema":
        name = args.args[0] if args.args else ""
        print(json.dumps(server.get_schema(name), indent=2, default=str))
    elif args.command == "list_websocket_messages":
        print(json.dumps(server.list_websocket_messages(), indent=2))
    elif args.command == "get_websocket_message":
        direction = args.args[0] if args.args else ""
        msg_type = args.args[1] if len(args.args) > 1 else ""
        print(json.dumps(server.get_websocket_message(direction, msg_type), indent=2, default=str))
    elif args.command == "validate_contract_drift":
        url = args.args[0] if args.args else "http://localhost:8000/openapi.json"
        print(json.dumps(server.validate_contract_drift(url), indent=2))
    else:
        print(f"Unknown command: {args.command}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
