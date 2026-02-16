#!/usr/bin/env python3
"""
Replay MCP Server — Physics Debugging Tool Server

Ticket: 0063_replay_mcp_server
Design: docs/designs/0063_replay_mcp_server/design.md

A standards-compliant MCP server that provides physics debugging tools
using the simulation recording database. Enables AI-assisted diagnosis
of physics simulation issues by exposing structured queries for frame
data, energy analysis, constraint forces, and solver diagnostics.

Usage:
    python -m replay.mcp_server

Environment:
    MCP_REPLAY_DB: Path to default recording database (optional)

Tools:
    Core Query:
    - list_recordings: List available .db recording files
    - load_recording: Open a recording database
    - get_metadata: Body properties and frame count
    - get_frame: Complete frame data (states, collisions, friction, solver)
    - get_body_state: Single body's kinematic state at a frame
    - get_contacts_for_body: All friction constraints involving a body

    Energy Analysis:
    - get_body_energy: Energy timeseries for one body
    - get_system_energy: System-level energy timeseries
    - find_energy_anomalies: Frames where |delta_e| exceeds threshold

    Diagnostic Comparison:
    - compare_body_across_frames: State delta between two frames
    - get_contact_history: Track contact pairs across frame range
    - get_solver_diagnostics: Solver convergence info for frame range

    Physics Validation:
    - check_friction_cone: Verify friction forces satisfy Coulomb cone
    - check_penetration: Find contacts with excessive penetration depth
    - check_resting_contact: Verify bodies at rest have near-zero velocity
"""

import json
import os
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

# Import SimulationService from replay package
sys.path.insert(0, str(Path(__file__).parent.parent))
from replay.services.simulation_service import SimulationService
from replay.models import FrameData, BodyState, FrictionConstraintInfo


class ReplayMCPServer:
    """MCP-compliant server for physics debugging tools."""

    def __init__(self):
        """Initialize MCP server with stdio transport and empty session."""
        self.service: Optional[SimulationService] = None  # Active recording session
        self.tools = self._define_tools()
        self.default_recording_dir = Path.cwd() / "build" / "Debug" / "debug" / "recordings"

    def _define_tools(self) -> list[dict]:
        """Define available tools with their JSON schemas."""
        return [
            # Session Management
            {
                "name": "list_recordings",
                "description": "List available .db recording files in a directory. Defaults to build/Debug/debug/recordings/",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "directory": {
                            "type": "string",
                            "description": "Directory path to search (default: build/Debug/debug/recordings/)"
                        }
                    }
                }
            },
            {
                "name": "load_recording",
                "description": "Open a recording database and set it as the active session for subsequent queries. Returns metadata summary.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "path": {
                            "type": "string",
                            "description": "Path to the .db recording file"
                        }
                    },
                    "required": ["path"]
                }
            },
            # Core Query Tools
            {
                "name": "get_metadata",
                "description": "Get simulation metadata (body properties, frame count) from the active recording session.",
                "inputSchema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "get_frame",
                "description": "Get complete frame data (body states, collisions, friction constraints, solver diagnostics) for a specific frame.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "frame_id": {
                            "type": "integer",
                            "description": "Frame ID to query"
                        }
                    },
                    "required": ["frame_id"]
                }
            },
            {
                "name": "get_body_state",
                "description": "Get a single body's kinematic state (position, velocity, orientation, angular velocity) at a specific frame.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "body_id": {
                            "type": "integer",
                            "description": "Body ID to query"
                        },
                        "frame_id": {
                            "type": "integer",
                            "description": "Frame ID to query"
                        }
                    },
                    "required": ["body_id", "frame_id"]
                }
            },
            {
                "name": "get_contacts_for_body",
                "description": "Get all friction constraints involving a specific body at a frame (contact forces, friction forces, tangent directions).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "body_id": {
                            "type": "integer",
                            "description": "Body ID to query"
                        },
                        "frame_id": {
                            "type": "integer",
                            "description": "Frame ID to query"
                        }
                    },
                    "required": ["body_id", "frame_id"]
                }
            },
            # Energy Analysis Tools
            {
                "name": "get_body_energy",
                "description": "Get energy timeseries for one body (linear KE, rotational KE, potential E, total E) across a frame range.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "body_id": {
                            "type": "integer",
                            "description": "Body ID to query"
                        },
                        "start_frame": {
                            "type": "integer",
                            "description": "Starting frame ID (optional, default: 0)"
                        },
                        "end_frame": {
                            "type": "integer",
                            "description": "Ending frame ID (optional, default: last frame)"
                        }
                    },
                    "required": ["body_id"]
                }
            },
            {
                "name": "get_system_energy",
                "description": "Get system-level energy timeseries (total system E, delta E per frame) across a frame range.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "start_frame": {
                            "type": "integer",
                            "description": "Starting frame ID (optional, default: 0)"
                        },
                        "end_frame": {
                            "type": "integer",
                            "description": "Ending frame ID (optional, default: last frame)"
                        }
                    }
                }
            },
            {
                "name": "find_energy_anomalies",
                "description": "Find frames where |delta_e| exceeds a threshold (detects energy injection or loss events).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "threshold": {
                            "type": "number",
                            "description": "Energy change threshold in Joules (default: 1.0 J)",
                            "default": 1.0
                        }
                    }
                }
            },
            # Diagnostic Comparison Tools
            {
                "name": "compare_body_across_frames",
                "description": "Compare a body's state between two frames (computes position/velocity delta with human-readable summary).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "body_id": {
                            "type": "integer",
                            "description": "Body ID to compare"
                        },
                        "frame_a": {
                            "type": "integer",
                            "description": "First frame ID"
                        },
                        "frame_b": {
                            "type": "integer",
                            "description": "Second frame ID"
                        }
                    },
                    "required": ["body_id", "frame_a", "frame_b"]
                }
            },
            {
                "name": "get_contact_history",
                "description": "Track a contact pair across a frame range (detects contact chatter, normal flipping, force oscillation).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "body_a_id": {
                            "type": "integer",
                            "description": "First body ID"
                        },
                        "body_b_id": {
                            "type": "integer",
                            "description": "Second body ID"
                        },
                        "start_frame": {
                            "type": "integer",
                            "description": "Starting frame ID"
                        },
                        "end_frame": {
                            "type": "integer",
                            "description": "Ending frame ID"
                        }
                    },
                    "required": ["body_a_id", "body_b_id", "start_frame", "end_frame"]
                }
            },
            {
                "name": "get_solver_diagnostics",
                "description": "Get solver convergence info for a frame range (iterations, residual, convergence status).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "start_frame": {
                            "type": "integer",
                            "description": "Starting frame ID (optional, default: 0)"
                        },
                        "end_frame": {
                            "type": "integer",
                            "description": "Ending frame ID (optional, default: last frame)"
                        }
                    }
                }
            },
            # Physics Validation Tools
            {
                "name": "check_friction_cone",
                "description": "Verify friction forces satisfy the Coulomb cone constraint (|f_tangent| <= mu * f_normal) at a frame.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "frame_id": {
                            "type": "integer",
                            "description": "Frame ID to check"
                        },
                        "body_id": {
                            "type": "integer",
                            "description": "Specific body ID to check (optional, default: check all bodies)"
                        }
                    },
                    "required": ["frame_id"]
                }
            },
            {
                "name": "check_penetration",
                "description": "Find contacts with excessive penetration depth (indicates collision resolution failure).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "frame_id": {
                            "type": "integer",
                            "description": "Frame ID to check"
                        },
                        "threshold": {
                            "type": "number",
                            "description": "Penetration depth threshold in meters (default: 0.01 m)",
                            "default": 0.01
                        }
                    },
                    "required": ["frame_id"]
                }
            },
            {
                "name": "check_resting_contact",
                "description": "Verify bodies at rest have near-zero velocity (detects creep/drift in resting contacts).",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "frame_id": {
                            "type": "integer",
                            "description": "Frame ID to check"
                        },
                        "velocity_threshold": {
                            "type": "number",
                            "description": "Velocity threshold in m/s (default: 0.01 m/s)",
                            "default": 0.01
                        }
                    },
                    "required": ["frame_id"]
                }
            },
        ]

    def handle_list_tools(self, request_id: Any) -> dict:
        """Handle tools/list request."""
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "tools": self.tools
            }
        }

    def handle_call_tool(self, request_id: Any, tool_name: str, arguments: dict) -> dict:
        """Dispatch tool call to appropriate handler."""
        try:
            # Session management tools
            if tool_name == "list_recordings":
                result = self._list_recordings(arguments.get("directory"))
            elif tool_name == "load_recording":
                result = self._load_recording(arguments["path"])
            # Core query tools
            elif tool_name == "get_metadata":
                result = self._get_metadata()
            elif tool_name == "get_frame":
                result = self._get_frame(arguments["frame_id"])
            elif tool_name == "get_body_state":
                result = self._get_body_state(arguments["body_id"], arguments["frame_id"])
            elif tool_name == "get_contacts_for_body":
                result = self._get_contacts_for_body(arguments["body_id"], arguments["frame_id"])
            # Energy analysis tools
            elif tool_name == "get_body_energy":
                result = self._get_body_energy(
                    arguments["body_id"],
                    arguments.get("start_frame"),
                    arguments.get("end_frame")
                )
            elif tool_name == "get_system_energy":
                result = self._get_system_energy(
                    arguments.get("start_frame"),
                    arguments.get("end_frame")
                )
            elif tool_name == "find_energy_anomalies":
                result = self._find_energy_anomalies(arguments.get("threshold", 1.0))
            # Diagnostic comparison tools
            elif tool_name == "compare_body_across_frames":
                result = self._compare_body_across_frames(
                    arguments["body_id"],
                    arguments["frame_a"],
                    arguments["frame_b"]
                )
            elif tool_name == "get_contact_history":
                result = self._get_contact_history(
                    arguments["body_a_id"],
                    arguments["body_b_id"],
                    arguments["start_frame"],
                    arguments["end_frame"]
                )
            elif tool_name == "get_solver_diagnostics":
                result = self._get_solver_diagnostics(
                    arguments.get("start_frame"),
                    arguments.get("end_frame")
                )
            # Physics validation tools
            elif tool_name == "check_friction_cone":
                result = self._check_friction_cone(
                    arguments["frame_id"],
                    arguments.get("body_id")
                )
            elif tool_name == "check_penetration":
                result = self._check_penetration(
                    arguments["frame_id"],
                    arguments.get("threshold", 0.01)
                )
            elif tool_name == "check_resting_contact":
                result = self._check_resting_contact(
                    arguments["frame_id"],
                    arguments.get("velocity_threshold", 0.01)
                )
            else:
                return {
                    "jsonrpc": "2.0",
                    "id": request_id,
                    "error": {
                        "code": -32601,
                        "message": f"Unknown tool: {tool_name}"
                    }
                }

            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {
                    "content": [
                        {
                            "type": "text",
                            "text": json.dumps(result, indent=2)
                        }
                    ]
                }
            }

        except Exception as e:
            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "error": {
                    "code": -32603,
                    "message": f"Internal error: {str(e)}"
                }
            }

    def handle_message(self, message: dict) -> dict:
        """MCP protocol message handler (stdio JSON-RPC)."""
        method = message.get("method")
        request_id = message.get("id")

        if method == "initialize":
            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {
                        "tools": {}
                    },
                    "serverInfo": {
                        "name": "replay-mcp-server",
                        "version": "1.0.0"
                    }
                }
            }
        elif method == "tools/list":
            return self.handle_list_tools(request_id)
        elif method == "tools/call":
            params = message.get("params", {})
            return self.handle_call_tool(
                request_id,
                params.get("name"),
                params.get("arguments", {})
            )
        elif method == "notifications/initialized":
            # Ignore initialized notification
            return None
        else:
            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "error": {
                    "code": -32601,
                    "message": f"Unknown method: {method}"
                }
            }

    def run(self) -> None:
        """Main stdio event loop."""
        for line in sys.stdin:
            try:
                message = json.loads(line)
                response = self.handle_message(message)
                if response is not None:
                    print(json.dumps(response), flush=True)
            except Exception as e:
                error_response = {
                    "jsonrpc": "2.0",
                    "error": {"code": -32603, "message": str(e)},
                    "id": message.get("id") if isinstance(message, dict) else None
                }
                print(json.dumps(error_response), flush=True)

    # Tool Implementation Methods

    def _list_recordings(self, directory: Optional[str] = None) -> dict:
        """List available .db recording files."""
        search_dir = Path(directory) if directory else self.default_recording_dir
        if not search_dir.exists():
            return {
                "directory": str(search_dir),
                "recordings": [],
                "summary": f"Directory not found: {search_dir}"
            }

        recordings = [
            {
                "path": str(db_file),
                "name": db_file.name,
                "size_bytes": db_file.stat().st_size
            }
            for db_file in search_dir.glob("*.db")
        ]

        return {
            "directory": str(search_dir),
            "recordings": recordings,
            "summary": f"Found {len(recordings)} recording(s) in {search_dir}"
        }

    def _load_recording(self, path: str) -> dict:
        """Set active recording session."""
        db_path = Path(path)
        if not db_path.exists():
            return {
                "error": f"Recording not found: {path}",
                "loaded": False
            }

        try:
            self.service = SimulationService(db_path)
            metadata = self.service.get_metadata()
            return {
                "loaded": True,
                "path": str(db_path),
                "bodies": len(metadata.bodies),
                "total_frames": metadata.total_frames,
                "summary": f"Loaded recording with {len(metadata.bodies)} bodies, {metadata.total_frames} frames"
            }
        except Exception as e:
            return {
                "error": f"Failed to load recording: {str(e)}",
                "loaded": False
            }

    def _require_session(self) -> dict:
        """Check if recording session is active."""
        if self.service is None:
            return {
                "error": "No recording loaded. Call load_recording first."
            }
        return {}

    def _get_metadata(self) -> dict:
        """Get simulation metadata."""
        error = self._require_session()
        if error:
            return error

        metadata = self.service.get_metadata()
        return {
            "bodies": [
                {
                    "body_id": body.body_id,
                    "mass": body.mass,
                    "restitution": body.restitution,
                    "friction": body.friction,
                    "asset_id": body.asset_id,
                    "is_environment": body.is_environment
                }
                for body in metadata.bodies
            ],
            "total_frames": metadata.total_frames,
            "summary": f"Simulation with {len(metadata.bodies)} bodies, {metadata.total_frames} frames"
        }

    def _get_frame(self, frame_id: int) -> dict:
        """Get complete frame data."""
        error = self._require_session()
        if error:
            return error

        try:
            frame = self.service.get_frame_data(frame_id)
            return json.loads(frame.model_dump_json())
        except Exception as e:
            return {"error": f"Failed to get frame {frame_id}: {str(e)}"}

    def _get_body_state(self, body_id: int, frame_id: int) -> dict:
        """Get single body's kinematic state."""
        error = self._require_session()
        if error:
            return error

        try:
            frame = self.service.get_frame_data(frame_id)
            state = next((s for s in frame.states if s.body_id == body_id), None)
            if state is None:
                return {"error": f"Body {body_id} not found in frame {frame_id}"}
            return json.loads(state.model_dump_json())
        except Exception as e:
            return {"error": f"Failed to get body state: {str(e)}"}

    def _get_contacts_for_body(self, body_id: int, frame_id: int) -> dict:
        """Get friction constraints involving body."""
        error = self._require_session()
        if error:
            return error

        try:
            frame = self.service.get_frame_data(frame_id)
            contacts = [
                json.loads(fc.model_dump_json())
                for fc in frame.friction_constraints
                if fc.body_a_id == body_id or fc.body_b_id == body_id
            ]
            return {
                "body_id": body_id,
                "frame_id": frame_id,
                "contacts": contacts,
                "summary": f"Found {len(contacts)} contact(s) involving body {body_id}"
            }
        except Exception as e:
            return {"error": f"Failed to get contacts: {str(e)}"}

    def _get_body_energy(
        self,
        body_id: int,
        start_frame: Optional[int] = None,
        end_frame: Optional[int] = None
    ) -> dict:
        """Get energy timeseries for one body."""
        error = self._require_session()
        if error:
            return error

        try:
            energy_points = self.service.get_energy_by_body(body_id)
            if start_frame is not None:
                energy_points = [e for e in energy_points if e.frame_id >= start_frame]
            if end_frame is not None:
                energy_points = [e for e in energy_points if e.frame_id <= end_frame]

            return {
                "body_id": body_id,
                "energy_points": [json.loads(e.model_dump_json()) for e in energy_points],
                "summary": f"Retrieved {len(energy_points)} energy point(s) for body {body_id}"
            }
        except Exception as e:
            return {"error": f"Failed to get body energy: {str(e)}"}

    def _get_system_energy(
        self,
        start_frame: Optional[int] = None,
        end_frame: Optional[int] = None
    ) -> dict:
        """Get system-level energy timeseries."""
        error = self._require_session()
        if error:
            return error

        try:
            energy_points = self.service.get_system_energy()
            if start_frame is not None:
                energy_points = [e for e in energy_points if e.frame_id >= start_frame]
            if end_frame is not None:
                energy_points = [e for e in energy_points if e.frame_id <= end_frame]

            return {
                "energy_points": [json.loads(e.model_dump_json()) for e in energy_points],
                "summary": f"Retrieved {len(energy_points)} system energy point(s)"
            }
        except Exception as e:
            return {"error": f"Failed to get system energy: {str(e)}"}

    def _find_energy_anomalies(self, threshold: float = 1.0) -> dict:
        """Find frames where |delta_e| exceeds threshold."""
        error = self._require_session()
        if error:
            return error

        try:
            energy_points = self.service.get_system_energy()
            anomalies = [
                {
                    "frame_id": e.frame_id,
                    "simulation_time": e.simulation_time,
                    "delta_e": e.delta_e,
                    "total_system_e": e.total_system_e
                }
                for e in energy_points
                if abs(e.delta_e) > threshold
            ]

            return {
                "threshold": threshold,
                "anomalies": anomalies,
                "summary": f"Found {len(anomalies)} anomal(y|ies) with |delta_e| > {threshold} J"
            }
        except Exception as e:
            return {"error": f"Failed to find energy anomalies: {str(e)}"}

    def _compare_body_across_frames(
        self,
        body_id: int,
        frame_a: int,
        frame_b: int
    ) -> dict:
        """Compare body state between two frames."""
        error = self._require_session()
        if error:
            return error

        try:
            frame_data_a = self.service.get_frame_data(frame_a)
            frame_data_b = self.service.get_frame_data(frame_b)

            state_a = next((s for s in frame_data_a.states if s.body_id == body_id), None)
            state_b = next((s for s in frame_data_b.states if s.body_id == body_id), None)

            if state_a is None or state_b is None:
                return {"error": f"Body {body_id} not found in one or both frames"}

            pos_delta = {
                "x": state_b.position.x - state_a.position.x,
                "y": state_b.position.y - state_a.position.y,
                "z": state_b.position.z - state_a.position.z,
            }
            vel_delta = {
                "x": state_b.velocity.x - state_a.velocity.x,
                "y": state_b.velocity.y - state_a.velocity.y,
                "z": state_b.velocity.z - state_a.velocity.z,
            }

            pos_mag = float(np.linalg.norm([pos_delta["x"], pos_delta["y"], pos_delta["z"]]))
            vel_mag = float(np.linalg.norm([vel_delta["x"], vel_delta["y"], vel_delta["z"]]))

            return {
                "body_id": body_id,
                "frame_a": frame_a,
                "frame_b": frame_b,
                "position_delta": pos_delta,
                "velocity_delta": vel_delta,
                "summary": f"Body moved {pos_mag:.4f}m, velocity changed by {vel_mag:.4f} m/s between frames {frame_a} and {frame_b}"
            }
        except Exception as e:
            return {"error": f"Failed to compare frames: {str(e)}"}

    def _get_contact_history(
        self,
        body_a_id: int,
        body_b_id: int,
        start_frame: int,
        end_frame: int
    ) -> dict:
        """Track contact pair across frame range."""
        error = self._require_session()
        if error:
            return error

        try:
            contact_history = []
            for frame_id in range(start_frame, end_frame + 1):
                try:
                    frame = self.service.get_frame_data(frame_id)
                    contacts = [
                        fc for fc in frame.friction_constraints
                        if (fc.body_a_id == body_a_id and fc.body_b_id == body_b_id) or
                           (fc.body_a_id == body_b_id and fc.body_b_id == body_a_id)
                    ]
                    if contacts:
                        contact_history.append({
                            "frame_id": frame_id,
                            "simulation_time": frame.simulation_time,
                            "num_contacts": len(contacts),
                            "contacts": [json.loads(c.model_dump_json()) for c in contacts]
                        })
                except Exception:
                    pass  # Frame may not exist or have contacts

            return {
                "body_a_id": body_a_id,
                "body_b_id": body_b_id,
                "start_frame": start_frame,
                "end_frame": end_frame,
                "contact_history": contact_history,
                "summary": f"Found contacts in {len(contact_history)} frame(s) between bodies {body_a_id} and {body_b_id}"
            }
        except Exception as e:
            return {"error": f"Failed to get contact history: {str(e)}"}

    def _get_solver_diagnostics(
        self,
        start_frame: Optional[int] = None,
        end_frame: Optional[int] = None
    ) -> dict:
        """Get solver convergence info for frame range."""
        error = self._require_session()
        if error:
            return error

        try:
            metadata = self.service.get_metadata()
            start = start_frame if start_frame is not None else 0
            end = end_frame if end_frame is not None else metadata.total_frames - 1

            diagnostics = []
            for frame_id in range(start, end + 1):
                try:
                    frame = self.service.get_frame_data(frame_id)
                    if frame.solver:
                        diagnostics.append({
                            "frame_id": frame_id,
                            "simulation_time": frame.simulation_time,
                            "iterations": frame.solver.iterations,
                            "residual": frame.solver.residual,
                            "converged": frame.solver.converged,
                            "num_constraints": frame.solver.num_constraints,
                            "num_contacts": frame.solver.num_contacts
                        })
                except Exception:
                    pass  # Frame may not have solver diagnostics

            return {
                "start_frame": start,
                "end_frame": end,
                "diagnostics": diagnostics,
                "summary": f"Retrieved solver diagnostics for {len(diagnostics)} frame(s)"
            }
        except Exception as e:
            return {"error": f"Failed to get solver diagnostics: {str(e)}"}

    def _check_friction_cone(
        self,
        frame_id: int,
        body_id: Optional[int] = None
    ) -> dict:
        """Verify friction forces satisfy Coulomb cone."""
        error = self._require_session()
        if error:
            return error

        try:
            frame = self.service.get_frame_data(frame_id)
            contacts = frame.friction_constraints
            if body_id is not None:
                contacts = [c for c in contacts if c.body_a_id == body_id or c.body_b_id == body_id]

            violations = []
            for contact in contacts:
                normal_force = float(abs(contact.normal_lambda))
                friction_limit = float(contact.friction_coefficient * normal_force)
                tangent_force = float(np.sqrt(
                    contact.tangent1_lambda**2 + contact.tangent2_lambda**2
                ))

                # Use 1% margin to account for floating-point epsilon
                margin = float(friction_limit * 1.01 - tangent_force)
                exceeds = bool(tangent_force > friction_limit * 1.01)

                violations.append({
                    "body_a_id": contact.body_a_id,
                    "body_b_id": contact.body_b_id,
                    "friction_coefficient": contact.friction_coefficient,
                    "normal_force": normal_force,
                    "friction_limit": friction_limit,
                    "tangent1_force": contact.tangent1_lambda,
                    "tangent2_force": contact.tangent2_lambda,
                    "total_tangent_force": tangent_force,
                    "exceeds_limit": exceeds,
                    "margin": margin
                })

            num_violations = sum(1 for v in violations if v["exceeds_limit"])
            return {
                "frame_id": frame_id,
                "body_id": body_id,
                "violations": violations,
                "summary": f"{len(contacts)} contact(s) checked. {num_violations} violation(s) detected."
            }
        except Exception as e:
            return {"error": f"Failed to check friction cone: {str(e)}"}

    def _check_penetration(self, frame_id: int, threshold: float = 0.01) -> dict:
        """Find contacts with excessive penetration depth."""
        error = self._require_session()
        if error:
            return error

        try:
            frame = self.service.get_frame_data(frame_id)
            excessive_penetrations = [
                {
                    "body_a_id": collision.body_a_id,
                    "body_b_id": collision.body_b_id,
                    "penetration_depth": collision.penetration_depth,
                    "normal": {
                        "x": collision.normal.x,
                        "y": collision.normal.y,
                        "z": collision.normal.z
                    }
                }
                for collision in frame.collisions
                if collision.penetration_depth > threshold
            ]

            return {
                "frame_id": frame_id,
                "threshold": threshold,
                "excessive_penetrations": excessive_penetrations,
                "summary": f"Found {len(excessive_penetrations)} collision(s) with penetration > {threshold}m"
            }
        except Exception as e:
            return {"error": f"Failed to check penetration: {str(e)}"}

    def _check_resting_contact(
        self,
        frame_id: int,
        velocity_threshold: float = 0.01
    ) -> dict:
        """Verify bodies at rest have near-zero velocity."""
        error = self._require_session()
        if error:
            return error

        try:
            frame = self.service.get_frame_data(frame_id)
            metadata = self.service.get_metadata()

            # Bodies with contacts (candidates for resting contact)
            body_ids_in_contact = set()
            for fc in frame.friction_constraints:
                body_ids_in_contact.add(fc.body_a_id)
                body_ids_in_contact.add(fc.body_b_id)

            # Exclude environment bodies (they are static by definition)
            environment_ids = {b.body_id for b in metadata.bodies if b.is_environment}
            body_ids_in_contact -= environment_ids

            # Check velocity for each body in contact
            moving_bodies = []
            for body_id in body_ids_in_contact:
                state = next((s for s in frame.states if s.body_id == body_id), None)
                if state:
                    vel_mag = float(np.linalg.norm([state.velocity.x, state.velocity.y, state.velocity.z]))
                    ang_vel_mag = float(np.linalg.norm([
                        state.angular_velocity.x,
                        state.angular_velocity.y,
                        state.angular_velocity.z
                    ]))
                    if vel_mag > velocity_threshold or ang_vel_mag > velocity_threshold:
                        moving_bodies.append({
                            "body_id": body_id,
                            "velocity_magnitude": vel_mag,
                            "angular_velocity_magnitude": ang_vel_mag
                        })

            return {
                "frame_id": frame_id,
                "velocity_threshold": velocity_threshold,
                "bodies_in_contact": len(body_ids_in_contact),
                "moving_bodies": moving_bodies,
                "summary": f"{len(moving_bodies)}/{len(body_ids_in_contact)} bodies in contact are moving (velocity > {velocity_threshold} m/s)"
            }
        except Exception as e:
            return {"error": f"Failed to check resting contact: {str(e)}"}


def main():
    """Entry point for MCP server."""
    server = ReplayMCPServer()
    server.run()


if __name__ == "__main__":
    main()
