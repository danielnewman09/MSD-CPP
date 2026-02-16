# Ticket 0063: Replay MCP Server for Physics Debugging

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [x] Ready for Implementation
- [x] Implementation Complete
- [ ] Implementation Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete — Awaiting Quality Gate
**Type**: Feature / Tooling
**Priority**: High
**Created**: 2026-02-14
**Related Tickets**: [0056_browser_simulation_replay](0056_browser_simulation_replay.md), [0056d_fastapi_backend](0056d_fastapi_backend.md), [0056c_python_bindings](0056c_python_bindings.md)

---

## Summary

Build an MCP (Model Context Protocol) server that exposes the replay simulation database to Claude Code. The goal is AI-assisted physics debugging: the user observes physically inaccurate behavior in the replay viewer (e.g., "at frame 45 the friction contact seems wrong") and Claude can query the MCP server to retrieve frame data, constraint forces, energy timeseries, and solver diagnostics to diagnose the root cause.

This is fundamentally different from the browser-based replay viewer — the viewer is for human visual inspection, while the MCP server is for structured programmatic queries that support AI reasoning about physical correctness.

---

## Motivation

Physics debugging today follows a manual workflow:
1. User watches replay, spots something wrong visually
2. User describes the issue to Claude in natural language
3. Claude has no access to the simulation data — can only reason from C++ source code
4. User must manually query the database, copy-paste results, go back and forth

With an MCP server, step 3 becomes: Claude directly queries the recording database for the exact frame, body, and constraint data relevant to the user's observation. This closes the loop between "I see something wrong" and "here's what the solver actually computed."

---

## Architecture

```
User: "At frame 45, body 1 seems to slide when it shouldn't"
                |
                v
    Claude Code (conversation)
                |
                v
    MCP Server (Python, stdio transport)
                |
                v
    SimulationService (existing query layer)
                |
                v
    Simulation Recording Database (.db)
```

The MCP server is a thin wrapper around the existing `SimulationService` class from `replay/replay/services/simulation_service.py`. It exposes the same queries as MCP tools with structured JSON responses.

### Transport

Use **stdio** transport (same as the existing codebase and traceability MCP servers). Configured in `.claude/settings.json` alongside the existing MCP servers.

### Dependencies

- `mcp` Python SDK (pip install)
- Existing `msd_reader` pybind11 module (already built)
- Existing `replay.services.simulation_service.SimulationService`

---

## MCP Tools

### Core Query Tools

| Tool | Description | Parameters | Use Case |
|------|-------------|------------|----------|
| `list_recordings` | List available .db recording files | `directory?` (default: test output dir) | Find which recordings are available |
| `load_recording` | Open a recording database | `path` | Set the active recording for subsequent queries |
| `get_metadata` | Body properties and frame count | — | Understand simulation setup (masses, restitution, friction coefficients, body count) |
| `get_frame` | Complete frame data (states, collisions, friction, solver) | `frame_id` | Inspect everything at a specific moment |
| `get_body_state` | Single body's kinematic state at a frame | `body_id`, `frame_id` | Check position, velocity, orientation, angular velocity of one body |
| `get_contacts_for_body` | All friction constraints involving a body at a frame | `body_id`, `frame_id` | Inspect contact forces, friction forces, tangent directions for a specific body |

### Energy Analysis Tools

| Tool | Description | Parameters | Use Case |
|------|-------------|------------|----------|
| `get_body_energy` | Energy timeseries for one body | `body_id`, `start_frame?`, `end_frame?` | Track KE/PE/total for a body over time |
| `get_system_energy` | System-level energy timeseries | `start_frame?`, `end_frame?` | Detect conservation violations |
| `find_energy_anomalies` | Frames where `|delta_e|` exceeds a threshold | `threshold?` (default: 1.0 J) | Automatically find energy injection/loss events |

### Diagnostic Comparison Tools

| Tool | Description | Parameters | Use Case |
|------|-------------|------------|----------|
| `compare_body_across_frames` | Diff a body's state between two frames | `body_id`, `frame_a`, `frame_b` | Quantify unexpected motion/drift between frames |
| `get_contact_history` | Track contact pairs across a frame range | `body_a_id`, `body_b_id`, `start_frame`, `end_frame` | Detect contact chatter, normal flipping, force oscillation |
| `get_solver_diagnostics` | Solver convergence info for a frame range | `start_frame?`, `end_frame?` | Find frames where solver didn't converge or had high residual |

### Physics Validation Tools

| Tool | Description | Parameters | Use Case |
|------|-------------|------------|----------|
| `check_friction_cone` | Verify friction forces satisfy Coulomb cone | `frame_id`, `body_id?` | Detect friction constraint violations |
| `check_penetration` | Find contacts with excessive penetration depth | `frame_id`, `threshold?` | Detect collision resolution failures |
| `check_resting_contact` | Verify bodies at rest have near-zero velocity | `frame_id`, `velocity_threshold?` | Detect creep/drift in resting contacts |

---

## Example Interaction

```
User: "At frame 45, body 1 seems to be sliding on the floor even though
       it should be at rest. The friction coefficient is 0.5."

Claude: [calls get_frame tool with frame_id=45]
        [calls get_contacts_for_body with body_id=1, frame_id=45]
        [calls check_friction_cone with frame_id=45, body_id=1]

Claude: "Looking at frame 45, body 1 has a friction constraint with body 2
         (the floor). The normal force is 98.1 N (correct for a 10 kg body
         under gravity). The friction limit is mu * Fn = 0.5 * 98.1 = 49.05 N.
         However, the tangent forces are t1=49.04 N, t2=0.12 N — the friction
         is saturated at the Coulomb limit. The residual velocity is
         (0.003, 0.0, 0.0) m/s along tangent1.

         This suggests the applied tangential impulse from a previous frame
         exceeds what static friction can resist. Let me check the contact
         history..."

        [calls get_contact_history with body_a=1, body_b=2, start=40, end=50]
        [calls compare_body_across_frames with body_id=1, frame_a=44, frame_b=45]

Claude: "Between frames 44 and 45, body 1 moved 0.002m in X. The contact
         normal flipped sign at frame 43, which caused the solver to
         re-initialize the friction basis. This is the root cause — the EPA
         normal is oscillating between faces."
```

---

## Implementation Plan

### File Structure

```
replay/
├── mcp_server/
│   ├── __init__.py
│   ├── server.py            # MCP server entry point (stdio transport)
│   └── tools.py             # Tool definitions and handlers
└── replay/
    └── services/
        └── simulation_service.py  # Existing (reused, possibly extended)
```

### Configuration

Add to `.claude/settings.json`:
```json
{
  "mcpServers": {
    "replay": {
      "type": "stdio",
      "command": "python",
      "args": ["-m", "replay.mcp_server.server"],
      "cwd": "/path/to/MSD-CPP/replay"
    }
  }
}
```

### Implementation Steps

1. **Install MCP SDK** — Add `mcp` to replay Python dependencies
2. **Implement core tools** — `list_recordings`, `load_recording`, `get_metadata`, `get_frame`, `get_body_state`, `get_contacts_for_body`
3. **Implement energy tools** — `get_body_energy`, `get_system_energy`, `find_energy_anomalies`
4. **Implement comparison tools** — `compare_body_across_frames`, `get_contact_history`, `get_solver_diagnostics`
5. **Implement validation tools** — `check_friction_cone`, `check_penetration`, `check_resting_contact`
6. **Configure in settings.json** — Wire up as stdio MCP server
7. **Test end-to-end** — Load a recording, query frame data, verify structured output

---

## Design Considerations

### State Management

The MCP server needs to maintain an active recording session. Options:
- **Session-based**: `load_recording` sets the active DB; subsequent tools query it. Simple, matches the user's mental model of "I'm looking at this recording."
- **Per-call path**: Every tool takes a `db_path` parameter. More flexible but verbose.

**Recommendation**: Session-based with `load_recording`. Most debugging sessions focus on one recording at a time.

### Reuse vs. Duplication

The existing `SimulationService` already implements all core queries. The MCP server should import and call it directly rather than duplicating SQL queries. The diagnostic/validation tools (compare, friction cone check, etc.) are new logic that wraps multiple `SimulationService` calls.

### Output Format

MCP tools return JSON strings. Use Pydantic's `.model_dump_json()` for structured responses. For diagnostic tools, include human-readable summaries alongside raw data to help Claude reason about the results.

### Recording Discovery

Default recording location: `build/Debug/debug/recordings/` (where `ReplayEnabledTest` writes .db files). The `list_recordings` tool should also accept an explicit directory path.

---

## Acceptance Criteria

1. MCP server starts via stdio and registers all tools
2. `list_recordings` finds .db files in the default recording directory
3. `load_recording` + `get_frame` returns complete frame data as structured JSON
4. `find_energy_anomalies` correctly identifies frames with energy conservation violations
5. `check_friction_cone` correctly identifies friction force violations
6. `compare_body_across_frames` shows meaningful state deltas
7. Server is configured in `.claude/settings.json` and tools appear in Claude Code
8. End-to-end: user describes a physics issue, Claude queries the MCP server and provides a diagnosis

---

## Out of Scope

- Browser viewer integration (the MCP server is independent of the web UI)
- Write operations (the MCP server is read-only)
- Real-time streaming (queries are per-frame, not live)
- Automated fix suggestions (Claude reasons about the data; fixes are separate tickets)

---

## Workflow Log

### Design Phase
- **Started**: 2026-02-16 11:55
- **Completed**: 2026-02-16 12:10
- **Branch**: 0063-replay-mcp-server
- **PR**: #65 (draft)
- **Artifacts**:
  - `docs/designs/0063_replay_mcp_server/design.md`
  - `docs/designs/0063_replay_mcp_server/0063_replay_mcp_server.puml`
- **Notes**:
  - Design follows established MCP server pattern from codebase server (scripts/mcp_server.py)
  - Reuses existing SimulationService without modification
  - Session-based design: load_recording sets active DB for subsequent queries
  - 14 MCP tools across 4 categories: core queries, energy analysis, diagnostics, validation
  - No open questions or prototype required
  - Ready for design review

### Design Review Phase
- **Started**: 2026-02-16 13:30
- **Completed**: 2026-02-16 13:45
- **Branch**: 0063-replay-mcp-server
- **PR**: #65 (draft)
- **Artifacts**:
  - Design review appended to `docs/designs/0063_replay_mcp_server/design.md`
- **Notes**:
  - Status: APPROVED (no revisions needed)
  - All criteria passed: architectural fit, Python design quality, feasibility, testability
  - All risks are low-likelihood/low-impact (schema variations, large recordings, epsilon handling)
  - No prototype phase required — proceed directly to implementation
  - Review summary posted to PR #65

### Implementation Phase
- **Started**: 2026-02-16 14:30
- **Completed**: 2026-02-16 15:15
- **Branch**: 0063-replay-mcp-server
- **PR**: #65 (ready for review)
- **Artifacts**:
  - `replay/mcp_server/server.py` (750 LOC)
  - `replay/mcp_server/__init__.py`
  - `replay/mcp_server/__main__.py`
  - `docs/designs/0063_replay_mcp_server/implementation-notes.md`
- **Notes**:
  - All 14 MCP tools implemented per design specification
  - Follows stdio JSON-RPC pattern from existing MCP servers
  - Session-based design: `load_recording` sets active `SimulationService` instance
  - Reuses existing `SimulationService` without modification (zero impact on existing code)
  - Added `mcp>=1.0` and `numpy>=1.24` dependencies to `replay/pyproject.toml`
  - Configured `.mcp.json` with replay server entry
  - Updated `replay/README.md` with MCP server documentation
  - No deviations from design (minor internal adjustments documented in implementation-notes.md)
  - Test suite pending (next phase per design document)
