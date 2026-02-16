# Design: Replay MCP Server for Physics Debugging

## Summary

Add an MCP (Model Context Protocol) server that exposes the simulation recording database to Claude Code via stdio transport. The server wraps the existing `SimulationService` query layer and provides structured tools for AI-assisted physics debugging. This enables Claude to directly query frame data, energy timeseries, constraint forces, and solver diagnostics when users observe physically incorrect behavior in the replay viewer.

## Architecture Changes

### PlantUML Diagram
See: `./0063_replay_mcp_server.puml`

### New Components

#### ReplayMCPServer
- **Purpose**: Stdio-based MCP server that registers physics debugging tools and maintains a session-based recording context
- **Module location**: `replay/mcp_server/server.py`
- **Entry point**: `replay/mcp_server/__main__.py` (for `python -m replay.mcp_server` invocation)
- **Key interfaces**:
  ```python
  class ReplayMCPServer:
      def __init__(self):
          """Initialize MCP server with stdio transport and empty session."""
          self.service: SimulationService | None = None  # Active recording session
          self.tools: list[dict] = self._define_tools()

      def handle_message(self, message: dict) -> dict:
          """MCP protocol message handler (stdio JSON-RPC)."""
          # Dispatch to appropriate tool handler based on message content
          pass

      def run(self) -> None:
          """Main stdio event loop (reads stdin, writes stdout)."""
          pass
  ```
- **Dependencies**:
  - `mcp` Python SDK (stdio transport)
  - `replay.services.simulation_service.SimulationService` (existing query layer)
  - `replay.models` (Pydantic response models)
- **Thread safety**: Single-threaded stdio transport, no concurrency concerns
- **Error handling**: JSON-RPC error responses with structured error codes; database errors wrapped with context

#### Tool Handler Methods
- **Purpose**: Individual tool implementations called by the MCP dispatcher
- **Location**: `replay/mcp_server/tools.py` (optionally separated from `server.py` for clarity)
- **Key interfaces**:
  ```python
  # Session management
  def _load_recording(self, path: str) -> dict:
      """Set active recording session. Returns metadata summary."""
      pass

  def _list_recordings(self, directory: str | None = None) -> dict:
      """List .db files in directory (default: build/Debug/debug/recordings/)."""
      pass

  # Core query tools
  def _get_metadata(self) -> dict:
      """Body properties and frame count from active session."""
      pass

  def _get_frame(self, frame_id: int) -> dict:
      """Complete FrameData as JSON."""
      pass

  def _get_body_state(self, body_id: int, frame_id: int) -> dict:
      """Single body's kinematic state."""
      pass

  def _get_contacts_for_body(self, body_id: int, frame_id: int) -> dict:
      """Friction constraints involving body at frame."""
      pass

  # Energy analysis tools
  def _get_body_energy(self, body_id: int,
                       start_frame: int | None = None,
                       end_frame: int | None = None) -> dict:
      """Energy timeseries for one body."""
      pass

  def _get_system_energy(self, start_frame: int | None = None,
                         end_frame: int | None = None) -> dict:
      """System-level energy timeseries."""
      pass

  def _find_energy_anomalies(self, threshold: float = 1.0) -> dict:
      """Frames where |delta_e| exceeds threshold."""
      pass

  # Diagnostic comparison tools
  def _compare_body_across_frames(self, body_id: int,
                                  frame_a: int, frame_b: int) -> dict:
      """State delta between two frames with human-readable summary."""
      pass

  def _get_contact_history(self, body_a_id: int, body_b_id: int,
                           start_frame: int, end_frame: int) -> dict:
      """Contact pair evolution (detect chatter, normal flipping)."""
      pass

  def _get_solver_diagnostics(self, start_frame: int | None = None,
                              end_frame: int | None = None) -> dict:
      """Solver convergence info across frame range."""
      pass

  # Physics validation tools
  def _check_friction_cone(self, frame_id: int,
                           body_id: int | None = None) -> dict:
      """Verify friction forces satisfy Coulomb cone constraint."""
      pass

  def _check_penetration(self, frame_id: int,
                         threshold: float = 0.01) -> dict:
      """Find contacts with excessive penetration depth."""
      pass

  def _check_resting_contact(self, frame_id: int,
                              velocity_threshold: float = 0.01) -> dict:
      """Verify bodies at rest have near-zero velocity."""
      pass
  ```
- **Dependencies**: `SimulationService` for all database queries, `numpy` for vector math in validation checks
- **Error handling**: Return structured error messages with context when queries fail or recording not loaded

### Modified Components

#### `replay/pyproject.toml`
- **Current location**: `replay/pyproject.toml`
- **Changes required**:
  - Add `mcp` to `dependencies` list
  - Add `numpy` to `dependencies` for vector math in validation tools
- **Backward compatibility**: No breaking changes; pure addition of dependencies

#### `replay/README.md`
- **Current location**: `replay/README.md`
- **Changes required**:
  - Add section "MCP Server for AI-Assisted Debugging"
  - Document installation steps (pip install with mcp dependency)
  - Document Claude Code configuration in `.claude/settings.json`
  - Provide example interaction workflow
- **Backward compatibility**: Documentation-only change

#### `.claude/settings.json` (repository root)
- **Current location**: `.claude/settings.json`
- **Changes required**:
  - Add `replay` MCP server entry with stdio transport
  - Configure `cwd` to point to `replay/` directory
  - Set command as `python -m replay.mcp_server`
- **Backward compatibility**: Additive change to JSON configuration

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| ReplayMCPServer | SimulationService | Direct import and instantiation | Server creates SimulationService instance on `load_recording` |
| ReplayMCPServer | mcp SDK StdioServer | Protocol implementation | Server follows stdio JSON-RPC message format |
| Tool handlers | FrameData, BodyState, etc. | Return Pydantic models as JSON | Use `.model_dump_json()` for structured responses |
| _check_friction_cone | FrictionConstraintInfo | Physics validation logic | Extract lambda values, verify against Coulomb cone |
| _find_energy_anomalies | SystemEnergyPoint | Threshold-based filtering | Scan delta_e field for outliers |

## Test Impact

### Existing Tests Affected

No existing tests are affected. The MCP server is an independent tooling layer that consumes the existing `SimulationService` API without modifying it.

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| ReplayMCPServer | `test_list_recordings_empty_directory` | Returns empty list when no .db files found |
| ReplayMCPServer | `test_load_recording_success` | Sets active service and returns metadata |
| ReplayMCPServer | `test_load_recording_nonexistent_file` | Returns structured error response |
| ReplayMCPServer | `test_get_frame_without_load` | Returns error when no recording loaded |
| Tool handlers | `test_get_metadata` | Returns correct body count and frame count |
| Tool handlers | `test_get_frame_valid` | Returns complete FrameData as JSON |
| Tool handlers | `test_get_body_state` | Returns single body state with all kinematic fields |
| Tool handlers | `test_get_contacts_for_body` | Filters friction constraints by body_id |
| Tool handlers | `test_find_energy_anomalies` | Identifies frames exceeding threshold |
| Tool handlers | `test_compare_body_across_frames` | Computes position/velocity delta correctly |
| Tool handlers | `test_check_friction_cone_pass` | Returns empty violations when forces satisfy cone |
| Tool handlers | `test_check_friction_cone_fail` | Detects when tangent force exceeds mu * Fn |
| Tool handlers | `test_check_penetration` | Detects contacts exceeding depth threshold |
| Tool handlers | `test_check_resting_contact` | Detects bodies with velocity above threshold |

**Test data source**: Use existing test recordings from `replay/tests/` (e.g., `test_drop_recording.db`) as fixture databases.

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `test_mcp_server_stdio_round_trip` | ReplayMCPServer, stdio transport | Full JSON-RPC message cycle (list tools, call tool, receive response) |
| `test_end_to_end_energy_anomaly_detection` | ReplayMCPServer + SimulationService + test DB | Load recording, find anomalies, verify against known energy violations |
| `test_friction_cone_validation_realistic_recording` | Tool handlers + test DB with known friction violations | Detect expected friction constraint failures in controlled test scenario |

**Integration test strategy**: Use pytest with test recordings that have known properties (e.g., a recording with an intentional energy injection at frame N). Validate that MCP tools correctly identify these known anomalies.

#### Manual End-to-End Test (Claude Code Integration)

| Test Case | Steps | Expected Outcome |
|-----------|-------|------------------|
| Configure and connect MCP server | Add config to `.claude/settings.json`, restart Claude Code | Server appears in available MCP servers, tools are listed |
| Physics debugging workflow | User says "At frame 45, body 1 is sliding", Claude calls tools | Claude queries `get_frame`, `get_contacts_for_body`, `check_friction_cone` and provides diagnosis |

## Session State Management

The MCP server uses a **session-based** design where `load_recording` sets the active database for all subsequent queries:

```python
class ReplayMCPServer:
    def __init__(self):
        self.service: SimulationService | None = None  # Active session

    def _load_recording(self, path: str) -> dict:
        self.service = SimulationService(Path(path))
        return self.service.get_metadata().model_dump()

    def _get_frame(self, frame_id: int) -> dict:
        if self.service is None:
            return {"error": "No recording loaded. Call load_recording first."}
        return self.service.get_frame_data(frame_id).model_dump()
```

**Rationale**: Most debugging sessions focus on one recording at a time. The session-based approach matches the user's mental model ("I'm looking at this recording") and avoids repetitive `db_path` parameters on every tool call.

**Alternative considered**: Per-call `db_path` parameter. Rejected due to verbosity and poor UX for the typical single-recording workflow.

## Diagnostic Tool Output Format

Diagnostic and validation tools return both **structured data** and **human-readable summaries** to help Claude reason about results:

```python
# Example: check_friction_cone output
{
    "frame_id": 45,
    "violations": [
        {
            "body_a_id": 1,
            "body_b_id": 2,
            "friction_coefficient": 0.5,
            "normal_force": 98.1,
            "friction_limit": 49.05,
            "tangent1_force": 49.04,
            "tangent2_force": 0.12,
            "total_tangent_force": 49.041,
            "exceeds_limit": False,
            "margin": 0.009
        }
    ],
    "summary": "1 contact checked. 0 violations detected. Body 1 is at friction limit (99.98% of mu*Fn)."
}
```

The `summary` field provides context for Claude to interpret the raw data. Similarly, `compare_body_across_frames` includes a delta summary:

```python
{
    "body_id": 1,
    "frame_a": 44,
    "frame_b": 45,
    "position_delta": {"x": 0.002, "y": 0.0, "z": 0.0},
    "velocity_delta": {"x": 0.003, "y": 0.0, "z": 0.0},
    "summary": "Body moved 0.002m in X between frames. Velocity increased by 0.003 m/s in X."
}
```

## Default Recording Directory

The `list_recordings` tool defaults to `build/Debug/debug/recordings/` which is where `ReplayEnabledTest` fixtures write `.db` files. This can be overridden with an explicit `directory` parameter for custom locations.

**Rationale**: Physics tests that record simulation data use this standard location. Defaulting to it reduces friction for the primary use case (debugging test failures).

## Open Questions

### Design Decisions (Human Input Needed)

None. The design follows the established MCP server pattern from the codebase server and reuses the existing `SimulationService` query layer without modification.

### Prototype Required

None. The design is straightforward Python wrapping of an existing service. No validation needed beyond unit/integration tests.

### Requirements Clarification

1. **Should the MCP server support multiple concurrent recording sessions?**
   - Current design: Single session (one active recording at a time)
   - Alternative: Session dictionary keyed by session ID
   - Recommendation: Single session is sufficient for the initial implementation. Multi-session can be added later if needed.

2. **What should `find_energy_anomalies` return when no anomalies are found?**
   - Option A: Empty list with summary "No anomalies detected"
   - Option B: Return all frames with their delta_e values
   - Recommendation: Option A — tool is explicitly for anomaly detection, not bulk energy export.

3. **Should validation tools (check_friction_cone, etc.) auto-load metadata or require explicit `get_metadata` first?**
   - Current design: Validation tools query metadata internally as needed
   - Alternative: Require user to call `get_metadata` separately
   - Recommendation: Auto-load internally — reduces tool call overhead and improves UX.

## Implementation Notes

### Dependency Installation

Add to `replay/pyproject.toml`:
```toml
dependencies = [
    "fastapi>=0.100",
    "uvicorn>=0.23",
    "pydantic>=2.0",
    "mcp>=1.0",  # NEW
    "numpy>=1.24",  # NEW (for vector math in validation)
]
```

Install in development mode:
```bash
cd replay
pip install -e .
```

### MCP Server Configuration

Add to `.claude/settings.json`:
```json
{
  "mcpServers": {
    "codebase": {
      "type": "stdio",
      "command": "python",
      "args": ["scripts/mcp_server.py", "build/Debug/docs/codebase.db"],
      "cwd": "/Users/danielnewman/Documents/GitHub/MSD-CPP"
    },
    "replay": {
      "type": "stdio",
      "command": "python",
      "args": ["-m", "replay.mcp_server"],
      "cwd": "/Users/danielnewman/Documents/GitHub/MSD-CPP/replay"
    }
  }
}
```

### Stdio Transport Implementation

The server uses the standard MCP stdio protocol (JSON-RPC over stdin/stdout):

```python
import sys
import json

class ReplayMCPServer:
    def run(self):
        """Main stdio event loop."""
        for line in sys.stdin:
            try:
                message = json.loads(line)
                response = self.handle_message(message)
                print(json.dumps(response), flush=True)
            except Exception as e:
                error_response = {
                    "jsonrpc": "2.0",
                    "error": {"code": -32603, "message": str(e)},
                    "id": message.get("id")
                }
                print(json.dumps(error_response), flush=True)

if __name__ == "__main__":
    server = ReplayMCPServer()
    server.run()
```

### Error Code Standards

Follow JSON-RPC 2.0 error codes:
- `-32700`: Parse error (invalid JSON)
- `-32600`: Invalid request (malformed message)
- `-32601`: Method not found (unknown tool)
- `-32602`: Invalid params (missing/wrong parameter types)
- `-32603`: Internal error (database errors, unhandled exceptions)

Custom application errors use code `-32000` with descriptive messages:
```python
{"code": -32000, "message": "No recording loaded. Call load_recording first."}
```

## Performance Considerations

### Query Performance
- **Assumption**: Typical recordings have 100-1000 frames
- **Frame queries**: `get_frame` involves SQL joins (sub-millisecond for single frame)
- **Bulk queries**: `get_system_energy` may return 1000+ points (~10ms)
- **Validation scans**: `find_energy_anomalies` scans all system energy records (linear time, acceptable)

**No benchmarking required** — the MCP server is human-in-the-loop tooling, not a hot path. Query latency <100ms is acceptable.

### Memory Footprint
- SimulationService holds one sqlite3 connection (minimal memory)
- Pydantic models are serialized and returned, not accumulated
- No caching needed — queries are infrequent and database is already on disk

## Code Quality Gates

### Build Quality
- **Type checking**: Use `mypy` to verify type annotations (all tool methods use explicit return types)
- **Linting**: Apply `ruff` for PEP 8 compliance
- **No warnings**: Python code must pass linting without warnings

### Test Coverage
- **Unit tests**: >80% coverage for tool handlers and server logic
- **Integration tests**: End-to-end stdio round-trip test with mock database
- **Manual test**: Document manual verification steps in replay README

### Documentation
- Docstrings for all public methods (NumPy style)
- MCP tool descriptions in `_define_tools()` provide clear guidance for Claude Code
- README section explains configuration and usage

## Future Extensions (Out of Scope)

1. **Streaming support**: Real-time frame data as simulation runs (requires WebSocket transport)
2. **Comparative analysis**: Compare two recordings side-by-side (e.g., before/after a physics fix)
3. **Automated fix suggestions**: Use LLM to propose code changes based on diagnostics (separate ticket)
4. **Visualization tools**: Generate plots/charts from MCP server data (can be handled in Claude Code with matplotlib)

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-16
**Status**: APPROVED
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Python PEP 8 style — snake_case for functions/variables, PascalCase for classes. Consistent with existing `replay/` codebase. |
| Namespace organization | ✓ | `replay/mcp_server/` structure mirrors the existing `replay/replay/services/` organization. Clean separation of MCP protocol layer from business logic. |
| File structure | ✓ | Follows established pattern from `scripts/mcp_server.py`. Entry point via `__main__.py` for `-m` invocation is appropriate. |
| Dependency direction | ✓ | Correct layering — MCP server depends on SimulationService, which depends on msd_reader. No circular dependencies introduced. |

**Summary**: The design integrates cleanly into the existing replay subsystem without introducing new architectural patterns. The MCP server is a thin protocol adapter over the existing query layer, which is the correct approach.

#### Python Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Type hints | ✓ | All methods specify return types (`-> dict`, `-> list[dict]`). Parameters use union types (`int \| None`) per PEP 604. |
| Error handling | ✓ | JSON-RPC error codes follow the spec (-32000 to -32603). Application errors use -32000 with descriptive messages. Database errors are wrapped with context. |
| Pydantic model usage | ✓ | `.model_dump_json()` used for structured responses. Leverages existing models from `replay.models`. |
| Session state | ✓ | Session-based design (`self.service: SimulationService \| None`) is simple and appropriate for single-recording debugging workflows. |
| Code organization | ✓ | Separation of concerns — server.py handles protocol, tools.py (optional) handles business logic. Clear separation from existing SimulationService. |
| Return values | ✓ | All tools return `dict` (JSON-RPC response format). Diagnostic tools include both structured data and human-readable summaries. |

**Summary**: Python design quality is strong. Type hints are comprehensive, error handling follows JSON-RPC standards, and the code leverages existing Pydantic models without duplication.

#### Feasibility Assessment

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Dependency availability | ✓ | `mcp` SDK available via pip. SimulationService already exists. `numpy` for vector math is standard. |
| Runtime performance | ✓ | Human-in-the-loop tooling — query latency <100ms is acceptable. No hot paths. SimulationService queries are sub-millisecond for single frames. |
| Integration complexity | ✓ | Stdio transport follows established pattern. No build system changes needed (pure Python). Configuration is additive (`.claude/settings.json`). |
| State management | ✓ | Single session model is simple and sufficient. No concurrency issues (stdio is single-threaded). |
| Testing strategy | ✓ | Unit tests use existing test recordings as fixtures. Integration tests validate stdio round-trip. Manual test validates Claude Code integration. |

**Summary**: Implementation is straightforward. All dependencies are available, integration points are well-defined, and the design reuses existing query infrastructure without modification.

#### Testability Assessment

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | Tool handlers can be tested independently by mocking SimulationService. Server can be tested with synthetic JSON-RPC messages. |
| Mockable dependencies | ✓ | SimulationService is the only external dependency and can be mocked for unit tests. |
| Observable state | ✓ | Session state (`self.service`) is directly inspectable. Tool return values are JSON-serializable and easy to assert against. |
| Test coverage | ✓ | Design specifies >80% coverage target with clear unit test matrix. Fixture data available from existing test recordings. |

**Summary**: Excellent testability. The design's separation of protocol (server.py) and business logic (tools.py) enables isolated testing of each layer.

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | MCP SDK API changes between versions | Technical | Low | Medium | Pin `mcp>=1.0,<2.0` in pyproject.toml. MCP spec is stable. | No |
| R2 | Recording databases with missing tables (older schema) | Integration | Medium | Low | SimulationService already handles missing tables gracefully (try/except with pass). Validation tools should do the same. | No |
| R3 | Large recordings (1000+ frames) causing slow bulk queries | Performance | Low | Low | Tool design already avoids bulk queries — most tools are frame-specific or support start/end ranges. `find_energy_anomalies` scans all frames but is acceptable for human-in-the-loop workflow. | No |
| R4 | Friction cone check false positives due to floating-point epsilon | Technical | Low | Low | Use appropriate epsilon (e.g., `margin = friction_limit * 1.01 - total_tangent_force`) when checking violations. Document epsilon choice. | No |

**Summary**: All risks are low-likelihood or low-impact. No high-risk items requiring prototypes. Existing code already handles schema variations and the performance profile is appropriate for the use case.

### Prototype Guidance

**No prototypes required.** The design follows established patterns (stdio MCP server, SimulationService query layer) and all integration points are well-understood. Implementation can proceed directly to unit/integration testing.

### Required Revisions

None. Design is approved as-is.

### Summary

This is a well-architected design that cleanly extends the replay subsystem with AI debugging capabilities. The MCP server is a thin protocol adapter over the existing SimulationService query layer, which is the correct approach — no duplication of business logic, no modification of existing code.

**Strengths**:
- Reuses SimulationService without modification (zero impact on existing code)
- Follows established stdio MCP pattern from codebase server
- Session-based design matches user mental model for debugging workflows
- Comprehensive tool set covers core queries, energy analysis, diagnostics, and validation
- Diagnostic tools include human-readable summaries alongside structured data
- Testability is excellent due to clear separation of protocol and business logic

**Next Steps**:
1. Proceed directly to implementation (no prototype phase needed)
2. Follow implementation plan from design document
3. Ensure >80% test coverage with unit tests using existing test recording fixtures
4. Manual end-to-end test with Claude Code to verify tool integration

**Design approved for implementation.**
