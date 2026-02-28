# Feature Ticket: Database-Backed Multi-Agent Orchestration

## Status
- [x] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Test Writing
- [ ] Test Writing Complete — Awaiting Quality Gate
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-27
- **Author**: Daniel Newman
- **Priority**: High
- **Estimated Complexity**: XL
- **Target Component(s)**: workflow infrastructure (standalone repo + MSD-CPP integration)
- **Languages**: Python
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**:

---

## Summary
Create a standalone `workflow-engine` repository containing a SQLite-backed work queue that enables multiple agents to operate concurrently across independent tickets. The engine provides a FastMCP server, scheduler, CLI, and markdown sync library that any consuming repository can adopt. MSD-CPP becomes the first consumer, providing project-specific phase definitions and agent type mappings via a `.workflow/` configuration directory. The engine is distributed via Docker (like the guidelines server in 0081) or direct Python installation, and integrates with consuming repos through `.mcp.json`.

## Motivation
The current workflow processes one ticket at a time, one phase at a time, through a single orchestrator agent. This creates several bottlenecks:

1. **Serial execution** — Only one ticket progresses at a time, even when tickets are independent
2. **No state visibility** — Determining which tickets are blocked, in-progress, or available requires reading markdown files
3. **No conflict detection** — Two agents could theoretically modify overlapping files without awareness
4. **No metrics** — No way to query phase durations, bottleneck identification, or throughput
5. **Human gates block everything** — A pending human review on one ticket prevents all other tickets from progressing
6. **No audit trail** — State transitions are logged in markdown but not queryable

Additionally, the workflow engine is not specific to MSD-CPP — any project using a ticket-based agent workflow could benefit from the same coordination infrastructure. Extracting it to a standalone repo (following the 0081 guidelines-server pattern) enables reuse across repositories.

## Requirements

### Functional Requirements
1. The engine shall maintain a SQLite database tracking ticket status, workflow phases, agent assignments, and human gates
2. The engine shall allow agents to atomically claim available work phases matching their capability
3. The engine shall enforce a configurable ticket state machine — phases must execute in the order defined by the consuming repo's phase configuration
4. The engine shall model human review gates as database records that block downstream phases
5. The engine shall track agent liveness via heartbeat and auto-release stale claims after a configurable timeout
6. The engine shall log all state transitions in an audit trail
7. The engine shall detect file-level conflicts when multiple agents work on overlapping files
8. The engine shall support querying tickets by status, priority, component, assigned agent, and blocking state
9. The engine shall sync ticket metadata between the database and markdown files in the consuming repo (markdown remains the content store; DB is the state/query store)
10. The engine shall expose the work queue via MCP tools so agents can claim/complete/query work
11. The engine shall support dependency relationships between tickets (ticket B blocks on ticket A)
12. The engine shall read phase definitions, agent type mappings, and configuration from the consuming repo's `.workflow/` directory
13. The engine shall be distributable via Docker image or direct Python package installation
14. MSD-CPP shall provide a `.workflow/` configuration that maps its existing ticket lifecycle and agent types

### Non-Functional Requirements
- **Performance**: Phase claiming must be atomic and complete in <100ms (SQLite transaction)
- **Reliability**: Database must be recoverable — WAL mode, periodic checkpoints
- **Consistency**: No two agents may claim the same phase simultaneously
- **Observability**: Dashboard-ready query interface for workflow state
- **Portability**: Engine must not contain any MSD-CPP-specific logic; all project knowledge comes from `.workflow/` configuration
- **Backward Compatibility**: Existing markdown tickets must continue to work; migration must be incremental

## Constraints
- Must use SQLite (consistent with existing infrastructure — codebase.db, traceability.db, guidelines.db)
- Must not require a persistent server/daemon — MCP servers start on-demand
- Must preserve existing agent definitions (.claude/agents/*.md) — agents gain MCP tools, not new architectures
- Must support incremental adoption — can coexist with the current file-based workflow during migration
- The database is a coordination layer, not a replacement for markdown content — ticket narrative, design documents, and artifacts remain in files
- Engine repo must be project-agnostic; all project-specific configuration lives in the consuming repo's `.workflow/` directory

## Acceptance Criteria
- [ ] `workflow-engine` repo contains schema, MCP server, scheduler, CLI, and markdown sync
- [ ] Engine reads phase definitions from consuming repo's `.workflow/phases.yaml`
- [ ] Engine reads configuration (timeouts, priority rules) from `.workflow/config.yaml`
- [ ] MSD-CPP `.workflow/phases.yaml` maps the full ticket lifecycle (all statuses from ticket.md.template)
- [ ] Agents can atomically claim phases via MCP tools without conflicts
- [ ] Two agents processing independent tickets can run concurrently without interference
- [ ] Human gates block only the specific ticket/phase, not all work
- [ ] Stale agent claims auto-release after configurable timeout (default: 10 minutes)
- [ ] All state transitions logged in audit table with actor, timestamp, context
- [ ] File conflict detection warns when two agents' phases touch overlapping files
- [ ] CLI tool can query: blocked tickets, available work by agent type, phase durations
- [ ] Existing markdown tickets can be imported into the database
- [ ] Docker image builds and serves MCP tools with consuming repo's `.workflow/` mounted
- [ ] Work queue MCP server integrates with consuming repo's `.mcp.json`

---

## Design Decisions (Human Input)

### Preferred Approaches
- SQLite with WAL mode for concurrent read access
- MCP server pattern consistent with existing codebase/traceability/guidelines servers
- Atomic claiming via `UPDATE ... WHERE status='pending' AND claimed_by IS NULL RETURNING *`
- Keep markdown as content store; database as state/coordination store
- Heartbeat-based liveness detection (agents report periodically; stale claims auto-release)
- Phase-granularity work items (not ticket-granularity) — each workflow phase is independently claimable
- Standalone repo following the 0081 guidelines-server extraction pattern
- `.workflow/` directory in consuming repos for project-specific configuration (same pattern as `.guidelines/` for project rules)

### Things to Avoid
- Do not embed any MSD-CPP-specific logic in the engine — all project knowledge comes from `.workflow/` config
- Do not require a persistent server/daemon — MCP servers are on-demand
- Do not replace markdown tickets with database records — the rich narrative content belongs in files
- Do not create new agent definition files — extend existing agents with MCP tool access
- Do not over-engineer the conflict detection — file-level overlap warnings are sufficient; no need for AST-level merging
- Do not introduce external dependencies beyond SQLite and Python stdlib (plus fastmcp, already in use)

### Open Questions
1. **Agent spawning model** — Should the orchestrator spawn agents as subprocesses, or should agents be launched independently and discover work via polling?
   - Option A: Orchestrator spawns — simpler lifecycle management, but orchestrator becomes a bottleneck
   - Option B: Independent polling — more scalable, but requires heartbeat/cleanup infrastructure
   - **Recommendation**: Option B (independent polling) — aligns with the goal of decoupling the orchestrator from execution
2. **Markdown sync direction** — Should the database be seeded from markdown (import), or should markdown be generated from the database (export)?
   - Option A: Import (markdown → DB) — markdown remains the source of truth for content; DB mirrors status
   - Option B: Bidirectional sync — either can update status; conflicts resolved by timestamp
   - **Recommendation**: Option A (import) with DB as authoritative for status fields only
3. **Phase granularity for multi-language tickets** — Should language-specific phases (Python Design, Frontend Design) be independent work items that can be claimed in parallel?
   - **Recommendation**: Yes — this is one of the key benefits of the database model
4. **Distribution model** — Docker-only or also pip-installable?
   - Option A: Docker-only (like 0081 guidelines-server)
   - Option B: Both Docker and pip-installable package
   - **Recommendation**: Option B — Docker for multi-repo CI, pip for local development with `python/.venv`

---

## References

### Related Code
- `scripts/mcp_codebase_server.py` — Existing MCP server pattern to follow
- `scripts/traceability/traceability_server.py` — Another MCP server reference
- `scripts/guidelines/guidelines_server.py` — Newest MCP server, closest template
- `scripts/guidelines/guidelines_schema.py` — Schema migration pattern to follow
- `.claude/agents/workflow-orchestrator.md` — Current orchestrator (to be refactored)

### Related Documentation
- `docs/workflows/` — Existing workflow documentation
- `.claude/templates/ticket.md.template` — Ticket schema that DB must model

### Related Tickets
- `tickets/0079_design_revision_feedback_loop.md` — Design Revision Loop must be supported in DB model
- `tickets/0081_guidelines_server_extraction.md` — MCP server extraction pattern (template for this work)

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**:
- **Completed**:
- **Artifacts**:
  - `docs/designs/0083_database_agent_orchestration/design.md`
  - `docs/designs/0083_database_agent_orchestration/0083_database_agent_orchestration.puml`
- **Design Decisions**:
- **Notes**:

### Design Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
  - P1: {name} — {result}
- **Artifacts**:
  - `docs/designs/0083_database_agent_orchestration/prototype-results.md`
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Artifacts**:
  - `docs/designs/0083_database_agent_orchestration/implementation-notes.md`
- **Notes**:

### Test Writing Phase
- **Started**:
- **Completed**:
- **Test Files Created**:
- **Test Coverage Summary**:
- **Test Failures Documented for Implementer**:
- **Notes**:

### Implementation Review Phase
- **Started**:
- **Completed**:
- **Status**:
- **Reviewer Notes**:

### Documentation Update Phase
- **Started**:
- **Completed**:
- **CLAUDE.md Updates**:
- **Diagrams Indexed**:
- **Notes**:

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}

### Feedback on Tests
{Your comments on test coverage, test quality, or missing test scenarios}
