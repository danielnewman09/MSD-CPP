# Design: Database-Backed Multi-Agent Orchestration

## Summary

This design creates a standalone `workflow-engine` repository containing a SQLite-backed work queue for concurrent multi-agent ticket orchestration. The engine is project-agnostic — it knows about tickets, phases, agents, and gates but nothing about spacecraft dynamics, C++, or any specific domain. All project-specific configuration (phase definitions, agent type mappings, timeouts) lives in the consuming repository's `.workflow/` directory. MSD-CPP becomes the first consumer.

The engine provides: a SQLite schema for coordination state, a FastMCP MCP server for agent interaction, a scheduler for phase seeding and availability resolution, a CLI for human review gates and querying, and a markdown sync library for bridging between file-based tickets and queryable database state.

Agents discover and claim work via MCP tools. Human gates block only the specific ticket/phase, not all work. Multiple tickets progress concurrently. The existing markdown tickets remain the content store; the database is the coordination and query layer.

This follows the extraction pattern established by ticket 0081 (guidelines-server): shared infrastructure lives in a standalone repo; project-specific configuration lives in the consuming repo.

## Architecture Changes

### PlantUML Diagram
See: `./0083_database_agent_orchestration.puml`

---

## Repository Structure

### Standalone Repository: `workflow-engine`

```
workflow-engine/
├── engine/
│   ├── __init__.py
│   ├── schema.py               # SQLite schema + migrations
│   ├── models.py               # Data classes for tickets, phases, gates, agents
│   ├── state_machine.py        # Phase status transitions + validation
│   ├── scheduler.py            # Phase seeder + availability resolver + stale cleanup
│   ├── claim.py                # Atomic claiming logic
│   ├── markdown_sync.py        # Import/export between markdown tickets and DB
│   ├── config.py               # Reads .workflow/ configuration from consuming repo
│   └── audit.py                # Audit log helpers
│
├── server/
│   ├── __init__.py
│   └── server.py               # FastMCP MCP server (stdio + SSE modes)
│
├── cli/
│   ├── __init__.py
│   └── cli.py                  # Human review CLI (approve/reject/query)
│
├── tests/
│   ├── test_schema.py
│   ├── test_claim.py
│   ├── test_phase_machine.py
│   ├── test_stale_recovery.py
│   ├── test_human_gates.py
│   ├── test_dependencies.py
│   ├── test_file_locks.py
│   ├── test_markdown_sync.py
│   ├── test_scheduler.py
│   ├── test_audit.py
│   ├── test_config.py
│   ├── test_mcp_tools.py
│   ├── test_concurrent_agents.py
│   └── test_full_lifecycle.py
│
├── Dockerfile
├── docker-compose.yaml
├── pyproject.toml              # Package metadata (pip-installable)
├── README.md
└── CLAUDE.md
```

### Consuming Repository: MSD-CPP (changes)

```
MSD-CPP/
├── .workflow/
│   ├── phases.yaml             # Phase definitions + agent type mappings
│   ├── config.yaml             # Timeouts, priority rules, stale thresholds
│   └── ticket_parser.py        # Optional: custom markdown parsing rules
│
├── .mcp.json                   # Updated: add workflow server entry
├── tickets/                    # Unchanged: ticket content remains here
├── .claude/agents/             # Updated: agents gain work queue preamble
└── .claude/agents/workflow-orchestrator.md  # Refactored: scheduler role
```

---

## New Components (workflow-engine repo)

### 1. `engine/schema.py` — Database Schema

SQLite database at a path specified by the consuming repo (e.g., `build/Debug/docs/workflow.db` for MSD-CPP, gitignored).

#### Tables

```sql
-- Core ticket tracking (mirrors ticket metadata from consuming repo)
CREATE TABLE tickets (
    id TEXT PRIMARY KEY,                    -- e.g., "0083"
    name TEXT NOT NULL,                     -- e.g., "database_agent_orchestration"
    full_name TEXT NOT NULL,                -- e.g., "0083_database_agent_orchestration"
    priority TEXT CHECK(priority IN ('Low', 'Medium', 'High', 'Critical')),
    complexity TEXT CHECK(complexity IN ('Small', 'Medium', 'Large', 'XL')),
    components TEXT,                        -- comma-separated
    languages TEXT DEFAULT 'C++',           -- comma-separated
    github_issue INTEGER,
    current_status TEXT NOT NULL,
    markdown_path TEXT NOT NULL,            -- path to tickets/*.md in consuming repo
    custom_metadata TEXT,                   -- JSON blob for project-specific fields
    created_at TEXT DEFAULT (datetime('now')),
    updated_at TEXT DEFAULT (datetime('now'))
);

-- Individual workflow phases as claimable work items
CREATE TABLE phases (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ticket_id TEXT NOT NULL REFERENCES tickets(id),
    phase_name TEXT NOT NULL,               -- e.g., "Design", "Design Review", "Implementation"
    phase_order INTEGER NOT NULL,           -- ordering within ticket
    status TEXT NOT NULL DEFAULT 'pending'
        CHECK(status IN ('pending', 'blocked', 'available', 'claimed', 'running', 'completed', 'failed', 'skipped')),
    agent_type TEXT,                        -- which agent type can claim this (from phases.yaml)
    claimed_by TEXT,                        -- agent instance ID
    claimed_at TEXT,
    heartbeat_at TEXT,
    started_at TEXT,
    completed_at TEXT,
    result_summary TEXT,                    -- brief outcome text
    error_details TEXT,                     -- on failure
    artifacts TEXT,                         -- JSON array of file paths produced
    parallel_group TEXT,                    -- non-null for phases that run in parallel (e.g., "impl")
    UNIQUE(ticket_id, phase_name)
);

-- Human review gates
CREATE TABLE human_gates (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    phase_id INTEGER NOT NULL REFERENCES phases(id),
    ticket_id TEXT NOT NULL REFERENCES tickets(id),
    gate_type TEXT NOT NULL,                -- e.g., "design_review", "prototype_review"
    status TEXT NOT NULL DEFAULT 'pending'
        CHECK(status IN ('pending', 'approved', 'rejected', 'changes_requested')),
    requested_at TEXT DEFAULT (datetime('now')),
    decided_at TEXT,
    decided_by TEXT,                        -- human reviewer identifier
    decision_notes TEXT,
    context TEXT,                           -- JSON blob with relevant context for reviewer
    UNIQUE(phase_id)
);

-- Inter-ticket dependencies
CREATE TABLE dependencies (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    blocked_ticket_id TEXT NOT NULL REFERENCES tickets(id),
    blocking_ticket_id TEXT NOT NULL REFERENCES tickets(id),
    dependency_type TEXT DEFAULT 'completion'
        CHECK(dependency_type IN ('completion', 'design', 'implementation')),
    resolved BOOLEAN DEFAULT 0,
    created_at TEXT DEFAULT (datetime('now')),
    resolved_at TEXT,
    UNIQUE(blocked_ticket_id, blocking_ticket_id)
);

-- Agent registry for liveness tracking
CREATE TABLE agents (
    id TEXT PRIMARY KEY,                    -- unique agent instance ID
    agent_type TEXT NOT NULL,               -- e.g., "cpp-architect", "cpp-implementer"
    status TEXT NOT NULL DEFAULT 'idle'
        CHECK(status IN ('idle', 'working', 'stale', 'terminated')),
    current_phase_id INTEGER REFERENCES phases(id),
    registered_at TEXT DEFAULT (datetime('now')),
    last_heartbeat TEXT DEFAULT (datetime('now')),
    metadata TEXT                           -- JSON blob (model, worktree path, etc.)
);

-- File lock tracking for conflict detection
CREATE TABLE file_locks (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    file_path TEXT NOT NULL,
    phase_id INTEGER NOT NULL REFERENCES phases(id),
    agent_id TEXT NOT NULL REFERENCES agents(id),
    acquired_at TEXT DEFAULT (datetime('now')),
    released_at TEXT,
    UNIQUE(file_path, phase_id)
);

-- Audit log for all state transitions
CREATE TABLE audit_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT DEFAULT (datetime('now')),
    actor TEXT NOT NULL,                    -- agent ID or "human:{name}" or "scheduler"
    action TEXT NOT NULL,                   -- e.g., "claim_phase", "complete_phase", "approve_gate"
    entity_type TEXT NOT NULL,              -- "ticket", "phase", "gate", "agent"
    entity_id TEXT NOT NULL,
    old_state TEXT,
    new_state TEXT,
    details TEXT                            -- JSON blob with additional context
);

-- Indexes for common queries
CREATE INDEX idx_phases_status ON phases(status);
CREATE INDEX idx_phases_agent_type ON phases(agent_type, status);
CREATE INDEX idx_phases_ticket ON phases(ticket_id);
CREATE INDEX idx_phases_parallel ON phases(parallel_group, ticket_id);
CREATE INDEX idx_agents_status ON agents(status);
CREATE INDEX idx_agents_heartbeat ON agents(last_heartbeat);
CREATE INDEX idx_audit_timestamp ON audit_log(timestamp);
CREATE INDEX idx_audit_entity ON audit_log(entity_type, entity_id);
CREATE INDEX idx_file_locks_active ON file_locks(file_path) WHERE released_at IS NULL;
```

#### Phase Status State Machine

```
pending → blocked    (dependency not met or human gate pending)
pending → available  (all prerequisites satisfied)
blocked → available  (dependency resolved or gate approved)
available → claimed  (agent atomically claims)
claimed → running    (agent begins execution)
running → completed  (agent reports success)
running → failed     (agent reports failure or stale claim cleanup)
pending → skipped    (conditional phase not applicable)
```

#### Atomic Claim Query

```sql
-- Agent claims next available phase matching its type
UPDATE phases
SET status = 'claimed',
    claimed_by = :agent_id,
    claimed_at = datetime('now')
WHERE id = (
    SELECT p.id FROM phases p
    JOIN tickets t ON p.ticket_id = t.id
    WHERE p.status = 'available'
      AND p.agent_type = :agent_type
      AND NOT EXISTS (
          SELECT 1 FROM dependencies d
          WHERE d.blocked_ticket_id = t.id
            AND d.resolved = 0
            AND d.dependency_type = 'completion'
      )
    ORDER BY
        CASE t.priority
            WHEN 'Critical' THEN 0
            WHEN 'High' THEN 1
            WHEN 'Medium' THEN 2
            WHEN 'Low' THEN 3
        END,
        p.id ASC
    LIMIT 1
)
RETURNING *;
```

---

### 2. `engine/config.py` — Configuration Reader

Reads project-specific configuration from the consuming repo's `.workflow/` directory.

#### `.workflow/phases.yaml` (in consuming repo)

```yaml
# Phase definitions for MSD-CPP ticket lifecycle
# Each phase maps to an agent type and optional conditions

phases:
  - name: "Math Design"
    agent_type: "math-designer"
    condition:
      field: "requires_math_design"
      value: true

  - name: "Math Design Review"
    agent_type: "math-reviewer"
    condition:
      field: "requires_math_design"
      value: true

  - name: "Design"
    agent_type: "cpp-architect"

  - name: "Design Review"
    agent_type: "design-reviewer"

  - name: "Integration Design"
    agent_type: "integration-designer"
    condition:
      field: "languages"
      has_multiple: true

  - name: "Integration Review"
    agent_type: "integration-reviewer"
    condition:
      field: "languages"
      has_multiple: true

  - name: "Python Design"
    agent_type: "python-architect"
    condition:
      field: "languages"
      contains: "Python"

  - name: "Python Design Review"
    agent_type: null  # human gate
    condition:
      field: "languages"
      contains: "Python"

  - name: "Frontend Design"
    agent_type: "frontend-architect"
    condition:
      field: "languages"
      contains: "Frontend"

  - name: "Frontend Design Review"
    agent_type: null  # human gate
    condition:
      field: "languages"
      contains: "Frontend"

  - name: "Prototype"
    agent_type: "cpp-prototyper"

  - name: "Prototype Review"
    agent_type: null  # human gate

  - name: "Implementation"
    agent_type: "cpp-implementer"

  - name: "Test Writing"
    agent_type: "cpp-test-writer"

  - name: "Quality Gate"
    agent_type: "code-quality-gate"

  - name: "Implementation Review"
    agent_type: "implementation-reviewer"

  - name: "Documentation"
    agent_type: "docs-updater"

  - name: "Tutorial"
    agent_type: "cpp-tutorial-generator"
    condition:
      field: "generate_tutorial"
      value: true

# Parallel phase groups: phases in the same group run concurrently
# All phases in a group must complete before the next sequential phase
parallel_groups:
  impl:
    after: "Integration Review"  # becomes available after this phase completes
    before: "Quality Gate"       # this phase waits for all group members
    phases:
      - name: "C++ Implementation"
        agent_type: "cpp-implementer"
        condition:
          field: "languages"
          contains: "C++"

      - name: "Python Implementation"
        agent_type: "python-implementer"
        condition:
          field: "languages"
          contains: "Python"

      - name: "Frontend Implementation"
        agent_type: "frontend-implementer"
        condition:
          field: "languages"
          contains: "Frontend"

# Metadata fields to extract from ticket markdown
# Engine uses these for condition evaluation and DB storage
ticket_metadata:
  - field: "requires_math_design"
    type: "boolean"
    default: false
    markdown_key: "Requires Math Design"

  - field: "generate_tutorial"
    type: "boolean"
    default: false
    markdown_key: "Generate Tutorial"

  - field: "languages"
    type: "list"
    default: ["C++"]
    markdown_key: "Languages"

  - field: "priority"
    type: "enum"
    values: ["Low", "Medium", "High", "Critical"]
    markdown_key: "Priority"

  - field: "complexity"
    type: "enum"
    values: ["Small", "Medium", "Large", "XL"]
    markdown_key: "Estimated Complexity"

  - field: "components"
    type: "list"
    markdown_key: "Target Component(s)"

  - field: "github_issue"
    type: "integer"
    markdown_key: "GitHub Issue"
```

#### `.workflow/config.yaml` (in consuming repo)

```yaml
# Workflow engine configuration for MSD-CPP

database:
  path: "build/Debug/docs/workflow.db"  # gitignored, derived state

tickets:
  directory: "tickets/"
  pattern: "*.md"
  id_regex: "^(\\d{4}[a-z]?)_"  # extracts ticket ID from filename

agents:
  stale_timeout_minutes: 10
  heartbeat_implicit: true  # update heartbeat on any MCP tool call

file_conflicts:
  mode: "advisory"  # "advisory" (warn) or "blocking" (prevent)

markdown_sync:
  status_update: "realtime"     # update checkboxes immediately on phase completion
  workflow_log_update: "batch"  # update workflow log section on CLI command

priority_order: ["Critical", "High", "Medium", "Low"]
```

The engine's `config.py` reads these YAML files and provides a typed configuration object to all other engine components. The engine has sensible defaults for all settings — `.workflow/config.yaml` is optional.

---

### 3. `server/server.py` — MCP Server

FastMCP server supporting both stdio (local development) and SSE (Docker) transports.

#### MCP Tools

| Tool | Parameters | Description |
|------|-----------|-------------|
| `register_agent` | `agent_type` | Register an agent instance, returns agent ID |
| `list_available_work` | `agent_type`, `limit?` | List phases available for claiming by agent type |
| `claim_phase` | `agent_id`, `phase_id?` | Atomically claim next available phase (or specific phase) |
| `heartbeat` | `agent_id` | Update agent liveness timestamp |
| `start_phase` | `agent_id`, `phase_id` | Mark claimed phase as running |
| `complete_phase` | `agent_id`, `phase_id`, `result_summary`, `artifacts?` | Report successful completion |
| `fail_phase` | `agent_id`, `phase_id`, `error_details` | Report phase failure |
| `release_phase` | `agent_id`, `phase_id` | Release a claimed phase back to available |
| `request_human_review` | `phase_id`, `gate_type`, `context` | Create a human gate blocking the next phase |
| `get_ticket_status` | `ticket_id` | Get full ticket status with all phases |
| `list_tickets` | `status_filter?`, `priority?`, `component?` | Query tickets by criteria |
| `list_blocked` | — | List all phases blocked on human gates or dependencies |
| `list_agents` | — | List registered agents and their current assignments |
| `declare_files` | `agent_id`, `phase_id`, `file_paths` | Declare files this phase will modify (conflict detection) |
| `check_conflicts` | `file_paths` | Check if any active phase has locks on these files |
| `get_phase_metrics` | `ticket_id?` | Get phase duration statistics |
| `get_audit_log` | `ticket_id?`, `limit?` | Query audit trail |

#### MCP Resources

| Resource | Description |
|----------|-------------|
| `workflow://dashboard` | Summary dashboard: active agents, pending gates, phase counts by status |
| `workflow://ticket/{id}` | Full ticket state with phase details |
| `workflow://queue/{agent_type}` | Work queue for a specific agent type |

#### Server Startup

The server receives the database path and the consuming repo's root directory as arguments:

```bash
# stdio mode (local development)
python -m workflow_engine.server /path/to/workflow.db --project-root /path/to/consuming-repo

# SSE mode (Docker)
python -m workflow_engine.server /app/workflow.db --project-root /app/project --transport sse --port 8080
```

On startup, the server reads `.workflow/phases.yaml` and `.workflow/config.yaml` from the project root to understand the phase definitions and configuration.

---

### 4. `engine/scheduler.py` — Phase Seeder

The scheduler is invoked on-demand (not a daemon). It:

1. **Imports tickets**: Reads `tickets/*.md` from the consuming repo, parses metadata per `ticket_metadata` config, creates/updates DB records
2. **Seeds phases**: For each ticket, creates phase rows matching the phase definitions in `phases.yaml` (respecting conditions)
3. **Resolves availability**: Marks phases as `available` when all prerequisites are met:
   - Prior phase in sequence is `completed`
   - No unresolved dependencies from other tickets
   - No pending human gate on the current phase
   - For parallel group phases: the `after` phase is `completed`
4. **Handles stale agents**: Checks `agents.last_heartbeat`; if stale (> timeout), releases their claimed phases
5. **Monitors completion**: When all phases for a ticket reach `completed`, updates the ticket markdown status

The scheduler reads all phase definitions from `phases.yaml` — it does not hardcode any phase names or agent types.

---

### 5. `cli/cli.py` — Human Review CLI

Command-line interface for human interaction with the work queue.

```bash
# All commands take --db and --project-root, or read from .workflow/config.yaml

# List pending human gates
workflow-engine gates

# Approve a gate
workflow-engine approve <gate-id> --notes "Looks good, proceed"

# Reject / request changes
workflow-engine reject <gate-id> --notes "Need to revisit the API surface"

# Query ticket status
workflow-engine status <ticket-id>

# List all tickets by state
workflow-engine list --status available --priority High

# Show blocked tickets
workflow-engine blocked

# Show active agents
workflow-engine agents

# Show work queue for an agent type
workflow-engine queue cpp-implementer

# Import tickets from markdown
workflow-engine import-tickets

# Sync DB status back to markdown
workflow-engine sync-markdown

# Show phase duration metrics
workflow-engine metrics

# Show audit log
workflow-engine audit --ticket 0083 --limit 50

# Add inter-ticket dependency
workflow-engine add-dep --blocked 0084 --blocking 0083

# Resolve a dependency manually
workflow-engine resolve-dep <dep-id>

# Run stale agent cleanup
workflow-engine cleanup-stale
```

When installed as a pip package, `workflow-engine` is available as a CLI entry point. When running from Docker, commands are prefixed with `docker exec`.

---

### 6. `engine/markdown_sync.py` — Ticket Markdown Synchronization

Project-agnostic markdown parsing and syncing:

**Import (markdown → DB):**
- Parse ticket markdown for metadata fields defined in `ticket_metadata` config
- Parse checkbox status to determine current workflow position
- Create/update ticket and phase records in DB
- Idempotent — can be run repeatedly

**Export (DB → markdown):**
- Update the status checkboxes in the ticket markdown to match DB state
- Update the Workflow Log section with timestamps, artifacts, and notes from phase records
- Preserve all human-authored content (requirements, design decisions, feedback)

**Conflict resolution:**
- DB is authoritative for status fields
- Markdown is authoritative for content fields
- If both have changed, log a warning and prefer DB status (human can override via CLI)

The parser uses the `ticket_metadata` configuration to know which fields to extract and their markdown keys. Custom parsing logic (if needed) can be provided via `.workflow/ticket_parser.py` in the consuming repo.

---

## Docker Distribution

```dockerfile
FROM python:3.12-slim
COPY engine/ /app/engine/
COPY server/ /app/server/
COPY cli/ /app/cli/
COPY pyproject.toml /app/
WORKDIR /app
RUN pip install .
EXPOSE 8080
# Mount consuming repo's .workflow/ and tickets/ at runtime
ENTRYPOINT ["workflow-engine-server"]
```

```yaml
# docker-compose.yaml in consuming repo
services:
  workflow:
    image: workflow-engine:latest
    volumes:
      - ./.workflow:/app/project/.workflow:ro      # Phase definitions + config
      - ./tickets:/app/project/tickets:ro           # Ticket markdown files
      - ./build/Debug/docs:/app/data                # DB storage (writable)
    environment:
      - PROJECT_ROOT=/app/project
      - DB_PATH=/app/data/workflow.db
    ports:
      - "8081:8080"                                 # SSE transport
```

### MCP Registration in Consuming Repos

**Option A: stdio via pip-installed package (local development)**
```json
{
  "mcpServers": {
    "workflow": {
      "command": "python/.venv/bin/python3",
      "args": ["-m", "workflow_engine.server", "build/Debug/docs/workflow.db", "--project-root", "."]
    }
  }
}
```

**Option B: SSE via Docker (multi-repo / CI)**
```json
{
  "mcpServers": {
    "workflow": {
      "url": "http://localhost:8081/sse"
    }
  }
}
```

---

## Modified Components (MSD-CPP repo)

### `.mcp.json` — Register Workflow MCP Server

Add the workflow server alongside existing servers (stdio mode for local dev):

```json
{
  "workflow": {
    "command": "python/.venv/bin/python3",
    "args": ["-m", "workflow_engine.server", "build/Debug/docs/workflow.db", "--project-root", "."]
  }
}
```

---

### `.workflow/phases.yaml` — MSD-CPP Phase Definitions

See the full YAML in the `engine/config.py` section above. This file encodes:
- The complete MSD-CPP ticket lifecycle (all statuses from `ticket.md.template`)
- Agent type mappings for each phase
- Conditional phases (math design, tutorial, multi-language)
- Parallel groups for multi-language implementation
- Metadata extraction rules for ticket markdown parsing

---

### `.workflow/config.yaml` — MSD-CPP Configuration

See the full YAML in the `engine/config.py` section above. This file sets:
- Database path (`build/Debug/docs/workflow.db`)
- Ticket directory and naming pattern
- Stale timeout and heartbeat mode
- File conflict mode (advisory)
- Markdown sync frequency

---

### `.claude/agents/workflow-orchestrator.md` — Refactor to Scheduler Role

**Current behavior**: Reads ticket markdown, determines current phase, spawns agent, waits for completion, updates markdown, repeats.

**New behavior**: The orchestrator becomes a thin coordinator that:
1. Calls `import-tickets` to sync markdown state into DB
2. Calls the scheduler to seed/update phase availability
3. Reports the current work queue state
4. Does NOT execute agents — agents are independent and claim work via MCP tools

The orchestrator is still useful for:
- Processing a single ticket through the DB pipeline (`Process ticket: 0083`)
- Checking overall status (`List tickets`, `Status: 0083`)
- Seeding new tickets into the DB

**Key changes to orchestrator prompt:**
- Remove all "invoke agent X" logic — agents self-dispatch
- Replace with "ensure phases are seeded and available"
- Add instructions to use `workflow` MCP tools for state management
- Preserve human gate enforcement (orchestrator can remind human of pending gates)

---

### Agent Definitions (all `.claude/agents/*.md`) — Add MCP Tool Usage

Each agent definition gains a standard preamble for interacting with the work queue:

```markdown
### Work Queue Integration

Before starting work:
1. Call `register_agent` with your agent type to get an agent ID
2. Call `claim_phase` to atomically claim an available phase
3. Call `declare_files` with the files you intend to modify
4. Call `check_conflicts` to verify no other agent holds locks on those files
5. Call `start_phase` to mark the phase as running

During work:
- Call `heartbeat` periodically (every 2 minutes) to maintain liveness
- If you need human input, call `request_human_review` with context

After completing work:
- Call `complete_phase` with result summary and artifacts list
- Or call `fail_phase` with error details if the phase cannot be completed
```

This is additive — existing agent instructions remain unchanged.

---

### `python/requirements.txt` — Add workflow-engine Dependency

```
# Workflow engine (installed from local clone or pip)
workflow-engine @ file:///path/to/workflow-engine
# or: workflow-engine>=0.1.0
```

The `python/setup.sh` script installs it into the shared venv alongside other tooling.

---

## Concurrency Model

### Within a Single Ticket
Phases execute **sequentially** — each phase's `available` status depends on the prior phase being `completed`. This preserves the existing workflow invariant.

Exception: Phases in a `parallel_group` (defined in `phases.yaml`) become available simultaneously and must all complete before the next sequential phase.

### Across Tickets
Independent tickets execute **concurrently** — agents can claim phases from different tickets simultaneously. The atomic claim query ensures no conflicts.

### Conflict Detection
When an agent declares files via `declare_files`, the server checks `file_locks` for any active (unreleased) locks on the same paths held by other agents. If conflicts exist, the agent is warned but not blocked (advisory, not mandatory). The human can resolve by reordering work or adding a dependency.

### Stale Agent Recovery
The scheduler (or `cleanup-stale` CLI command) checks `agents.last_heartbeat`. If an agent hasn't heartbeated in > `stale_timeout_minutes`:
1. Mark agent status as `stale`
2. Release any claimed phases back to `available`
3. Release any file locks
4. Log to audit trail

---

## Design Revision Loop Support

The engine supports MSD-CPP's Design Revision Loop (ticket 0079) through project-specific metadata stored in `tickets.custom_metadata`:

```json
{
  "design_revision_count": 0,
  "previous_design_approaches": []
}
```

The engine doesn't interpret these fields — it stores them as a JSON blob. The consuming repo's agents and orchestrator read/update them via `get_ticket_status` and a `update_ticket_metadata` MCP tool.

Additional MCP tool:

| Tool | Parameters | Description |
|------|-----------|-------------|
| `update_ticket_metadata` | `ticket_id`, `metadata` (JSON) | Update project-specific metadata fields |

This keeps the engine project-agnostic while allowing MSD-CPP's agents to track revision state.

---

## Migration Strategy

### Phase 1: Create workflow-engine repo
- Set up repo structure, pyproject.toml, Dockerfile
- Implement schema, config reader, scheduler, claim logic
- Implement MCP server with all tools
- Implement CLI
- Write tests
- Publish Docker image

### Phase 2: Integrate MSD-CPP as first consumer
- Create `.workflow/phases.yaml` and `.workflow/config.yaml`
- Add `workflow-engine` to `python/requirements.txt`
- Register workflow MCP server in `.mcp.json`
- Import existing tickets into DB
- Test with agents claiming work manually

### Phase 3: Agent integration (follow-up ticket)
- Add work queue preamble to all agent definitions
- Agents use MCP tools for claim/complete/heartbeat
- Orchestrator transitions to scheduler role

### Phase 4: Deprecate legacy mode (follow-up ticket)
- Remove direct agent invocation from orchestrator
- All work routing goes through the database
- Add dashboard/metrics reporting

---

## Test Impact

### Existing Tests Affected
None — this is new infrastructure in a separate repository. No MSD-CPP C++ source is modified.

### New Tests Required (workflow-engine repo)

#### Unit Tests (`tests/`)
| Test | What It Validates |
|------|-------------------|
| `test_schema.py` | Schema creation, table constraints, indexes |
| `test_claim.py` | Atomic claiming, no double-claims, priority ordering |
| `test_phase_machine.py` | Phase state transitions, invalid transition rejection |
| `test_stale_recovery.py` | Stale agent detection, phase release, lock cleanup |
| `test_human_gates.py` | Gate creation, approval/rejection, downstream unblocking |
| `test_dependencies.py` | Inter-ticket dependencies, resolution, cascade |
| `test_file_locks.py` | Lock acquisition, conflict detection, release |
| `test_markdown_sync.py` | Import from markdown, export to markdown, conflict resolution |
| `test_scheduler.py` | Phase seeding, conditional phases, parallel phases |
| `test_audit.py` | Audit log entries for all state transitions |
| `test_config.py` | phases.yaml parsing, config.yaml defaults, condition evaluation |

#### Integration Tests
| Test | What It Validates |
|------|-------------------|
| `test_mcp_tools.py` | MCP tool registration, parameter validation, tool execution |
| `test_concurrent_agents.py` | Two simulated agents claiming from same queue without conflict |
| `test_full_lifecycle.py` | Ticket from import through all phases to completion |

---

## Open Questions

### Design Decisions (Human Input Needed)

1. **Database location** — `build/{build_type}/docs/workflow.db` (gitignored, like other DBs) or project root (tracked)?
   - **Recommendation**: Gitignored in build directory. The DB is derived state; markdown tickets are the durable record. Each developer/worktree runs `workflow-engine import-tickets` on setup.

2. **Agent spawning** — Should the system provide a launcher script that spawns N agent processes, or should agents be launched manually?
   - Option A: Manual launch — developer runs `claude --agent cpp-implementer` in separate terminals
   - Option B: Launcher script — `workflow-engine launch --agents 4` spawns a pool
   - **Recommendation**: Start with Option A (manual); add Option B as a convenience in Phase 4

3. **Heartbeat mechanism** — How should agents heartbeat when they're Claude Code subagents (not long-running processes)?
   - Option A: Heartbeat on each MCP tool call (implicit — server updates timestamp on any tool invocation by that agent)
   - Option B: Explicit heartbeat calls at intervals
   - **Recommendation**: Option A (implicit heartbeat on any tool call) — simpler, no timer needed in agent prompts

4. **Markdown sync frequency** — Should the DB update markdown in real-time (after each phase completion) or in batch (on CLI command)?
   - **Recommendation**: Real-time for status updates (lightweight); batch for Workflow Log updates (heavier parsing)

5. **Distribution model** — Docker-only or also pip-installable?
   - Option A: Docker-only (like 0081 guidelines-server)
   - Option B: Both Docker and pip-installable package
   - **Recommendation**: Option B — Docker for multi-repo CI, pip for local development with `python/.venv`

---

## Design Decisions

### DD-0083-001: Standalone Repository
- **Affects**: All workflow infrastructure
- **Rationale**: The workflow engine is project-agnostic — it knows about tickets, phases, agents, and gates but nothing about spacecraft dynamics, C++, or any specific domain. Extracting it to a standalone repo enables reuse across repositories and follows the precedent set by ticket 0081 (guidelines-server extraction). Project-specific configuration lives in the consuming repo's `.workflow/` directory.
- **Alternatives Considered**:
  - In-tree (scripts/workflow/): Simpler initially, but creates tight coupling and prevents reuse. Every project that wants workflow orchestration would need to copy or fork the code.
  - Monorepo with guidelines-server: They serve different purposes, have different release cadences, and coupling them creates unnecessary coordination overhead.
- **Trade-offs**: Adds a repository to maintain and a dependency to manage. Acceptable given the reuse benefit and the established extraction pattern.
- **Status**: active

### DD-0083-002: SQLite as Coordination Store
- **Affects**: All workflow infrastructure
- **Rationale**: SQLite is already used for codebase.db, traceability.db, and guidelines.db in MSD-CPP. It requires no server process, supports concurrent reads via WAL mode, and provides ACID transactions for atomic claiming.
- **Alternatives Considered**:
  - PostgreSQL: Requires running a server; overkill for single-developer workflow coordination
  - Redis: Better for pub/sub but adds an external dependency; not persistent by default
  - JSON file: No atomicity guarantees; prone to corruption under concurrent access
- **Trade-offs**: SQLite's write concurrency is limited (one writer at a time), but WAL mode mitigates this for the expected workload.
- **Status**: active

### DD-0083-003: Markdown Remains Content Store
- **Affects**: markdown_sync.py, ticket workflow
- **Rationale**: Ticket markdown contains rich narrative (requirements, design decisions, feedback, workflow logs) that is best authored and read as text files, version-controlled in git. The database mirrors only the queryable metadata. This avoids duplicating content and keeps the existing ticket authoring workflow intact.
- **Alternatives Considered**:
  - Database-only tickets: Would lose git history, diffability, and the ability to review tickets in PRs
  - Full bidirectional sync: Complex conflict resolution; markdown is not a reliable structured data format for round-tripping
- **Trade-offs**: Requires a sync mechanism. Import is straightforward (parse markdown → update DB). Export is limited to status checkboxes and workflow log timestamps.
- **Status**: active

### DD-0083-004: Configuration-Driven Phase Definitions
- **Affects**: phases.yaml, config.py, scheduler.py
- **Rationale**: Hardcoding phase definitions in the engine would make it MSD-CPP-specific. Reading phase definitions from `.workflow/phases.yaml` makes the engine fully generic — any project can define its own lifecycle, agent types, conditions, and parallel groups. The engine interprets the YAML schema; it doesn't know what "cpp-architect" means.
- **Alternatives Considered**:
  - Plugin system: More flexible but more complex; YAML configuration covers all known use cases
  - Python config file: More powerful (lambdas for conditions) but less portable and harder to validate
- **Trade-offs**: YAML conditions are limited to simple field matching (equality, contains, has_multiple). Complex conditions require the optional `ticket_parser.py` extension point.
- **Status**: active

### DD-0083-005: Phase-Granularity Work Items
- **Affects**: phases table, agent claiming, scheduler
- **Rationale**: Phases are the natural unit of work — each maps to exactly one agent type and produces defined artifacts. Phase-level claiming enables maximum parallelism within the state machine constraints, including parallel groups for multi-language tickets.
- **Alternatives Considered**:
  - Ticket-level claiming: Simpler but prevents intra-ticket parallelism
  - Task-level claiming (sub-phase): Too granular; agents manage their own internal task decomposition
- **Trade-offs**: More rows in the phases table; slightly more complex availability resolution. Acceptable given the benefits.
- **Status**: active

### DD-0083-006: Advisory File Conflict Detection
- **Affects**: file_locks table, declare_files/check_conflicts tools
- **Rationale**: Hard file locking would block agents and require deadlock detection. Advisory warnings are sufficient — the human can reorder work or add a dependency.
- **Alternatives Considered**:
  - Hard locking with blocking: Complex deadlock scenarios; agents could stall indefinitely
  - No conflict detection: Risk of silent merge conflicts discovered only at commit time
- **Trade-offs**: Advisory conflicts may be ignored. Acceptable for the current scale.
- **Status**: active

### DD-0083-007: Implicit Heartbeat on MCP Tool Calls
- **Affects**: agents table, server.py, stale recovery
- **Rationale**: Claude Code agents don't have timer-based callbacks. Updating `last_heartbeat` on every tool call provides a natural liveness signal without requiring explicit heartbeat instructions in every agent prompt.
- **Alternatives Considered**:
  - Explicit heartbeat tool: Requires agents to remember to call it; adds noise to agent prompts
  - Process-level monitoring: Claude Code agents are subprocesses managed by the CLI; no direct process monitoring API
- **Trade-offs**: An agent that goes quiet for > timeout will be marked stale. The timeout should be generous (10+ minutes).
- **Status**: active

### DD-0083-008: Dual Distribution (Docker + pip)
- **Affects**: pyproject.toml, Dockerfile, consuming repo setup
- **Rationale**: Docker provides hermetic packaging for multi-repo CI (same pattern as 0081 guidelines-server). Pip installation provides a simpler local development experience — install into `python/.venv` and register as stdio MCP server, no container management needed. Supporting both maximizes adoption flexibility.
- **Alternatives Considered**:
  - Docker-only: Forces container management for local development, which adds friction for a single-developer workflow
  - Pip-only: Loses the hermetic packaging benefit for CI and multi-repo deployments
- **Trade-offs**: Two distribution paths to maintain and test. Acceptable given the different use cases.
- **Status**: active

### DD-0083-009: Project-Specific Metadata as JSON Blob
- **Affects**: tickets.custom_metadata, update_ticket_metadata tool
- **Rationale**: Features like MSD-CPP's Design Revision Loop (0079) require project-specific ticket metadata (revision count, previous approaches). Rather than adding project-specific columns to the engine's schema, a `custom_metadata` JSON blob allows any consuming repo to store arbitrary fields. The engine stores and retrieves them; it doesn't interpret them.
- **Alternatives Considered**:
  - Project-specific columns: Would break the engine's project-agnosticism
  - Separate metadata table with key-value rows: More queryable but adds complexity for what is currently a simple use case
- **Trade-offs**: JSON blob fields are not individually queryable via SQL. If advanced querying of custom metadata is needed later, a migration to a key-value table can be considered.
- **Status**: active

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-27
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Module organization | ✓ | `engine/`, `server/`, `cli/` separation is clean. Mirrors the guidelines-server pattern (schema.py, server.py, seed script). |
| Naming conventions | ✓ | Python modules follow `snake_case`. Public symbols consistent with existing MSD-CPP Python tooling. |
| Dependency direction | ✓ | `server/` depends on `engine/`; `cli/` depends on `engine/`. No cycles. `engine/` has no dependency on consuming repo code — it reads YAML config. |
| Consistency with existing MCP servers | ✓ | FastMCP pattern with class + factory function matches `guidelines_server.py` and `mcp_codebase_server.py`. |
| Standalone repo separation | ✓ | Engine is fully project-agnostic. MSD-CPP-specific knowledge is externalized to `.workflow/phases.yaml` and `.workflow/config.yaml`, following the same pattern as `.guidelines/` in ticket 0081. |
| SQLite consistency | ✓ | Follows the existing pattern (codebase.db, traceability.db, guidelines.db). WAL mode + PRAGMA foreign_keys = ON is correct. |

#### Python Design Quality

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Type annotations | ~ | Design describes data classes in `models.py` but does not specify whether they use `dataclasses.dataclass`, `pydantic.BaseModel`, or plain classes. Given the existing replay server uses Pydantic, this should be clarified. |
| Async patterns | ✓ | Not required. FastMCP in stdio/SSE modes handles concurrency at the MCP layer. Synchronous SQLite access is appropriate — SQLite's WAL mode provides read concurrency; atomic claiming serializes writes by design. |
| Error handling | ~ | The design specifies error_details in the schema but does not describe the Python exception hierarchy in `engine/` or how errors surface to MCP tool callers (error dicts vs. exceptions). Should match guidelines_server.py's error-dict-on-not-found pattern. |
| Resource management | ✓ | Not specified explicitly, but SQLite connections via context managers (`with conn:`) are the established pattern in the codebase and must be used. The design implies this via WAL mode and transaction semantics. |
| Idempotency | ✓ | `import-tickets` idempotency explicitly stated. Scheduler `seed phases` must be idempotent on repeated runs — the UNIQUE(ticket_id, phase_name) constraint enforces this at the DB level. |
| Configuration validation | ~ | `config.py` reads YAML but the design does not specify how invalid configuration is reported (missing required fields, unknown agent types, invalid condition syntax). Validation errors should fail fast with clear messages. |

#### Feasibility

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Atomic claiming | ✓ | The `UPDATE ... WHERE ... RETURNING *` query is correct for SQLite 3.35+. Python 3.12 ships SQLite 3.39+. This is safe. |
| WAL mode concurrency | ✓ | WAL mode permits concurrent readers + single writer. The write load is low (one claim at a time) and brief. Appropriate for the expected single-developer workload. |
| SSE transport | ✓ | FastMCP supports SSE natively. Docker Compose mounts are correctly specified. |
| pip + Docker distribution | ✓ | `pyproject.toml` entry point (`workflow-engine`) + Dockerfile is the right approach. Consistent with guidelines-server extraction plan in 0081. |
| `.workflow/` configuration parsing | ✓ | YAML parsing with PyYAML (already in `python/requirements.txt` for guidelines seeder) is appropriate. Condition evaluation for `contains`, `has_multiple`, and equality is straightforward. |
| markdown_sync parsing | ~ | Parsing checkbox state from markdown is fragile if ticket authors deviate from the `- [x]` format. The design mentions a `ticket_parser.py` extension point in the consuming repo, which is the correct escape hatch. The base parser must be defensive. |
| Parallel phase group resolution | ✓ | The `parallel_group` column + `after`/`before` semantics in phases.yaml are well-specified. Availability resolution: mark all group members available when `after` phase completes; block next sequential phase until all group members complete. |
| Python 3.12 requirement | ✓ | Implied by the slim Docker base image. The `|` union type syntax used in guidelines_server.py requires Python 3.10+. Explicitly state minimum version in `pyproject.toml`. |

#### Testability

| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Isolation possible | ✓ | SQLite in-memory databases (`:memory:`) enable fully isolated unit tests for schema, claim, state_machine, scheduler, and audit. |
| Mockable dependencies | ✓ | `engine/` modules take explicit `conn` or `db_path` parameters, enabling test injection. Config reader takes a path, enabling temp directories. |
| Observable state | ✓ | All state is in the SQLite DB — tests can query directly after calling engine functions. |
| MCP tool integration tests | ✓ | FastMCP supports in-process testing via the Python API without a network connection. |
| Concurrent agent test | ✓ | `test_concurrent_agents.py` specified. Using Python `threading` + in-memory SQLite will validate the atomic claim query correctly. Note: in-memory SQLite does not share state across connections by default — tests should use a temp file or `sqlite3.connect("file::memory:?cache=shared&uri=true")`. |

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | Heartbeat implicit model may produce false-positive stale detection if an agent calls no MCP tools for > 10 min during a long-running phase (e.g., a compilation or test run inside the agent's work) | Reliability | Med | Med | Document minimum heartbeat-call frequency in agent prompt preamble; consider a 30-min default timeout rather than 10-min for Phase 1 | No |
| R2 | markdown_sync checkbox parser breaks if ticket markdown deviates from the expected checkbox format (e.g., extra spaces, different heading structure) | Feasibility | Low | Low | Defensive regex with test fixtures covering edge cases; `ticket_parser.py` extension point is the escape hatch | No |
| R3 | `phases.yaml` condition evaluation is limited (contains, has_multiple, equality) — complex conditions (e.g., "C++ AND Frontend but not Python") are not expressible | Feasibility | Low | Low | Acceptable for current phase definitions; `ticket_parser.py` extension point handles complex cases | No |
| R4 | Two-phase worktree scenario: a developer working in a git worktree and a CI agent running concurrently will share the DB at `build/Debug/docs/workflow.db` only if they use the same build directory. If worktrees use separate build directories, they have separate DBs with no cross-worktree coordination | Architecture | Med | Low | Document that the DB is per-worktree; cross-worktree coordination would require a shared DB path in config | No |
| R5 | The `RETURNING *` clause in the atomic claim query requires that Python's `sqlite3` module uses autocommit or that the UPDATE is inside an explicit transaction. If FastMCP or SQLite connection setup disables autocommit, the UPDATE may not be visible to other connections immediately | Reliability | Low | High | Prototype the atomic claim in isolation before implementation to verify transaction semantics | Yes |

### Prototype Guidance

#### Prototype P1: Atomic Claim Transaction Semantics

**Risk addressed**: R5
**Question to answer**: Does `UPDATE ... RETURNING *` inside a Python `sqlite3` transaction correctly serialize concurrent claims from two threads connecting to the same WAL-mode database file, with no double-claims?

**Success criteria**:
- 100 concurrent claim attempts from 2 threads produce exactly N unique claims (where N = number of rows in `phases` table)
- No two threads receive the same `phase_id` from the `RETURNING *` result
- No deadlocks or `sqlite3.OperationalError: database is locked` errors

**Prototype approach**:
```
Location: prototypes/0083_database_agent_orchestration/p1_atomic_claim/
Type: Standalone Python script (pytest fixture + threading)

Steps:
1. Create a temp file SQLite DB with WAL mode enabled
2. Insert 10 phases all with status='available'
3. Spawn 2 threads, each attempting to claim all 10 phases sequentially
4. Collect all phase_ids returned by RETURNING *
5. Assert no duplicates and total claim count == 10
```

**Time box**: 30 minutes

**If prototype fails**:
- Wrap the UPDATE in an explicit `BEGIN IMMEDIATE` transaction to serialize writers
- Alternatively, use `sqlite3.connect(check_same_thread=False)` with a Python threading.Lock for the claim operation

### Open Question Resolutions

The following open questions from the design were resolved in the Design Decisions section. No human input is needed to proceed:

| Question | Resolution | Decision |
|----------|------------|----------|
| Database location | Gitignored in build directory | DD-0083-001 (derived state, not tracked) |
| Agent spawning model | Manual launch first (Option A) | DD-0083-001 (defer launcher script to Phase 4) |
| Heartbeat mechanism | Implicit on any MCP tool call | DD-0083-007 |
| Markdown sync frequency | Real-time for status; batch for Workflow Log | Design section 6 |
| Distribution model | Both Docker and pip | DD-0083-008 |

### Minor Notes (No Changes Required)

1. **`models.py` data classes**: Recommend using `@dataclasses.dataclass(frozen=True)` for immutable record types (phases, tickets) and mutable `@dataclasses.dataclass` for agent state. Pydantic is acceptable if the team prefers runtime validation. Document the choice in `models.py`.

2. **`pyproject.toml` Python version floor**: Set `requires-python = ">=3.10"` (union type syntax) or `>=3.12` (matches Docker base image). Do not leave it unspecified.

3. **WAL checkpoint strategy**: The design mentions WAL mode and periodic checkpoints. Explicitly specify that the server runs `PRAGMA wal_autocheckpoint = 1000` (default) — this is sufficient for the expected write volume and requires no additional configuration.

4. **CLI entry point naming**: `workflow-engine` as a CLI entry point conflicts with the package name as a Python import (`workflow_engine`). Prefer `wfe` or `workflow-cli` as the entry point name to avoid shell confusion (hyphen in Python import names requires importlib or workarounds).

5. **`phases.yaml` parallel groups structure**: The current design nests parallel group phases inside a `parallel_groups` top-level key separate from the sequential `phases` list. This creates two lookups to build the full ordered phase sequence. Consider flattening to a single `phases` list where parallel phases carry `parallel_group: impl` as an attribute — consistent with the `parallel_group` column in the `phases` DB table.

### Summary

The design is architecturally sound and well-specified. The standalone repo extraction, SQLite coordination layer, FastMCP server pattern, and `.workflow/` configuration externalization are all consistent with established MSD-CPP infrastructure patterns. The schema is appropriately normalized, the atomic claim query is correct for SQLite 3.35+, and the test plan covers all major components.

One prototype is recommended (P1: Atomic Claim Transaction Semantics) before full implementation to validate concurrent claim behavior. This is low-risk but high-impact to verify before building the full claim infrastructure.

The design is ready for human review and proceeds to the Prototype phase upon approval.
