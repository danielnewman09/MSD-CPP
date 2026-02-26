# Iteration Log — 0078_cpp_guidelines_mcp_server

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/0078_cpp_guidelines_mcp_server/iteration-log.md`
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0078_cpp_guidelines_mcp_server
**Branch**: 0078-cpp-guidelines-mcp-server
**Baseline**: N/A — pure Python implementation, no C++ tests affected

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — 2026-02-26 16:30

**Commit**: (pending — first and only iteration)
**Hypothesis**: Implement all 4 phases of the design in one pass — the design is fully
specified with no open questions, no novel algorithms, and follows established patterns
from `mcp_codebase_server.py` and `scripts/traceability/`. A single iteration should
suffice.

**Changes**:
- `scripts/guidelines/guidelines_schema.py`: Created — SQLite DDL for 6 tables + FTS5 triggers
- `scripts/guidelines/seed_guidelines.py`: Created — Pydantic-validated YAML → SQLite pipeline
- `scripts/guidelines/guidelines_server.py`: Created — FastMCP server with 5 tools + CLI mode
- `scripts/guidelines/data/project_rules.yaml`: Created — 10 MSD project rules
- `scripts/guidelines/data/cpp_core_guidelines.yaml`: Created — empty stub
- `scripts/guidelines/data/misra_rules.yaml`: Created — empty stub
- `python/requirements.txt`: Added `pyyaml==6.0.2`
- `.mcp.json`: Added `guidelines` server entry
- `.claude/settings.local.json`: Added `"guidelines"` to `enabledMcpjsonServers`
- `CMakeLists.txt`: Added `guidelines-seed` custom target
- `CMakeUserPresets.json`: Added `debug-guidelines` preset
- `docs/designs/0078_cpp_guidelines_mcp_server/implementation-notes.md`: Created

**Build Result**: PASS — `cmake --build --preset debug-guidelines` succeeded on first run.
Seeded 10 rules. No reconfiguration errors after `cmake --preset conan-debug` refresh.

**Test Result** (CLI smoke tests): 8/8 acceptance criteria PASS

| Test | Result |
|------|--------|
| `search_guidelines("brace initialization")` → MSD-INIT-002 | PASS |
| `get_rule("MSD-RES-001")` → full rule + tags + cross-refs + status | PASS |
| `list_categories()` → 6 categories with correct rule counts | PASS |
| `get_category("Initialization")` → summary mode | PASS |
| `get_category("Initialization", --detailed)` → full rationale + examples | PASS |
| `get_rules_by_tag("safety")` → MSD-INIT-001, MSD-RES-001, MSD-RES-003 | PASS |
| FTS porter stemming: "initialize" → MSD-INIT-001, MSD-INIT-002 | PASS |
| `get_rule("MSD-DOES-NOT-EXIST")` → `{"error": "Rule not found", ...}` | PASS |

**Impact vs Previous**: N/A — baseline is zero (new feature, no prior test suite)
**Assessment**: All design requirements implemented correctly on first pass. No circles,
no regressions. Ready for quality gate / implementation review.
