# Ticket 0081: Extract Guidelines Server to Standalone Multi-Language Service

## Status
- [x] Draft
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Implementation
- [ ] Implementation Complete
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Design Approved — Ready for Implementation
**Type**: Infrastructure / Extraction
**Priority**: Medium
**Created**: 2026-02-26
**Generate Tutorial**: No
**Depends On**: 0078 (base implementation), 0078b (C++ Core Guidelines population)

---

## Summary

Extract the guidelines MCP server from `scripts/guidelines/` into a standalone repository that supports multiple programming languages (C++, Python, JavaScript, HTML/CSS) and serves shared guidelines to multiple consuming repositories via Docker. Project-specific rules remain in each consuming repo and are injected at seed time.

---

## Problem

The guidelines server (0078) was built as an in-tree tool for a single C++ project. Three limitations have emerged:

1. **Single-repo scope** — Other repositories that should follow the same C++ Core Guidelines / MISRA rules cannot access the server without duplicating code
2. **Single-language** — The schema assumes C++ implicitly. Rules for Python (PEP 8, type hints), JavaScript (ESLint), and HTML/CSS (accessibility, BEM) have no home
3. **Entangled data** — Shared guidelines (C++ Core Guidelines, PEP 8) and project-specific conventions (MSD-INIT-001) live in the same YAML files with no separation boundary

---

## Solution

### New Repository: `guidelines-server`

```
guidelines-server/                    # Standalone repo
├── data/
│   ├── cpp/
│   │   ├── core_guidelines.yaml      # C++ Core Guidelines (from 0078b)
│   │   └── misra.yaml                # MISRA rules (from 0078c)
│   ├── python/
│   │   ├── pep8.yaml                 # PEP 8 style rules
│   │   └── typing.yaml               # Type hint conventions
│   ├── javascript/
│   │   ├── eslint_recommended.yaml   # ESLint recommended rules
│   │   └── typescript.yaml            # TypeScript-specific rules
│   └── html_css/
│       └── accessibility.yaml         # WCAG / accessibility rules
│
├── server/
│   ├── schema.py                      # Extended schema with language dimension
│   ├── seeder.py                      # YAML -> SQLite indexer (supports project overlay)
│   └── server.py                      # FastMCP server (stdio + SSE modes)
│
├── Dockerfile
├── docker-compose.yaml
├── pyproject.toml                     # Package metadata
└── README.md
```

### Consuming Repo Layout

Each repo that uses the guidelines server provides project-specific rules:

```
MSD-CPP/
└── .guidelines/
    └── project_rules.yaml             # MSD-INIT-001, MSD-RES-002, etc.

other-repo/
└── .guidelines/
    └── project_rules.yaml             # That repo's conventions
```

### Schema Changes

#### New: `language` column on rules

```sql
ALTER TABLE rules ADD COLUMN language TEXT NOT NULL DEFAULT 'cpp';
```

- Valid values: `cpp`, `python`, `javascript`, `html_css`
- All existing rules get `language = 'cpp'`
- FTS5 index includes `language` for filtered search

#### `language` parameter on MCP tools

All query tools require a `language` parameter (except `get_rule`, where rule_id is globally unique):

| Tool | Parameter |
|------|-----------|
| `search_guidelines` | `language: string` — required, filter results to one language |
| `list_categories` | `language: string` — required, categories for a specific language |
| `get_category` | `language: string` — required, rules in category for a specific language |
| `get_rules_by_tag` | `language: string` — required, tagged rules for a specific language |
| `get_rule` | (no change — rule_id is globally unique) |

### Rule ID Conventions (Extended)

| Source | Pattern | Example |
|--------|---------|---------|
| C++ Core Guidelines | `CPP-{section}.{number}` | `CPP-R.11` |
| MISRA | `MISRA-{rule}` | `MISRA-6.2` |
| PEP 8 | `PEP8-{section}` | `PEP8-E501` |
| Python typing | `PY-TYPE-{NNN}` | `PY-TYPE-001` |
| ESLint | `JS-{rule-name}` | `JS-NO-VAR` |
| TypeScript | `TS-{rule-name}` | `TS-STRICT-NULL` |
| WCAG | `WCAG-{criterion}` | `WCAG-1.1.1` |
| Project (any repo) | `{PREFIX}-{CAT}-{NNN}` | `MSD-INIT-001` |

### Docker Distribution

```dockerfile
FROM python:3.12-slim
COPY server/ /app/server/
COPY data/ /app/data/
WORKDIR /app
RUN pip install fastmcp pyyaml
EXPOSE 8080
# Seed DB from shared data + any mounted project rules, then start server
ENTRYPOINT ["python", "server/entrypoint.py"]
```

```yaml
# docker-compose.yaml in consuming repo
services:
  guidelines:
    image: guidelines-server:latest
    volumes:
      - ./.guidelines:/app/project_data:ro    # Mount project-specific rules
    environment:
      - LANGUAGES=cpp,python                   # Which languages to seed
    ports:
      - "8080:8080"                            # SSE transport
```

### MCP Registration in Consuming Repos

**Option A: stdio via docker exec**
```json
{
  "mcpServers": {
    "guidelines": {
      "command": "docker",
      "args": ["exec", "-i", "guidelines-server", "python", "server/server.py", "/app/guidelines.db"]
    }
  }
}
```

**Option B: SSE transport (preferred for multi-repo)**
```json
{
  "mcpServers": {
    "guidelines": {
      "url": "http://localhost:8080/sse"
    }
  }
}
```

### Data Migration Path

| Current Location | Destination |
|------------------|-------------|
| `scripts/guidelines/data/cpp_core_guidelines.yaml` | `guidelines-server/data/cpp/core_guidelines.yaml` |
| `scripts/guidelines/data/misra_rules.yaml` | `guidelines-server/data/cpp/misra.yaml` |
| `scripts/guidelines/data/project_rules.yaml` | `MSD-CPP/.guidelines/project_rules.yaml` |
| `scripts/guidelines/guidelines_schema.py` | `guidelines-server/server/schema.py` (extended) |
| `scripts/guidelines/seed_guidelines.py` | `guidelines-server/server/seeder.py` (extended) |
| `scripts/guidelines/guidelines_server.py` | `guidelines-server/server/server.py` (extended) |

---

## Design Decisions

### DD-0081-001: Project rules stay in consuming repos
**Rationale**: Rules like `MSD-INIT-001` reference project-specific patterns, ticket IDs, and class names. They belong under the project's version control, not in a shared server repo. The server merges them at seed time via volume mount.

### DD-0081-002: Language as a column, not separate databases
**Rationale**: A single DB with a `language` column enables cross-language search (e.g., "brace initialization" matches C++ rules, "bracket spacing" matches JS rules). Separate DBs would require running multiple server instances and lose cross-cutting queries.

### DD-0081-003: Docker as primary distribution
**Rationale**: Multi-repo consumers need a distribution mechanism that doesn't require cloning the guidelines repo or managing Python dependencies. Docker provides hermetic packaging. SSE transport is simpler to configure than stdio-over-docker-exec.

### DD-0081-004: LANGUAGES environment variable for selective seeding
**Rationale**: A C++-only project shouldn't need to seed Python/JS rules. The `LANGUAGES` env var controls which `data/{language}/` directories are loaded, keeping the DB and FTS index focused.

### DD-0081-005: `language` is a required parameter
**Rationale**: This is a new service, not an evolution of the in-tree server. Requiring `language` on all query tools (except `get_rule`, where rule_id is globally unique) enforces intentional usage and prevents accidental cross-language noise in results. No backward compatibility concern since there are no existing consumers of this new API.

---

## Implementation Steps

### Phase 1: Create Standalone Repository
1. Create `guidelines-server` repo with directory structure
2. Port schema, seeder, and server code with `language` as required column
3. Port shared YAML data (`cpp_core_guidelines.yaml`, `misra.yaml`) into `data/cpp/`
4. Add `language` field to all YAML files and make `language` required on query tools
5. Add `pyproject.toml`, `Dockerfile`, `docker-compose.yaml`
6. Add entrypoint script that seeds DB then starts server

### Phase 2: Add New Language Rules
7. Create `data/python/pep8.yaml` — initial population of key PEP 8 rules
8. Create `data/javascript/eslint_recommended.yaml` — initial ESLint rules
9. Create `data/html_css/accessibility.yaml` — initial WCAG rules
10. Each file follows existing YAML schema with `language` field

### Phase 3: Update MSD-CPP as Consumer
11. Move `scripts/guidelines/data/project_rules.yaml` to `.guidelines/project_rules.yaml`
12. Remove `scripts/guidelines/` directory
13. Remove `guidelines-seed` CMake target and `debug-guidelines` preset
14. Update `.mcp.json` to point to Docker-based server
15. Update `CLAUDE.md` to document new setup

### Phase 4: Verification
16. Docker build and run with MSD-CPP project rules mounted
17. Verify all existing MSD-* rules are queryable with `language="cpp"`
18. Verify new Python/JS/HTML rules are queryable
19. Verify missing `language` param raises error on query tools

---

## Acceptance Criteria

- [ ] `search_guidelines("brace initialization", language="cpp")` returns C++ rules
- [ ] `search_guidelines("type hints", language="python")` returns Python typing rules
- [ ] `search_guidelines("brace initialization")` without `language` raises an error (required param)
- [ ] `get_rule("MSD-INIT-001")` works (project rules injected from volume mount)
- [ ] `get_rule("PEP8-E501")` returns PEP 8 line length rule
- [ ] `list_categories(language="javascript")` returns only JS categories
- [ ] Docker container starts, seeds DB, and serves MCP tools over SSE
- [ ] Project rules from `.guidelines/` are merged into DB at container startup
- [ ] `LANGUAGES=cpp` env var seeds only C++ rules (no Python/JS in DB)

---

## Open Questions

1. **SSE vs stdio**: Should the Docker container serve over SSE (HTTP) by default, or should consuming repos use `docker exec` for stdio? SSE is simpler for multi-repo but requires a running container.
2. **Versioning strategy**: Should consuming repos pin to a specific guidelines-server image tag, or track `latest`? Pinning is safer but adds maintenance.
3. **CI integration**: Should consuming repos run guidelines-server in CI for automated rule checking, or is this purely an AI-agent-time tool?
4. **Rule override/disable**: Should project-level YAML be able to override or disable a shared rule (e.g., disable `PEP8-E501` for a specific repo)?

---

## Follow-Up Work

- **0081a**: Populate Python PEP 8 + typing rules (initial set)
- **0081b**: Populate JavaScript ESLint + TypeScript rules (initial set)
- **0081c**: Populate HTML/CSS WCAG accessibility rules (initial set)
- **0081d**: Add rule override/disable mechanism for project-level YAML
- **0081e**: CI integration — run guidelines checks in GitHub Actions

---

## Workflow Log

### Draft Phase
- **Started**: 2026-02-26
- **Notes**: Extracted from discussion about guidelines server scalability. Key drivers: multi-repo sharing and multi-language support. Schema extension (language column) is designed to be backward-compatible with existing 0078 infrastructure.
