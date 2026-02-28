# Design: Extract Guidelines Server to Standalone Multi-Language Service (0081)

## Summary

Extract the in-tree guidelines MCP server (`scripts/guidelines/`) into a standalone repository (`guidelines-server`) that supports multiple programming languages (C++, Python, JavaScript, HTML/CSS) and serves shared coding guidelines to multiple consuming repositories via Docker. The service uses SQLite + FTS5 for retrieval, FastMCP for the tool interface, and a two-layer data model: shared rules ship with the Docker image, project-specific rules are injected via volume mount at startup.

---

## Architecture Overview

### System Context

```
┌─────────────────────────────────────┐
│         guidelines-server           │
│            (Docker)                 │
│                                     │
│  ┌───────────┐  ┌────────────────┐  │
│  │ Shared     │  │ Project Rules  │  │
│  │ YAML Data  │  │ (volume mount) │  │
│  │ cpp/       │  │ .guidelines/   │  │
│  │ python/    │  └───────┬────────┘  │
│  │ javascript/│          │           │
│  │ html_css/  │          │           │
│  └─────┬─────┘          │           │
│        │    ┌────────────┘           │
│        ▼    ▼                        │
│  ┌──────────────┐                   │
│  │   seeder.py  │                   │
│  │  (on start)  │                   │
│  └──────┬───────┘                   │
│         ▼                            │
│  ┌──────────────┐                   │
│  │ guidelines.db│                   │
│  │ SQLite+FTS5  │                   │
│  └──────┬───────┘                   │
│         ▼                            │
│  ┌──────────────┐                   │
│  │  server.py   │──── SSE :8080     │
│  │  (FastMCP)   │                   │
│  └──────────────┘                   │
└─────────────────────────────────────┘
         ▲              ▲
         │              │
    ┌────┴────┐   ┌─────┴─────┐
    │ MSD-CPP │   │ other-repo│
    │ agents  │   │ agents    │
    └─────────┘   └───────────┘
```

### Repository Structure

```
guidelines-server/
├── data/                              # Shared rules (ship with image)
│   ├── cpp/
│   │   ├── core_guidelines.yaml       # C++ Core Guidelines (R, C, ES sections)
│   │   └── misra.yaml                 # MISRA rules
│   ├── python/
│   │   ├── pep8.yaml                  # PEP 8 style rules
│   │   └── typing.yaml               # Type hint conventions
│   ├── javascript/
│   │   ├── eslint_recommended.yaml    # ESLint recommended rules
│   │   └── typescript.yaml            # TypeScript-specific rules
│   └── html_css/
│       └── accessibility.yaml         # WCAG / accessibility rules
│
├── server/
│   ├── __init__.py
│   ├── schema.py                      # SQLite schema (extended with language)
│   ├── seeder.py                      # YAML -> SQLite indexer
│   ├── server.py                      # FastMCP server (SSE + stdio)
│   └── entrypoint.py                  # Docker entrypoint: seed then serve
│
├── tests/
│   ├── test_schema.py                 # Schema creation tests
│   ├── test_seeder.py                 # Seeder validation tests
│   └── test_server.py                 # MCP tool integration tests
│
├── Dockerfile
├── docker-compose.yaml                # Reference compose for consumers
├── pyproject.toml
├── README.md
└── .github/
    └── workflows/
        └── ci.yaml                    # Build image, run tests, push to registry
```

### Consuming Repo Layout

```
MSD-CPP/
├── .guidelines/
│   └── project_rules.yaml             # MSD-INIT-001, MSD-RES-002, etc.
├── docker-compose.yaml                # (or add guidelines service to existing)
└── .mcp.json                          # Points to guidelines SSE endpoint
```

---

## Schema Design

### Extended `rules` Table

The key schema change is adding a `language` column to the `rules` table:

```sql
CREATE TABLE IF NOT EXISTS rules (
    rule_id            TEXT PRIMARY KEY,
    category_id        INTEGER NOT NULL REFERENCES categories(id),
    language           TEXT NOT NULL CHECK(language IN ('cpp', 'python', 'javascript', 'html_css')),
    source             TEXT NOT NULL CHECK(source IN (
                           'project',
                           'cpp_core_guidelines', 'misra', 'clang_tidy',
                           'pep8', 'python_typing',
                           'eslint', 'typescript',
                           'wcag'
                       )),
    severity           TEXT NOT NULL CHECK(severity IN ('required', 'recommended', 'advisory')),
    status             TEXT NOT NULL DEFAULT 'active'
                           CHECK(status IN ('proposed', 'active', 'deprecated')),
    title              TEXT NOT NULL,
    rationale          TEXT NOT NULL,
    enforcement_notes  TEXT NOT NULL,
    enforcement_check  TEXT,
    good_example       TEXT,
    bad_example        TEXT
)
```

All other tables (`categories`, `rule_cross_refs`, `tags`, `rule_tags`, `rules_fts`) remain structurally unchanged. The FTS5 virtual table continues to index title, rationale, enforcement_notes, and examples — language filtering happens at the SQL query level by joining back to `rules.language`.

### Category Scoping

Categories are **per-language**. A category named "Initialization" in C++ and "Initialization" in Python are the same row in `categories` — but queries filter by `rules.language`, so `list_categories(language="cpp")` only returns categories that have C++ rules. This avoids category table bloat while keeping cross-language category names consistent where they happen to overlap.

### YAML Schema Extension

Each YAML seed file adds a top-level `language` key:

```yaml
language: python          # NEW — required
rules:
  - rule_id: PEP8-E501
    category: Line Length
    source: pep8
    severity: recommended
    title: "Limit all lines to 79 characters"
    rationale: "..."
    enforcement_notes: "..."
    tags: [style, readability]
```

The seeder reads `language` from each file and applies it to all rules in that file. A file without `language` is rejected with a validation error.

---

## MCP Tool Interface

### `search_guidelines(query, language, source?, category?, severity?, limit?)`

**`language` is required.** Returns rules matching the FTS5 query for the specified language.

```python
@mcp.tool()
def search_guidelines(
    query: str,
    language: str,
    source: str | None = None,
    category: str | None = None,
    severity: str | None = None,
    limit: int = 20,
) -> str:
```

### `get_rule(rule_id)`

**No `language` parameter.** Rule IDs are globally unique; language is returned in the response.

```python
@mcp.tool()
def get_rule(rule_id: str) -> str:
```

Response now includes `"language": "cpp"` (or whichever language the rule belongs to).

### `list_categories(language)`

**`language` is required.** Returns categories that have at least one rule in the specified language.

```python
@mcp.tool()
def list_categories(language: str) -> str:
```

SQL change:
```sql
SELECT c.name AS category, COUNT(r.rule_id) AS total_rules, ...
FROM categories c
JOIN rules r ON r.category_id = c.id
WHERE r.language = ?
GROUP BY c.id, c.name
ORDER BY c.name
```

### `get_category(name, language, detailed?)`

**`language` is required.** Returns rules in a category filtered by language.

```python
@mcp.tool()
def get_category(name: str, language: str, detailed: bool = False) -> str:
```

### `get_rules_by_tag(tag, language)`

**`language` is required.** Returns tagged rules filtered by language.

```python
@mcp.tool()
def get_rules_by_tag(tag: str, language: str) -> str:
```

---

## Docker & Distribution

### Dockerfile

```dockerfile
FROM python:3.12-slim

WORKDIR /app
COPY pyproject.toml .
RUN pip install --no-cache-dir .

COPY data/ /app/data/
COPY server/ /app/server/

# DB is seeded at startup, stored in a tmpfs-friendly location
ENV DB_PATH=/app/guidelines.db
ENV LANGUAGES=cpp,python,javascript,html_css
ENV PROJECT_DATA_DIR=/app/project_data
ENV PORT=8080

EXPOSE 8080

ENTRYPOINT ["python", "-m", "server.entrypoint"]
```

### Entrypoint Logic

`server/entrypoint.py`:
1. Read `LANGUAGES` env var, parse to list
2. Seed DB from `data/{lang}/*.yaml` for each requested language
3. If `PROJECT_DATA_DIR` exists and contains YAML files, seed those too (project overlay)
4. Start FastMCP server on `PORT` with SSE transport

```python
def main():
    languages = os.environ.get("LANGUAGES", "cpp,python,javascript,html_css").split(",")
    db_path = os.environ.get("DB_PATH", "/app/guidelines.db")
    project_dir = os.environ.get("PROJECT_DATA_DIR", "/app/project_data")
    port = int(os.environ.get("PORT", "8080"))

    # Seed shared rules
    all_rules = []
    for lang in languages:
        lang_dir = Path(f"/app/data/{lang.strip()}")
        if lang_dir.exists():
            all_rules.extend(collect_rules_from_dir(lang_dir, language=lang.strip()))

    # Seed project rules (overlay)
    project_path = Path(project_dir)
    if project_path.exists():
        for yaml_file in sorted(project_path.glob("*.yaml")):
            all_rules.extend(load_project_rules(yaml_file))

    # Create and populate DB
    seed_database(db_path, all_rules)

    # Start MCP server
    mcp = create_mcp_server(db_path)
    mcp.run(transport="sse", port=port)
```

### Consumer docker-compose.yaml

```yaml
services:
  guidelines:
    image: guidelines-server:latest
    volumes:
      - ./.guidelines:/app/project_data:ro
    environment:
      - LANGUAGES=cpp,python
      - PORT=8080
    ports:
      - "8080:8080"
```

### Consumer .mcp.json

```json
{
  "mcpServers": {
    "guidelines": {
      "url": "http://localhost:8080/sse"
    }
  }
}
```

---

## Seeder Changes

### Two-Layer Data Model

The seeder handles two types of YAML files:

1. **Shared rules** (`data/{language}/*.yaml`) — ship with the Docker image, language is specified at the file level
2. **Project rules** (`project_data/*.yaml`) — mounted at runtime, each rule specifies its own language

### Project Rule YAML Format

Project rules include `language` per-rule since a project might have conventions spanning multiple languages:

```yaml
rules:
  - rule_id: MSD-INIT-001
    language: cpp                       # Required per-rule for project files
    category: Initialization
    source: project
    severity: required
    title: "Use NaN for uninitialized floating-point members"
    rationale: "..."
    enforcement_notes: "..."
    tags: [initialization, safety]
```

### Validation

The Pydantic `RuleModel` gains a `language` field:

```python
class RuleModel(BaseModel):
    rule_id: str
    language: str  # Required — 'cpp', 'python', 'javascript', 'html_css'
    category: str
    source: str
    # ... rest unchanged
```

The `rule_id` prefix validator is extended:

```python
VALID_PREFIXES = {
    "cpp": ("MSD-", "CPP-", "MISRA-", "TIDY-"),
    "python": ("PEP8-", "PY-"),
    "javascript": ("JS-", "TS-"),
    "html_css": ("WCAG-", "CSS-", "HTML-"),
}
# Plus any prefix for source='project' rules
```

---

## Data Migration

### From MSD-CPP In-Tree to Standalone

| Current Path | New Location | Notes |
|---|---|---|
| `scripts/guidelines/data/cpp_core_guidelines.yaml` | `guidelines-server/data/cpp/core_guidelines.yaml` | Add `language: cpp` header |
| `scripts/guidelines/data/misra_rules.yaml` | `guidelines-server/data/cpp/misra.yaml` | Add `language: cpp` header |
| `scripts/guidelines/data/project_rules.yaml` | `MSD-CPP/.guidelines/project_rules.yaml` | Add `language: cpp` per-rule |
| `scripts/guidelines/guidelines_schema.py` | `guidelines-server/server/schema.py` | Add `language` column |
| `scripts/guidelines/seed_guidelines.py` | `guidelines-server/server/seeder.py` | Two-layer loading |
| `scripts/guidelines/guidelines_server.py` | `guidelines-server/server/server.py` | Add `language` param |

### MSD-CPP Cleanup

After the guidelines-server repo is stood up and MSD-CPP is consuming it via Docker:

1. Remove `scripts/guidelines/` directory entirely
2. Remove `guidelines-seed` CMake target from `CMakeLists.txt`
3. Remove `debug-guidelines` preset from `CMakeUserPresets.json`
4. Remove guidelines server entry from `.mcp.json` (replaced with SSE URL)
5. Remove `pyyaml` from `python/requirements.txt` (if no other consumer)
6. Update `CLAUDE.md` Guidelines MCP Server section to document Docker setup

---

## Design Decisions

### DD-0081-001: Project rules stay in consuming repos
**Decision**: Project-specific rules (e.g., `MSD-INIT-001`) live in each consuming repo under `.guidelines/`, not in the shared guidelines-server repo.
**Rationale**: These rules reference project-specific patterns, ticket IDs, and class names. They belong under the project's version control. The server merges them at seed time via Docker volume mount.
**Alternatives considered**: (1) All rules in guidelines-server with repo-specific YAML directories — rejected because it couples every project's release cycle to the guidelines repo. (2) Git submodule for shared rules — rejected because it adds git complexity without clear benefit over Docker.

### DD-0081-002: Language as a column, not separate databases
**Decision**: Single `guidelines.db` with a `language` column on the `rules` table.
**Rationale**: Enables cross-language queries if ever needed (e.g., "initialization" conventions across all languages). Avoids running multiple server instances or managing multiple DB files. FTS5 index covers all languages; SQL filters narrow results.
**Alternatives considered**: Separate DB per language — rejected because it loses cross-cutting queries and requires multiple server instances or a routing layer.

### DD-0081-003: Docker with SSE as primary distribution
**Decision**: Docker image with SSE (HTTP) transport as the primary MCP connection method.
**Rationale**: (1) Multi-repo consumers don't need to clone the guidelines repo or manage Python dependencies. (2) SSE is simpler to configure in `.mcp.json` than stdio-over-docker-exec. (3) Docker provides hermetic, reproducible builds.
**Alternatives considered**: (1) pip package — viable but requires Python env management in each consuming repo. (2) stdio via `docker exec` — works but requires the container to be running and named correctly.

### DD-0081-004: LANGUAGES env var for selective seeding
**Decision**: The `LANGUAGES` environment variable controls which `data/{language}/` directories are seeded.
**Rationale**: A C++-only project shouldn't seed Python/JS/HTML rules. Selective seeding keeps the DB and FTS index focused, improving search relevance and reducing noise.

### DD-0081-005: `language` is required on query tools
**Decision**: All query tools (except `get_rule`) require a `language` parameter. There is no "search all languages" default.
**Rationale**: This is a new service with no existing consumers. Requiring `language` enforces intentional, focused queries and prevents cross-language noise in agent results. If cross-language search is needed later, a separate tool can be added.

---

## Testing Strategy

### Unit Tests (in guidelines-server repo)

| Test | Description |
|------|-------------|
| `test_schema.py` | Schema creation, language column exists, CHECK constraints enforce valid values |
| `test_seeder.py` | YAML loading with language field, project overlay merging, duplicate detection, validation errors on missing language |
| `test_server.py` | All 5 MCP tools with language filtering, get_rule returns language field, error on missing language param |

### Integration Tests

| Test | Description |
|------|-------------|
| Docker build | Image builds successfully |
| Docker seed + serve | Container starts, seeds DB, responds to SSE health check |
| Project overlay | Mount `.guidelines/` volume, verify project rules are queryable |
| LANGUAGES filter | Set `LANGUAGES=cpp`, verify Python rules are absent |

### Smoke Tests (in consuming repo)

After MSD-CPP switches to Docker-based guidelines:
- `search_guidelines("brace initialization", language="cpp")` returns MSD-INIT-002
- `get_rule("MSD-INIT-001")` returns project rule with `language: cpp`
- Agent prompts continue to work with updated tool signatures

---

## Open Questions (from ticket)

1. **Versioning strategy** — Pin image tag or track latest? Recommend: pin to semver tags, update via Dependabot-style PR.
2. **CI integration** — Run guidelines-server in CI for automated checks? Recommend: defer to follow-up 0081e.
3. **Rule override/disable** — Can project YAML disable a shared rule? Recommend: defer to follow-up 0081d. Initial version: project rules can only add, not override.
