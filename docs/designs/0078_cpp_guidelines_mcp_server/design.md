# Design: C++ Guidelines MCP Server (0078)

## Summary

This design adds a FastMCP server backed by a SQLite + FTS5 database that stores structured C++ coding guidelines from three sources: project-specific MSD conventions, the C++ Core Guidelines, and MISRA rules. AI agents (architect, reviewer, implementer) query the server during design and code review to retrieve rationale-backed recommendations with traceable rule IDs. This is a lightweight RAG (Retrieval-Augmented Generation) system where FTS5 handles retrieval, MCP handles augmentation, and the AI handles generation.

---

## Architecture Changes

### PlantUML Diagram

See: `./0078_cpp_guidelines_mcp_server.puml`

---

### New Components

#### `scripts/guidelines/guidelines_schema.py`

- **Purpose**: Defines the SQLite schema as Python constants (CREATE TABLE DDL + FTS5 virtual table). Imported by both the seed script and the server so schema is the single source of truth.
- **File location**: `scripts/guidelines/guidelines_schema.py`
- **Key interface**:
  ```python
  SCHEMA_STATEMENTS: list[str]  # All DDL statements in dependency order
  create_schema(conn: sqlite3.Connection) -> None
  ```
- **Tables defined**:
  | Table | Purpose |
  |-------|---------|
  | `categories` | Broad groupings (Memory Management, Initialization, etc.) |
  | `rules` | Individual guidelines with rule_id, source, severity, status, title, rationale, enforcement_notes, enforcement_check, good_example, bad_example |
  | `rule_cross_refs` | Relationships: derived_from, related, supersedes, conflicts_with |
  | `tags` | Cross-cutting concern labels (memory, ownership, safety, performance) |
  | `rule_tags` | Many-to-many join between rules and tags |
  | `rules_fts` | FTS5 virtual table over title + rationale + enforcement_notes + examples; porter tokenizer |

- **Schema detail — `rules` table columns**:
  | Column | Type | Notes |
  |--------|------|-------|
  | `rule_id` | TEXT PRIMARY KEY | e.g. MSD-INIT-001 |
  | `category_id` | INTEGER FK | References `categories` |
  | `source` | TEXT | `project`, `cpp_core_guidelines`, `misra` |
  | `severity` | TEXT | `required`, `recommended`, `advisory` |
  | `status` | TEXT | `proposed`, `active`, `deprecated` |
  | `title` | TEXT | One-line summary |
  | `rationale` | TEXT | Why the rule exists |
  | `enforcement_notes` | TEXT | How to detect violations |
  | `enforcement_check` | TEXT NULL | clang-tidy / cppcheck ID if applicable |
  | `good_example` | TEXT NULL | Code snippet showing correct usage |
  | `bad_example` | TEXT NULL | Code snippet showing violation |

- **Design decision — status column (DD-0078-002)**: Rules have lifecycles (proposed → active → deprecated). Without status, deprecated rules would still be returned by search. Agents distinguish current rules from historical ones by filtering on `status = 'active'`.
- **Design decision — enforcement_check column (DD-0078-003)**: Maps rules to clang-tidy or cppcheck IDs (e.g., `cppcoreguidelines-special-member-functions`). Enables future tooling: given a linter warning, look up the project rule and rationale. Initially nullable — populated incrementally.

#### `scripts/guidelines/seed_guidelines.py`

- **Purpose**: CLI tool that reads YAML seed files, validates them with Pydantic models, and populates `guidelines.db`. Fully idempotent (DROP + CREATE on each run). Serves as the single pipeline from data sources to database.
- **File location**: `scripts/guidelines/seed_guidelines.py`
- **Key interface**:
  ```python
  # CLI usage
  python seed_guidelines.py [--db PATH] [--data-dir PATH]

  # Pydantic models for validation
  class RuleModel(BaseModel):
      rule_id: str
      category: str
      source: Literal["project", "cpp_core_guidelines", "misra"]
      severity: Literal["required", "recommended", "advisory"]
      status: Literal["proposed", "active", "deprecated"] = "active"
      title: str
      rationale: str
      enforcement_notes: str
      enforcement_check: str | None = None
      good_example: str | None = None
      bad_example: str | None = None
      tags: list[str] = []
      cross_refs: list[CrossRefModel] = []
  ```
- **Dependencies**: `pyyaml`, `pydantic` (already in `python/requirements.txt`)
- **Error handling**: Raises `ValidationError` on bad YAML; aborts before writing if validation fails (atomic: all-or-nothing)

#### `scripts/guidelines/guidelines_server.py`

- **Purpose**: FastMCP server that wraps `GuidelinesServer` class with 5 MCP tools and a CLI mode for smoke testing. Follows identical structural pattern to `scripts/mcp_codebase_server.py` (class + `create_mcp_server` factory function).
- **File location**: `scripts/guidelines/guidelines_server.py`
- **Key interface**:
  ```python
  class GuidelinesServer:
      def __init__(self, db_path: str): ...
      def search_guidelines(self, query: str, source: str | None = None,
                            category: str | None = None,
                            severity: str | None = None,
                            limit: int = 20) -> list[dict]: ...
      def get_rule(self, rule_id: str) -> dict: ...
      def list_categories(self) -> list[dict]: ...
      def get_category(self, name: str, detailed: bool = False) -> dict: ...
      def get_rules_by_tag(self, tag: str) -> list[dict]: ...

  def create_mcp_server(db_path: str) -> FastMCP: ...
  ```
- **MCP tools (5)**:
  | Tool | Parameters | Returns |
  |------|-----------|---------|
  | `search_guidelines` | `query, source?, category?, severity?, limit?=20` | Matching rules via FTS5 with BM25 ranking |
  | `get_rule` | `rule_id` | Full rule: category, tags, cross_refs, status, examples |
  | `list_categories` | none | All categories with rule counts |
  | `get_category` | `name, detailed?=false` | `detailed=false`: rule_id+title+severity+status only; `detailed=true`: full rationale+examples |
  | `get_rules_by_tag` | `tag` | All rules with the given tag |

- **Design decision — merged get_related_rules (DD-0078-001)**: `get_rule` already returns cross-refs and tags. A separate `get_related_rules` tool adds a round-trip for information available in one call. Replaced by `get_rules_by_tag` which covers a query axis no other tool addresses.
- **Design decision — get_category default mode (DD-0078-005)**: CppCoreGuidelines categories can contain 100+ rules. Returning full rationale + examples for all would blow the AI context window. Default is summary mode (`{rule_id, title, severity, status}`); `detailed=true` includes everything.
- **FTS5 configuration**: `porter` tokenizer for stemming (e.g., "initialize" matches "initialization"). BM25 ranking via `bm25(rules_fts)`.

#### `scripts/guidelines/data/project_rules.yaml`

- **Purpose**: Seed data encoding the 10 project-specific rules currently in `CLAUDE.md` in structured form.
- **File location**: `scripts/guidelines/data/project_rules.yaml`
- **Initial rules**:
  | Rule ID | Category | Title |
  |---------|----------|-------|
  | MSD-INIT-001 | Initialization | Use NaN for uninitialized floating-point members |
  | MSD-INIT-002 | Initialization | Always use brace initialization |
  | MSD-RES-001 | Resource Management | All-or-Nothing Rule (Rule of Five/Zero) |
  | MSD-RES-002 | Resource Management | unique_ptr for ownership, references for non-owning |
  | MSD-RES-003 | Resource Management | Optional reference wrapper only for truly optional lookups |
  | MSD-NAME-001 | Naming Conventions | Follow project naming conventions |
  | MSD-NAME-002 | Naming Conventions | No misleading `cached` prefix |
  | MSD-FUNC-001 | Function Design | Prefer return values over output parameters |
  | MSD-ORG-001 | Code Organization | One class per header |
  | MSD-DOC-001 | Documentation | Public APIs require Doxygen comments |

#### `scripts/guidelines/data/cpp_core_guidelines.yaml`

- **Purpose**: Stub file for incremental population of C++ Core Guidelines rules (R, C, ES sections in follow-up 0078b).
- **File location**: `scripts/guidelines/data/cpp_core_guidelines.yaml`
- **Initial state**: Empty stub with correct YAML structure and schema comment.

#### `scripts/guidelines/data/misra_rules.yaml`

- **Purpose**: Stub file for incremental population of MISRA C++ rules (memory and initialization categories in follow-up 0078c).
- **File location**: `scripts/guidelines/data/misra_rules.yaml`
- **Initial state**: Empty stub with correct YAML structure and schema comment.

---

### Modified Components

#### `.mcp.json`

- **Current location**: `/.mcp.json`
- **Changes required**: Add `guidelines` server entry pointing to `guidelines_server.py` with the database path `build/Debug/docs/guidelines.db`.
- **Backward compatibility**: Additive change; existing server entries untouched.

#### `.claude/settings.local.json`

- **Current location**: `/.claude/settings.local.json`
- **Changes required**: Add `"guidelines"` to `enabledMcpjsonServers` list. Add tool permissions for `mcp__guidelines__*` tools.
- **Backward compatibility**: Additive change only.

#### `CMakeLists.txt`

- **Current location**: `/CMakeLists.txt`
- **Changes required**: Add `guidelines-seed` custom target that invokes `seed_guidelines.py` to populate `build/Debug/docs/guidelines.db`. Follows pattern of `traceability` targets.
- **Backward compatibility**: New target only; no changes to existing targets.

#### `CMakeUserPresets.json`

- **Current location**: `/CMakeUserPresets.json`
- **Changes required**: Add `debug-guidelines` build preset targeting `guidelines-seed`.
- **Backward compatibility**: Additive only.

#### `python/requirements.txt`

- **Current location**: `/python/requirements.txt`
- **Changes required**: Add `pyyaml` (version pinned). `pydantic` is already present.
- **Backward compatibility**: Pure addition; existing entries untouched.

---

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|--------------------|------------------|-------|
| `guidelines_server.py` | `mcp_codebase_server.py` | Pattern reuse | Same class + factory structure |
| `guidelines_server.py` | `.mcp.json` | Registration | New server entry |
| `seed_guidelines.py` | `CMakeLists.txt` | Build target | `guidelines-seed` custom target |
| `guidelines_schema.py` | `seed_guidelines.py` | Module import | Schema DDL shared |
| `guidelines_schema.py` | `guidelines_server.py` | Module import | Schema constants for queries |
| `python/requirements.txt` | Python venv | Dependency | `pyyaml` added |

---

## Data Flow

```
YAML seed files
    │
    ▼  (pydantic validation)
seed_guidelines.py
    │
    ▼  (INSERT via guidelines_schema.py DDL)
guidelines.db (SQLite + FTS5)
    │
    ▼  (SQL queries via GuidelinesServer)
guidelines_server.py (FastMCP)
    │
    ▼  (MCP stdio transport)
AI agents (architect, reviewer, implementer)
```

---

## Database Location

The database is stored at `build/{build_type}/docs/guidelines.db` (parallel to `codebase.db` and `traceability.db`). It is gitignored and fully rebuildable from YAML seed files.

---

## Test Impact

### Existing Tests Affected

None — this is a new Python module with no C++ changes and no modifications to existing Python infrastructure.

### New Tests Required

#### CLI Smoke Tests (Phase 4 verification — not automated test files)

| Command | Validates |
|---------|-----------|
| `python seed_guidelines.py` | Database created, 10 rules inserted |
| `python guidelines_server.py guidelines.db search_guidelines "brace initialization"` | Returns MSD-INIT-002 |
| `python guidelines_server.py guidelines.db get_rule MSD-RES-001` | Full rule with tags and cross-refs |
| `python guidelines_server.py guidelines.db list_categories` | All categories with rule counts |
| `python guidelines_server.py guidelines.db get_category Initialization` | Summary mode (rule_id + title) |
| `python guidelines_server.py guidelines.db get_category Initialization --detailed` | Full rationale and examples |
| `python guidelines_server.py guidelines.db get_rules_by_tag safety` | All rules tagged "safety" |

#### FTS Stemming Verification

| Query | Expected Match |
|-------|---------------|
| `"initialize"` | MSD-INIT-001, MSD-INIT-002 (porter stemmer: initialize ≈ initialization) |
| `"ownership"` | MSD-RES-002, MSD-RES-003 |
| `"pointer"` | MSD-RES-002 (rationale mentions raw pointers) |

---

## Open Questions

### Design Decisions (Resolved — documented for traceability)

All design decisions were resolved upfront in the ticket. No open questions blocking implementation.

1. **DD-0078-001**: `get_related_rules` merged into `get_rule` (returns cross-refs + tags in one call). Replaced by `get_rules_by_tag` for tag-axis queries.
2. **DD-0078-002**: `status` column added (proposed/active/deprecated lifecycle).
3. **DD-0078-003**: `enforcement_check` column added (nullable, for clang-tidy/cppcheck IDs).
4. **DD-0078-004**: CLAUDE.md migration tracked as follow-up 0078d.
5. **DD-0078-005**: `get_category` defaults to summary mode to avoid context window overflow.

### Prototype Required

None — all components are straightforward Python with no novel algorithms. The FTS5 + porter stemmer is a SQLite built-in; no prototype validation needed.

### Requirements Clarification

None — requirements and design are fully specified in the ticket.

---

## Implementation Order

The 4 phases from the ticket map directly to implementation:

1. **Phase 1**: `guidelines_schema.py` → `seed_guidelines.py` → YAML data files → `python/requirements.txt`
2. **Phase 2**: `guidelines_server.py` with 5 tools + CLI mode
3. **Phase 3**: `.mcp.json` → `.claude/settings.local.json` → `CMakeLists.txt` → `CMakeUserPresets.json`
4. **Phase 4**: Seed database, run CLI smoke tests, restart session and verify MCP tools

---

## Follow-Up Work

- **0078a**: Update agent prompts to reference guidelines MCP tools and add directive: "Only cite rules returned by `search_guidelines`. Do not invent rule IDs."
- **0078b**: Populate C++ Core Guidelines rules (R, C, ES sections) with enforcement_check clang-tidy mappings.
- **0078c**: Populate MISRA rules (memory and initialization categories).
- **0078d**: Migrate CLAUDE.md coding standards section to a pointer — replace prose with directive to query the guidelines MCP server (DD-0078-004).
