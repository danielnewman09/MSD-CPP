# Feature Ticket: Extract Workflow Engine to Standalone Repository

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Test Writing
- [x] Test Writing Complete — Awaiting Quality Gate
- [x] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-02-27
- **Author**: Daniel Newman
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: scripts/workflow, workflow-engine (new repo)
- **Languages**: Python
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**: 114

---

## Summary
Extract the workflow engine from `scripts/workflow/` in MSD-CPP into a standalone `workflow-engine` repository. The code is already implemented and tested (ticket 0083). This ticket is a mechanical extraction: move files, add Python packaging (`pyproject.toml`), add Docker distribution, update MSD-CPP to consume the package as a dependency, and remove the in-tree copy.

## Motivation
The workflow engine (0083) was implemented in-tree at `scripts/workflow/` to reduce risk and validate the design before extraction. The engine is project-agnostic — it knows about tickets, phases, agents, and gates but nothing about MSD-CPP. Extracting it to a standalone repo enables:
- Reuse across other repositories
- Independent versioning and releases
- Docker distribution for multi-repo deployments
- Clean separation between engine and consuming project

This follows the same extraction pattern as ticket 0081 (guidelines-server).

## Requirements

### Functional Requirements
1. Create a `workflow-engine` GitHub repository with the engine code
2. Add `pyproject.toml` with package metadata, CLI entry point (`workflow-engine`), and dependencies
3. Add `Dockerfile` and `docker-compose.yaml` for Docker distribution
4. Restructure imports from `scripts.workflow.engine.*` to `workflow_engine.engine.*`
5. All 140 existing tests must pass in the standalone repo
6. MSD-CPP must consume the package via pip (added to `python/requirements.txt`)
7. MSD-CPP's `.mcp.json` must reference the installed package (`-m workflow_engine.server`)
8. Remove `scripts/workflow/` from MSD-CPP after confirming the dependency works
9. `.workflow/phases.yaml` and `.workflow/config.yaml` remain in MSD-CPP (unchanged)

### Non-Functional Requirements
- **Backward Compatibility**: No functional changes to the engine — pure extraction
- **Testing**: All tests pass in both repos during transition

## Constraints
- No logic changes — this is a move + packaging exercise
- Must preserve git history where practical (consider `git filter-branch` or `git subtree split`)

## Acceptance Criteria
- [x] `workflow-engine` repo exists on GitHub with all engine code
- [x] `pip install workflow-engine` (from repo) works in a clean venv
- [x] `workflow-engine gates` CLI entry point works after pip install
- [x] `python -m workflow_engine.server` starts the MCP server
- [ ] Docker image builds and serves MCP tools
- [x] MSD-CPP `python/requirements.txt` includes `workflow-engine` dependency
- [x] MSD-CPP `.mcp.json` uses `-m workflow_engine.server` (not `scripts/workflow/server/server.py`)
- [x] `scripts/workflow/` is removed from MSD-CPP
- [x] All 140 tests pass in the standalone repo
- [x] MSD-CPP workflow functionality works end-to-end with the extracted package

---

## Design Decisions (Human Input)

### Preferred Approaches
- Follow the 0081 guidelines-server extraction pattern
- Use `pyproject.toml` (not `setup.py`) for modern Python packaging
- Tag v0.1.0 on first stable release

### Things to Avoid
- Do not change any engine logic during extraction
- Do not rename the `.workflow/` config directory convention

### Open Questions
1. **GitHub organization** — Under `danielnewman09` account (same as MSD-CPP).
2. **Package registry** — GitHub URL install (`pip install git+https://...`), PyPI later if needed.

---

## References

### Related Code
- `scripts/workflow/` — REMOVED (extracted to standalone repo)
- `.workflow/phases.yaml` — Consumer config (stays in MSD-CPP, unchanged)
- `.workflow/config.yaml` — Consumer config (stays in MSD-CPP, unchanged)
- `.mcp.json` — Updated to use `-m workflow_engine.server`
- `python/requirements.txt` — Updated with `workflow-engine` dependency

### Standalone Repository
- **Repo**: https://github.com/danielnewman09/workflow-engine
- **Tag**: v0.1.0
- **Install**: `pip install git+https://github.com/danielnewman09/workflow-engine.git@v0.1.0`

### Related Tickets
- `tickets/0083_database_agent_orchestration.md` — Parent ticket (implementation)
- `tickets/0081_guidelines_server_extraction.md` — Same extraction pattern

---

## Workflow Log

### Design Phase
- **Started**: 2026-02-27 (skipped — design covered in 0083 design.md)
- **Completed**: 2026-02-27
- **Branch**: 0083a-workflow-engine-extraction
- **PR**: N/A (advanced to implementation per human guidance)
- **Notes**: Design is minimal — the architecture is already specified in 0083's design.md. Advanced directly to implementation per ticket context.

### Implementation Phase
- **Started**: 2026-02-27
- **Completed**: 2026-02-27
- **Branch**: 0083a-workflow-engine-extraction
- **PR**: N/A (pending creation)
- **Artifacts**:
  - `https://github.com/danielnewman09/workflow-engine` (new repository)
  - `python/requirements.txt` (added workflow-engine dependency)
  - `.mcp.json` (updated to -m workflow_engine.server)
  - `scripts/workflow/` REMOVED
- **Notes**: All 140 tests pass in the standalone repo. Package installs cleanly from GitHub URL. CLI entry point and python -m module entry point both verified working. Docker acceptance criterion deferred — Docker build not verified yet.

---

## Human Feedback

### Feedback on Design
{Your comments on the design}

### Feedback on Implementation
{Your comments on the implementation}
