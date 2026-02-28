# Feature Ticket: Extract Workflow Engine to Standalone Repository

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
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: scripts/workflow, workflow-engine (new repo)
- **Languages**: Python
- **Generate Tutorial**: No
- **Requires Math Design**: No
- **GitHub Issue**:

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
- [ ] `workflow-engine` repo exists on GitHub with all engine code
- [ ] `pip install workflow-engine` (from repo) works in a clean venv
- [ ] `workflow-engine gates` CLI entry point works after pip install
- [ ] `python -m workflow_engine.server` starts the MCP server
- [ ] Docker image builds and serves MCP tools
- [ ] MSD-CPP `python/requirements.txt` includes `workflow-engine` dependency
- [ ] MSD-CPP `.mcp.json` uses `-m workflow_engine.server` (not `scripts/workflow/server/server.py`)
- [ ] `scripts/workflow/` is removed from MSD-CPP
- [ ] All 140 tests pass in the standalone repo
- [ ] MSD-CPP workflow functionality works end-to-end with the extracted package

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
1. **GitHub organization** — Should the repo be under the same GitHub account or a separate org?
2. **Package registry** — Publish to PyPI or install from GitHub URL only?
   - **Recommendation**: GitHub URL for now (`pip install git+https://...`), PyPI later if needed

---

## References

### Related Code
- `scripts/workflow/` — Current in-tree implementation to extract
- `.workflow/phases.yaml` — Consumer config (stays in MSD-CPP)
- `.workflow/config.yaml` — Consumer config (stays in MSD-CPP)
- `.mcp.json` — Must be updated to reference installed package

### Related Tickets
- `tickets/0083_database_agent_orchestration.md` — Parent ticket (implementation)
- `tickets/0081_guidelines_server_extraction.md` — Same extraction pattern

---

## Workflow Log

### Design Phase
- **Started**:
- **Completed**:
- **Notes**: Design is minimal — the architecture is already specified in 0083's design.md. This ticket primarily needs an implementation plan for the file moves and packaging.

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
- **Notes**:

---

## Human Feedback

### Feedback on Design
{Your comments on the design}

### Feedback on Implementation
{Your comments on the implementation}
