# Ticket 0078b: C++ Core Guidelines Population

## Status
- [x] Draft
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Implementation
- [x] Implementation Complete
- [ ] Quality Gate Passed — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

**Current Phase**: Implementation Complete
**Type**: Tooling / Infrastructure
**Priority**: Medium
**Created**: 2026-02-26
**Generate Tutorial**: No
**Parent**: 0078_cpp_guidelines_mcp_server

---

## Summary

Populate the C++ Core Guidelines rules in the guidelines database, starting with the R (Resource Management), C (Classes and Class Hierarchies), and ES (Expressions and Statements) sections. Include `enforcement_check` mappings to clang-tidy checks where applicable.

---

## Problem

The guidelines database (0078) ships with a stub `cpp_core_guidelines.yaml` file. Without populated C++ Core Guidelines rules:

1. **No authoritative backing** — Project rules (MSD-*) lack cross-references to the standard they derive from
2. **Incomplete search results** — `search_guidelines` only returns project-specific rules, missing broader C++ best practices
3. **No clang-tidy mapping** — The `enforcement_check` column exists but is unpopulated, preventing future automated enforcement

---

## Solution

### Sections to Populate

| Section | Prefix | Approximate Rules | Priority |
|---------|--------|-------------------|----------|
| R (Resource Management) | CPP-R.* | ~15 rules | High — directly backs MSD-RES-* |
| C (Classes and Class Hierarchies) | CPP-C.* | ~20 rules | High — backs MSD-RES-001 (Rule of Five/Zero) |
| ES (Expressions and Statements) | CPP-ES.* | ~15 rules | Medium — backs MSD-INIT-002 (brace init) |

### Rule ID Convention

Per 0078 design: `CPP-{section}.{number}` (e.g., `CPP-R.11`, `CPP-C.21`, `CPP-ES.23`)

### Cross-References

Add `rule_cross_refs` entries linking project rules to their C++ Core Guidelines origins:

| Project Rule | C++ Core Guideline | Relationship |
|-------------|-------------------|--------------|
| MSD-RES-001 | CPP-C.21 | derived_from |
| MSD-RES-002 | CPP-R.11 | derived_from |
| MSD-INIT-002 | CPP-ES.23 | derived_from |

### Enforcement Check Mappings

Populate `enforcement_check` for rules that map to clang-tidy checks:

| Rule | clang-tidy Check |
|------|-----------------|
| CPP-C.21 | `cppcoreguidelines-special-member-functions` |
| CPP-R.11 | `cppcoreguidelines-owning-memory` |
| CPP-ES.23 | `modernize-use-default-member-init` |

---

## Implementation Steps

1. Research and extract rules from C++ Core Guidelines R section
2. Research and extract rules from C++ Core Guidelines C section
3. Research and extract rules from C++ Core Guidelines ES section
4. Add all rules to `scripts/guidelines/data/cpp_core_guidelines.yaml`
5. Add cross-reference entries linking MSD-* rules to CPP-* rules
6. Add `enforcement_check` mappings for rules with clang-tidy equivalents
7. Re-seed database and verify with CLI smoke tests

---

## Acceptance Criteria

- [x] R section rules populated in `cpp_core_guidelines.yaml` with rationale and examples
- [x] C section rules populated in `cpp_core_guidelines.yaml` with rationale and examples
- [x] ES section rules populated in `cpp_core_guidelines.yaml` with rationale and examples
- [x] Cross-references link MSD-RES-001 → CPP-C.21, MSD-RES-002 → CPP-R.11, MSD-INIT-002 → CPP-ES.23
- [x] `enforcement_check` populated for rules with clang-tidy equivalents
- [x] `search_guidelines("resource management")` returns both MSD-RES-* and CPP-R.* rules
- [x] `get_rule("CPP-R.11")` returns full rule with cross-refs to MSD-RES-002
- [x] Database is fully rebuildable from updated YAML seed files

---

## Dependencies

- **Requires**: 0078_cpp_guidelines_mcp_server (Complete)
- **Blocked By**: None (0078 is complete)

---

## Workflow Log

### Implementation Phase
- **Started**: 2026-02-26 00:00
- **Completed**: 2026-02-26 00:00
- **Branch**: 0078b-cpp-core-guidelines-population
- **PR**: N/A
- **Artifacts**:
  - `scripts/guidelines/data/cpp_core_guidelines.yaml` — 57 rules: R section (15), C section (16), ES section (26)
  - `scripts/guidelines/data/project_rules.yaml` — added cross-refs to CPP-C.21, CPP-R.11, CPP-ES.23 in MSD-RES-001, MSD-RES-002, MSD-INIT-002
- **Notes**: >
    Skipped design/review phases per ticket guidance — this is a data population task
    with no new Python or C++ code. Seeder validation confirms 67 total rules (57 CPP +
    10 project). All 8 acceptance criteria verified via CLI smoke tests. Rules include
    enforcement_check mappings: CPP-C.21 → cppcoreguidelines-special-member-functions,
    CPP-R.11 → cppcoreguidelines-owning-memory, CPP-ES.23 → modernize-use-default-member-init,
    CPP-ES.20 → cppcoreguidelines-init-variables, CPP-ES.46 → cppcoreguidelines-narrowing-conversions,
    CPP-ES.47 → modernize-use-nullptr, CPP-ES.30 → cppcoreguidelines-macro-usage,
    CPP-C.37 → cppcoreguidelines-virtual-class-destructor,
    CPP-NAME-001 (via MSD-NAME-001) → readability-identifier-naming.
