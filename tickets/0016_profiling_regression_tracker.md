# Feature Ticket: Profiling Regression Tracker

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [ ] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-10
- **Author**: Claude Code
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: scripts, profiling infrastructure

---

## Summary

Create a Python-based profiling regression tracker that compares Time Profiler results against golden baselines, detecting when functions consume meaningfully more CPU time after code changes. This mirrors the workflow established in ticket 0014 (benchmark metrics tracker) but applies to profiling data, enabling tracking of CPU hotspots across test executables over time.

## Motivation

As the project grows and new features are added, it's important to detect unintentional performance regressions at the function level. Currently, profiling results are generated but there's no automated way to:

1. Track baseline CPU usage per function over time
2. Detect when code changes cause specific functions to consume more CPU
3. Generate comparison reports for design/code review
4. Integrate with CI to catch profiling regressions before merge
5. Track profiling results across different test executables (not just benchmarks)

This feature provides visibility into function-level performance trends and prevents unintentional CPU usage increases.

## Requirements

### Functional Requirements

1. The system shall compare profiling JSON results against a committed golden baseline
2. The system shall flag regressions when a function's sample percentage exceeds a configurable relative threshold (default: 50% increase)
3. The system shall generate JSON comparison reports with detailed per-function metrics
4. The system shall provide color-coded console output matching existing script conventions
5. The system shall support setting current results as new baseline via CLI flag (`--set-baseline`)
6. The system shall return non-zero exit code on regressions for CI integration (`--strict` flag)
7. The system shall support multiple executable baselines (e.g., `msd_sim_test`, `msd_sim_bench`)
8. The system shall match functions by exact name for comparison
9. The system shall report new hotspots (functions not in baseline but now in top N)
10. The system shall report disappeared functions (previously in top N, now absent)
11. The system shall only track top N functions by sample percentage (default: 10, configurable via `--top`)
12. The system shall average the most recent M profile runs for stability (default: 5, configurable via `--runs`)

### Non-Functional Requirements
- **Performance**: Script execution under 2 seconds for typical result sets
- **Memory**: Minimal memory footprint (load JSON files sequentially)
- **Thread Safety**: N/A (single-threaded script)
- **Backward Compatibility**: Works with existing `parse-profile.py` JSON schema

## Constraints
- Python 3.9+ (use built-in generics: `dict`, `list` not `Dict`, `List`)
- No external dependencies (stdlib only: json, argparse, pathlib)
- Must follow existing project conventions (color scheme, directory structure)
- Must work with `--project-only` filtered profiles (only msd_* namespaces)

## Acceptance Criteria
- [ ] `scripts/compare-profiles.py` exists and is executable
- [ ] Running `./scripts/compare-profiles.py` compares latest profile against baseline
- [ ] Running `./scripts/compare-profiles.py --set-baseline` creates/updates baseline
- [ ] Running `./scripts/compare-profiles.py --strict` returns exit code 1 on regressions
- [ ] JSON comparison report generated in `profile_results/comparison_{timestamp}.json`
- [ ] Console output uses color coding matching `compare_benchmarks.py` (GREEN/YELLOW/RED)
- [ ] Baseline files stored in `profile_baselines/{executable}/baseline.json`
- [ ] `profile_baselines/` directory is NOT gitignored (baselines are committed)
- [ ] Supports comparison of different executables (msd_sim_test, msd_sim_bench, etc.)
- [ ] Detects new hotspots that weren't in baseline
- [ ] CLAUDE.md updated with usage documentation

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Baseline type**: Golden baseline file (manually committed, updated when performance changes are expected)
- **Default threshold**: 50% relative increase triggers warning (e.g., 10% → 15% is a 50% increase)
- **Top N functions**: Only track top 10 functions by sample percentage (configurable via `--top`)
- **Run averaging**: Average the top 5 most recent profile runs for stability
- **Primary metric**: Compare sample percentage (not absolute sample count, which varies with run duration)
- **Output format**: JSON comparison report + console output (both by default)
- **Function matching**: Match by exact function name (demangled C++ symbols)

### Things to Avoid
- External Python dependencies (no pip install required)
- Comparing absolute sample counts (varies with trace duration)
- Complex statistical analysis in v1 (keep it simple: percentage comparison)
- Modifying existing profiling infrastructure (additive changes only)

### Open Questions
1. **Threshold type**: Absolute percentage point difference (5pp) vs relative increase (50%)? **→ Relative increase (default 50%)**
2. **Minimum significance**: Ignore functions below X% in both baseline and current? **→ Retain top N functions only (default top 10)**
3. **Multiple runs**: Average multiple profile runs for stability? **→ Yes, average top 5 runs**

---

## References

### Related Code
- `scripts/compare_benchmarks.py` — Similar comparison tool for benchmarks (primary reference)
- `scripts/parse-profile.py` — Produces JSON profiles for comparison
- `profile_results/*.json` — Profile JSON output format

### Related Documentation
- `docs/designs/0014_benchmark_metrics_tracker/design.md` — Benchmark comparison design (similar pattern)
- `docs/designs/0015_profiling_trace_parser/design.md` — Profile parser design
- `CLAUDE.md` — Profiling section to update

### Related Tickets
- `tickets/0014_benchmark_metrics_tracker.md` — Similar tool for benchmark comparison (template)
- `tickets/0015_profiling_trace_parser.md` — Profile parser that generates input JSON

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-10 15:00
- **Completed**: 2026-01-10 15:07
- **Artifacts**:
  - `docs/designs/0016_profiling_regression_tracker/design.md`
  - `docs/designs/0016_profiling_regression_tracker/0016_profiling_regression_tracker.puml`
- **Notes**: Design follows compare_benchmarks.py architecture with adaptations for profiling data. Key additions: multi-run averaging algorithm, function matching by exact name, percentage-based comparison metric. All design decisions from ticket have been incorporated.

### Design Review Phase
- **Started**: 2026-01-10 15:15
- **Completed**: 2026-01-10 15:30
- **Status**: APPROVED
- **Reviewer Notes**: Design approved without revision. Excellent architectural fit with compare_benchmarks.py. Algorithm is feasible and well-specified. JSON schema compatibility verified against actual parse-profile.py output. All criteria passed. No prototyping required. Estimated implementation: 4-6 hours.

### Prototype Phase
- **Started**:
- **Completed**:
- **Prototypes**:
- **Artifacts**:
- **Notes**:

### Implementation Phase
- **Started**:
- **Completed**:
- **Files Created**:
- **Files Modified**:
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
