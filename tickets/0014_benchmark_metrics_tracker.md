# Feature Ticket: Benchmark Metrics Tracker

## Status
- [x] Draft
- [x] Ready for Design
- [x] Design Complete — Awaiting Review
- [x] Design Approved — Ready for Prototype
- [x] Prototype Complete — Awaiting Review
- [x] Ready for Implementation
- [x] Implementation Complete — Awaiting Review
- [x] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 2026-01-08
- **Author**: Claude Code
- **Priority**: Medium
- **Estimated Complexity**: Medium
- **Target Component(s)**: scripts, benchmarking infrastructure

---

## Summary

Create a Python-based benchmark metrics tracker that compares Google Benchmark results against a golden baseline, detecting performance regressions with configurable thresholds and generating JSON comparison reports. This enables tracking benchmark values as functionality evolves and flags warnings when performance degrades beyond acceptable limits.

## Motivation

As the project grows and new features are added, it's important to ensure performance doesn't regress. Currently, benchmark results are generated but there's no automated way to:
1. Track baseline performance metrics over time
2. Detect when code changes cause performance regressions
3. Generate comparison reports for design/code review
4. Integrate with CI to catch regressions before merge

This feature provides visibility into performance trends and prevents unintentional performance degradation.

## Requirements

### Functional Requirements
1. The system shall compare benchmark results against a committed golden baseline
2. The system shall flag regressions when benchmark times exceed a configurable threshold (default: 10%)
3. The system shall generate JSON comparison reports with detailed metrics
4. The system shall provide color-coded console output matching existing script conventions
5. The system shall support setting current results as new baseline via CLI flag
6. The system shall return non-zero exit code on regressions for CI integration (with --strict flag)
7. The system shall skip aggregate rows (BigO, RMS) from comparison while including them in reports
8. The system shall match parameterized benchmarks by exact name (e.g., `BM_ConvexHull_Construction/8`)


### Non-Functional Requirements
- **Performance**: Script execution under 1 second for typical result sets
- **Memory**: Minimal memory footprint (load JSON files sequentially if needed)
- **Thread Safety**: N/A (single-threaded script)
- **Backward Compatibility**: Works with existing Google Benchmark JSON schema v1

## Constraints
- Python 3.9+ (use built-in generics: `dict`, `list` not `Dict`, `List`)
- No external dependencies (stdlib only: json, argparse, pathlib, subprocess)
- Must follow existing project conventions (color scheme, directory structure)

## Acceptance Criteria
- [ ] `scripts/compare_benchmarks.py` exists and is executable
- [ ] Running `./scripts/compare_benchmarks.py` compares latest results against baseline
- [ ] Running `./scripts/compare_benchmarks.py --set-baseline` creates/updates baseline
- [ ] Running `./scripts/compare_benchmarks.py --strict` returns exit code 1 on regressions
- [ ] JSON comparison report generated in `benchmark_results/{suite}/comparison_{timestamp}.json`
- [ ] Console output uses color coding matching `run_benchmarks.sh` (GREEN/YELLOW/RED)
- [ ] Baseline files stored in `benchmark_baselines/{suite}/baseline.json`
- [ ] `benchmark_baselines/` directory is NOT gitignored (baselines are committed)
- [ ] CLAUDE.md updated with usage documentation

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Baseline type**: Golden baseline file (manually committed, updated when performance changes are expected)
- **Default threshold**: 10% regression triggers warning
- **Primary metric**: Compare `cpu_time` (less affected by system load than `real_time`)
- **Output format**: JSON comparison report + console output (both by default)

### Things to Avoid
- External Python dependencies (no pip install required)
- Modifying existing benchmark infrastructure (additive changes only)
- Complex statistical analysis in v1 (keep it simple: percentage comparison)

### Open Questions
- None (clarified during planning)

---

## References

### Related Code
- `scripts/run_benchmarks.sh` — Existing benchmark runner, follow conventions
- `benchmark_results/msd_sim_bench/benchmark_latest.json` — Example result format
- `msd/msd-sim/bench/ConvexHullBench.cpp` — Current benchmark implementations

### Related Documentation
- `docs/designs/0011_add_google_benchmark/design.md` — Benchmark infrastructure design
- `CLAUDE.md` — Benchmarking section to update

### Related Tickets
- `tickets/0011_add_google_benchmark.md` — Parent ticket for benchmark infrastructure

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Artifacts**:
  - `/Users/danielnewman/Documents/GitHub/MSD-CPP/docs/designs/0014_benchmark_metrics_tracker/design.md`
- **Notes**: Comprehensive design document created for Python benchmark comparison utility. Includes data flow diagrams, JSON schemas, CLI interface specification, and integration points with existing benchmark infrastructure.

### Design Review Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Status**: APPROVED
- **Reviewer Notes**: Design approved without revision. No prototypes required. All criteria pass: proper integration with existing infrastructure, comprehensive error handling, clear CLI design, and well-defined data structures. Ready for direct implementation. See full review appended to design document.

### Prototype Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Prototypes**: None required (design review determined no high-risk areas needing validation)
- **Artifacts**: N/A
- **Notes**: Design reviewer assessed all risks as low-likelihood/low-impact with clear mitigation strategies. Implementation is straightforward with well-understood technologies (Python stdlib, JSON parsing). Skipping directly to implementation phase.

### Implementation Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Files Created**:
  - `/Users/danielnewman/Documents/GitHub/MSD-CPP/scripts/compare_benchmarks.py` — Python benchmark comparison script with CLI interface
  - `/Users/danielnewman/Documents/GitHub/MSD-CPP/benchmark_baselines/msd_sim_bench/baseline.json` — Initial golden baseline from existing ConvexHull benchmarks
- **Files Modified**:
  - `/Users/danielnewman/Documents/GitHub/MSD-CPP/CLAUDE.md` — Added "Benchmark Regression Detection" section and "Recent Architectural Changes" entry
- **Notes**: Implementation completed successfully. All acceptance criteria met. The script provides comprehensive benchmark comparison with color-coded output, JSON reports, and flexible CLI options. Tested with existing ConvexHull benchmarks showing correct comparison (all PASS with 0% difference when comparing against itself). The .gitignore already correctly excluded benchmark_results/ while allowing benchmark_baselines/ to be committed.

### Implementation Review Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Status**: APPROVED
- **Reviewer Notes**: Implementation fully conforms to design specification with no deviations. All acceptance criteria met. Code quality excellent with proper error handling, type hints, and comprehensive documentation. Manual testing verified all functionality. Three minor stylistic suggestions noted but no action required. Feature is production-ready and approved for merge. See full review: `/Users/danielnewman/Documents/GitHub/MSD-CPP/docs/designs/0014_benchmark_metrics_tracker/implementation-review.md`

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
