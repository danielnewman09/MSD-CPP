# Feature Ticket: Profiling Trace Parser

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
- **Target Component(s)**: scripts, profiling infrastructure

---

## Summary

Extend the macOS profiling infrastructure (ticket 0012) with XML export capability and a Python parser that extracts Time Profiler data into JSON format. This enables programmatic analysis of profiling results for any executable—tests, benchmarks, or applications—without requiring manual inspection in the Instruments GUI.

## Motivation

The current profiling workflow (ticket 0012) generates `.trace` files that must be opened manually in Xcode Instruments. While useful for interactive analysis, this approach has limitations:

1. **No automation**: Cannot programmatically extract hotspots or function timings
2. **No CI integration**: Cannot track profiling results over time or detect performance regressions
3. **Manual comparison**: Comparing profiles before/after code changes requires manual side-by-side inspection
4. **Test profiling**: Profiling test executables produces the same opaque `.trace` files with no quick summary

This feature provides structured JSON output from profiling sessions, enabling:
- Quick console review of top functions by CPU time
- Machine-readable data for future CI integration
- Profiling any executable (tests, benchmarks, main application)
- Foundation for future profile comparison tooling

## Requirements

### Functional Requirements

1. **XML Export Enhancement**: Modify `profile-instruments.sh` to optionally export trace data to XML after recording
   - Add `--export-xml` / `-x` flag to enable automatic XML export
   - Use `xctrace export --toc` to discover available data tables
   - Use `xctrace export --xpath` to extract Time Profiler sample data
   - Save XML file alongside the `.trace` file (same directory, same base name)

2. **Parser Script**: Create `scripts/parse-profile.py` Python script
   - Accept `.trace` file as input (will internally call xctrace export)
   - Parse Time Profiler XML schema to extract:
     - Total sample count
     - Per-function sample weights
     - Function names (demangled C++ symbols)
     - Source file and line attribution (when available)
   - Output JSON with configurable top N functions (default: 20)
   - Support direct XML input (skip xctrace export if XML already exists)

3. **JSON Output Schema**:
   ```json
   {
     "metadata": {
       "trace_file": "profile_20260108_181837.trace",
       "template": "Time Profiler",
       "export_timestamp": "2026-01-08T18:20:00Z",
       "executable": "/path/to/msd_sim_test"
     },
     "summary": {
       "total_samples": 12345,
       "total_time_ms": 5000.0
     },
     "top_functions": [
       {
         "rank": 1,
         "name": "msd_sim::ConvexHull::computeHull<msd_sim::Coordinate>",
         "samples": 4567,
         "percentage": 37.0,
         "source_file": "ConvexHull.cpp",
         "line": 142
       }
     ]
   }
   ```

4. **Console Output**: Provide human-readable summary when not using `--json-only`
   - Table of top functions with sample counts and percentages
   - Color-coded output matching existing script conventions

### Non-Functional Requirements
- **Performance**: Parser execution under 2 seconds for typical trace files
- **Memory**: Stream XML parsing if files exceed 100MB
- **Dependencies**: Python stdlib only (xml.etree.ElementTree, json, argparse, subprocess, pathlib)
- **Compatibility**: macOS 12.0+ (same as ticket 0012)
- **Thread Safety**: N/A (single-threaded script)

## Constraints
- Python 3.9+ (use built-in generics: `dict`, `list` not `Dict`, `List`)
- No external Python dependencies (stdlib only)
- Must follow existing project conventions (color scheme, argument parsing style)
- Time Profiler template only (Allocations and other templates out of scope for v1)
- XML schema may vary between Xcode versions—design for graceful degradation

## Acceptance Criteria
- [ ] `profile-instruments.sh` supports `--export-xml` / `-x` flag
- [ ] XML file generated alongside `.trace` when `--export-xml` is used
- [ ] `scripts/parse-profile.py` exists and is executable
- [ ] `parse-profile.py` accepts `.trace` file and outputs JSON to stdout
- [ ] `parse-profile.py --output report.json` writes JSON to specified file
- [ ] `parse-profile.py --top 10` limits output to top 10 functions
- [ ] JSON output includes top functions by sample count with percentages
- [ ] Works with test executables (e.g., `msd_sim_test`)
- [ ] Works with benchmark executables (e.g., `msd_sim_bench`)
- [ ] Graceful error handling for missing/invalid trace files
- [ ] CLAUDE.md updated with parser documentation and examples
- [ ] JSON reports go to `profile_results/` 

---

## Design Decisions (Human Input)

### Preferred Approaches
- **Output format**: JSON (machine-readable, matches benchmark infrastructure)
- **Template support**: Time Profiler only (CPU profiling)
- **Integration**: Standalone tool (no CI integration in v1)
- **XML handling**: Parser calls xctrace internally; user doesn't need to manage XML files

### Things to Avoid
- External Python dependencies
- Complex call tree analysis in v1 (focus on flat function list)
- Modifying existing profiling behavior (additive changes only)
- Allocations/Leaks template support (defer to future ticket)

### Open Questions
2. **Call tree**: Include hierarchical call tree in v1, or defer to future enhancement? Defer
3. **Comparison tool**: Should this ticket include basic profile comparison, or defer? Defer

---

## References

### Related Code
- `scripts/profile-instruments.sh` — Existing profiling script to enhance
- `scripts/compare_benchmarks.py` — Reference for Python script conventions
- `scripts/run_benchmarks.sh` — Reference for bash script conventions

### Related Documentation
- `docs/designs/0012_add_macos_profiling_support/design.md` — Profiling infrastructure design
- `docs/designs/0014_benchmark_metrics_tracker/design.md` — Benchmark comparison design (similar pattern)
- `CLAUDE.md` — Profiling section to update

### Related Tickets
- `tickets/0012_add_macos_profiling_support.md` — Parent ticket for profiling infrastructure
- `tickets/0014_benchmark_metrics_tracker.md` — Similar tool for benchmark comparison

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Artifacts**:
  - `docs/designs/0015_profiling_trace_parser/design.md`
  - `docs/designs/0015_profiling_trace_parser/0015_profiling_trace_parser.puml`
- **Notes**:
  - Created comprehensive design for profiling trace parser
  - Designed parse-profile.py Python script following project conventions
  - Enhanced profile-instruments.sh with XML export capability (--export-xml flag)
  - Defined JSON output schema matching benchmark infrastructure patterns
  - Addressed human design decisions from ticket (profile_results/ directory, defer call tree, defer comparison tool)
  - Identified 3 prototypes needed: XML schema validation, symbol demangling verification, export performance testing
  - Design follows existing patterns from compare_benchmarks.py and run_benchmarks.sh
  - All requirements clarified, no blocking open questions

### Design Review Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Status**: APPROVED
- **Reviewer Notes**:
  - Design approved on first review (no revision needed)
  - Follows established project patterns (mirrors benchmark infrastructure)
  - Python 3.9+ stdlib-only implementation (no external dependencies)
  - Comprehensive error handling and defensive XML parsing
  - Two prototypes required for validation:
    - P1: Validate xctrace XML schema and symbol demangling (45 min)
    - P2: Test XML export performance (20 min)
  - Total prototype time: 65 minutes
  - All design criteria passed (architectural fit, feasibility, testability)
  - Risks identified and mitigated appropriately
  - Ready for prototype phase after human review

### Prototype Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Prototypes**:
  - P1: Validate xctrace XML schema structure and symbol demangling (45 min) — VALIDATED
  - P2: Test XML export performance characteristics (20 min) — VALIDATED
- **Artifacts**:
  - `docs/designs/0015_profiling_trace_parser/prototype-results.md`
  - `prototypes/0015_profiling_trace_parser/p1_xctrace_schema/test_parser.py`
  - `prototypes/0015_profiling_trace_parser/p1_xctrace_schema/test_export.xml` (7,821 samples)
  - `prototypes/0015_profiling_trace_parser/p1_xctrace_schema/toc.txt`
- **Notes**:
  - P1 validated xctrace XML schema uses time-sample based format (not direct node weights as expected)
  - Function names are fully demangled (no c++filt needed)
  - Source locations present but optional (14% of msd_sim functions had source file/line)
  - P2 validated XML export performance: 2.08s wall clock for 8.6s trace
  - Export produces 3.1 MB XML from 5.3 MB trace (0.58x ratio)
  - Both prototypes PASSED all success criteria
  - Minor design adjustment needed: parser must accumulate sample counts across time-sample rows
  - Implementation can proceed immediately with no blocking issues

### Implementation Phase
- **Started**: 2026-01-08
- **Completed**: 2026-01-08
- **Files Created**:
  - `scripts/parse-profile.py` — Python script to parse Time Profiler XML and generate JSON reports
- **Files Modified**:
  - `scripts/profile-instruments.sh` — Added `--export-xml` / `-x` flag, `--output-dir` / `-d` option, and XML export functionality
  - `.gitignore` — Added `profile_results/` directory
  - `CLAUDE.md` — Added comprehensive profiling parser documentation and recent architectural changes entry
- **Notes**:
  - Implementation completed all acceptance criteria from ticket
  - Enhanced profile-instruments.sh with:
    - New argument parsing supporting `-t`/`--template`, `-d`/`--output-dir`, `-x`/`--export-xml`, `-h`/`--help` flags
    - `export_trace_to_xml()` function that calls xctrace export with Time Profiler xpath
    - Default output directory changed to `profile_results/` (matches benchmark infrastructure)
    - Automatic directory creation if it doesn't exist
    - Suggestion to use parse-profile.py displayed after profiling completes
  - Created parse-profile.py with:
    - Time-sample based XML parsing (per prototype P1 findings)
    - Accumulates sample counts per function across all time samples
    - Handles backtrace references gracefully (skips ref-only backtraces in v1)
    - Automatic symbol demangling (symbols already demangled in XML, no c++filt needed)
    - Optional source file and line attribution
    - Color-coded console output with top N functions table
    - JSON output schema matching benchmark infrastructure patterns
    - Comprehensive error handling (missing trace file, xctrace not available, XML parse errors, empty profiles)
    - Default output to `profile_results/profile_<timestamp>.json`
    - Made executable with `chmod +x`
  - Updated CLAUDE.md with:
    - New "Parsing Profiling Data" section under Profiling with complete usage examples
    - JSON output schema documentation
    - Console output example
    - Script options table
    - Use cases (hotspot identification, CI integration, test profiling)
    - Limitations section (Time Profiler only, flat list, optional source locations, backtrace references)
    - Added entry to "Recent Architectural Changes" section
    - Added diagram to "Diagrams Index"
  - Updated .gitignore to exclude entire `profile_results/` directory
  - All implementation follows prototype findings:
    - Time-sample based accumulation (not direct node weights)
    - No c++filt needed (symbols auto-demangled)
    - Source locations optional (gracefully handles missing data)
    - XML export performance acceptable (~2s for 8.6s trace)

### Implementation Review Phase
- **Started**: 2026-01-10
- **Completed**: 2026-01-10
- **Status**: APPROVED
- **Artifacts**:
  - `docs/designs/0015_profiling_trace_parser/implementation-review.md`
- **Reviewer Notes**:
  - Implementation review APPROVED on first review (no revisions needed)
  - Design conformance: PASS - All components exist in correct locations, interfaces match design exactly
  - Prototype application: PASS - Time-sample accumulation, auto-demangled symbols, optional source locations all correctly applied
  - Code quality: PASS - Modern Python 3.9+ practices, follows project script conventions perfectly
  - Manual testing: PASS - All 12 acceptance criteria verified through execution
  - JSON schema validated against actual output (6,668 samples from msd_sim_bench)
  - Integration verified: profile-instruments.sh --export-xml → parse-profile.py workflow works correctly
  - CLAUDE.md documentation comprehensive and accurate
  - Two minor suggestions for future enhancement (metadata.executable extraction, backtrace reference documentation)
  - Implementation is production-ready and textbook quality
  - Ready to advance to "Approved — Ready to Merge" status

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
