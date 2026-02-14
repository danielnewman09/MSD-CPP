---
name: sync-records
description: Regenerate pybind11 bindings and Pydantic leaf models from C++ msd-transfer records, then refresh traceability database. Use when records have been added/modified or to verify generated layers are current.

<example>
Context: User added a new field to EnergyRecord and wants to regenerate downstream layers.
user: "/sync-records"
assistant: "I'll regenerate the pybind11 bindings and Pydantic models from the updated C++ records."
<Runs scripts/generate_record_layers.py --update-traceability, then index_record_mappings.py for composites>
</example>

<example>
Context: User wants to check if generated files are current with C++ source.
user: "Are the Python bindings up to date with the C++ records?"
assistant: "I'll run /sync-records to verify and regenerate if needed."
<Runs skill>
</example>

model: sonnet
---

# /sync-records Skill

## What This Does

Regenerates pybind11 bindings and Pydantic leaf models from C++ transfer records (the source of truth), writes all four generator-managed layers (cpp, sql, pybind, leaf-pydantic) directly to the traceability database, then indexes hand-written composite Pydantic models.

This skill ensures downstream representation layers stay synchronized with C++ record schema changes.

## Prerequisites

- `scripts/.venv` with tree-sitter and tree-sitter-cpp installed
- msd-transfer headers at `msd/msd-transfer/src/*.hpp`
- Traceability database at `build/Debug/docs/traceability.db` (optional)

## Steps

### 1. Generate Bindings and Update Traceability

Run the generator script with the traceability flag:
```bash
source scripts/.venv/bin/activate
python scripts/generate_record_layers.py --update-traceability build/Debug/docs/traceability.db
```

This single command:
- Parses all `BOOST_DESCRIBE_STRUCT` macros from `msd-transfer/src/*.hpp`
- Generates `msd/msd-pybind/src/record_bindings.cpp` (pybind11 bindings)
- Generates `replay/replay/generated_models.py` (Pydantic leaf models)
- Writes cpp, sql, pybind, and leaf-pydantic layer data to the traceability database

### 2. Review Generated Files

Check what was regenerated:
```bash
git status msd/msd-pybind/src/record_bindings.cpp replay/replay/generated_models.py
```

If files are unchanged, generation is idempotent and records are current.

### 3. Index Composite Pydantic Models

Run the indexer for hand-written composite models (FrameData, BodyState, etc.):
```bash
python scripts/traceability/index_record_mappings.py build/Debug/docs/traceability.db --repo .
```

The indexer handles only composite models — the generator already populated the four authoritative layers.

### 4. Check for Drift in Hand-Written Composites

Compare generated leaf model fields with hand-written composite models in `replay/replay/models.py`:

```python
# Pseudo-code for drift check
import ast
from pathlib import Path

generated = Path("replay/replay/generated_models.py")
hand_written = Path("replay/replay/models.py")

# Parse both files with ast.parse()
# For each hand-written composite (FrameData, BodyState, etc.):
#   - Check if it references generated leaf models
#   - If yes, check if all fields from generated model are exposed
#   - Report any new fields in generated models not used in composites
```

**Advisory only** — drift does not block, but highlights opportunities to update API models.

### 5. Report Summary

Provide structured output:
```
## Record Sync Results

### Generated Files Updated
- msd/msd-pybind/src/record_bindings.cpp (28 records, 2 new fields in EnergyRecord)
- replay/replay/generated_models.py (6 leaf models, EnergyPoint updated)

### Drift Check
- ⚠ WARNING: EnergyRecord has new field `angular_momentum` — check hand-written models:
  - replay/replay/models.py:FrameData (references EnergyPoint)

### Manual Review Needed
- [ ] Consider exposing `angular_momentum` in FrameData API
```

## Configuration

The generator uses a hard-coded name mapping in `scripts/generate_record_layers.py`:

```python
NAME_MAPPING = {
    "CoordinateRecord": "Vec3",
    "VelocityRecord": "Vec3",
    "QuaternionDRecord": "Quaternion",
    "ContactPointRecord": "ContactPoint",
    "SolverDiagnosticRecord": "SolverDiagnostics",
    "EnergyRecord": "EnergyPoint",
    "SystemEnergyRecord": "SystemEnergyPoint",
    # ... (see scripts/generate_record_layers.py for full mapping)
}
```

To add a new record to Pydantic generation, add it to `NAME_MAPPING`.

## Error Handling

- If tree-sitter parsing fails: Report header file and line number, suggest manual inspection
- If pybind bindings don't compile: Report C++ error, may indicate type mismatch or missing include
- If Pydantic models have import errors: Check NAME_MAPPING for circular dependencies
- If indexer fails: Non-blocking, skill completes but traceability not updated

## Workflow Integration

This skill is invoked automatically by the docs-updater agent (Phase 6.5) when a ticket touches `msd-transfer/src/*.hpp` files. Manual invocation is also supported for ad-hoc regeneration.

---

## Example Output

```
============================================================
Record Layer Generator
============================================================

[1/3] Parsing msd-transfer headers...
  Parsed AccelerationRecord (Tier 2)
  Parsed CoordinateRecord (Tier 2)
  Parsed EnergyRecord (Tier 1)
  ... (25 more records)
  Found 28 records

[2/3] Generating pybind11 bindings...
  Wrote msd/msd-pybind/src/record_bindings.cpp

[3/3] Generating Pydantic leaf models...
  Generating 6 unique models from 12 records
  Wrote replay/replay/generated_models.py

✓ Generated 28 record bindings
✓ Generated 6 Pydantic leaf models

[4/4] Updating traceability database: build/Debug/docs/traceability.db...
  ✓ Wrote 28 records across 4 layers (cpp, sql, pybind, pydantic)

============================================================
Generation complete
============================================================

[Composite Indexer]
  Pydantic composites: 14 models parsed
  Generated leaf models: 6 (handled by generator)
✓ Indexed 14 composite Pydantic models

[Drift Check]
No drift detected — all hand-written composites are current
```
