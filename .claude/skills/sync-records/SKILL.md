---
name: sync-records
description: Regenerate pybind11 bindings and Pydantic leaf models from C++ msd-transfer records, then refresh traceability database. Use when records have been added/modified or to verify generated layers are current.

<example>
Context: User added a new field to EnergyRecord and wants to regenerate downstream layers.
user: "/sync-records"
assistant: "I'll regenerate the pybind11 bindings and Pydantic models from the updated C++ records."
<Runs scripts/generate_record_layers.py, then scripts/traceability/index_record_mappings.py>
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

Regenerates pybind11 bindings and Pydantic leaf models from C++ transfer records (the source of truth), then runs the 0061 cross-layer mapping indexer to refresh traceability and check for drift in hand-written composite models.

This skill ensures downstream representation layers stay synchronized with C++ record schema changes.

## Prerequisites

- `scripts/.venv` with tree-sitter and tree-sitter-cpp installed
- msd-transfer headers at `msd/msd-transfer/src/*.hpp`
- 0061 mapping indexer at `scripts/traceability/index_record_mappings.py` (optional)

## Steps

### 1. Parse C++ Transfer Records

Run the generator script:
```bash
source scripts/.venv/bin/activate
python scripts/generate_record_layers.py
```

This:
- Parses all `BOOST_DESCRIBE_STRUCT` macros from `msd-transfer/src/*.hpp`
- Extracts record names, field names, and field types
- Classifies fields into primitives, nested sub-records, ForeignKeys, RepeatedFields

### 2. Review Generated Files

Check what was regenerated:
```bash
git status msd/msd-pybind/src/record_bindings.cpp replay/replay/generated_models.py
```

Expected output:
- `msd/msd-pybind/src/record_bindings.cpp` — pybind11 bindings (if changed)
- `replay/replay/generated_models.py` — Pydantic leaf models (if changed)

If files are unchanged, generation is idempotent and records are current.

### 3. Run 0061 Mapping Indexer (Optional)

If the indexer exists, refresh traceability mappings:
```bash
# Check if indexer exists
if [ -f scripts/traceability/index_record_mappings.py ]; then
  source scripts/.venv/bin/activate
  python scripts/traceability/index_record_mappings.py
  echo "✓ Traceability database refreshed"
else
  echo "⚠ Mapping indexer not found (ticket 0061 may not be implemented)"
fi
```

The indexer:
- Parses `Maps-to:` docstring annotations from generated Pydantic models
- Links Python classes → C++ records deterministically (no heuristics)
- Validates that generated models have non-NULL linkage

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

============================================================
Generation complete
============================================================

[Optional: 0061 Indexer]
✓ Traceability database refreshed
✓ All generated models have deterministic C++ linkage

[Drift Check]
No drift detected — all hand-written composites are current
```
