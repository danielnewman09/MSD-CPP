# msd-pybind — Python Bindings and Code Generation

> Python bindings for MSD transfer records via pybind11, with automated code generation from C++ record definitions.

## Overview

msd-pybind provides a Python module (`msd_reader`) that exposes msd-transfer C++ record types and database query operations to Python. Bindings are **auto-generated** from `BOOST_DESCRIBE_STRUCT` macros in msd-transfer headers using `scripts/generate_record_layers.py`.

**Location**: `msd/msd-pybind/`
**Type**: pybind11 Python module
**Dependencies**: msd-assets (transitively provides msd-transfer)
**External Dependencies**: pybind11, Python 3.x

---

## File Structure

```
msd-pybind/
├── CMakeLists.txt
├── CLAUDE.md                          # This file
├── src/
│   ├── msd_bindings.cpp              # Module definition (manual)
│   ├── record_bindings.cpp           # Record type bindings (AUTO-GENERATED)
│   ├── database_bindings.cpp         # Database query wrappers (manual)
│   ├── geometry_bindings.cpp         # Geometry deserialization helpers (manual)
│   └── asset_registry_bindings.cpp   # AssetRegistry wrapper (manual)
└── test/
    └── test_msd_reader.py            # Python test suite
```

---

## Record Layer Code Generation

### What Gets Generated

| Target File | Purpose | Source |
|-------------|---------|--------|
| `msd/msd-pybind/src/record_bindings.cpp` | pybind11 bindings for all 28+ transfer records | `BOOST_DESCRIBE_STRUCT` macros in msd-transfer |
| `replay/replay/generated_models.py` | Pydantic leaf models for REST API consumption | Same C++ records, with name mapping |

### Field Type Patterns

The generator uses mechanical patterns based on C++ field types:

| C++ Field Type | pybind11 Pattern | Pydantic Pattern |
|----------------|------------------|------------------|
| Primitive (`double`, `uint32_t`, `bool`) | `.def_readonly("name", &Record::name)` | `name: float \| int \| bool` |
| Nested sub-record (`CoordinateRecord`) | `.def_readonly("name", &Record::name)` | `name: Vec3` (via NAME_MAPPING) |
| `ForeignKey<T>` | `.def_property_readonly("name_id", lambda)` | `name_id: int` |
| `RepeatedFieldTransferObject<T>` | `.def_property_readonly("name", lambda)` | `name: list[T]` |

### C++ to Pydantic Name Mapping

Multiple C++ records can map to the same Pydantic class when structurally equivalent:

```python
# From scripts/generate_record_layers.py NAME_MAPPING
"CoordinateRecord": "Vec3"        # position, velocity, acceleration
"VelocityRecord": "Vec3"
"Vector3DRecord": "Vec3"
"QuaternionDRecord": "Quaternion"
"ContactPointRecord": "ContactPoint"
"EnergyRecord": "EnergyPoint"
"SystemEnergyRecord": "SystemEnergyPoint"
"SolverDiagnosticRecord": "SolverDiagnostics"
```

See `.claude/skills/sync-records/references/record-mapping.md` for the complete mapping rationale.

### Usage

**Recommended**: Use the `/sync-records` skill after modifying transfer record headers:

```bash
# In Claude Code CLI
/sync-records
```

**Manual invocation**:

```bash
# Regenerate both pybind bindings and Pydantic models
python scripts/generate_record_layers.py

# Check if generated files are current (exits non-zero if stale)
python scripts/generate_record_layers.py --check-only
```

### When to Regenerate

Run the generator (or `/sync-records`) when:
- Adding a new `BOOST_DESCRIBE_STRUCT` record in msd-transfer
- Adding or removing fields from an existing record
- Changing field types in a record

The generated files are committed to the repo. CI validates with `--check-only`.

### Design Decision Linkage

Generated Pydantic models include `Maps-to:` docstring annotations:

```python
class Vec3(BaseModel):
    """Generated from CoordinateRecord. Maps-to: CoordinateRecord"""
    x: float
    y: float
    z: float
```

These annotations enable the ticket 0061 cross-layer mapping indexer to deterministically link Python models to C++ records in the traceability database.

---

## Workflow Integration

- **Developer workflow**: Use `/sync-records` skill after modifying transfer record headers
- **Docs-updater integration**: When a ticket touches `msd-transfer/src/*.hpp`, the docs-updater agent runs the sync (Phase 6.5)
- **CI drift check**: `python scripts/generate_record_layers.py --check-only` fails if generated files are stale

---

## Building

```bash
# Build the pybind module (requires ENABLE_PYBIND=ON in Conan)
conan install . --build=missing -s build_type=Debug -o "&:enable_pybind=True"
cmake --preset conan-debug
cmake --build --preset debug-pybind-only
```

Output: `build/Debug/debug/msd_reader.cpython-{version}-darwin.so`

## Testing

```bash
# Run from build directory
cd build/Debug/debug
python3 -m pytest ../../../msd/msd-pybind/test/test_msd_reader.py -v
```

---

## References

- **Generator script**: [`scripts/generate_record_layers.py`](../../scripts/generate_record_layers.py)
- **Sync skill**: [`.claude/skills/sync-records/SKILL.md`](../../.claude/skills/sync-records/SKILL.md)
- **Cross-layer mapping**: [Ticket 0061](../../tickets/0061_cross_layer_record_mapping.md)
- **Design document**: [`docs/designs/0062_pybind_codegen_from_boost_describe/design.md`](../../docs/designs/0062_pybind_codegen_from_boost_describe/design.md)
