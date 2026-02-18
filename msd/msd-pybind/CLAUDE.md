# msd-pybind — Python Bindings and Code Generation

> Python bindings for MSD transfer records, simulation engine control, and asset querying via pybind11, with automated code generation from C++ record definitions.

## Overview

msd-pybind provides a Python module (`msd_reader`) that exposes:
- msd-transfer C++ record types and database query operations (auto-generated)
- `msd_sim::Engine` live simulation control via `EngineWrapper` (manual)
- Asset registry and geometry query operations (manual)

Record bindings are **auto-generated** from `BOOST_DESCRIBE_STRUCT` macros in msd-transfer headers using `scripts/generate_record_layers.py`.

**Location**: `msd/msd-pybind/`
**Type**: pybind11 Python module
**Dependencies**: msd-assets (transitively provides msd-transfer), msd-sim
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
│   ├── asset_registry_bindings.cpp   # AssetRegistry wrapper (manual)
│   └── engine_bindings.cpp           # Engine simulation wrapper (manual)
└── test/
    ├── test_msd_reader.py            # Python test suite (records, DB, assets)
    └── test_engine_bindings.py       # Engine binding test suite (36 tests)
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

## Engine Bindings

**Location**: `src/engine_bindings.cpp`
**Introduced**: [Ticket: 0072a_engine_pybind_bindings](../../tickets/0072a_engine_pybind_bindings.md)

### Purpose

Exposes `msd_sim::Engine` to Python so the FastAPI replay/simulation server can instantiate simulations, spawn objects, step the physics, and extract frame state — all from Python. Used by the live browser simulation feature (Ticket 0072).

### Key Classes

| Class | Header | Responsibility |
|-------|--------|----------------|
| `EngineWrapper` | `engine_bindings.cpp` | Python wrapper around `msd_sim::Engine`; converts all Eigen types to Python dicts/tuples |

Exposed in Python as `msd_reader.Engine`.

### Key Interfaces

```python
import msd_reader

# Construct from assets database
engine = msd_reader.Engine("path/to/assets.db")

# Spawn objects
result = engine.spawn_inertial_object(
    "cube", x=0.0, y=0.0, z=5.0,
    pitch=0.0, roll=0.0, yaw=0.0,
    mass=10.0, restitution=0.5, friction=0.5)
# Returns: {"instance_id": int, "asset_id": int}

result = engine.spawn_environment_object(
    "large_cube", x=0.0, y=0.0, z=0.0)
# Returns: {"instance_id": int, "asset_id": int}

# Step simulation (absolute time, not delta)
engine.update(16)   # 16ms — first tick
engine.update(32)   # 32ms — second tick

# Extract current state
frame = engine.get_frame_state()
# Returns: {
#   "simulation_time": float,       # seconds
#   "states": [
#     {"body_id": int, "asset_id": int,
#      "position":         {"x": float, "y": float, "z": float},
#      "velocity":         {"x": float, "y": float, "z": float},
#      "orientation":      {"w": float, "x": float, "y": float, "z": float},
#      "angular_velocity": {"x": float, "y": float, "z": float}}
#   ]
# }

# Query available assets
assets = engine.list_assets()        # [(asset_id, "name"), ...]

# Get geometry for Three.js rendering
vertices = engine.get_collision_vertices(asset_id)  # [(x, y, z), ...]
```

### Design Pattern

Follows the `DatabaseWrapper` / `AssetRegistryWrapper` pattern: all C++ to Python type conversion (Eigen `Coordinate`, `AngularCoordinate`, quaternions) is done inside the C++ wrapper. No pybind11 Eigen type casters are used. The Python API only sees plain dicts, lists, and tuples.

### Important: update() takes absolute time

`engine.update(ms)` receives the **absolute simulation time** in milliseconds, not a delta. Callers must track cumulative time and pass increasing values:

```python
# Correct (60 FPS loop)
for i in range(1, 61):
    engine.update(i * 16)    # 16ms, 32ms, 48ms, ...

# Wrong
for _ in range(60):
    engine.update(16)        # Resets simulation to 16ms each call
```

### Thread Safety

Not thread-safe. Only one thread should call `Engine` methods at a time. This mirrors the underlying `msd_sim::Engine` constraint.

### Error Handling

- Constructor raises `RuntimeError` if the assets database path is invalid or cannot be opened
- `spawn_inertial_object` / `spawn_environment_object` raise `RuntimeError` if the named asset is not found or has no collision geometry

### Memory Management

- `EngineWrapper` owns `msd_sim::Engine` by value (exclusive ownership)
- pybind11 manages `EngineWrapper` lifetime via Python reference counting
- All returned dicts/lists/tuples are Python-owned objects

### Dependencies

- `msd-sim` — `msd_sim::Engine`, `Coordinate`, `AngularCoordinate`, `AngularVelocity`
- `msd-assets` — `AssetRegistry` (accessed via `Engine::getAssetRegistry()`)
- `pybind11/stl.h` — STL container conversion (for `list_assets`, `get_collision_vertices`)

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

# Record/database/asset tests
python3 -m pytest ../../../msd/msd-pybind/test/test_msd_reader.py -v

# Engine binding tests (requires valid assets.db)
python3 -m pytest ../../../msd/msd-pybind/test/test_engine_bindings.py -v
```

---

## References

- **Generator script**: [`scripts/generate_record_layers.py`](../../scripts/generate_record_layers.py)
- **Sync skill**: [`.claude/skills/sync-records/SKILL.md`](../../.claude/skills/sync-records/SKILL.md)
- **Cross-layer mapping**: [Ticket 0061](../../tickets/0061_cross_layer_record_mapping.md)
- **Design document**: [`docs/designs/0062_pybind_codegen_from_boost_describe/design.md`](../../docs/designs/0062_pybind_codegen_from_boost_describe/design.md)
- **Engine bindings**: [Ticket 0072a](../../tickets/0072a_engine_pybind_bindings.md)
- **Live simulation**: [Ticket 0072](../../tickets/0072_live_browser_simulation.md)
