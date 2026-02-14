# C++ Record → Pydantic Name Mapping

> Configuration reference for `scripts/generate_record_layers.py`
>
> Ticket: 0062_pybind_codegen_from_boost_describe

## Purpose

The generator uses a hard-coded mapping dictionary to control which C++ records are exposed as Pydantic models and what names they receive. This allows:

1. **Structural Equivalence**: Multiple C++ records with identical field layout can map to a single Pydantic class
2. **Selective Generation**: Only records in the mapping are generated (escape hatch for custom models)
3. **API-Friendly Names**: C++ naming (e.g., `CoordinateRecord`) can be mapped to terser Python names (e.g., `Vec3`)

## Current Mapping

| C++ Record | Pydantic Class | Rationale |
|------------|---------------|-----------|
| `CoordinateRecord` | `Vec3` | Shared 3D vector representation |
| `VelocityRecord` | `Vec3` | Structurally identical to CoordinateRecord |
| `AccelerationRecord` | `Vec3` | Structurally identical to CoordinateRecord |
| `Vector3DRecord` | `Vec3` | Structurally identical to CoordinateRecord |
| `ForceVectorRecord` | `Vec3` | Structurally identical to CoordinateRecord |
| `TorqueVectorRecord` | `Vec3` | Structurally identical to CoordinateRecord |
| `QuaternionDRecord` | `Quaternion` | Quaternion orientation (w, x, y, z) |
| `Vector4DRecord` | `Quaternion` | Structurally identical to QuaternionDRecord |
| `ContactPointRecord` | `ContactPoint` | Contact manifold geometry |
| `SolverDiagnosticRecord` | `SolverDiagnostics` | Constraint solver convergence metrics |
| `EnergyRecord` | `EnergyPoint` | Per-body energy values at one frame |
| `SystemEnergyRecord` | `SystemEnergyPoint` | System-level energy summary |

**Total**: 12 C++ records → 6 unique Pydantic classes

## Not Generated

Records **not** in the mapping are skipped during Pydantic generation. This includes:

- Top-level records without API exposure: `MeshRecord`, `ObjectRecord`, `MaterialRecord`, `PhysicsTemplateRecord`
- Internal records: `AssetDynamicStateRecord`, `AssetPhysicalStaticRecord`, `AssetPhysicalDynamicRecord`
- Constraint records (Tier 3): `ContactConstraintRecord`, `FrictionConstraintRecord`
- Angular sub-records (not currently exposed in API): `AngularVelocityRecord`, `AngularAccelerationRecord`, `AngularCoordinateRecord`
- Frame timestamping: `SimulationFrameRecord`, `InertialStateRecord`

These records still get pybind11 bindings (all 28 records are bound), but Pydantic models are not generated.

## Adding a New Mapping

To expose a new record in the Pydantic API:

1. **Edit** `scripts/generate_record_layers.py`
2. **Add** entry to `NAME_MAPPING` dict:
   ```python
   NAME_MAPPING = {
       # ... existing mappings
       "NewRecord": "NewModel",  # C++ record name → Python class name
   }
   ```
3. **Run** `/sync-records` skill to regenerate

The generator will:
- Parse the C++ record from msd-transfer headers
- Generate a Pydantic class with the specified name
- Include `Maps-to: NewRecord` docstring annotation
- Transform field names (camelCase → snake_case, FK → _id)

## Structural Equivalence Validation

**Current behavior**: The generator **trusts the mapping configuration** and does not validate that records mapped to the same Pydantic class have identical field layouts.

**Future enhancement** (ticket 0062 follow-up): Add validation that checks structural equivalence:
```python
# Pseudo-code
records_for_vec3 = ["CoordinateRecord", "VelocityRecord", "AccelerationRecord", ...]
field_sets = [set(record.fields) for record in records_for_vec3]
assert all(fs == field_sets[0] for fs in field_sets), "Vec3 records are not structurally equivalent"
```

This would catch drift if, for example, `VelocityRecord` adds a field not present in `CoordinateRecord`.

## Field Transformations

The generator applies these transformations to all generated models:

| C++ Pattern | Pydantic Pattern | Example |
|-------------|------------------|---------|
| `double field` | `field: float` | `x: float` |
| `uint32_t field` | `field: int` | `body_id: int` |
| `bool field` | `field: bool` | `converged: bool` |
| `ForeignKey<T> fk` | `fk_id: int` | `body: ForeignKey<...>` → `body_id: int` |
| `camelCase` | `snake_case` | `penetrationDepth` → `penetration_depth` |
| `NestedRecord sub` | `sub: GeneratedModel` | `normal: Vector3DRecord` → `normal: Vec3` |
| `RepeatedField<T> items` | `items: list[GeneratedModel]` | `contacts: RepeatedField<ContactPointRecord>` → `contacts: list[ContactPoint]` |

## Usage in Hand-Written Models

Hand-written composite models in `replay/replay/models.py` should **import from `generated_models.py`** rather than redefining leaf models:

```python
# Good
from replay.generated_models import Vec3, Quaternion, EnergyPoint

class BodyState(BaseModel):
    position: Vec3
    orientation: Quaternion
    energy: EnergyPoint

# Bad (duplicate definition)
class Vec3(BaseModel):
    x: float
    y: float
    z: float
```

This ensures a single source of truth and automatic updates when C++ schema changes.
