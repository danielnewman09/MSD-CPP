# Design: Record Layer Code Generation from BOOST_DESCRIBE

## Summary

This feature automates the generation of pybind11 bindings and leaf Pydantic models from msd-transfer C++ record headers. Currently, both layers are manually written following mechanical patterns determined entirely by C++ field types. By parsing BOOST_DESCRIBE_STRUCT macros and struct member declarations with tree-sitter, we can emit both `record_bindings.cpp` (pybind11) and `generated_models.py` (Pydantic leaf models) automatically. A `/sync-records` skill provides a developer-facing entry point, and the docs-updater agent runs the sync as part of the feature workflow to keep downstream layers current.

This design eliminates manual boilerplate, enforces consistency across layers, provides deterministic C++ → Python linkage for the 0061 mapping indexer, and integrates with the existing workflow via the docs-updater agent.

## Architecture Changes

### PlantUML Diagram
See: `./0062_pybind_codegen_from_boost_describe.puml`

### New Components

#### RecordLayerGenerator (scripts/generate_record_layers.py)
- **Purpose**: Main generator script that orchestrates parsing, code generation, and file writing
- **Header location**: N/A (Python script)
- **Source location**: `scripts/generate_record_layers.py`
- **Key interfaces**:
  ```python
  def main():
      """Entry point — reads headers, generates files."""
      parser = argparse.ArgumentParser()
      parser.add_argument("--check-only", action="store_true")
      args = parser.parse_args()

      # Parse all record headers
      records = parse_record_headers(
          headers_dir="msd/msd-transfer/src/",
          include_path="msd-transfer/src/Records.hpp"
      )

      # Generate pybind bindings
      pybind_code = generate_pybind_bindings(records)
      write_file("msd/msd-pybind/src/record_bindings.cpp", pybind_code)

      # Generate Pydantic models
      pydantic_code = generate_pydantic_models(records, NAME_MAPPING)
      write_file("replay/replay/generated_models.py", pydantic_code)

      # Optionally check for drift
      if args.check_only:
          check_drift_and_exit()

  def parse_record_headers(headers_dir: Path, include_path: str) -> list[RecordInfo]:
      """Parse all headers included by Records.hpp."""
      parser = RecordParser()
      records = []
      for header in get_included_headers(include_path):
          record_info = parser.parse_header(headers_dir / header)
          if record_info:
              records.append(record_info)
      return classify_tiers(records)

  def generate_pybind_bindings(records: list[RecordInfo]) -> str:
      """Generate record_bindings.cpp content."""
      codegen = PybindCodegen()
      return codegen.generate(records)

  def generate_pydantic_models(records: list[RecordInfo], mapping: dict) -> str:
      """Generate generated_models.py content."""
      codegen = PydanticCodegen(name_mapping=mapping)
      return codegen.generate(records)
  ```
- **Dependencies**: tree-sitter, tree-sitter-cpp (already in scripts/.venv)
- **Thread safety**: Single-threaded script, no concurrency
- **Error handling**: Exit with non-zero on parse errors, file write errors

#### RecordParser (scripts/generate_record_layers.py)
- **Purpose**: Parse C++ record headers to extract record name, base class, field list, and field types
- **Key interfaces**:
  ```python
  class RecordParser:
      def __init__(self):
          self.parser = self._create_parser()

      def parse_header(self, header_path: Path) -> RecordInfo | None:
          """Extract record metadata from a header file."""
          source = header_path.read_bytes()
          tree = self.parser.parse(source)

          # Find BOOST_DESCRIBE_STRUCT macro
          describe_macro = self._find_boost_describe(tree.root_node)
          if not describe_macro:
              return None

          record_name, base_class, field_names = self._parse_macro_args(describe_macro)

          # Find struct definition
          struct_node = self._find_struct(tree.root_node, record_name)
          if not struct_node:
              raise ParseError(f"BOOST_DESCRIBE found but no struct {record_name}")

          # Extract field types from struct body
          fields = self._extract_fields(struct_node, field_names)

          return RecordInfo(
              record_name=record_name,
              base_class=base_class,
              fields=fields,
              header_path=header_path
          )

      def _classify_field_type(self, type_str: str) -> FieldType:
          """Classify field into PRIMITIVE, NESTED_SUB_RECORD, FOREIGN_KEY, or REPEATED_FIELD."""
          if "ForeignKey<" in type_str:
              return FieldType.FOREIGN_KEY
          elif "RepeatedFieldTransferObject<" in type_str:
              return FieldType.REPEATED_FIELD
          elif type_str.endswith("Record"):
              return FieldType.NESTED_SUB_RECORD
          elif type_str in ("double", "float", "uint32_t", "int", "bool", "std::string"):
              return FieldType.PRIMITIVE
          else:
              raise ValueError(f"Unknown field type: {type_str}")
  ```
- **Dependencies**: tree-sitter, tree-sitter-cpp
- **Error handling**: Raises ParseError on malformed headers, invalid BOOST_DESCRIBE syntax

#### PybindCodegen (scripts/generate_record_layers.py)
- **Purpose**: Generate `record_bindings.cpp` from RecordInfo list
- **Key interfaces**:
  ```python
  class PybindCodegen:
      BINDING_PATTERNS = {
          FieldType.PRIMITIVE: lambda name, record, field:
              f'.def_readonly("{name}", &{record}::{name})',
          FieldType.NESTED_SUB_RECORD: lambda name, record, field:
              f'.def_readonly("{name}", &{record}::{name})',
          FieldType.FOREIGN_KEY: lambda name, record, field:
              f'.def_property_readonly("{name}_id", []({record} const& r) {{ return r.{name}.id; }})',
          FieldType.REPEATED_FIELD: lambda name, record, field:
              f'.def_property_readonly("{name}", []({record} const& r) {{ return r.{name}.data; }})'
      }

      def generate(self, records: list[RecordInfo]) -> str:
          """Generate full record_bindings.cpp content."""
          header = self._generate_header()

          # Group by tier: Tier 2 (sub-records), Tier 1 (top-level), Tier 3 (extended)
          tier2 = [r for r in records if r.tier == 2]
          tier1 = [r for r in records if r.tier == 1]
          tier3 = [r for r in records if r.tier == 3]

          body = []
          body.append(self._generate_tier_comment("Tier 2: Sub-Records"))
          for record in tier2:
              body.append(self._generate_class_binding(record))

          body.append(self._generate_tier_comment("Tier 1: Top-Level Records"))
          for record in tier1:
              body.append(self._generate_class_binding(record))

          body.append(self._generate_tier_comment("Tier 3: Extended Records"))
          for record in tier3:
              body.append(self._generate_class_binding(record))

          return header + "\n".join(body) + self._generate_footer()

      def _generate_class_binding(self, record: RecordInfo) -> str:
          """Generate py::class_<> block for a single record."""
          bindings = [f'  py::class_<msd_transfer::{record.record_name}>(m, "{record.record_name}")']
          bindings.append('    .def(py::init<>())')
          bindings.append('    .def_readonly("id", &msd_transfer::{record.record_name}::id)')

          for field in record.fields:
              pattern = self.BINDING_PATTERNS[field.field_type]
              binding_line = pattern(field.name, f"msd_transfer::{record.record_name}", field)
              bindings.append(f'    {binding_line}')

          bindings.append('    ;')
          return "\n".join(bindings)
  ```
- **Dependencies**: None (pure Python codegen)
- **Error handling**: Validates RecordInfo before generation

#### PydanticCodegen (scripts/generate_record_layers.py)
- **Purpose**: Generate `generated_models.py` with leaf Pydantic models
- **Key interfaces**:
  ```python
  class PydanticCodegen:
      TYPE_MAPPING = {
          "double": "float",
          "float": "float",
          "uint32_t": "int",
          "int": "int",
          "bool": "bool",
          "std::string": "str",
      }

      def __init__(self, name_mapping: dict[str, str]):
          """
          Args:
              name_mapping: C++ record name → Pydantic class name
                            e.g., {"CoordinateRecord": "Vec3", ...}
          """
          self.name_mapping = name_mapping

      def generate(self, records: list[RecordInfo]) -> str:
          """Generate full generated_models.py content."""
          header = self._generate_header()
          imports = "from pydantic import BaseModel\n\n"

          # Only generate models for records in the mapping
          models = []
          for record in records:
              if record.record_name in self.name_mapping:
                  model_code = self._generate_model(record)
                  models.append(model_code)

          return header + imports + "\n\n".join(models)

      def _generate_model(self, record: RecordInfo) -> str:
          """Generate a single Pydantic model."""
          pydantic_name = self.name_mapping[record.record_name]

          lines = [
              f'class {pydantic_name}(BaseModel):',
              f'    """Generated from {record.record_name}. Maps-to: {record.record_name}"""',
              ''
          ]

          for field in record.fields:
              py_field_name = self._camel_to_snake(field.name)
              py_type = self._map_field_type(field)
              lines.append(f'    {py_field_name}: {py_type}')

          return "\n".join(lines)

      def _map_field_type(self, field: FieldInfo) -> str:
          """Map C++ field type to Python type annotation."""
          if field.field_type == FieldType.PRIMITIVE:
              return self.TYPE_MAPPING[field.type]
          elif field.field_type == FieldType.FOREIGN_KEY:
              return "int"
          elif field.field_type == FieldType.NESTED_SUB_RECORD:
              # Look up the nested record's Pydantic name
              nested_record_name = field.type
              if nested_record_name in self.name_mapping:
                  return self.name_mapping[nested_record_name]
              else:
                  return f"{nested_record_name}Model"
          elif field.field_type == FieldType.REPEATED_FIELD:
              element_type = self._extract_template_arg(field.type)
              py_element_type = self.name_mapping.get(element_type, f"{element_type}Model")
              return f"list[{py_element_type}]"

      def _camel_to_snake(self, name: str) -> str:
          """Transform camelCase to snake_case."""
          import re
          s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
          return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()
  ```
- **Dependencies**: None (pure Python codegen)
- **Error handling**: Raises error on unmapped nested record types

#### RecordInfo / FieldInfo (Data Classes)
- **Purpose**: Data transfer objects for parsed record metadata
- **Interfaces**:
  ```python
  from dataclasses import dataclass
  from enum import Enum
  from pathlib import Path

  class FieldType(Enum):
      PRIMITIVE = "primitive"
      NESTED_SUB_RECORD = "nested_sub_record"
      FOREIGN_KEY = "foreign_key"
      REPEATED_FIELD = "repeated_field"

  @dataclass
  class FieldInfo:
      name: str           # Field name from BOOST_DESCRIBE
      type: str           # C++ type string (e.g., "double", "ForeignKey<T>")
      field_type: FieldType  # Classified type

  @dataclass
  class RecordInfo:
      record_name: str    # e.g., "CoordinateRecord"
      base_class: str     # e.g., "cpp_sqlite::BaseTransferObject"
      fields: list[FieldInfo]
      header_path: Path
      tier: int = 0       # 1=top-level, 2=sub-record, 3=extended
  ```

#### /sync-records Skill (.claude/skills/sync-records/SKILL.md)
- **Purpose**: Developer-facing skill to regenerate all record layers and validate with 0061 indexer
- **Location**: `.claude/skills/sync-records/SKILL.md`
- **Key workflow**:
  ```markdown
  # /sync-records Skill

  ## What This Does

  Regenerates pybind11 bindings and Pydantic leaf models from C++ transfer records, then runs the 0061 cross-layer mapping indexer to refresh traceability and check for drift in hand-written composite models.

  ## Steps

  ### 1. Parse C++ Transfer Records
  - Read all `BOOST_DESCRIBE_STRUCT` macros from `msd-transfer/src/*.hpp`
  - Extract record names, field names, and field types
  - Classify fields into primitives, nested sub-records, ForeignKeys, RepeatedFields

  ### 2. Regenerate Pybind Bindings
  - Generate `msd/msd-pybind/src/record_bindings.cpp`
  - Use mechanical patterns based on field type
  - Preserve Tier 2 → Tier 1 → Tier 3 ordering
  - Mark file as AUTO-GENERATED

  ### 3. Regenerate Pydantic Leaf Models
  - Generate `replay/replay/generated_models.py`
  - Apply C++ → Pydantic name mapping (e.g., CoordinateRecord → Vec3)
  - Transform camelCase → snake_case
  - Include `Maps-to:` docstrings for 0061 linkage
  - Mark file as AUTO-GENERATED

  ### 4. Run 0061 Mapping Indexer
  - Execute `scripts/traceability/index_record_mappings.py`
  - Refresh traceability database with new linkages
  - Validate that generated models have deterministic C++ linkage

  ### 5. Check for Drift in Hand-Written Composites
  - Compare fields in generated models vs. hand-written `models.py`
  - Report if new C++ fields are available but not yet used in composites
  - Advisory only — does not block

  ### 6. Report Summary
  - List generated files and record counts
  - Report drift status (OK / WARNING with details)
  - Highlight manual review needed for composite models

  ## Usage

  ```
  /sync-records
  ```

  No arguments — always syncs all records.
  ```
- **Dependencies**: RecordLayerGenerator script, 0061 indexer
- **Error handling**: Reports parse errors, file write errors, indexer failures

### Modified Components

#### msd/msd-pybind/src/record_bindings.cpp
- **Current location**: `msd/msd-pybind/src/record_bindings.cpp`
- **Changes required**:
  - Replace entire file with AUTO-GENERATED header
  - Contents generated by `scripts/generate_record_layers.py`
  - Manual version archived or compared for validation
- **Backward compatibility**:
  - Generated bindings are semantically equivalent to current hand-written version
  - All existing `test_msd_reader.py` tests must pass without modification
  - No API changes at the Python layer

#### replay/replay/models.py
- **Current location**: `replay/replay/models.py`
- **Changes required**:
  - Remove leaf model definitions (Vec3, Quaternion, ContactPoint, SolverDiagnostics, EnergyPoint, SystemEnergyPoint)
  - Add import: `from .generated_models import Vec3, Quaternion, ContactPoint, SolverDiagnostics, EnergyPoint, SystemEnergyPoint`
  - Keep composite models (FrameData, BodyState, BodyMetadata, SimulationMetadata, SimulationInfo, FrameInfo, AssetGeometry) unchanged
- **Backward compatibility**:
  - Public API unchanged — consumers still import from `replay.models`
  - Generated models re-exported to maintain import paths
  - All existing API tests must pass

#### .claude/agents/docs-updater.md
- **Current location**: `.claude/agents/docs-updater.md`
- **Changes required**:
  - Add conditional step after Phase 6 (documentation update):
    ```markdown
    ### Phase 6.5: Record Layer Sync (Conditional)

    **Trigger**: If ticket's file changes include any `msd-transfer/src/*.hpp` files

    **Action**:
    1. Run `scripts/generate_record_layers.py` to regenerate bindings and models
    2. Run `scripts/traceability/index_record_mappings.py` to update traceability
    3. Check for drift in hand-written `models.py` composite models

    **Report**: Append "Record Layer Sync" section to `doc-sync-summary.md`:
    ```markdown
    ## Record Layer Sync
    - pybind bindings: regenerated (N records)
    - Pydantic leaf models: regenerated (N models)
    - Drift check: {PASS / WARNING: field X added to RecordY, check composite models}
    ```

    **Non-blocking**: Drift warnings are advisory — do not block documentation phase
    ```
- **Backward compatibility**: Existing workflow unchanged for tickets that don't touch msd-transfer

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| RecordLayerGenerator | msd-transfer headers | Read dependency | Parses BOOST_DESCRIBE and struct definitions |
| RecordLayerGenerator | msd-pybind/src/record_bindings.cpp | Write dependency | Overwrites entire file |
| RecordLayerGenerator | replay/replay/generated_models.py | Write dependency | Creates new file |
| /sync-records skill | RecordLayerGenerator | Invocation | Runs generator script |
| /sync-records skill | 0061 indexer | Invocation | Runs after generation for validation |
| docs-updater agent | /sync-records skill | Conditional invocation | Runs if msd-transfer touched |
| replay/models.py | replay/generated_models.py | Import dependency | Imports leaf models |

## Configuration and Mapping

### C++ → Pydantic Name Mapping

The generator uses a configuration dictionary to map C++ record names to Pydantic class names:

```python
# In scripts/generate_record_layers.py
NAME_MAPPING = {
    # Multiple C++ records → same Pydantic class (structural equivalence)
    "CoordinateRecord": "Vec3",
    "VelocityRecord": "Vec3",
    "AccelerationRecord": "Vec3",
    "Vector3DRecord": "Vec3",
    "ForceVectorRecord": "Vec3",
    "TorqueVectorRecord": "Vec3",

    # Quaternion types
    "QuaternionDRecord": "Quaternion",
    "Vector4DRecord": "Quaternion",  # Structural match

    # Contact geometry
    "ContactPointRecord": "ContactPoint",

    # Diagnostics and energy
    "SolverDiagnosticRecord": "SolverDiagnostics",
    "EnergyRecord": "EnergyPoint",
    "SystemEnergyRecord": "SystemEnergyPoint",

    # Angular sub-records (if needed for future API endpoints)
    "AngularVelocityRecord": "EulerAngles",
    "AngularAccelerationRecord": "EulerAngles",
    "AngularCoordinateRecord": "EulerAngles",
}
```

**Records not in the mapping** are skipped during Pydantic generation. This allows:
- Selective generation (only records exposed to the API)
- Custom hand-written models for complex cases
- Escape hatch for non-standard bindings

### Tier Classification

Records are classified into tiers to preserve dependency order in generated bindings:

| Tier | Description | Examples |
|------|-------------|----------|
| **Tier 2** | Sub-records (no FKs, used as nested fields) | CoordinateRecord, QuaternionDRecord, ContactPointRecord |
| **Tier 1** | Top-level records (own DB tables, may have FKs) | SimulationFrameRecord, EnergyRecord, InertialStateRecord |
| **Tier 3** | Extended records (forward compatibility, complex nesting) | AssetDynamicStateRecord, AssetPhysicalStaticRecord |

**Tier detection algorithm**:
1. **Tier 2**: No ForeignKey fields, name in TIER2_SUBRECORDS list
2. **Tier 1**: Has ForeignKey fields OR in TIER1_TOPLEVEL list
3. **Tier 3**: Everything else (extended/future records)

```python
TIER2_SUBRECORDS = {
    "CoordinateRecord", "VelocityRecord", "AccelerationRecord",
    "QuaternionDRecord", "Vector4DRecord", "Vector3DRecord",
    "AngularAccelerationRecord", "AngularVelocityRecord", "AngularCoordinateRecord",
    "ContactPointRecord", "ForceVectorRecord", "TorqueVectorRecord",
    "ExternalForceRecord",  # Nested in AssetDynamicStateRecord
}

TIER1_TOPLEVEL = {
    "SimulationFrameRecord", "AssetInertialStaticRecord", "InertialStateRecord",
    "EnergyRecord", "SystemEnergyRecord", "CollisionResultRecord",
    "SolverDiagnosticRecord", "MeshRecord", "ObjectRecord",
    "MaterialRecord", "PhysicsTemplateRecord",
}
```

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `test/msd-pybind/test_msd_reader.py` | All pybind import tests | Generated bindings replace manual | Must pass without changes (validation) |
| `replay/test_services.py` | FastAPI endpoint tests | Imports from generated_models | Must pass without changes (validation) |
| `replay/test_models.py` | Pydantic model validation | Leaf models now imported | Update imports if separate tests exist |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| RecordParser | `test_parse_coordinate_record()` | Extracts record name, fields, types correctly |
| RecordParser | `test_classify_field_types()` | Classifies primitives, FKs, nested, repeated |
| RecordParser | `test_parse_error_handling()` | Raises ParseError on malformed headers |
| PybindCodegen | `test_generate_primitive_field()` | Emits `.def_readonly()` for primitives |
| PybindCodegen | `test_generate_foreign_key_field()` | Emits `.def_property_readonly("{name}_id", ...)` |
| PybindCodegen | `test_generate_repeated_field()` | Emits `.def_property_readonly()` with `.data` accessor |
| PybindCodegen | `test_tier_ordering()` | Tier 2 → Tier 1 → Tier 3 preserved |
| PydanticCodegen | `test_camel_to_snake()` | `penetrationDepth` → `penetration_depth` |
| PydanticCodegen | `test_name_mapping()` | `CoordinateRecord` → `Vec3` |
| PydanticCodegen | `test_maps_to_docstring()` | Docstring includes `Maps-to: {RecordName}` |
| RecordLayerGenerator | `test_idempotency()` | Running twice produces identical output |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `test_generated_bindings_compile()` | RecordLayerGenerator, pybind11 | Generated `record_bindings.cpp` compiles without errors |
| `test_generated_models_import()` | RecordLayerGenerator, Pydantic | Generated `generated_models.py` imports without errors |
| `test_msd_reader_api_unchanged()` | Generated bindings, msd_reader | All record types accessible, field access works |
| `test_pydantic_field_access()` | Generated models, FastAPI | Generated models serialize/deserialize correctly |
| `test_0061_indexer_linkage()` | Generated models, 0061 indexer | `Maps-to:` annotations parsed correctly, no NULL linkage |
| `test_sync_records_skill()` | /sync-records skill, generator, indexer | Skill runs end-to-end, reports drift correctly |
| `test_docs_updater_integration()` | docs-updater agent, /sync-records | Agent runs record sync when msd-transfer touched |

#### Regression Tests

| Test Case | What It Validates |
|-----------|-------------------|
| `test_bindings_semantic_equivalence()` | Diff generated vs. current `record_bindings.cpp` — field-by-field comparison |
| `test_pybind_test_suite()` | All existing pybind tests pass with generated bindings |
| `test_replay_api_suite()` | All existing replay API tests pass with generated leaf models |
| `test_no_import_breakage()` | Composite models in `models.py` import from `generated_models.py` successfully |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Build Integration: Build-Time vs. Manual Generation**
   - **Option A (Build-Time)**: Add CMake custom target that runs generator at build time
     - **Pros**: Generated files always fresh, no risk of stale bindings
     - **Cons**: Slower builds (tree-sitter parsing), requires Python in build environment
   - **Option B (Manual Generation with CI Check)**: Run manually via `/sync-records` skill, commit generated files
     - **Pros**: Faster builds, no Python dependency at build time, generated files versioned
     - **Cons**: Risk of forgetting to regenerate, requires CI check for drift detection
   - **Recommendation**: **Option B** — Manual generation with CI drift check. Rationale:
     - Record schema changes are infrequent (few times per ticket)
     - Build-time cost is high relative to update frequency
     - `/sync-records` skill + docs-updater integration provide automatic sync in workflow
     - CI can fail if `record_bindings.cpp` or `generated_models.py` are out-of-date

2. **Escape Hatch: Skip List vs. Comment Convention**
   - **Option A (Skip List)**: Hard-coded list in generator script
     - **Pros**: Simple, centralized, easy to audit
     - **Cons**: Not discoverable in headers
   - **Option B (Comment Convention)**: `// @skip-codegen` comment in header
     - **Pros**: Self-documenting, discoverable in headers
     - **Cons**: Requires parsing comments (easy with tree-sitter)
   - **Recommendation**: **Option A** — Skip list in generator script. Rationale:
     - Currently no records need custom bindings (all follow mechanical patterns)
     - If custom bindings are needed, they live in separate files (`geometry_bindings.cpp`, `asset_registry_bindings.cpp`)
     - Skip list can reference these files in comments for traceability

3. **Generated File Header Format**
   - **Option A**: Minimal header (`// AUTO-GENERATED — do not edit`)
   - **Option B**: Detailed header with generator invocation, timestamp, source files
   - **Recommendation**: **Option B** — Include metadata for debugging. Example:
     ```cpp
     // AUTO-GENERATED by scripts/generate_record_layers.py
     // DO NOT EDIT MANUALLY
     // Generated: 2026-02-13 14:32:15
     // Source: msd-transfer/src/Records.hpp (28 records)
     // Ticket: 0062_pybind_codegen_from_boost_describe
     ```

### Requirements Clarification

1. **Database Binding Generation (R3)**
   - The ticket mentions generating `database_bindings.cpp` for `selectAll<>`, `selectById<>`, etc.
   - **Question**: Should this be part of the initial implementation, or deferred to a follow-up?
   - **Context**: Current `database_bindings.cpp` is minimal (~50 lines) and follows trivial patterns
   - **Recommendation**: Defer to follow-up ticket. Rationale:
     - Focus on record bindings (R2) and Pydantic models (R7) first
     - Database bindings are less complex and can be added later without blocking workflow

2. **Handling Multiple Records → Same Pydantic Class**
   - CoordinateRecord, VelocityRecord, AccelerationRecord all map to Vec3
   - **Question**: Should the generator validate structural equivalence, or trust the mapping configuration?
   - **Recommendation**: Trust configuration for initial implementation, add validation in follow-up. Rationale:
     - Structural equivalence check requires deep field-by-field comparison
     - Configuration is manually maintained and reviewed (low risk)
     - Can add validation in CI as a safeguard later

3. **Drift Reporting Detail Level**
   - When new C++ fields are added, how detailed should the drift report be?
   - **Option A**: List all new fields across all composite models
   - **Option B**: Only report records with new fields, let developer inspect
   - **Recommendation**: **Option A** — List all new fields. Rationale:
     - Composite models like `FrameData` are hand-written and may not expose all C++ fields
     - Explicit list helps developer decide what to expose

## Implementation Phases

This design can be implemented in phases for incremental delivery:

### Phase 1: Core Generator (Pybind Only)
- RecordParser with tree-sitter
- PybindCodegen
- Generate `record_bindings.cpp`
- Validate against current hand-written version
- **Deliverable**: Generated pybind bindings compile and pass all existing tests

### Phase 2: Pydantic Generation
- PydanticCodegen with name mapping
- Generate `generated_models.py`
- Update `models.py` to import from generated file
- **Deliverable**: Replay API tests pass with generated leaf models

### Phase 3: Skill and Workflow Integration
- Create `/sync-records` skill
- Integrate with docs-updater agent (Phase 6.5 conditional step)
- Drift reporting for composite models
- **Deliverable**: End-to-end workflow from `/sync-records` to traceability refresh

### Phase 4: CI Integration and Validation
- CMake target or CI check for drift detection
- 0061 indexer validation (Maps-to annotations)
- Documentation updates
- **Deliverable**: Automated drift detection prevents stale bindings

## Acceptance Criteria Mapping

This design addresses all acceptance criteria from the ticket:

| AC | Requirement | Design Element |
|----|-------------|----------------|
| AC1 | Generator parses BOOST_DESCRIBE and classifies field types | RecordParser component |
| AC2 | Generated `record_bindings.cpp` compiles | PybindCodegen + integration test |
| AC3 | Existing pybind tests pass | Regression test suite |
| AC4 | Generator is idempotent | Unit test + file hashing |
| AC5 | Non-record bindings unaffected | Generator only writes `record_bindings.cpp` |
| AC6 | Build integration via CMake or manual | Open Question #1 (Option B recommended) |
| AC7 | AUTO-GENERATED header | PybindCodegen / PydanticCodegen file headers |
| AC8 | Generated `generated_models.py` contains leaf models | PydanticCodegen component |
| AC9 | Models include `Maps-to:` docstrings | PydanticCodegen._generate_model() |
| AC10 | Hand-written `models.py` imports generated models | Modified component: models.py |
| AC11 | Replay API tests pass | Regression test suite |
| AC12 | 0061 indexer parses `Maps-to:` annotations | Integration test with indexer |
| AC13 | Skill definition at `.claude/skills/sync-records/SKILL.md` | New component: /sync-records skill |
| AC14 | Skill regenerates both pybind and Pydantic | Skill workflow steps 2-3 |
| AC15 | Skill runs 0061 indexer | Skill workflow step 4 |
| AC16 | Skill reports drift in composites | Skill workflow step 5 |
| AC17 | Docs-updater runs record sync conditionally | Modified component: docs-updater agent |
| AC18 | Doc-sync-summary includes "Record Layer Sync" | docs-updater Phase 6.5 report format |

## Dependencies and Prerequisites

### External Tools
- tree-sitter (already in `scripts/.venv` from traceability indexer)
- tree-sitter-cpp (already in `scripts/.venv`)

### Existing Infrastructure
- `scripts/traceability/index_symbols.py` — Reference implementation for tree-sitter parsing
- `msd-transfer/src/Records.hpp` — Umbrella include defining source files
- Current `record_bindings.cpp` — Reference implementation for validation

### Workflow Integration
- `.claude/agents/docs-updater.md` — Requires modification for Phase 6.5
- Ticket 0061 indexer — Required for validation and drift reporting

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Tree-sitter parsing fails on unusual syntax | Low | High | Regression tests on all 28 existing records |
| Generated code has subtle API differences | Medium | High | Diff validation against current bindings, full test suite |
| Developers forget to regenerate after changes | Medium | Medium | docs-updater auto-sync + CI drift check |
| Name mapping conflicts (multiple records → same model) | Low | Medium | Configuration review, structural validation in follow-up |
| Performance regression (tree-sitter overhead) | Low (Option B) | Low | Manual generation, no build-time cost |

## Future Enhancements

Potential follow-up tickets:
1. **Database Binding Generation**: Extend generator to emit `database_bindings.cpp` (R3 deferred)
2. **Structural Equivalence Validation**: Verify records mapped to the same Pydantic class have identical field lists
3. **FastAPI Endpoint Generation**: Auto-generate REST endpoints for new record types
4. **GraphQL Schema Generation**: Emit GraphQL schema from record definitions
5. **TypeScript Type Generation**: Generate TypeScript interfaces for frontend consumption

---

## Design Review

**Reviewer**: Design Review Agent
**Date**: 2026-02-13
**Status**: APPROVED WITH NOTES
**Iteration**: 0 of 1 (no revision needed)

### Criteria Assessment

#### Architectural Fit
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Naming conventions | ✓ | Python script follows snake_case (RecordParser, PybindCodegen). Generated C++ uses project conventions (PascalCase for classes, camelCase for methods). |
| Namespace organization | ✓ | Generated bindings preserve existing `msd_transfer::` namespace. Python modules follow `replay.generated_models` pattern. |
| File structure | ✓ | Generator placed in `scripts/` alongside existing traceability tools. Skill in `.claude/skills/sync-records/` follows convention. |
| Dependency direction | ✓ | Generator reads from msd-transfer (source of truth), writes to msd-pybind and replay. No cycles. Respects layering: msd-transfer → msd-pybind → replay. |

**Overall**: ✓ PASS — Design follows project conventions and integrates cleanly with existing structure.

#### Python Tooling Quality
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Parsing robustness | ✓ | Uses tree-sitter (proven in traceability indexer) rather than fragile regex. Handles formatting variations. |
| Error handling | ✓ | Explicit ParseError on malformed headers. Non-zero exit on failures. Generator validates inputs before codegen. |
| Idempotency | ✓ | Designed for deterministic output. Acceptance criteria include idempotency test (AC4). |
| Type safety | ✓ | Uses dataclasses (RecordInfo, FieldInfo) and Enum (FieldType). Type hints throughout. Python 3.10+ union syntax (`RecordInfo \| None`). |
| Configuration management | ✓ | NAME_MAPPING dictionary is explicit, centralized, and reviewable. Escape hatch via skip list (Open Question #2, Option A recommended). |

**Overall**: ✓ PASS — Follows Python best practices, leverages proven parsing infrastructure, and includes proper error handling.

#### Feasibility Assessment
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Parsing complexity | ✓ | BOOST_DESCRIBE macros are well-formed. tree-sitter handles C++ syntax robustly. Reference implementation exists (`index_symbols.py`). |
| Code generation patterns | ✓ | Pybind patterns are mechanical and deterministic from field types. Four field types map to four binding patterns (Table in ticket R1). |
| Build integration | ✓ | Option B (manual generation + CI check) defers Python dependency to development time. CMake target optional for convenience. |
| Backward compatibility | ✓ | Generated bindings designed to be semantically equivalent to current hand-written code. Regression tests verify existing test suite passes (AC3, AC11). |
| Dependencies available | ✓ | tree-sitter and tree-sitter-cpp already in `scripts/.venv` from ticket 0061 infrastructure. No new external dependencies. |

**Overall**: ✓ PASS — Implementation is feasible with existing infrastructure. Incremental phasing reduces risk.

#### Testability Assessment
| Criterion | Pass/Fail | Notes |
|-----------|-----------|-------|
| Parser unit testable | ✓ | RecordParser can be tested in isolation with mock header files. Individual methods (`_classify_field_type`, `_parse_macro_args`) testable separately. |
| Codegen unit testable | ✓ | PybindCodegen and PydanticCodegen consume RecordInfo objects. Can test with hand-crafted RecordInfo fixtures without parsing. |
| Integration testable | ✓ | Generated C++ compiles (build test). Generated Python imports (import test). Existing test suites validate runtime behavior (AC3, AC11). |
| Validation against baseline | ✓ | Diff comparison against current `record_bindings.cpp` (regression test). Field-by-field equivalence check. |
| Skill testable | ✓ | `/sync-records` skill orchestrates components end-to-end. Can test drift reporting with mocked composite models. |

**Overall**: ✓ PASS — All components are testable in isolation and integration. Comprehensive test plan provided.

### Risks Identified

| ID | Risk Description | Category | Likelihood | Impact | Mitigation | Prototype? |
|----|------------------|----------|------------|--------|------------|------------|
| R1 | tree-sitter fails to parse edge-case C++ syntax (e.g., multi-line macros, unusual formatting) | Technical | Low | High | Regression tests on all 28 existing records. Parser error messages reference specific header/line for debugging. | No |
| R2 | Generated pybind bindings have subtle semantic differences (e.g., lifetime, reference semantics) | Technical | Medium | High | Diff validation against current bindings. Full test suite (AC3) must pass. Manual review of first generated output. | No |
| R3 | Developers forget to run `/sync-records` after msd-transfer changes | Maintenance | Medium | Medium | docs-updater agent auto-invokes (Phase 6.5). CI drift check fails if bindings out-of-date (Option B, Open Question #1). | No |
| R4 | NAME_MAPPING conflicts: multiple records map to same Pydantic class, but fields diverge over time | Maintenance | Low | Medium | Configuration reviewed in PR. Follow-up ticket for structural equivalence validation (Future Enhancement #2). | No |
| R5 | Performance regression from build-time tree-sitter parsing | Performance | Low (Option B) | Low | Option B (manual generation) eliminates build-time cost. Generator runs only when invoked by developer. | No |
| R6 | 0061 indexer fails to parse `Maps-to:` annotations correctly | Integration | Low | Medium | Integration test (AC12) validates parsing. `Maps-to:` format is simple, deterministic, and matches existing patterns. | No |

### Notes for Implementation

1. **Start with Phase 1 (Pybind Only)**: Validate tree-sitter parsing and codegen correctness before adding Pydantic layer. This isolates risks R1 and R2.

2. **Diff Validation First**: Before replacing `record_bindings.cpp`, generate it and run a line-by-line diff against the current manual version. Investigate any discrepancies (expected: whitespace, comment style). This addresses R2.

3. **Regression Test Baseline**: Capture current test suite output as baseline. After generator integration, assert identical results. This addresses R2.

4. **CI Drift Check**: Add a CI step that runs generator with `--check-only` flag and fails if generated files differ from committed versions. This addresses R3. Example:
   ```yaml
   - name: Check record layer drift
     run: |
       python scripts/generate_record_layers.py --check-only
       git diff --exit-code msd/msd-pybind/src/record_bindings.cpp replay/replay/generated_models.py
   ```

5. **NAME_MAPPING Documentation**: Add comments in the mapping dict explaining why certain records share Pydantic classes. This helps future maintainers and addresses R4.

6. **Error Messages**: Parser errors should include:
   - Header file path and line number
   - Snippet of problematic code
   - Expected vs. actual structure
   This addresses R1 by making debugging easier.

7. **Open Question Decisions**: The design defers three decisions to human input (Build Integration, Escape Hatch, Generated File Header). These are well-analyzed with clear recommendations. **Recommendation**: Accept Option B for Build Integration, Option A for Escape Hatch, Option B for Generated File Header.

8. **Database Binding Generation (R3)**: Deferring to follow-up ticket is correct. Focus on core pybind and Pydantic generation first. Database bindings are trivial patterns and can be added incrementally.

### Integration with Existing Workflow

| Integration Point | Status | Notes |
|-------------------|--------|-------|
| `.claude/agents/docs-updater.md` | Requires modification | Phase 6.5 conditional step is well-specified. Non-blocking advisory reporting is appropriate. |
| `/sync-records` skill | New component | Workflow steps are clear. Skill invokes both generator and 0061 indexer in sequence. |
| Ticket 0061 indexer | Dependency | `Maps-to:` annotations solve the heuristic matching problem. Integration test (AC12) validates end-to-end. |
| CI pipeline | Enhancement | Drift check required (not blocking if Option B chosen, but strongly recommended for preventing stale bindings). |

### Summary

This design is **APPROVED WITH NOTES**. The architecture is sound, leverages existing infrastructure (tree-sitter, traceability tools), and follows project conventions. The phased implementation plan reduces risk by validating each layer incrementally.

**Key Strengths**:
- Eliminates manual boilerplate for 28 records (~200 lines of pybind + 12 Pydantic models)
- Provides deterministic C++ → Python linkage (`Maps-to:` annotations)
- Integrates cleanly with docs-updater agent and workflow
- Comprehensive test plan with regression, unit, and integration coverage

**Notes for Implementation**:
1. Start with Phase 1 (pybind only) and validate diff against current bindings
2. Add CI drift check to prevent stale generated files
3. Document NAME_MAPPING rationale for maintainability
4. Accept recommended Open Question resolutions (Option B for build integration, Option A for escape hatch)

**Next Steps**:
1. Human review and approval of design
2. Proceed to implementation (no prototype needed — risks are low-to-medium and mitigated)
3. Phase 1: Core generator (pybind bindings)
4. Phase 2: Pydantic generation
5. Phase 3: Skill and docs-updater integration
6. Phase 4: CI drift check

No blocking issues identified. Design is ready for implementation.
