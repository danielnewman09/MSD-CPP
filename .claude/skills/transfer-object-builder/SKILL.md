---
name: transfer-object-builder
description: Build the database transfer interface for an arbitrary domain object. Given a target class, this skill analyzes its data members, generates a transfer record in msd-transfer, adds fromRecord/toRecord factory methods to the domain class, and generates round-trip GTest tests. Invoke with the fully-qualified class name and library location.
---

# Transfer Object Builder

Build the complete database persistence interface for a domain object: transfer record struct, factory methods, and round-trip tests.

## Invocation

The user provides:
- **Target class**: The domain class to expose to the database (e.g., `AssetInertial`, `InertialState`)
- **Target library**: Which MSD library it lives in (e.g., `msd-sim`, `msd-assets`)
- **Direction**: `read-only` (fromRecord only), `write-only` (toRecord only), or `both` (default: `both`)

## Workflow Overview

```
1. ANALYZE     -> Read the domain class, inventory data members
2. CLASSIFY    -> Map each member to a transfer field type
3. GENERATE    -> Create transfer record in msd-transfer/src/
4. INTEGRATE   -> Add fromRecord/toRecord to the domain class
5. TEST        -> Generate round-trip GTest tests
6. REGISTER    -> Update CMakeLists.txt files if needed
7. CHECKPOINT  -> Present summary for human review
```

## Step 1: Analyze the Domain Class

Read the target class header and implementation files. Extract:

- All data members (name, type, default value)
- Constructor parameters
- Existing factory methods (check for `fromRecord`, `fromObjectRecord`, etc.)
- Dependencies on other domain objects that already have transfer records
- Whether the class is movable/copyable

**Document findings as a member inventory table:**

```markdown
| Member | Type | Transfer Strategy | Notes |
|--------|------|-------------------|-------|
| mass_  | double | scalar | Direct mapping |
| name_  | std::string | scalar | Direct mapping |
| inertiaTensor_ | Eigen::Matrix3d | BLOB (72 bytes) | 9 doubles, memcpy |
| hull_  | const ConvexHull& | SKIP | Derived from mesh, not persisted |
| mesh_  | reference | FK<MeshRecord> | Already has transfer record |
```

## Step 2: Classify Members

Apply these rules to determine how each member maps to a transfer field:

### Classification Rules

| Domain Type | Transfer Type | Strategy |
|---|---|---|
| `double`, `float`, `uint32_t`, `int` | Same type | Direct scalar field |
| `std::string` | `std::string` | Direct scalar field |
| `bool` | `uint32_t` | Store as 0/1 integer (SQLite has no bool) |
| `msd_sim::Vector3D` | `std::vector<uint8_t>` | BLOB: 24 bytes (3 doubles) |
| `Eigen::Vector4d` | `std::vector<uint8_t>` | BLOB: 32 bytes (4 doubles) |
| `Eigen::Quaterniond` | `std::vector<uint8_t>` | BLOB: 32 bytes (4 doubles, w,x,y,z) |
| `Eigen::Matrix3d` | `std::vector<uint8_t>` | BLOB: 72 bytes (9 doubles) |
| `std::vector<msd_sim::Vector3D>` | `std::vector<uint8_t>` | BLOB: N*24 bytes + separate count field |
| `std::vector<T>` (POD) | `std::vector<uint8_t>` | BLOB: N*sizeof(T) + separate count field |
| Reference to class with existing record | `cpp_sqlite::ForeignKey<TRecord>` | FK reference |
| `std::optional<T>` | nullable field or optional FK | FK with `.isSet()` check |
| Computed/derived values | **SKIP** | Not persisted (recomputed from other data) |
| Const references to external objects | **SKIP** | Ownership belongs elsewhere |

### BLOB Serialization Convention

All BLOB fields use `std::memcpy` for serialization/deserialization:

```cpp
// Serialize Eigen::Matrix3d -> BLOB
record.inertia_tensor.resize(9 * sizeof(double));
std::memcpy(record.inertia_tensor.data(), matrix.data(), 9 * sizeof(double));

// Deserialize BLOB -> Eigen::Matrix3d
Eigen::Matrix3d matrix;
std::memcpy(matrix.data(), record.inertia_tensor.data(), 9 * sizeof(double));
```

### Skip Rules

Do NOT create transfer fields for:
- Members that are computed/derived from other persisted data (e.g., `inverseInertiaTensor_` computed from `inertiaTensor_`)
- Non-owning references to external objects (e.g., `const Database& db_`)
- Cached values that are recomputed on construction
- Runtime-only state (e.g., mutex, condition variable, thread handles)

**CHECKPOINT**: Present the classification table to the user. Ask:
- Are any members incorrectly classified?
- Should any skipped members be included?
- Are there additional fields not in the class that should be in the record?

## Step 3: Generate Transfer Record

Create the transfer record header in `msd-transfer/src/`.

### Naming Convention
- Domain class `Foo` -> transfer record `FooRecord`
- Domain class `AssetInertial` -> transfer record `InertialAssetRecord` (prefer DB-centric naming)
- Header file: `msd-transfer/src/{RecordName}.hpp`
- Header guard: `MSD_TRANSFER_{RECORD_NAME_UPPER}_HPP`

### File Structure

Use the template in `assets/record-template.hpp.md` as the starting point. Key rules:

1. Struct inherits from `cpp_sqlite::BaseTransferObject`
2. Include `<boost/describe.hpp>` and the cpp_sqlite headers
3. Use `<cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>` include path (NOT `sqlite_db/`)
4. Use `<cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>` for foreign keys
5. All scalar fields have sensible defaults (use project convention: `quiet_NaN()` for uninitialized floats/doubles, `0` for integers, `""` for strings)
6. BLOB fields default to empty `std::vector<uint8_t>`
7. `BOOST_DESCRIBE_STRUCT` macro placed OUTSIDE the namespace block, listing ALL fields except inherited `id`
8. Doxygen comment block on the struct explaining what it represents
9. Include any headers for ForeignKey target types

### BOOST_DESCRIBE_STRUCT Placement

```cpp
namespace msd_transfer {
struct FooRecord : public cpp_sqlite::BaseTransferObject { /* ... */ };
}  // namespace msd_transfer

// Register OUTSIDE namespace
BOOST_DESCRIBE_STRUCT(msd_transfer::FooRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (field1, field2, field3));
```

## Step 4: Integrate with Domain Class

Add `fromRecord` and/or `toRecord` methods to the domain class.

### fromRecord (Database -> Domain)

```cpp
// In header (.hpp):
static ClassName fromRecord(msd_transfer::FooRecord& record,
                            cpp_sqlite::Database& db);

// In implementation (.cpp):
ClassName ClassName::fromRecord(msd_transfer::FooRecord& record,
                                cpp_sqlite::Database& db)
{
  // 1. Resolve foreign keys
  //    record.someFK.resolve(db) returns std::optional<std::reference_wrapper<T>>

  // 2. Deserialize BLOBs
  //    Validate size before memcpy

  // 3. Construct domain object via private constructor or builder
  return ClassName{/* ... */};
}
```

Rules:
- `record` parameter is non-const reference (FK resolution mutates internal state)
- `db` parameter is non-const reference
- Validate BLOB sizes before `std::memcpy` — throw `std::runtime_error` on mismatch
- Handle optional FKs: check `record.fk.isSet()` before `record.fk.resolve(db)`
- Use brace initialization `{}` per project convention

### toRecord (Domain -> Database)

```cpp
// In header (.hpp):
msd_transfer::FooRecord toRecord() const;

// In implementation (.cpp):
msd_transfer::FooRecord ClassName::toRecord() const
{
  msd_transfer::FooRecord record;

  // 1. Map scalar fields
  record.name = name_;

  // 2. Serialize BLOBs
  record.matrix_data.resize(9 * sizeof(double));
  std::memcpy(record.matrix_data.data(), matrix_.data(), 9 * sizeof(double));

  // 3. Set foreign key IDs (if known)
  // record.meshFK.id = meshId_;  // Only if the domain object tracks the FK ID

  return record;
}
```

Rules:
- Method is `const` — does not modify the domain object
- Returns a new record by value
- Does NOT set `record.id` — that is assigned by the DAO on insert
- FK IDs are set only if the domain object has knowledge of the related record's ID

### Include Updates

Add necessary includes to the domain class header:
```cpp
#include "msd-transfer/src/FooRecord.hpp"
```

And in the implementation file:
```cpp
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
```

## Step 5: Generate Tests

Create a test file at `{library}/test/{ClassName}TransferTest.cpp`.

### Required Test Categories

#### 1. Round-trip Test (REQUIRED)

```cpp
TEST_F(FooTransferTest, RoundTrip_AllFields)
{
  // 1. Create domain object with known values
  // 2. Serialize: auto record = original.toRecord();
  // 3. Insert: dao.insert(record);
  // 4. Retrieve: auto retrieved = dao.selectById(record.id);
  // 5. Deserialize: auto restored = Foo::fromRecord(*retrieved, db);
  // 6. Compare ALL fields between original and restored
}
```

Field comparison rules:
- `double`: Use `EXPECT_DOUBLE_EQ` (exact match through round-trip)
- `float`: Use `EXPECT_FLOAT_EQ`
- `msd_sim::Vector3D`: Compare component-wise with `EXPECT_DOUBLE_EQ`
- `Eigen::Matrix3d`: Compare all 9 elements
- `Eigen::Quaterniond`: Compare w,x,y,z components
- `std::string`: Use `EXPECT_EQ`
- `uint32_t`: Use `EXPECT_EQ`

#### 2. Optional FK Test (REQUIRED if any FK fields exist)

```cpp
TEST_F(FooTransferTest, FromRecord_MissingOptionalFK)
{
  // Create record WITHOUT setting optional FK
  // Verify domain object handles absence gracefully
}
```

#### 3. BLOB Integrity Test (REQUIRED if any BLOB fields exist)

```cpp
TEST_F(FooTransferTest, BlobRoundTrip_Matrix)
{
  // Create object with specific matrix values
  // Round-trip through database
  // Verify all matrix elements match exactly
}
```

#### 4. Invalid BLOB Test (REQUIRED if any BLOB fields exist)

```cpp
TEST_F(FooTransferTest, FromRecord_InvalidBlobSize_Throws)
{
  // Create record with corrupted BLOB (wrong size)
  // Verify fromRecord throws std::runtime_error
}
```

#### 5. Nonexistent Record Test (RECOMMENDED)

```cpp
TEST_F(FooTransferTest, SelectNonexistent_ReturnsNullopt)
{
  auto result = dao.selectById(99999);
  EXPECT_FALSE(result.has_value());
}
```

### Test Fixture Pattern

```cpp
#include <gtest/gtest.h>
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
#include <filesystem>
#include <memory>
// ... domain and transfer includes ...

class FooTransferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testDbPath_ = "test_foo_transfer.db";
    if (std::filesystem::exists(testDbPath_))
    {
      std::filesystem::remove(testDbPath_);
    }
    auto& logger = cpp_sqlite::Logger::getInstance();
    db_ = std::make_unique<cpp_sqlite::Database>(
      testDbPath_, true, logger.getLogger());
  }

  void TearDown() override
  {
    db_.reset();
  }

  std::string testDbPath_;
  std::unique_ptr<cpp_sqlite::Database> db_;
};
```

### Test Naming Convention

Follow project convention: `TEST_F(ClassTransferTest, Category_Behavior)`

Ticket reference in test description where applicable:
```cpp
// Ticket: {ticket-name}
TEST_F(FooTransferTest, RoundTrip_AllFields)
```

## Step 6: Register in CMakeLists.txt

Check and update:

1. **msd-transfer CMakeLists.txt**: Add the new record header to the library's source list if headers are explicitly listed
2. **Domain library CMakeLists.txt**: Add test source file to test target
3. **Domain library CMakeLists.txt**: Verify `msd-transfer` is already a dependency (it should be)

Only modify CMakeLists.txt if the project explicitly lists source files. If it uses glob patterns, no change is needed.

## Step 7: Checkpoint

Present a summary to the user:

```markdown
## Transfer Object Builder Summary

### Target: ClassName (library-name)

### Files Created/Modified:
- [NEW] msd-transfer/src/FooRecord.hpp
- [MOD] library/src/ClassName.hpp (added fromRecord/toRecord declarations)
- [MOD] library/src/ClassName.cpp (added fromRecord/toRecord implementations)
- [NEW] library/test/ClassNameTransferTest.cpp

### Transfer Record Fields:
| Field | Type | Source Member |
|-------|------|---------------|
| ... | ... | ... |

### Skipped Members:
| Member | Reason |
|--------|--------|
| ... | ... |

### Tests Generated:
- RoundTrip_AllFields
- FromRecord_MissingOptionalFK
- BlobRoundTrip_Matrix
- FromRecord_InvalidBlobSize_Throws
- SelectNonexistent_ReturnsNullopt

### Build Verification:
Run: `cmake --build --preset debug-{library}-only`
Run: `cmake --build --preset debug-tests-only`
```

Wait for human approval before proceeding to build verification.

## Error Handling

### If the domain class already has transfer methods
- Report existing methods
- Ask user whether to replace, extend, or skip

### If a transfer record already exists for this class
- Report existing record
- Ask user whether to update it (add missing fields) or create a new one

### If the class has members of unknown type
- Flag them in the classification table as UNKNOWN
- Ask user for guidance before proceeding

## References

- See `references/transfer-pattern-guide.md` for the complete field type mapping and BLOB conventions
- See `assets/record-template.hpp.md` for the transfer record header template
- Existing exemplar: `msd-transfer/src/MeshRecord.hpp` + `msd-assets/src/Geometry.hpp` + `msd-assets/test/GeometryDatabaseTest.cpp`
