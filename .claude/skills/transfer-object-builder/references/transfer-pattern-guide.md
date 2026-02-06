# Transfer Object Pattern Reference Guide

Quick reference for the MSD-CPP transfer object pattern. This document codifies the conventions established by the existing `MeshRecord`/`Geometry` and `ObjectRecord`/`Asset` pairs.

## Architecture Overview

```
Database (SQLite)
    |
    v
cpp_sqlite::DAO<T>          -- CRUD operations
    |
    v
msd_transfer::FooRecord     -- Transfer object (plain struct, no logic)
    |
    v
DomainClass::fromRecord()   -- Deserialization + FK resolution
DomainClass::toRecord()     -- Serialization
    |
    v
DomainClass                 -- Rich domain object (business logic, computed properties)
```

## Field Type Mapping

### Scalar Types (Direct Mapping)

| C++ Domain Type | Transfer Record Type | Default Value | Notes |
|---|---|---|---|
| `double` | `double` | `std::numeric_limits<double>::quiet_NaN()` | Project convention for uninitialized |
| `float` | `float` | `std::numeric_limits<float>::quiet_NaN()` | Project convention for uninitialized |
| `uint32_t` | `uint32_t` | `0` | |
| `int32_t` | `int32_t` | `0` | |
| `std::string` | `std::string` | `""` (empty) | |
| `bool` | `uint32_t` | `0` | SQLite has no native bool; use 0/1 |

### BLOB Types (Binary Serialization)

| C++ Domain Type | Transfer Type | Size (bytes) | Serialization |
|---|---|---|---|
| `msd_sim::Vector3D` | `std::vector<uint8_t>` | 24 | 3 doubles, `memcpy` |
| `Eigen::Vector4d` | `std::vector<uint8_t>` | 32 | 4 doubles, `memcpy` |
| `Eigen::Quaterniond` | `std::vector<uint8_t>` | 32 | 4 doubles (w,x,y,z), `memcpy` via `.coeffs()` |
| `Eigen::Matrix3d` | `std::vector<uint8_t>` | 72 | 9 doubles, `memcpy` via `.data()` |
| `Eigen::Matrix4d` | `std::vector<uint8_t>` | 128 | 16 doubles, `memcpy` via `.data()` |
| `std::vector<msd_sim::Vector3D>` | `std::vector<uint8_t>` | N*24 | + `uint32_t count` field |
| `std::vector<Vertex>` | `std::vector<uint8_t>` | N*36 | + `uint32_t count` field |
| `float[3]` | `std::vector<uint8_t>` | 12 | 3 floats, `memcpy` |

### Relationship Types

| C++ Domain Type | Transfer Type | Pattern |
|---|---|---|
| Reference to class with record | `cpp_sqlite::ForeignKey<TRecord>` | Set `.id`, resolve with `.resolve(db)` |
| `std::optional<T>` where T has record | `cpp_sqlite::ForeignKey<TRecord>` | Check `.isSet()` before `.resolve()` |
| Owned sub-object without record | Inline fields | Flatten into parent record |

### Skip Rules (Do NOT Persist)

| Pattern | Reason |
|---|---|
| Inverse/derived matrices | Recomputed from source data |
| Cached computed values | Recomputed on construction |
| Non-owning references (`const T&`) | Ownership belongs elsewhere |
| Runtime state (mutexes, threads) | Not serializable |
| Bounding boxes from vertex data | Recomputed from geometry |

## BLOB Serialization Patterns

### Eigen::Matrix3d

```cpp
// Serialize (domain -> record)
record.matrix_blob.resize(9 * sizeof(double));
std::memcpy(record.matrix_blob.data(), matrix.data(), 9 * sizeof(double));

// Deserialize (record -> domain)
Eigen::Matrix3d matrix;
if (record.matrix_blob.size() != 9 * sizeof(double)) {
  throw std::runtime_error("Invalid matrix BLOB size");
}
std::memcpy(matrix.data(), record.matrix_blob.data(), 9 * sizeof(double));
```

### Eigen::Quaterniond

```cpp
// Serialize (domain -> record)
record.quat_blob.resize(4 * sizeof(double));
std::memcpy(record.quat_blob.data(), quat.coeffs().data(), 4 * sizeof(double));

// Deserialize (record -> domain)
Eigen::Quaterniond quat;
if (record.quat_blob.size() != 4 * sizeof(double)) {
  throw std::runtime_error("Invalid quaternion BLOB size");
}
std::memcpy(quat.coeffs().data(), record.quat_blob.data(), 4 * sizeof(double));
```

**Eigen::Quaterniond storage order**: `.coeffs()` returns `[x, y, z, w]` (NOT `[w, x, y, z]`). This is Eigen's internal convention. The BLOB preserves this order, so round-trips are exact.

### msd_sim::Vector3D

```cpp
// Serialize (domain -> record)
record.vec_blob.resize(3 * sizeof(double));
std::memcpy(record.vec_blob.data(), vec.data(), 3 * sizeof(double));

// Deserialize (record -> domain)
msd_sim::Vector3D vec;
if (record.vec_blob.size() != 3 * sizeof(double)) {
  throw std::runtime_error("Invalid vector BLOB size");
}
std::memcpy(vec.data(), record.vec_blob.data(), 3 * sizeof(double));
```

### Variable-Length Arrays (e.g., `std::vector<msd_sim::Vector3D>`)

```cpp
// Serialize
const size_t blobSize = vertices.size() * sizeof(msd_sim::Vector3D);
record.vertex_data.resize(blobSize);
std::memcpy(record.vertex_data.data(), vertices.data(), blobSize);
record.vertex_count = static_cast<uint32_t>(vertices.size());

// Deserialize
if (record.vertex_data.size() % sizeof(msd_sim::Vector3D) != 0) {
  throw std::runtime_error("Invalid vertex BLOB size");
}
const size_t count = record.vertex_data.size() / sizeof(msd_sim::Vector3D);
std::vector<msd_sim::Vector3D> vertices(count);
std::memcpy(vertices.data(), record.vertex_data.data(), record.vertex_data.size());
```

## Foreign Key Patterns

### Setting FK on Record Creation

```cpp
msd_transfer::ObjectRecord record;
record.meshRecord.id = meshRecordId;  // Set FK by ID
```

### Resolving FK on Record Loading

```cpp
if (record.meshRecord.isSet()) {
  auto resolved = record.meshRecord.resolve(db);
  if (resolved) {
    // resolved is std::optional<std::reference_wrapper<MeshRecord>>
    auto& meshRecord = resolved->get();
    // Use meshRecord...
  }
}
```

### Optional vs Required FKs

- **Required FK**: Always check `.isSet()` and throw if missing
- **Optional FK**: Check `.isSet()`, construct with `std::nullopt` if absent

```cpp
// Optional FK pattern (from Asset::fromObjectRecord)
std::optional<VisualGeometry> visualGeom = std::nullopt;
if (record.meshRecord.isSet()) {
  visualGeom = VisualGeometry{record.meshRecord.resolve(db)->get(), record.id};
}
```

## Include Path Convention

Use these include paths for cpp_sqlite headers:

```cpp
// Transfer record headers:
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

// Domain class implementations (for DB operations):
#include <cpp_sqlite/src/cpp_sqlite/DBDataAccessObject.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBDatabase.hpp>
```

## Naming Conventions

| Item | Convention | Example |
|---|---|---|
| Transfer record struct | `{Name}Record` | `PhysicsTemplateRecord` |
| Transfer record header | `{Name}Record.hpp` | `PhysicsTemplateRecord.hpp` |
| Header guard | `MSD_TRANSFER_{NAME}_RECORD_HPP` | `MSD_TRANSFER_PHYSICS_TEMPLATE_RECORD_HPP` |
| Namespace | `msd_transfer` | Always |
| Record fields | `snake_case` | `linear_damping`, `vertex_count` |
| BLOB fields | `snake_case` descriptive | `inertia_tensor`, `vertex_data` |
| Count companion fields | `{blob_name}_count` or `{noun}_count` | `vertex_count` |
| Factory method (read) | `fromRecord` or `from{RecordName}` | `fromObjectRecord` |
| Factory method (write) | `toRecord` or `populate{RecordName}` | `populateMeshRecord` |
| Test file | `{ClassName}TransferTest.cpp` | `AssetInertialTransferTest.cpp` |
| Test fixture | `{ClassName}TransferTest` | `AssetInertialTransferTest` |

## Test Comparison Helpers

```cpp
// Compare msd_sim::Vector3D components
void expectVector3dEq(const msd_sim::Vector3D& a, const msd_sim::Vector3D& b) {
  EXPECT_DOUBLE_EQ(a.x(), b.x());
  EXPECT_DOUBLE_EQ(a.y(), b.y());
  EXPECT_DOUBLE_EQ(a.z(), b.z());
}

// Compare Eigen::Quaterniond components
void expectQuaterniondEq(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  EXPECT_DOUBLE_EQ(a.w(), b.w());
  EXPECT_DOUBLE_EQ(a.x(), b.x());
  EXPECT_DOUBLE_EQ(a.y(), b.y());
  EXPECT_DOUBLE_EQ(a.z(), b.z());
}

// Compare Eigen::Matrix3d (all 9 elements)
void expectMatrix3dEq(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(a(i, j), b(i, j));
    }
  }
}
```

## Existing Exemplars

These are the canonical reference implementations:

| Transfer Object | Domain Object | Location |
|---|---|---|
| `MeshRecord` | `BaseGeometry<T>` | `msd-transfer/src/MeshRecord.hpp` -> `msd-assets/src/Geometry.hpp` |
| `ObjectRecord` | `Asset` | `msd-transfer/src/MeshRecord.hpp` -> `msd-assets/src/Asset.hpp` |
| `PhysicsTemplateRecord` | *(not yet consumed)* | `msd-transfer/src/PhysicsTemplateRecord.hpp` |
| `MaterialRecord` | *(not yet consumed)* | `msd-transfer/src/MaterialRecord.hpp` |