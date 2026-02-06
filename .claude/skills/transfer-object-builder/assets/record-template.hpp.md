# Transfer Record Header Template

Use this template when generating a new transfer record. Replace all `{PLACEHOLDERS}` with actual values.

---

```cpp
#ifndef MSD_TRANSFER_{RECORD_NAME_UPPER}_HPP
#define MSD_TRANSFER_{RECORD_NAME_UPPER}_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <boost/describe.hpp>
#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
// Include if foreign keys are needed:
// #include <cpp_sqlite/src/cpp_sqlite/DBForeignKey.hpp>

// Include headers for FK target types:
// #include "msd-transfer/src/{TargetRecord}.hpp"

namespace msd_transfer
{

/**
 * @brief {Brief description of what this record represents}
 *
 * {Longer description explaining the record's role, what domain object
 *  it maps to, and how it relates to other records.}
 */
struct {RecordName} : public cpp_sqlite::BaseTransferObject
{
  // --- Scalar fields ---
  // std::string name;
  // double mass{std::numeric_limits<double>::quiet_NaN()};
  // uint32_t count{0};

  // --- BLOB fields ---
  // std::vector<uint8_t> matrix_data;  // {N} bytes ({description})

  // --- Foreign key relationships ---
  // cpp_sqlite::ForeignKey<{TargetRecord}> target;
};

}  // namespace msd_transfer

// Register with Boost.Describe for cpp_sqlite ORM
BOOST_DESCRIBE_STRUCT(msd_transfer::{RecordName},
                      (cpp_sqlite::BaseTransferObject),
                      ({field1},
                       {field2},
                       {field3}));

#endif  // MSD_TRANSFER_{RECORD_NAME_UPPER}_HPP
```

---

## Placeholder Reference

| Placeholder | Description | Example |
|---|---|---|
| `{RECORD_NAME_UPPER}` | Header guard name, SCREAMING_SNAKE | `INERTIAL_STATE_RECORD` |
| `{RecordName}` | Struct name, PascalCase | `InertialStateRecord` |
| `{Brief description}` | One-line Doxygen summary | `Database record for rigid body state snapshots` |
| `{TargetRecord}` | FK target struct name | `MeshRecord` |
| `{field1}, {field2}` | All non-inherited fields for BOOST_DESCRIBE | `name, mass, inertia_tensor` |

## Rules

1. Header guard uses `MSD_TRANSFER_` prefix
2. `BOOST_DESCRIBE_STRUCT` goes OUTSIDE the namespace block
3. List ALL fields in `BOOST_DESCRIBE_STRUCT` except inherited `id`
4. Use `std::numeric_limits<double>::quiet_NaN()` for uninitialized doubles/floats
5. Use `{0}` for uninitialized integers
6. BLOB fields are always `std::vector<uint8_t>` with a comment noting expected size
7. Add a `_count` companion field for variable-length BLOBs