#include <boost/describe.hpp>

#include "cpp_sqlite/sqlite_db/DBBaseTransferObject.hpp"

namespace msd_assets
{

struct VertexTO : public cpp_sqlite::BaseTransferObject
{
  float position[3];  // Position (x, y, z)
  float color[3];     // Color (r, g, b)
  float normal[3];    // Normal vector (x, y, z)
};

// Register the test class with boost::describe
BOOST_DESCRIBE_STRUCT(VertexTO,
                      (cpp_sqlite::BaseTransferObject),
                      (position, color, normal));

}  // namespace msd_assets