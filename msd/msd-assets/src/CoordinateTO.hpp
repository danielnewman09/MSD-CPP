
#include <boost/describe.hpp>

#include "cpp_sqlite/sqlite_db/DBBaseTransferObject.hpp"

namespace msd_assets
{

struct CoordinateTO : public cpp_sqlite::BaseTransferObject
{
  double x;
  double y;
  double z;
};

// Register the test class with boost::describe
BOOST_DESCRIBE_STRUCT(CoordinateTO,
                      (cpp_sqlite::BaseTransferObject),
                      (x, y, z));

}  // namespace msd_assets