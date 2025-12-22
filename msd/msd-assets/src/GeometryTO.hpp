#include "cpp_sqlite/sqlite_db/DBBaseTransferObject.hpp"

namespace msd_assets
{

// Test TransferObject class for demonstration
struct TestProduct : public cpp_sqlite::BaseTransferObject
{
  std::vector<uint8_t> vertexBlob;
};

// Register the test class with boost::describe
BOOST_DESCRIBE_STRUCT(TestProduct,
                      (cpp_sqlite::BaseTransferObject),
                      (name, price, quantity, in_stock, children));

}  // namespace msd_assets