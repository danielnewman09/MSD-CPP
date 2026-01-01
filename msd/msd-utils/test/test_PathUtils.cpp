#include <iostream>
#include "msd-utils/src/PathUtils.hpp"

int main()
{
  try
  {
    // Test 1: Simple relative path
    auto path1 = msd_utils::absolutePath("data/config.json");
    std::cout << "Test 1 - Relative path 'data/config.json':\n  " << path1
              << "\n\n";

    // Test 2: Parent directory navigation
    auto path2 = msd_utils::absolutePath("../assets/model.obj");
    std::cout << "Test 2 - Parent directory '../assets/model.obj':\n  " << path2
              << "\n\n";

    // Test 3: Absolute path gets normalized
    auto path3 = msd_utils::absolutePath("./config/settings.ini");
    std::cout << "Test 3 - Current directory './config/settings.ini':\n  "
              << path3 << "\n\n";

    // Test 4: Show executable directory
    auto execDir = msd_utils::absolutePath(".");
    std::cout << "Test 4 - Executable directory '.':\n  " << execDir << "\n\n";

    std::cout << "All tests completed successfully!\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
