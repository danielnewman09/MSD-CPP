set(MSD_TRANSFER_NAME msd_transfer)

# Header-only library for database transfer objects
add_library(${MSD_TRANSFER_NAME} INTERFACE)

# Find dependencies
# Use CONFIG mode to find Conan-generated packages instead of CMake's built-in FindSQLite3.cmake
find_package(cpp_sqlite REQUIRED CONFIG)

# Link dependencies (INTERFACE - header-only)
# Note: SQLite3 is explicitly linked because Conan CMakeDeps has issues
# generating include paths for the local sqlite3 recipe with Emscripten
target_link_libraries(${MSD_TRANSFER_NAME}
  INTERFACE
    cpp_sqlite::cpp_sqlite
    SQLite3::SQLite3
)

# Add include directories
target_include_directories(${MSD_TRANSFER_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>
)

# Installation rules
install(TARGETS ${MSD_TRANSFER_NAME}
  EXPORT ${MSD_TRANSFER_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
  DESTINATION include/msd-transfer
  FILES_MATCHING PATTERN "*.hpp"
)
