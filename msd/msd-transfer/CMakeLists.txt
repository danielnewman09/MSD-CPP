set(MSD_TRANSFER_NAME msd_transfer)

# Header-only library for database transfer objects
add_library(${MSD_TRANSFER_NAME} INTERFACE)

# Find dependencies
find_package(cpp_sqlite REQUIRED)

# Link dependencies (INTERFACE - header-only)
target_link_libraries(${MSD_TRANSFER_NAME}
  INTERFACE
    cpp_sqlite::cpp_sqlite
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
