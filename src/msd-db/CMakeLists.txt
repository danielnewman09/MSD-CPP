# src/msd-db/CMakeLists.txt

# Define our library name
set(MSD_DB_NAME msd_db)

# Add subdirectories
add_subdirectory(src)

# Create the library
add_library(${MSD_DB_NAME})

# Find required packages
find_package(SQLite3 REQUIRED)
find_package(spdlog REQUIRED)

# Set up include directories
target_include_directories(${MSD_DB_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link dependencies
target_link_libraries(${MSD_DB_NAME} 
  PUBLIC
    SQLite::SQLite3
    spdlog::spdlog
)

# Set language properties
set_target_properties(${MSD_DB_NAME} PROPERTIES 
  LINKER_LANGUAGE CXX
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# Add sources (pulled from the src/CMakeLists.txt)
target_sources(${MSD_DB_NAME} PRIVATE ${MSD_DB_SOURCES})

# Add include files (for installation purposes)
set(MSD_DB_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/msd-db/Database.hpp
)

# Add test directory if tests are enabled
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# Install rules
install(TARGETS ${MSD_DB_NAME}
  EXPORT ${MSD_DB_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(FILES ${MSD_DB_HEADERS}
  DESTINATION include/msd-db
)