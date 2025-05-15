# src/msd-db/test/CMakeLists.txt

# Check if the root testing framework is available
if(NOT COMMAND add_msd_test)
  message(FATAL_ERROR "Test framework not found. Please include the top-level test/CMakeLists.txt first.")
endif()

# Database tests
add_msd_test(
  NAME db_unit_tests
  SOURCES 
    DatabaseTest.cpp
    # Add more test files here as you create them
  LIBS 
    ${MSD_DB_NAME}
  INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)