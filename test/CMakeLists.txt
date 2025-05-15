# test/CMakeLists.txt


# Create a test helper function to be used by all test directories
function(add_msd_test)

    # Find the GTest package for all tests
    find_package(GTest REQUIRED)
    include(GoogleTest)
  # Parse function arguments
  set(options "")
  set(oneValueArgs NAME)
  set(multiValueArgs SOURCES LIBS INCLUDES)
  cmake_parse_arguments(TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  
  # Create test executable
  add_executable(${TEST_NAME} ${TEST_SOURCES})
  
  # Set standard properties for all tests
  target_link_libraries(${TEST_NAME} 
    PRIVATE ${TEST_LIBS} GTest::gtest GTest::gtest_main
  )
  
  target_include_directories(${TEST_NAME} PRIVATE
    ${TEST_INCLUDES}
    ${CMAKE_SOURCE_DIR}/src  # Allow includes relative to src
  )
  
  # Set compile features
  target_compile_features(${TEST_NAME} PRIVATE cxx_std_17)
  
  # Add to CTest using gtest_discover_tests which will properly register tests
  gtest_discover_tests(${TEST_NAME}
    PROPERTIES
      TIMEOUT 300  # 5-minute timeout
      PROCESSORS 1 # How many processors the test needs
  )
endfunction()

# Include all subdirectories with integration tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/integration)
  add_subdirectory(integration)
endif()