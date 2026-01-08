set(MSD_GUI_TEST_NAME msd_gui_test)

find_package(GTest REQUIRED)

add_executable(${MSD_GUI_TEST_NAME}
    ShaderTransformTest.cpp
    unit/shader_policy_test.cpp
)

target_link_libraries(${MSD_GUI_TEST_NAME}
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        msd_gui
)

target_include_directories(${MSD_GUI_TEST_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

include(GoogleTest)
gtest_discover_tests(${MSD_GUI_TEST_NAME})

# Add coverage target for msd-gui tests
if(ENABLE_COVERAGE)
  # Find required programs
  find_program(LCOV_PATH lcov)
  find_program(GENHTML_PATH genhtml)

  if(LCOV_PATH AND GENHTML_PATH)
    # Add custom target for coverage
    add_custom_target(coverage-msd-gui
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/coverage

      # Zero coverage counters
      COMMAND ${LCOV_PATH} --directory . --zerocounters

      # Run tests directly (allow test failures to continue coverage generation)
      COMMAND $<TARGET_FILE:${MSD_GUI_TEST_NAME}> || (exit 0)

      # Capture coverage data (ignore inconsistent errors from header-only functions)
      COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage.info --ignore-errors inconsistent,unsupported,format

      # Filter to only include project files (exclude system headers, test files, conan cache, and build artifacts)
      COMMAND ${LCOV_PATH} --extract coverage.info '${CMAKE_SOURCE_DIR}/*' --output-file coverage_project.info --ignore-errors inconsistent,unsupported,format
      COMMAND ${LCOV_PATH} --remove coverage_project.info '*/test/*' '*/build/*' --output-file coverage_filtered.info --ignore-errors inconsistent,unsupported,format,unused

      # Generate HTML report
      COMMAND ${GENHTML_PATH} coverage_filtered.info --output-directory ${CMAKE_BINARY_DIR}/coverage/msd-gui --ignore-errors inconsistent,unsupported,format,corrupt,category

      # Print summary
      COMMAND ${LCOV_PATH} --summary coverage_filtered.info --ignore-errors inconsistent,unsupported,format

      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Generating code coverage report for msd-gui tests"
      DEPENDS ${MSD_GUI_TEST_NAME}
    )

    message(STATUS "Coverage target 'coverage-msd-gui' added. HTML report will be in ${CMAKE_BINARY_DIR}/coverage/msd-gui")
  else()
    message(WARNING "lcov and/or genhtml not found. Coverage target will not be available.")
  endif()
endif()
