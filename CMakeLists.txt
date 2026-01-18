cmake_minimum_required(VERSION 3.15)

set(PROJECT_NAME msd)

project(msd VERSION 1.0 LANGUAGES CXX)

# Enable testing at the top level
enable_testing()

# Set compiler options
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(BUILD_TESTING "Build the testing tree" ON)
option(ENABLE_COVERAGE "Enable code coverage" OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
option(ENABLE_BENCHMARKS "Build performance benchmarks" OFF)
option(ENABLE_PROFILING "Enable profiling support (macOS only)" OFF)

# Enable warnings as errors for Release builds by default
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(WARNINGS_AS_ERRORS_DEFAULT ON)
else()
  set(WARNINGS_AS_ERRORS_DEFAULT OFF)
endif()
option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ${WARNINGS_AS_ERRORS_DEFAULT})

# Add useful compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion        # Warn on implicit type conversions that may lose data
    -Wshadow            # Warn when a local variable shadows another
    -Wnon-virtual-dtor  # Warn on classes with virtual functions but non-virtual destructors
    -Wold-style-cast    # Warn on C-style casts
    -Wnull-dereference  # Warn if null pointer dereference is detected
    -Wdouble-promotion  # Warn when float is implicitly promoted to double
    -Wimplicit-float-conversion  # Warn when double is implicitly converted to float
  )
elseif(MSVC)
  add_compile_options(/W4)
endif()

# Warnings as errors configuration
if(WARNINGS_AS_ERRORS)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Werror)
    message(STATUS "Warnings as errors enabled (-Werror)")
  elseif(MSVC)
    add_compile_options(/WX)
    message(STATUS "Warnings as errors enabled (/WX)")
  endif()
endif()

# Coverage configuration
if(ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Add coverage flags
    add_compile_options(--coverage -O0 -fno-inline -fno-elide-constructors)
    add_link_options(--coverage)
    message(STATUS "Code coverage enabled")

    # Find required programs for coverage reporting
    find_program(LCOV_PATH lcov)
    find_program(GENHTML_PATH genhtml)

    if(LCOV_PATH AND GENHTML_PATH)
      # Add project-wide coverage target that runs all tests
      add_custom_target(coverage
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/coverage

        # Zero coverage counters
        COMMAND ${LCOV_PATH} --directory . --zerocounters

        # Capture baseline (all instrumented files with 0% coverage) BEFORE running tests
        COMMAND ${LCOV_PATH} --directory . --capture --initial --output-file coverage_baseline.info --ignore-errors inconsistent,unsupported,format,gcov,source,range --filter range

        # Run all tests via ctest (allow test failures to continue coverage generation)
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure || (exit 0)

        # Capture coverage data after tests (ignore inconsistent errors from header-only functions)
        COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage_test.info --ignore-errors inconsistent,unsupported,format,gcov,source,range --filter range

        # Merge baseline and test coverage (baseline ensures untested files appear with 0%)
        COMMAND ${LCOV_PATH} --add-tracefile coverage_baseline.info --add-tracefile coverage_test.info --output-file coverage.info --ignore-errors inconsistent,unsupported,format,source,range

        # Filter to only include project files (exclude system headers, test files, conan cache, and build artifacts)
        COMMAND ${LCOV_PATH} --extract coverage.info '${CMAKE_SOURCE_DIR}/*' --output-file coverage_project.info --ignore-errors inconsistent,unsupported,format
        COMMAND ${LCOV_PATH} --remove coverage_project.info '*/test/*' '*/build/*' '*/bench/*' --output-file coverage_filtered.info --ignore-errors inconsistent,unsupported,format,unused

        # Generate HTML report (filter missing files and out-of-range lines from stale gcov data)
        COMMAND ${GENHTML_PATH} coverage_filtered.info --output-directory ${CMAKE_BINARY_DIR}/coverage --ignore-errors inconsistent,unsupported,format,corrupt,category,source,range --filter missing,range

        # Print summary
        COMMAND ${LCOV_PATH} --summary coverage_filtered.info --ignore-errors inconsistent,unsupported,format

        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating project-wide code coverage report"
      )

      message(STATUS "Coverage target 'coverage' added. HTML report will be in ${CMAKE_BINARY_DIR}/coverage")
    else()
      message(WARNING "lcov and/or genhtml not found. Coverage target will not be available.")
    endif()
  else()
    message(WARNING "Code coverage is only supported with GCC or Clang")
  endif()
endif()

# Profiling configuration (macOS only)
if(ENABLE_PROFILING)
  if(APPLE)
    # Add debug symbols with Release optimizations
    # -g: Generate debug symbols for Instruments
    # -O2: Release-level optimizations for realistic performance
    # Keep assertions enabled (no -DNDEBUG)
    add_compile_options(-g -O2)
    message(STATUS "Profiling support enabled (debug symbols + optimizations)")
  else()
    message(WARNING "Profiling support is only available on macOS (Xcode Instruments)")
  endif()
endif()

# Clang-tidy configuration
if(ENABLE_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
  if(CLANG_TIDY_EXE)
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  else()
    message(WARNING "clang-tidy requested but not found")
  endif()
endif()

# Always include the test directory first to define testing functions
# This ensures the testing framework is available to all modules
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(msd)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
