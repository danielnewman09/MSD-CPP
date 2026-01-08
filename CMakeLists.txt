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
  else()
    message(WARNING "Code coverage is only supported with GCC or Clang")
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
