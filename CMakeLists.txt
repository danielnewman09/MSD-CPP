cmake_minimum_required(VERSION 3.15)

set(PROJECT_NAME msd)

project(msd VERSION 1.0 LANGUAGES CXX)

# Enable testing at the top level
enable_testing()

# Set compiler options
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add useful compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(/W4)
endif()

# Option for building tests
option(BUILD_TESTING "Build the testing tree" ON)

# Always include the test directory first to define testing functions
# This ensures the testing framework is available to all modules
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(msd)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
