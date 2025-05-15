# CMake diagnostic script to check GTest findability
# Save this as check_gtest.cmake and run with: cmake -P check_gtest.cmake

# Set build directory relative to this script
get_filename_component(SCRIPT_DIR ${CMAKE_SCRIPT_MODE_FILE} DIRECTORY)
set(BUILD_DIR "${SCRIPT_DIR}/build")
set(BUILD_TYPE "Release") # Or change to match your build type

# Potential locations for the toolchain file
set(TOOLCHAIN_FILE_1 "${BUILD_DIR}/${BUILD_TYPE}/generators/conan_toolchain.cmake")
set(TOOLCHAIN_FILE_2 "${BUILD_DIR}/generators/conan_toolchain.cmake")

# Check if toolchain files exist and report
if(EXISTS ${TOOLCHAIN_FILE_1})
  message(STATUS "Toolchain file found at: ${TOOLCHAIN_FILE_1}")
  set(TOOLCHAIN_FILE ${TOOLCHAIN_FILE_1})
else()
  message(STATUS "Toolchain file not found at: ${TOOLCHAIN_FILE_1}")
endif()

if(EXISTS ${TOOLCHAIN_FILE_2})
  message(STATUS "Toolchain file found at: ${TOOLCHAIN_FILE_2}")
  set(TOOLCHAIN_FILE ${TOOLCHAIN_FILE_2})
else()
  message(STATUS "Toolchain file not found at: ${TOOLCHAIN_FILE_2}")
endif()

if(NOT DEFINED TOOLCHAIN_FILE)
  message(FATAL_ERROR "No toolchain file found!")
endif()

# Print some details about the toolchain file
file(READ ${TOOLCHAIN_FILE} TOOLCHAIN_CONTENT)
message(STATUS "Toolchain file contents preview (first 300 chars):")
string(SUBSTRING "${TOOLCHAIN_CONTENT}" 0 300 TOOLCHAIN_PREVIEW)
message(STATUS "${TOOLCHAIN_PREVIEW}...")

# Check for GTest find module files
set(GTEST_CONFIG_1 "${BUILD_DIR}/${BUILD_TYPE}/generators/GTestConfig.cmake")
set(GTEST_CONFIG_2 "${BUILD_DIR}/generators/GTestConfig.cmake")

if(EXISTS ${GTEST_CONFIG_1})
  message(STATUS "GTestConfig.cmake found at: ${GTEST_CONFIG_1}")
else()
  message(WARNING "GTestConfig.cmake not found at: ${GTEST_CONFIG_1}")
endif()

if(EXISTS ${GTEST_CONFIG_2})
  message(STATUS "GTestConfig.cmake found at: ${GTEST_CONFIG_2}")
else()
  message(WARNING "GTestConfig.cmake not found at: ${GTEST_CONFIG_2}")
endif()

# Try to include the toolchain file and find GTest
include(${TOOLCHAIN_FILE})

# Now try to find GTest
find_package(GTest QUIET)
if(GTest_FOUND)
  message(STATUS "GTest found! GTest_DIR = ${GTest_DIR}")
else()
  message(WARNING "GTest not found after including toolchain file")
endif()

# Check CMAKE_MODULE_PATH
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

# Check CMAKE_PREFIX_PATH 
message(STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

message(STATUS "Diagnostic complete")