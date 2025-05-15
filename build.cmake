# build.cmake
# CMake build script for MSD project
# Run with: cmake -P build.cmake

# Function to execute a command and check status
function(execute_process_with_check)
  execute_process(${ARGN} RESULT_VARIABLE CMD_RESULT)
  if(NOT CMD_RESULT EQUAL 0)
    message(FATAL_ERROR "Command failed with exit code: ${CMD_RESULT}")
  endif()
endfunction()

# Set build directory relative to this script
get_filename_component(SCRIPT_DIR ${CMAKE_SCRIPT_MODE_FILE} DIRECTORY)

# Allow configuring build type from command line or default to Debug
if(NOT DEFINED BUILD_TYPE)
  set(BUILD_TYPE "Debug")
endif()
message(STATUS "Build type: ${BUILD_TYPE}")

# With cmake_layout(), Conan creates folders like build/Debug, build/Release
set(BUILD_DIR "${SCRIPT_DIR}/build")
set(BUILD_TYPE_DIR "${BUILD_DIR}/${BUILD_TYPE}")

# Create build directories if they don't exist
if(NOT EXISTS ${BUILD_DIR})
  file(MAKE_DIRECTORY ${BUILD_DIR})
endif()
if(NOT EXISTS ${BUILD_TYPE_DIR})
  file(MAKE_DIRECTORY ${BUILD_TYPE_DIR})
endif()

# Run Conan to install dependencies
message(STATUS "Running Conan to install dependencies...")
execute_process_with_check(
  COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPT_DIR}
  conan install . --output-folder=${SCRIPT_DIR} --build=missing -s build_type=${BUILD_TYPE}
)

# Check if GTest was installed by listing packages
message(STATUS "Checking installed Conan packages...")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E chdir ${SCRIPT_DIR}
  conan list "*gtest*"
  OUTPUT_VARIABLE GTEST_PACKAGES
  ERROR_VARIABLE GTEST_LIST_ERROR
)

if(GTEST_PACKAGES)
  message(STATUS "Found GTest packages: ${GTEST_PACKAGES}")
else()
  message(WARNING "No GTest packages found in Conan cache: ${GTEST_LIST_ERROR}")
endif()

# Find the toolchain file - check both potential locations
set(TOOLCHAIN_FILE "${BUILD_TYPE_DIR}/generators/conan_toolchain.cmake")
if(NOT EXISTS ${TOOLCHAIN_FILE})
  # Try alternative location
  set(TOOLCHAIN_FILE "${BUILD_DIR}/generators/conan_toolchain.cmake")
  if(NOT EXISTS ${TOOLCHAIN_FILE})
    message(FATAL_ERROR "Toolchain file not found in either:\n- ${BUILD_TYPE_DIR}/generators/conan_toolchain.cmake\n- ${BUILD_DIR}/generators/conan_toolchain.cmake\nConan generation might have failed.")
  endif()
endif()

# Run CMake configure step
message(STATUS "Configuring project...")
message(STATUS "Using toolchain file: ${TOOLCHAIN_FILE}")
execute_process_with_check(
  COMMAND ${CMAKE_COMMAND} -E chdir ${BUILD_TYPE_DIR}
  ${CMAKE_COMMAND} ${SCRIPT_DIR}
  -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}
  -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
)

# Run CMake build step
message(STATUS "Building project...")
execute_process_with_check(
  COMMAND ${CMAKE_COMMAND} -E chdir ${BUILD_TYPE_DIR}
  ${CMAKE_COMMAND} --build . --config ${BUILD_TYPE}
)

# Run tests if requested
if(RUN_TESTS)
  message(STATUS "Running tests...")
  execute_process_with_check(
    COMMAND ${CMAKE_COMMAND} -E chdir ${BUILD_TYPE_DIR}
    ctest -C ${BUILD_TYPE} --output-on-failure
  )
endif()

message(STATUS "Build completed successfully!")