cmake_minimum_required(VERSION 3.16)
project(prototype_p2_overhead)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find Eigen3 - try Conan first, then system
# Use Release build for release, Debug for debug
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../../build/Release/generators" ${CMAKE_PREFIX_PATH})
else()
    set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../../build/Debug/generators" ${CMAKE_PREFIX_PATH})
endif()
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    # Fallback to system Eigen3 (macOS Homebrew location)
    set(EIGEN3_INCLUDE_DIR "/opt/homebrew/include/eigen3")
    if(EXISTS ${EIGEN3_INCLUDE_DIR})
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        target_include_directories(Eigen3::Eigen INTERFACE ${EIGEN3_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "Could not find Eigen3")
    endif()
endif()

# Create executable
add_executable(constraint_overhead_bench
    main.cpp
)

target_link_libraries(constraint_overhead_bench
    PRIVATE
    Eigen3::Eigen
)
