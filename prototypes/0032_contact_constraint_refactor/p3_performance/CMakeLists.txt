cmake_minimum_required(VERSION 3.16)
project(prototype_p3_performance)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find Eigen3 - match build type with Conan generators
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../../build/Release/generators")
else()
    list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../../build/Debug/generators")
endif()

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    # Fallback to system Eigen3 (macOS Homebrew location)
    set(EIGEN3_INCLUDE_DIR "/opt/homebrew/include/eigen3")
    if(EXISTS ${EIGEN3_INCLUDE_DIR})
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        target_include_directories(Eigen3::Eigen INTERFACE ${EIGEN3_INCLUDE_DIR})
        message(STATUS "Using system Eigen3 from ${EIGEN3_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Could not find Eigen3. Please run 'conan install' or install via Homebrew.")
    endif()
else()
    message(STATUS "Found Eigen3 via Conan")
endif()

# Create executable
add_executable(performance_test
    main.cpp
)

target_link_libraries(performance_test
    PRIVATE
    Eigen3::Eigen
)
