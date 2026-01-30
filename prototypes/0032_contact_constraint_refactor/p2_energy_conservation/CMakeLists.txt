cmake_minimum_required(VERSION 3.16)
project(prototype_p2_energy_conservation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3 - try Conan first, then system
set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../../build/Debug/generators" ${CMAKE_PREFIX_PATH})
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

# Enable testing
enable_testing()

# Create executable
add_executable(energy_conservation_test
    main.cpp
)

target_link_libraries(energy_conservation_test
    PRIVATE
    Eigen3::Eigen
)

# Add test
add_test(NAME EnergyConservationTest COMMAND energy_conservation_test)
