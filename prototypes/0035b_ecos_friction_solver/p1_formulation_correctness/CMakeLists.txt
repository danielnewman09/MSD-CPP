cmake_minimum_required(VERSION 3.16)
project(prototype_p1_formulation_correctness)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Find ECOS (assume it's installed via Conan in parent project)
# For prototype, we'll try to use the system ECOS or Conan-installed one
find_package(ecos QUIET CONFIG)

if(NOT ecos_FOUND)
    message(STATUS "ECOS not found via find_package. Attempting manual configuration...")
    # Try to find ECOS headers and library manually
    # This prototype assumes ECOS is built in the parent project's Conan cache
    # For now, we'll create a simplified version that doesn't actually call ECOS
    # but validates the formulation logic
    add_definitions(-DECOS_NOT_AVAILABLE)
endif()

add_executable(prototype main.cpp)

if(ecos_FOUND)
    target_link_libraries(prototype PRIVATE ecos::ecos)
    message(STATUS "Linking against ECOS library")
else()
    message(WARNING "ECOS not available - prototype will simulate ECOS behavior")
endif()
