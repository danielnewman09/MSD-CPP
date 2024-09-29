cmake_minimum_required(VERSION 3.15)
project(msd VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set the output directories for binaries
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Needs to go before we add the subdirectories
add_executable(msd-exe msd/src/msd-exe/main.cpp)

# add_library(msd-sim STATIC
#     # List your .cpp files here
#     simulation.cpp
#     simulation.hpp
# )

# Define the main project source directories
# add_subdirectory(msd/src/msd-sim)
# add_subdirectory(msd/src/msd-gui)
add_subdirectory(msd/src/msd-exe)

# Include tests
# enable_testing()
# add_subdirectory(msd/test/msd-sim)
# add_subdirectory(msd/test/msd-gui)

message(STATUS "Parsing Debug/Release...\n")

# Set up additional configurations
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DRELEASE)
endif()



# target_link_libraries(msd-gui
#     sfml-graphics
# )

# target_link_libraries(msd-exe
#     msd-gui
#     msd-sim)

install(TARGETS msd-exe DESTINATION "." 
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)