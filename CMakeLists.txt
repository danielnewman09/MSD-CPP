cmake_minimum_required(VERSION 3.15)
project(msd-exe VERSION 1.0 LANGUAGES CXX)

include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_SOURCE_DIR}/lib)

# Executable for the main msd
add_executable(msd-exe src/msd-exe/main.cpp)

# find_package(glfw3 3.4 REQUIRED)
find_package(SFML CONFIG REQUIRED)

# Library for msd-sim
add_library(msd src/msd-sim/msd-sim.cpp)

# Link the library to the executable
target_link_libraries(${PROJECT_NAME} sfml-graphics)
target_link_libraries(msd-exe sfml-graphics msd-sim)


# target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

install(TARGETS msd-exe DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Add tests
enable_testing()
add_subdirectory(test)
