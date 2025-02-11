cmake_minimum_required(VERSION 3.15)
project(msd-exe VERSION 1.0 LANGUAGES CXX)

add_subdirectory(src)   


set(MSD_SIM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MSD_ENV_DIR ${MSD_SIM_DIR}/Environment)
set(MSD_AGENT_DIR ${MSD_SIM_DIR}/Agent) 

include_directories(${MSD_SIM_DIR})


# find_package(glfw3 3.4 REQUIRED)
find_package(SFML CONFIG REQUIRED)

# Executable for the main msd
add_executable(msd-exe src/msd-exe/main.cpp)


# Link the library to the executable
target_link_libraries(${PROJECT_NAME} sfml-graphics)
target_link_libraries(msd-exe sfml-graphics msd_sim)



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
