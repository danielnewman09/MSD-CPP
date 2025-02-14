cmake_minimum_required(VERSION 3.15)
project(msd VERSION 1.0 LANGUAGES CXX)

include(sourcelist.cmake)

add_subdirectory(src)   


include_directories(msd_sim ${MSD_SIM_DIR}../)
include_directories(msd_exe ${MSD_EXE_DIR}../)

# find_package(glfw3 3.4 REQUIRED)
find_package(SFML CONFIG REQUIRED)

# Link the library to the executable
# target_link_libraries(${PROJECT_NAME} sfml-graphics)

message(STATUS exe src ${EXE_SRC_FILES})

# target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics)
# target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

install(TARGETS ${MSD_EXE_NAME} DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Add tests
# enable_testing()
# add_subdirectory(test)
