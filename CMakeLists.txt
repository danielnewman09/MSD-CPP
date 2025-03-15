cmake_minimum_required(VERSION 3.15)
project(msd VERSION 1.0 LANGUAGES CXX)

include(sourcelist.cmake)

add_subdirectory(src)   

include_directories(msd_exe ${MSD_EXE_DIR}../)

# find_package(glfw3 3.4 REQUIRED)

install(TARGETS ${MSD_EXE_NAME} DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

        