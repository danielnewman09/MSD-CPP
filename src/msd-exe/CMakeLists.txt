add_subdirectory(src)

set(EXE_SRC_FILES ${EXE_SRC_FILES} 
    src/main.cpp)


set(MSD_EXE_NAME msd_exe)
message(STATUS "EXE: ${MSD_ENV_DIR}")

# Executable for the main msd
add_executable(${MSD_EXE_NAME})

find_package(SFML CONFIG REQUIRED)

set_target_properties(${MSD_EXE_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(msd_exe PRIVATE 
    sfml-graphics
        msd_sim)


# Add all relevant include directories
target_include_directories(${MSD_EXE_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}  # Allows includes like #include "msd-sim/Agent/Agent.hpp"
    ${MSD_EXE_DIR}
    ${MSD_ENV_DIR}
)

# Add all sources (adjust as needed)
target_sources(${MSD_EXE_NAME} PRIVATE
    ${EXE_SRC_FILES}
)

target_compile_features(${MSD_EXE_NAME} PRIVATE cxx_std_17)

