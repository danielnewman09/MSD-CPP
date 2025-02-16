add_subdirectory(Agent)
add_subdirectory(Algorithms)
add_subdirectory(Environment)
add_subdirectory(Logging)

set(MSD_SIM_NAME msd_sim)

add_library(${MSD_SIM_NAME} ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${MSD_SIM_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Add all relevant include directories
target_include_directories(${MSD_SIM_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}  # Allows includes like #include "msd-sim/Agent/Agent.hpp"
    ${MSD_ENV_DIR}
)

# Add all sources (adjust as needed)
target_sources(${MSD_SIM_NAME} PRIVATE
    ${ENV_SRC_FILES}
    ${AGENT_SRC_FILES}
)
