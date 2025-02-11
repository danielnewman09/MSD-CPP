include(sourcelist.cmake)

# add_library(environment SHARED ${ENV_SRC_FILES})
# set_target_properties(environment PROPERTIES
#     INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})

#     # Collect all include directories under msd-sim
# target_include_directories(msd_sim PUBLIC
# ${MSD_SIM_DIR}  # Ensures `#include "msd-sim/..."`
# )