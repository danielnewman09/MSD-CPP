set(ENV_SRC_FILES ${ENV_SRC_FILES} 
    WorldModel.cpp
    Platform.cpp
    PlatformState.cpp
)

set(MSD_HPP_FILES ${MSD_HPP_FILES}
    ${CMAKE_CURRENT_SOURCE_DIR}/WorldModel.hpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PlatformState.hpp)
# add_library(environment SHARED ${ENV_SRC_FILES})
# set_target_properties(environment PROPERTIES
#     INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})

#     # Collect all include directories under msd-sim
# target_include_directories(msd_sim PUBLIC
# ${MSD_SIM_DIR}  # Ensures `#include "msd-sim/..."`
# )