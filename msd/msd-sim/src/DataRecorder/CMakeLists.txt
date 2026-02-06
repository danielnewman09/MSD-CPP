# DataRecorder sources
# Ticket: 0038_simulation_data_recorder

target_sources(${MSD_SIM_NAME} PRIVATE
    DataRecorder.cpp
)

set(
    HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/DataRecorder.hpp
    PARENT_SCOPE
)
