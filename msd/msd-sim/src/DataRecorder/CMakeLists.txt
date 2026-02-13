# DataRecorder sources
# Ticket: 0038_simulation_data_recorder
# Ticket: 0057_contact_tangent_recording

target_sources(${MSD_SIM_NAME} PRIVATE
    DataRecorder.cpp
    DataRecorderVisitor.cpp
)

set(
    HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/DataRecorder.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DataRecorderVisitor.hpp
    PARENT_SCOPE
)
