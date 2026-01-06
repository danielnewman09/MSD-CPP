# Ticket: 0004_gui_framerate
target_sources(${MSD_SIM_NAME} PRIVATE
    InputControlAgent.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/BaseAgent.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InputCommands.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InputControlAgent.hpp
# Add more headers here as needed
    PARENT_SCOPE
)
