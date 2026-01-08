# Ticket: 0004_gui_framerate
target_sources(${MSD_GUI_NAME} PRIVATE
  Camera3D.cpp
  InputState.cpp
  InputHandler.cpp
  SDLApp.cpp
  SDLUtils.cpp
  ShaderPolicy.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Camera3D.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InputState.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InputHandler.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SDLApp.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SDLGPUManager.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SDLUtils.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ShaderPolicy.hpp
# Add more headers here as needed
    PARENT_SCOPE
)
