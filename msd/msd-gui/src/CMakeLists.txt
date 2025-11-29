target_sources(${MSD_GUI_NAME} PRIVATE
  SDLApp.cpp
  SDLGPUManager.cpp
  SDLUtils.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/SDLApp.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SDLGPUManager.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SDLUtils.hpp
# Add more headers here as needed
    PARENT_SCOPE
)
