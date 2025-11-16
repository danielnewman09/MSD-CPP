add_subdirectory(Agent)
add_subdirectory(Algorithms)
add_subdirectory(Environment)


target_sources(${MSD_SIM_NAME} PRIVATE
    Engine.cpp
)

set(
    HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Engine.hpp
# Add more headers here as needed
    PARENT_SCOPE
)