
add_subdirectory(RigidBody)

target_sources(${MSD_SIM_NAME} PRIVATE
    GJK.cpp
)

set(
    PHYSICS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/GJK.hpp
    PARENT_SCOPE
)