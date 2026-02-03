
add_subdirectory(RigidBody)
add_subdirectory(Constraints)
add_subdirectory(Integration)
add_subdirectory(PotentialEnergy)
add_subdirectory(Collision)

target_sources(${MSD_SIM_NAME} PRIVATE
    SupportFunction.cpp
)

set(
    PHYSICS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Facet.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SupportFunction.hpp
    PARENT_SCOPE
)