
add_subdirectory(RigidBody)
add_subdirectory(Constraints)
add_subdirectory(Integration)
add_subdirectory(PotentialEnergy)

target_sources(${MSD_SIM_NAME} PRIVATE
    GJK.cpp
    EPA.cpp
    CollisionHandler.cpp
    SupportFunction.cpp
    CollisionResponse.cpp
)

set(
    PHYSICS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/GJK.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EPA.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Facet.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CollisionHandler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CollisionResult.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SupportFunction.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CollisionResponse.hpp
    PARENT_SCOPE
)