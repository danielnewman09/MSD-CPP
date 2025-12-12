# Physics module CMakeLists.txt

# Add source files
target_sources(${MSD_SIM_NAME} PRIVATE
    ConvexHull.cpp
    PhysicalAsset.cpp
    RigidBodyProperties.cpp
    GJK.cpp
)

# Header files for installation
set(
    PHYSICS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/ConvexHull.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PhysicalAsset.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Force.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ForceAccumulator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RigidBodyProperties.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/GJK.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PhysicsState.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PhysicsUtils.hpp
    PARENT_SCOPE
)
