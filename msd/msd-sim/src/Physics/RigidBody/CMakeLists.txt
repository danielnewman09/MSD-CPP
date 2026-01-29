target_sources(${MSD_SIM_NAME} PRIVATE
    AssetEnvironment.cpp
    AssetInertial.cpp
    AssetPhysical.cpp
    ConvexHull.cpp
    InertialCalculations.cpp
    InertialState.cpp
)

set(
    PHYSICS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/AssetEnvironment.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/AssetInertial.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/AssetPhysical.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ConvexHull.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/InertialCalculations.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/InertialState.hpp
    PARENT_SCOPE
)
