target_sources(${MSD_SIM_NAME} PRIVATE
    # AssetEnvironment.cpp
    AssetInertial.cpp
    AssetPhysical.hpp
    ConvexHull.cpp
    InertialCalculations.cpp
)

set(
    PHYSICS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/AssetEnvironment.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/AssetInertial.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/AssetPhysical.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ConvexHull.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DynamicState.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/InertialCalculations.hpp
    PARENT_SCOPE
)
