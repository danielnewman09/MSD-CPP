target_sources(${MSD_ASSETS_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ConvexHull.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/STLLoader.cpp
)

set(v
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/CoordianteTO.hpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/ConvexHull.hpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryTO.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GeometryFactory.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/STLLoader.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/VertexTO.hpp
    PARENT_SCOPE
)
