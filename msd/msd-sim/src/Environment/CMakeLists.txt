target_sources(${MSD_SIM_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/WorldModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InertialState.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ReferenceFrame.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/Polyhedron.cpp
)

set(
    HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Angle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Coordinate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/EulerAngles.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InertialState.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ReferenceFrame.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/WorldModel.hpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/Polyhedron.hpp
# Add more headers here as needed
    PARENT_SCOPE
)
