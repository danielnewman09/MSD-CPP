
target_sources(${MSD_SIM_NAME} PRIVATE
    GJK.cpp
    EPA.cpp
    CollisionHandler.cpp
    CollisionPipeline.cpp
    VertexFaceDetector.cpp
    VertexFaceManifoldGenerator.cpp
)

set(
    PHYSICS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/GJK.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EPA.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CollisionHandler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CollisionResult.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CollisionPipeline.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/VertexFaceDetector.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/VertexFaceManifoldGenerator.hpp
    PARENT_SCOPE
)