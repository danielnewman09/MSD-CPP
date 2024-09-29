

target_include_directories(msd-sim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../installs/eigen/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../installs/SFML/include
)

target_link_libraries(msd-sim
    ${CONAN_LIBS}  # Link with Conan libraries
)
