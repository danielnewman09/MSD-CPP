add_library(msd-gui STATIC
    # List your .cpp files here
    gui.cpp
    gui.hpp
)

target_include_directories(msd-gui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../installs/SFML/include
)


