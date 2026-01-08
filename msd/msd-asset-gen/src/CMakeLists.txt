# Add executable for asset generation
add_executable(generate_assets generate_assets.cpp)

# Link against required libraries
# Note: msd_transfer, cpp_sqlite, spdlog are provided transitively via msd_assets
target_link_libraries(generate_assets
    PRIVATE
        msd_assets
)

# Set C++20 standard
set_target_properties(generate_assets PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Add include directories
target_include_directories(generate_assets
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Installation
install(TARGETS generate_assets
    RUNTIME DESTINATION bin
)
