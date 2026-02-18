# Ticket: 0056c_python_bindings
# Python bindings for MSD simulation recording database access

cmake_minimum_required(VERSION 3.22)

# Find Python and pybind11 (Python3 hints set in root CMakeLists.txt)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)

# Create Python module target
pybind11_add_module(msd_reader
    src/msd_bindings.cpp
    src/record_bindings.cpp
    src/database_bindings.cpp
    src/geometry_bindings.cpp
    src/asset_registry_bindings.cpp
    src/engine_bindings.cpp
)

# Link to msd_assets (provides AssetRegistry, transitively provides msd_transfer)
# Link to msd_sim (provides Engine for live simulation — ticket 0072a)
target_link_libraries(msd_reader PRIVATE
    msd_assets
    msd_sim
)

# Set output directory to build/{build_type}
set_target_properties(msd_reader PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}"
)

# Set C++20 standard
target_compile_features(msd_reader PRIVATE cxx_std_20)
