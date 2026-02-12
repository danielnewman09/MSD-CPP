add_subdirectory(msd-utils)
add_subdirectory(msd-transfer)
add_subdirectory(msd-sim)
add_subdirectory(msd-assets)
add_subdirectory(msd-gui)
add_subdirectory(msd-exe)
add_subdirectory(msd-asset-gen)

# Optional Python bindings
if(ENABLE_PYBIND)
  add_subdirectory(msd-pybind)
endif()