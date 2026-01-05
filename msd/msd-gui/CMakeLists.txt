
set(MSD_GUI_NAME msd_gui)

add_library(${MSD_GUI_NAME})

set_target_properties(${MSD_GUI_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Find required packages
find_package(spdlog REQUIRED)

# SDL3 handling: For Emscripten, SDL comes from Emscripten ports (USE_SDL=3)
# For native builds, we use Conan packages
if(NOT EMSCRIPTEN)
    find_package(SDL3 REQUIRED CONFIG)
    find_package(SDL3_image REQUIRED CONFIG)
    find_package(SDL3_mixer REQUIRED CONFIG)
    find_package(SDL3_ttf REQUIRED CONFIG)
endif()

add_subdirectory(src)

# Link libraries - conditionally include SDL packages for native builds
target_link_libraries(${MSD_GUI_NAME}
  PUBLIC
    msd_sim
    msd_assets
  PRIVATE
    spdlog::spdlog)

if(NOT EMSCRIPTEN)
    # Native builds: link SDL3 shared libraries from Conan
    target_link_libraries(${MSD_GUI_NAME}
      PUBLIC
        SDL3::SDL3-shared
        SDL3_image::SDL3_image-shared
        SDL3_mixer::SDL3_mixer-shared
        SDL3_ttf::SDL3_ttf-shared)
endif()

# Add include directories to support "msd-sim/src/..." includes
target_include_directories(${MSD_GUI_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:include>
)

# Collect all compiled shader files (including WGSL for WebGPU/Emscripten)
file(GLOB_RECURSE SHADER_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Content/Shaders/Compiled/*.msl"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Content/Shaders/Compiled/*.spv"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Content/Shaders/Compiled/*.dxil"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Content/Shaders/Compiled/*.wgsl"
)

# Create a custom target that always runs to copy Content folder
add_custom_target(CopyContent ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Content
        $<TARGET_FILE_DIR:${MSD_GUI_NAME}>/Content
    COMMENT "Copying Content folder to build directory"
    DEPENDS ${SHADER_FILES}
)

# Make sure CopyContent runs after the library is built
add_dependencies(CopyContent ${MSD_GUI_NAME})

# Installation rules
install(TARGETS ${MSD_GUI_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Install Content folder to installation directory
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/Content
        DESTINATION bin
        )

# Install headers using the variable
install(FILES ${HEADER_FILES}
        DESTINATION include/${MSD_SIM_NAME})

# Testing
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
