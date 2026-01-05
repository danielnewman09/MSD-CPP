
set(MSD_EXE_NAME msd_exe)

add_executable(${MSD_EXE_NAME})
set_target_properties(${MSD_EXE_NAME} PROPERTIES LINKER_LANGUAGE CXX)

find_package(spdlog REQUIRED)

target_link_libraries(${MSD_EXE_NAME}
    PRIVATE
      msd_sim
      msd_gui
      msd_utils
      spdlog::spdlog)

# Add all relevant include directories
target_include_directories(${MSD_EXE_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)

# Add all sources (adjust as needed)
target_sources(${MSD_EXE_NAME} PRIVATE
src/main.cpp
    )

# Emscripten-specific configuration for WebAssembly builds
if(EMSCRIPTEN)
    # Output HTML file
    set_target_properties(${MSD_EXE_NAME} PROPERTIES SUFFIX ".html")

    # Define paths for embedded assets
    set(ASSET_DB_PATH "${CMAKE_SOURCE_DIR}/build/Debug/debug/example_assets.db")
    set(SHADERS_PATH "${CMAKE_SOURCE_DIR}/msd/msd-gui/src/Content/Shaders/Compiled")
    set(SHELL_HTML_PATH "${CMAKE_SOURCE_DIR}/web/shell.html")

    # Embed assets into the WebAssembly virtual filesystem
    # --preload-file <host_path>@<virtual_path>
    target_link_options(${MSD_EXE_NAME} PRIVATE
        # Embed the asset database
        "SHELL:--preload-file ${ASSET_DB_PATH}@/assets/example_assets.db"
        # Embed compiled shaders
        "SHELL:--preload-file ${SHADERS_PATH}@/Content/Shaders/Compiled"
        # Use custom HTML shell template
        "SHELL:--shell-file ${SHELL_HTML_PATH}"
    )

    message(STATUS "Emscripten build configured for msd_exe")
    message(STATUS "  Asset DB: ${ASSET_DB_PATH}")
    message(STATUS "  Shaders: ${SHADERS_PATH}")
    message(STATUS "  Shell HTML: ${SHELL_HTML_PATH}")
endif()

