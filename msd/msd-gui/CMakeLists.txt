
set(MSD_GUI_NAME msd_gui)

add_executable(${MSD_GUI_NAME})

set_target_properties(${MSD_GUI_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Find all SDL3 packages
find_package(spdlog REQUIRED)
find_package(SDL3 REQUIRED CONFIG)
find_package(SDL3_image REQUIRED CONFIG)
find_package(SDL3_mixer REQUIRED CONFIG)
find_package(SDL3_ttf REQUIRED CONFIG)

target_link_libraries(${MSD_GUI_NAME}
  PRIVATE
    spdlog::spdlog
    msd_sim
    SDL3::SDL3-shared
    SDL3_image::SDL3_image-shared
    SDL3_mixer::SDL3_mixer-shared
    SDL3_ttf::SDL3_ttf-shared)


# Add all relevant include directories
target_include_directories(${MSD_GUI_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>  
)

# Add all sources (adjust as needed)
target_sources(${MSD_GUI_NAME} PRIVATE
src/gui.cpp
    )

