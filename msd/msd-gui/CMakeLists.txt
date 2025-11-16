
set(MSD_GUI_NAME msd_gui)

find_package(SFML 3 COMPONENTS Graphics REQUIRED)

add_executable(${MSD_GUI_NAME})

set_target_properties(${MSD_GUI_NAME} PROPERTIES LINKER_LANGUAGE CXX)

find_package(spdlog REQUIRED)

target_link_libraries(${MSD_GUI_NAME}
 
  PRIVATE
    spdlog::spdlog
    msd_sim
    SFML::Graphics)


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

