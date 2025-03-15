
set(MSD_GUI_NAME msd_gui)

find_package(SFML 2 CONFIG REQUIRED COMPONENTS graphics audio network)


add_executable(${MSD_GUI_NAME} ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${MSD_GUI_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${MSD_GUI_NAME} 
  PUBLIC
    sfml-graphics sfml-audio sfml-network
  PRIVATE 
    msd_sim )

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

