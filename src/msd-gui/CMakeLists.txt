set(GUI_SRC_FILES) # Initialize SOURCE_FILES as empty
set(LIBRARY_DIR Gui) # Set LIBRARY_DIR to "Environment"

# Iterate over the list of source files in LIBRARY_DIR
foreach(FILE ${CMAKE_CURRENT_SOURCE_DIR}/${LIBRARY_DIR}/*.cpp)
  set(GUI_SRC_FILES "${GUI_SRC_FILES} ${FILE}") # Append FILE to SOURCE_FILES
endforeach()