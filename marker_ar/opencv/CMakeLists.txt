# SPECIFY THE MINIMUM VERSION OF CMAKE REQUIRED
cmake_minimum_required(VERSION 3.8.0)

# SPECIFY YOUR PROJECT NAME
PROJECT(Project_Name)

# ENABLE C++11
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# MAKE SURE OPENCV IS INSTALLED CORRECTLY
find_package( OpenCV REQUIRED )

# INCLUDE OPENCV DIRECTORIES
include_directories( ${OpenCV_INCLUDE_DIRS})

# MACRO TO COMPILE CPP FILES
# Do Not Edit
MACRO(add_example name)
  ADD_EXECUTABLE(${name} ${name}.cpp)
  TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS} )
ENDMACRO()

MACRO(add_dll name)
  add_library(${name}_dll SHARED ${name}.cpp)
  TARGET_LINK_LIBRARIES(${name}_dll ${OpenCV_LIBS} )
ENDMACRO()

# COMPILE CPP FILES USING THIS LINE
######## EDIT THE FILE NAMES ########
add_example(create_marker)
add_example(create_board)
add_example(calibrate_camera)
add_example(detect_markers)
add_example(aruco_unity)

add_dll(aruco_unity)