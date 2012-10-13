## This CMakeLists.txt file builds 3 sub-projects: image_store,
## image_map_display, and image_map_info.
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
## Setup the ROS libraries
rosbuild_init()
# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


## Setup the OGRE libraries for image_map_display
set(CMAKE_MODULE_PATH "/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
find_package(OGRE REQUIRED)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

rosbuild_add_library(${PROJECT_NAME}
    src/image_map_display.cpp
    src/image_map_visual.cpp
)
target_link_libraries(${PROJECT_NAME} ${OGRE_LIBRARIES})