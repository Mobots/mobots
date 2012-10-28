## This CMakeLists.txt file builds 3 sub-projects: image_store,
## image_map_display, and image_map_info.
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
## Setup the ROS libraries
rosbuild_init()
# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
# Run the moc pre-processor on the qt files
add_definitions(-DQT_NO_KEYWORDS)
set(qt_srcs
    src/main.cpp
    src/image_map_info.cpp
    src/image_map_model.cpp
    src/image_map_waypoint.cpp)
set(qt_hdrs
    src/image_map_info.h
    src/image_map_model.h
    src/image_map_waypoint.h)
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})
#QT4_WRAP_UI(uis_h src/image_map_info.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})


rosbuild_add_executable(image_map_info
    ${qt_srcs} ${qt_moc_srcs} #${uis_h}
)
target_link_libraries(image_map_info ${QT_LIBRARIES})

#rosbuild_link_boost(image_map_info bind)