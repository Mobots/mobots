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

rosbuild_gensrv()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
# Run the moc pre-processor on the qt files
add_definitions(-DQT_NO_KEYWORDS)
set(qt_srcs
    src/image_map_panel.cpp
    src/image_map_info.cpp
    src/image_map_model.cpp)
set(qt_hdrs
    src/image_map_panel.h
    src/image_map_info.h
    src/image_map_model.h)
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})
QT4_WRAP_UI(uis_h src/image_map_info.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})


rosbuild_add_library(${PROJECT_NAME}
  ${uis_h} ${qt_srcs} ${qt_moc_srcs}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})