## Setup the OGRE libraries for image_map_display
set(CMAKE_MODULE_PATH "/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
find_package(OGRE REQUIRED)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
# Run the moc pre-processor on the qt files
add_definitions(-DQT_NO_KEYWORDS)
set(qt_srcs
  image_map_display.cpp
  image_map_info.cpp)
set(qt_hdrs
  image_map_display.h
  image_map_info.h)
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})
QT4_WRAP_UI(uis_h image_map_info.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})


rosbuild_add_library(${PROJECT_NAME}
  image_map_visual.cpp
  ${uis_h} ${qt_srcs} ${qt_moc_srcs}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})
rosbuild_add_executable(image_map_test image_map_test.cpp)