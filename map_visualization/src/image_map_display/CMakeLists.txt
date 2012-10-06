## Setup the Qt libraries
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

# Run the moc pre-processor on the qt files
add_definitions(-DQT_NO_KEYWORDS)
set(qt_srcs
		mobots_info.cpp)
set(qt_hdrs
		mobots_info.h)
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})
QT4_WRAP_UI(uis_h srcmobots_info.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rosbuild_add_library(${PROJECT_NAME}
		image_map_display.cpp
		image_map_visual.cpp
)

target_link_libraries(${PROJECT_NAME} ${OGRE_LIBRARIES} ${QT_LIBRARIES})

rosbuild_add_executable(image_map_test image_map_test.cpp)
