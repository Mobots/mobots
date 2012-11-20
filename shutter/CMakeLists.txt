cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()
set(ROS_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

include_directories(/opt/boost_1_51_0)
#include_directories(/usr/local/boost_1_49_0)
rosbuild_add_boost_directories()

#ADD_LIBRARY(geometry STATIC IMPORTED)
#if (CMAKE_SIZEOF_VOID_P EQUAL 8)  # 64-bit project
#SET_TARGET_PROPERTIES(geometry PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libgeometry64.a)
#else()  # 32-bit project
#SET_TARGET_PROPERTIES(geometry PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libgeometry32.a)
#endif()
rosbuild_add_library(new_usb_cam_lib src/libusb_cam/usb_cam.cpp)
target_link_libraries(new_usb_cam_lib avcodec swscale)
rosbuild_add_executable(shutter src/shutter3.cpp src/geometry.cpp)
target_link_libraries(shutter new_usb_cam_lib)
rosbuild_add_executable(shutter4 src/shutter4.cpp src/geometry.cpp)
target_link_libraries(shutter4 new_usb_cam_lib)
rosbuild_add_executable(shutterold src/shutter.cpp src/shutter2.cpp src/geometry.cpp)
target_link_libraries(shutterold new_usb_cam_lib)
#rosbuild_add_executable(manualShutter src/manualShutter.cpp)
#rosbuild_link_boost(shutter geometry)

#rosbuild_find_ros_package(image_transport)

#rosbuild_add_boost_directories()

#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()
#rosbuild_add_boost_directories() 

