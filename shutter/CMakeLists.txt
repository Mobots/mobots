cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_boost_directories(/usr/local/boost_1_49_0)


ADD_LIBRARY(geometry STATIC IMPORTED)
if (CMAKE_SIZEOF_VOID_P EQUAL 8)  # 64-bit project
SET_TARGET_PROPERTIES(geometry PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libgeometry64.a) 
else()  # 32-bit project
SET_TARGET_PROPERTIES(geometry PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libgeometry32.a)  
endif()

EXEC_PROGRAM("rosstack find mobots" OUTPUT_VARIABLE MOBOT_PATH)
#include_directories(${MOBOT_PATH}/util/include/)
include_directories(../util/include/)
rosbuild_add_executable(shutter src/shutter.cpp)
rosbuild_add_executable(manualShutter src/manualShutter.cpp)
target_link_libraries(shutter geometry)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_find_ros_package(image_transport)

#rosbuild_add_boost_directories()

#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()
#rosbuild_add_boost_directories() 

