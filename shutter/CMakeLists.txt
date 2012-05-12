cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(shutter src/shutter.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_find_ros_package(image_transport)


rosbuild_add_boost_directories()
#rosbuild_add_executable(shutter_node src/shutter.cpp)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
rosbuild_add_boost_directories() 

SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "/usr/local/boost_1_49_0")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/usr/local/boost_1_49_0/libs")
set(BOOST_ROOT /usr/local/boost_1_49_0)
set(Boost_INCLUDE_DIR /usr/local/boost_1_49_0)
set(BOOST_LIBRARYDIR /usr/local/boost_1_49_0/libs)
   set(Boost_USE_STATIC_LIBS   ON)
   set(Boost_USE_MULTITHREADED ON)
   find_package( Boost 1.49.0 COMPONENTS filesystem date_time graph )

   if(Boost_FOUND)
      include_directories(${Boost_INCLUDE_DIRS})
      add_executable(geo geometry.cpp geometry.h)
      target_link_libraries(geo ${Boost_LIBRARIES})
   endif()