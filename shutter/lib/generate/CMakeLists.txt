cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_init()

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(Boost_DEBUG ON)
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "/usr/include/boost_1_49_0")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/usr/include/boost_1_49_0/libs")
set(BOOST_ROOT /usr/include/boost_1_49_0)
set(Boost_INCLUDE_DIR /usr/include/boost_1_49_0)
set(BOOST_LIBRARYDIR /usr/include/boost_1_49_0/libs)
    set(Boost_USE_STATIC_LIBS   ON)
    set(Boost_USE_MULTITHREADED ON)
    find_package(Boost 1.49.0 )

   if(Boost_FOUND)
      include_directories(${Boost_INCLUDE_DIRS})
	add_library(geometry geometry.cpp)
	target_link_libraries(geometry ${Boost_LIBRARIES})
	add_executable(aus ausgabe.cc)
	target_link_libraries(aus geometry)
   endif()




#ADD_LIBRARY(geometry STATIC IMPORTED)
#SET_TARGET_PROPERTIES(geometry PROPERTIES IMPORTED_LOCATION libgeometry.a)
#add_executable(aus ausgabe.cc)
#target_link_libraries(aus geometry)




#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_find_ros_package(image_transport)

#rosbuild_add_boost_directories()

#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()
#rosbuild_add_boost_directories() 

