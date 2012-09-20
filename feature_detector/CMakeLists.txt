cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#include_directories(include/)
rosbuild_add_library(messageBridge src/MessageBridge.cpp)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(feature_detector src/FeatureDetector.cpp src/FeaturesFinder.cpp)
target_link_libraries(feature_detector messageBridge)
rosbuild_add_executable(testFeatures src/ror.cpp src/qsrt_bsrch.cpp src/testFeatures.cpp src/FeaturesFinder.cpp src/FeaturesMatcher.cpp src/ror3.cpp src/draw.cpp src/ror2.cpp )
target_link_libraries(testFeatures messageBridge m)
rosbuild_add_executable(test2 src/test2.cpp)
#target_link_libraries(example ${PROJECT_NAME})
