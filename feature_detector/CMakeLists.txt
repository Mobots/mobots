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
#include_directories(/opt/boost_1_51_0/)
rosbuild_add_boost_directories()
rosbuild_add_library(messageBridge src/MessageBridge.cpp)
rosbuild_add_library(featuresMatcher src/FeaturesMatcher.cpp src/test/draw.cpp src/gpc/gpc.c)
rosbuild_add_library(parallelMatcher src/ParallelMatcher.cpp)
target_link_libraries(parallelMatcher featuresMatcher)
rosbuild_link_boost(parallelMatcher thread system)


rosbuild_add_executable(feature_detector src/FeatureDetector.cpp src/FeaturesFinder.cpp)

##test files
rosbuild_add_executable(camera_calibration src/test/camera_calibration.cpp)
rosbuild_add_executable(removeDistortion src/test/removeDistortion.cpp)
rosbuild_add_executable(correctBrightness src/test/brightnessCorrection.cpp)
rosbuild_add_executable(testParallelMatcher src/test/testParallelMatcher.cpp)
rosbuild_add_executable(makeMatch src/test/makeMatch.cpp)
rosbuild_add_executable(testFeatures src/test/testFeatures.cpp)
rosbuild_add_executable(testPics src/test/testPics.cpp)
rosbuild_add_executable(imgtest src/test/imgtest.cpp)

target_link_libraries(feature_detector messageBridge)
target_link_libraries(testFeatures messageBridge featuresMatcher)
target_link_libraries(testParallelMatcher messageBridge parallelMatcher)
target_link_libraries(makeMatch messageBridge featuresMatcher)
