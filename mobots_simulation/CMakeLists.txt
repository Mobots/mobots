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
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(gazebo_simul src/gazebo_simul.cpp ../feature_detector/src/FeaturesMatcher.cpp ../feature_detector/src/ror.cpp ../feature_detector/src/draw.cpp src/ImageHandler.cpp) ##ugly for now
rosbuild_add_executable(mobox_simul src/mobox_simul.cpp ../feature_detector/src/FeaturesMatcher.cpp ../feature_detector/src/ror.cpp ../feature_detector/src/FeaturesFinder.cpp ../feature_detector/src/draw.cpp src/ImageHandler.cpp)
#rosbuild_add_executable(keyboard src/keyboard.cpp ../feature_detector/src/FeaturesMatcher.cpp ../feature_detector/src/ror.cpp ../feature_detector/src/draw.cpp)
#rosbuild_add_executable(meh src/test.cpp ../feature_detector/src/FeaturesMatcher.cpp ../feature_detector/src/ror.cpp ../feature_detector/src/draw.cpp)
#rosbuild_add_compile_flags(mobox_simul -fstack-protector)
#target_link_libraries(gazebo_simul pthread)
#target_link_libraries(mobox_simul pthread)
target_link_libraries(mobox_simul pthread)
#target_link_libraries(mobox_simul mudflap)
