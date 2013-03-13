#!/bin/sh
 source /opt/ros/fuerte/setup.bash
 export ROS_ROOT=/opt/ros/fuerte/share/ros
 export PATH=$ROS_ROOT/bin:$PATH
 export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
 export ROS_PACKAGE_PATH="/home/moritz/Dokumente/UniBremen/Mobots/:$ROS_PACKAGE_PATH"
 export ROS_WORKSPACE="/home/moritz/Dokumente/UniBremen/Mobots/mobots/"
 export OGRE_DIR="/usr/share/OGRE/cmake/modules"
