#!/bin/sh
#export OGRE_RTT_MODE=Copy #bug workaround
trap "killall python; killall image_view; killall gazebo; \n" INT HUP TERM 

roscore &
roscore_pid=$(echo $!)
sleep 2

rosrun usb_cam usb_cam_node _video_device:="/dev/video0" _pixel_format:="yuyv" &
rosrun mobox feature_detector &
rosrun image_view image_view image:=/usb_cam/image_raw &
rosrun image_view image_view image:=/my_cam/featured &

wait $roscore_pid
