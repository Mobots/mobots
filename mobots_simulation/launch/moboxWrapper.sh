#!/bin/sh
#export OGRE_RTT_MODE=Copy #bug workaround
trap "killall python; killall image_view; killall gazebo;" INT HUP TERM 

roscore &
roscore_pid=$(echo $!)
sleep 2

rosrun usb_cam usb_cam_node _video_device:="/dev/video1" _pixel_format:="yuyv"&
rosrun feature_detector featureDetector &
rosrun image_view image_view image:=/usb_cam/image_raw &
xterm -e rosrun mobots_simulation mobox_simul _mousePath:="/dev/input/mouse2" &
xterm -e rosrun shutter shutter &
wait $roscore_pid
