#!/bin/sh
#export OGRE_RTT_MODE=Copy #bug workaround
trap "killall python; killall image_view; killall gazebo;" INT HUP TERM 

roscore &
roscore_pid=$(echo $!)
sleep 2

rosrun usb_cam usb_cam_node _video_device:="/dev/video1" _pixel_format:="yuyv"&
rosrun mobox feature_detector &
rosrun image_view image_view image:=/usb_cam/image_raw &
rosrun image_view image_view image:=/my_cam/featured &
roslaunch mobots_simulation mobots_world.launch &
rosrun gazebo spawn_model -file $(rospack find mobox)/models/mobot.urdf -urdf -model mobot -z 0.3
wait $roscore_pid
