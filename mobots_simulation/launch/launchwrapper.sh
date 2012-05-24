#!/bin/sh
export OGRE_RTT_MODE=Copy #bug workaround
trap "killall python; killall image_view; killall gazebo;" INT HUP TERM 
roscore &
sleep 2
#rosrun image_view image_view image:=/my_cam/image &
#rosrun image_view image_view image:=/my_cam/featured &
xterm -e rosrun mobots_simulation keyboard _linear_vel:=0.05 _linear_vel_fast_vel:=0.1 _angular_vel:=0.2 &
xterm -e rosrun shutter shutter &
#rosrun mobots_simulation mobots_teleop_bridge &
#rosrun mobots_simulation image_worker &
#rosparam load $(rospack find mobots_simulation)/models/mobot.urdf mobot
roslaunch mobots_simulation mobots_world.launch &
gazebo_pid=$(echo $!)
sleep 2
rosrun gazebo spawn_model -file $(rospack find mobots_simulation)/models/mobot.urdf -urdf -model mobot -z 0.4
wait $gazebo_pid
