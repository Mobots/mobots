/*
 * Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTA	L, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <pthread.h>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "mobots_msgs/Twist2D.h"
#include "path_planner/KeyboardRequest.h"


#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

#define KEYCODE_SPACE 0x20

using namespace std;

double linear_vel = 0.5;
double angular_vel = 0.5;
double linear_vel_fast = 1;
double angular_vel_fast = 1;
int mobotID;

ros::Publisher velocity_pub;
ros::ServiceClient client;
ros::NodeHandle* nh;

int kfd = 0;
struct termios cooked, raw;

void keyboardLoop();

void siginthandler(int param){
	path_planner::KeyboardRequest::Request req;
	path_planner::KeyboardRequest::Response res;
	req.mobot_id = mobotID;
	req.enable = false;
	if(!client.call(req, res))
		ROS_WARN("[mobot_keyboard_teleop] No path_planner found while detaching teleop");
	tcsetattr(kfd, TCSANOW, &cooked);
	mobots_msgs::Twist2D msg;
	msg.x = 100;
	msg.y = 100;
	msg.theta = 100;
	velocity_pub.publish(msg);
	exit(1);
}


int main(int argc, char** argv){
	cout << "specify a mobot to control (enter the mobot id): ";
	cin >> mobotID;
	cout << endl;
	std::stringstream namess;
	namess << "mobot_keyboard_teleop_" << mobotID;
	ros::init(argc, argv, namess.str(), ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle;
	string waypointPath;
	string globalPosePath;
	stringstream ss;
	ss << "/mobot" << mobotID << "/velocity";
	waypointPath = ss.str();
	/*ss.clear();
	ss.str("");
	ss << "/mobot" << mobotID << "/mouse";*/
	client = nh->serviceClient<path_planner::KeyboardRequest>("/path_planner/keyboard_request");
	path_planner::KeyboardRequest::Request req;
	path_planner::KeyboardRequest::Response res;
	req.mobot_id = mobotID;
	req.enable = true;
	if(client.call(req, res)){
		if(!res.enabled){
			cout << "path_planner does currently not allow keyboard controlling for this id" << endl;
			exit(1);
		}
	}else{
		ROS_WARN("[%s] No path_planner found while attaching teleop", namess.str().c_str());
	}
	globalPosePath = ss.str();
	velocity_pub = nh->advertise<mobots_msgs::Twist2D>(ss.str(), 2);
	cout << "paths: " << endl << waypointPath << endl;
	signal(SIGINT, siginthandler);
  //boost::thread t = boost::thread(keyboardLoop);
  //t.interrupt(); //?
  //t.join();
 //ros::spin();
    keyboardLoop();

  return(0);
}

mobots_msgs::Twist2D twist;
mobots_msgs::Twist2D lastTwist;

void keyboardLoop(){
  char c;
  double max_tv = linear_vel;
  double max_rv = angular_vel;
  bool dirty = false;
  double speed = 0;
  int turn = 0;
  
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  
  puts("Reading from keyboard");
  puts("Use W/S to drive forward/backward");
  puts("Use A/D to turn around");
  puts("Press Shift to move faster");
  //puts("Press space to shutter");
  
  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
  
  for(;;){
    //boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
		
    if((num = poll(&ufd, 1, 500)) < 0){
      perror("poll():");
      return;
    }
    else if(num > 0){
      if(read(kfd, &c, 1) < 0){
	perror("read():");
	return;
      }
    }
    else{
      if (dirty == true){
				mobots_msgs::Twist2D pub_pose;
				pub_pose.x = 0;
				pub_pose.y = 0;
				pub_pose.theta = 0;
				velocity_pub.publish(pub_pose);
				dirty = false;
      }
      continue;
    }
    //float heading = currentPosition.theta;
    switch(c){
	case KEYCODE_W:
	    max_tv = linear_vel;
	    speed = 1;
	    turn = 0;
	    dirty = true;
	    break;
	case KEYCODE_S:
	    max_tv = linear_vel;
	    speed = -1;
	    turn = 0;
	    dirty = true;
	    break;
	case KEYCODE_A:
	    max_rv = angular_vel;
	    speed = 0;
	    turn = 1;
	    dirty = true;
	    break;
	case KEYCODE_D:
	    max_rv = angular_vel;
	    speed = 0;
	    turn = -1;
	    dirty = true;
	    break;
	    
	case KEYCODE_W_CAP:
	    max_tv = linear_vel_fast;
	    speed = 1;
	    turn = 0;
	    dirty = true;
	    break;
	case KEYCODE_S_CAP:
	    max_tv = linear_vel_fast;
	    speed = -1;
	    turn = 0;
	    dirty = true;
	    break;
	case KEYCODE_A_CAP:
	    max_rv = angular_vel_fast;
	    speed = 0;
	    turn = 1;
	    dirty = true;
	    break;
	case KEYCODE_D_CAP:
	    max_rv = angular_vel_fast;
	    speed = 0;
	    turn = -1;
	    dirty = true;
	    break;
	/*case KEYCODE_SPACE:
	  imageHandler->shutterCallback();
	  speed = 0;
	  turn = 0;
	  break;*/
    }
    twist.x = speed*max_tv;
    twist.y = 0;
    twist.theta = turn*max_rv;
		if(twist.x == lastTwist.x && twist.y == lastTwist.y && twist.theta == lastTwist.theta)
			continue;
		velocity_pub.publish(twist);
		lastTwist = twist;
  }
}
