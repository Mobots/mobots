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

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "mobots_msgs/Pose2DPrio.h"


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

double linear_vel = 0.1;
double angular_vel = 0.1;
double linear_vel_fast = 0.3;
double angular_vel_fast = 0.3;

geometry_msgs::Pose2D currentPosition;
ros::Publisher targetPose_pub;
ros::Subscriber globalPose_sub;
ros::NodeHandle* nh;

int kfd = 0;
struct termios cooked, raw;

void keyboardLoop();

void globalPoseCallback(const geometry_msgs::Pose2D& msg){
	currentPosition = msg;
}


int main(int argc, char** argv){

	
	cout << "specify a mobot to control (enter the mobot id): ";
	int mobotID;
	cin >> mobotID;
	cout << endl;
	std::stringstream namess;
	namess << "mobot_keyboard_teleop-" << mobotID;
	ros::init(argc, argv, namess.str(), ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle;
	string waypointPath;
	string globalPosePath;
	stringstream ss;
	ss << "/mobot" << mobotID << "/waypoint_rel";
	waypointPath = ss.str();
	/*ss.clear();
	ss.str("");
	ss << "/mobot" << mobotID << "/mouse";*/
	globalPosePath = ss.str();
  targetPose_pub = nh->advertise<mobots_msgs::Pose2DPrio>(ss.str(), 2);
	//globalPose_sub = nh->subscribe(globalPosePath, 2, globalPoseCallback);s
	cout << "paths: " << endl << waypointPath << endl;/* << globalPosePath << endl << endl;*/
	
  boost::thread t = boost::thread(keyboardLoop);
  ros::spin();
  
  t.interrupt(); //?
  t.join();
  tcsetattr(kfd, TCSANOW, &cooked);
  
  return(0);
}

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
    boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
    
    if((num = poll(&ufd, 1, 250)) < 0){
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
    mobots_msgs::Pose2DPrio pub_pose;
    pub_pose.pose.x = /*currentPosition.x + (heading)**/speed*max_tv;
    pub_pose.pose.y = /*currentPosition.y + (heading)**/0;
    pub_pose.pose.theta =/*currentPosition.theta +*/ turn*max_rv;
    pub_pose.prio = 0;
		//cout << pub_pose.pose.x << " and " << speed*max_tv << endl;
	 targetPose_pub.publish(pub_pose);
  }
}