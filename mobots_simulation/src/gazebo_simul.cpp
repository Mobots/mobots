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
#include "geometry_msgs/Pose2D.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <opencv2/core/core.hpp>
#include "ImageHandler.h"


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
using namespace cv;

gazebo_msgs::GetModelState getStateRequest;
gazebo_msgs::ModelState modelState;
gazebo_msgs::SetModelState setModelStateRequest;
ros::ServiceClient setClient;
ros::ServiceClient getClient;
Ptr<ImageHandler> imageHandler;

double linear_vel;
double angular_vel;
double linear_vel_fast;
double angular_vel_fast;

int kfd = 0;
struct termios cooked, raw;

void keyboardLoop();

// --- DeltaPose Node --- 
geometry_msgs::Pose2D lastPose;
geometry_msgs::Pose2D deltaPose;
ros::Publisher posePublisher;

void* deltaPoseThread(void* data){
  ros::Rate rate(50); //50Hz
  while(ros::ok()){
    getClient.call(getStateRequest);
    double theta  = 2*acos(getStateRequest.response.pose.orientation.w);
    deltaPose.x = lastPose.x - getStateRequest.response.pose.position.x;
    deltaPose.y = lastPose.y - getStateRequest.response.pose.position.y;
    deltaPose.theta = lastPose.theta - theta;
    posePublisher.publish(deltaPose);
    lastPose.x = getStateRequest.response.pose.position.x;
    lastPose.y = getStateRequest.response.pose.position.y;
    lastPose.theta = theta;
    rate.sleep();
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "MobotKeyboardController", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  
  ros::NodeHandle handle("~");
  handle.param("linear_vel", linear_vel, 0.5);
  handle.param("linear_vel_fast", linear_vel_fast, 1.0);
  handle.param("angular_vel", angular_vel, 0.5);
  handle.param("angular_vel_fast", angular_vel_fast, 1.5);
  
  ros::NodeHandle nodeHandle;
  setClient = handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  setClient.waitForExistence();
  getClient = handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
  getClient.waitForExistence();
  getStateRequest.request.model_name = std::string("mobot");
  getStateRequest.request.relative_entity_name = std::string("world");
  modelState.model_name = std::string("mobot");
  modelState.reference_frame = std::string("world");
  
  
  imageHandler = new ImageHandler;
  ros::Subscriber sub3 = nodeHandle.subscribe("/mobot_pose/ImageWithDeltaPoseAndID", 2, &ImageHandler::shutterCallback, &(*imageHandler));
  posePublisher = nodeHandle.advertise<geometry_msgs::Pose2D>("/mouse/pose", 10);
  boost::thread t = boost::thread(keyboardLoop);
  pthread_t t2;
  pthread_create(&t2, 0, deltaPoseThread, 0);
  ros::spin();
  
  t.interrupt(); //?
  t.join();
  tcsetattr(kfd, TCSANOW, &cooked);
  
  return(0);
}

void stopRobot(){
  getClient.call(getStateRequest);
  modelState.pose = getStateRequest.response.pose;
  modelState.twist.linear.x = 0;
  modelState.twist.linear.y = 0;
  modelState.twist.angular.z = 0;
  setModelStateRequest.request.model_state = modelState;
  setClient.call(setModelStateRequest);
}

void keyboardLoop(){
  char c;
  double max_tv = linear_vel;
  double max_rv = angular_vel;
  bool dirty = false;
  int speed = 0;
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
	  stopRobot();
	  dirty = false;
      }
      continue;
    }
    getClient.call(getStateRequest);
    float heading = 2*acos(getStateRequest.response.pose.orientation.w);
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
    modelState.pose = getStateRequest.response.pose;
    modelState.twist.linear.x = sin(heading)*speed*max_tv;
    modelState.twist.linear.y = cos(heading)*speed*max_tv;
    modelState.twist.angular.z = turn*max_rv;
    setModelStateRequest.request.model_state = modelState;
    setClient.call(setModelStateRequest);
  }
}