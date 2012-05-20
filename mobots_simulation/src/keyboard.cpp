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
#include "shutter/ImagePoseID.h"
#include "geometry_msgs/Pose2D.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include "../../feature_detector/src/FeaturesMatcher.h" //debug and shit
#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/MessageBridge.h"
#include "mobots_msgs/FeatureSetWithDeltaPose.h"
#include "mobots_msgs/ImageWithPoseDebug.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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

const bool simulation = false;

class MobotKeyboardController{
private:
  double linear_vel;
  double angular_vel;
  double linear_vel_fast;
  double angular_vel_fast;

  ros::NodeHandle nodeHandle; //TODO wtf put this into constructor and gazebo goes fubar
      
	
public:
  MobotKeyboardController(){
    //true für persistente Verbindung <> GEHT NICHT IN DIESEM FALL >_<
    setClient = nodeHandle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    if(simulation)
      setClient.waitForExistence();
    getClient = nodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
    if(simulation)
      getClient.waitForExistence();
    nodeHandle.param("linear_vel", linear_vel, 0.5);
    nodeHandle.param("linear_vel_fast", linear_vel_fast, 1.0);
    nodeHandle.param("angular_vel", angular_vel, 0.5);
    nodeHandle.param("angular_vel_fast", angular_vel_fast, 1.5);
    getStateRequest.request.model_name = std::string("mobot");
    getStateRequest.request.relative_entity_name = std::string("world");
    modelState.model_name = std::string("mobot");
    modelState.reference_frame = std::string("world"); 
  }
        
  ~MobotKeyboardController(){}
  void keyboardLoop();

  void stopRobot(){
    getClient.call(getStateRequest);
    modelState.pose = getStateRequest.response.pose;
    modelState.twist.linear.x = 0;
    modelState.twist.linear.y = 0;
    modelState.twist.angular.z = 0;
    setModelStateRequest.request.model_state = modelState;
    setClient.call(setModelStateRequest);
  }
};

MobotKeyboardController* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;
char imagePos = 0;
char shutterPos = 0;
char featurePos = 0;
ros::Publisher publisher;

cv::Mat image1;
cv::Mat image2;
sensor_msgs::Image images[2];
ImageFeatures features1;
ImageFeatures features2;

// --- DeltaPose Node --- 
geometry_msgs::Pose2D lastPose;
geometry_msgs::Pose2D deltaPose;
ros::Publisher posePublisher;

//forward declarations
void shutterCallback();


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


/**
 * angle is in radian kk
 */
void findRotationMatrix2D(Point2f center, double angle, Mat& rotMat){
    double alpha = cos(angle);
    double beta = sin(angle);
    rotMat.create(2, 3, CV_64F);
    double* m = (double*)rotMat.data;

    m[0] = alpha;
    m[1] = beta;
    m[2] = (1-alpha)*center.x - beta*center.y;
    m[3] = -beta;
    m[4] = alpha;
    m[5] = beta*center.x + (1-alpha)*center.y;
}

inline double toDegree(double rad){
  return rad * 180 / M_PI;
}

void imageCallback(const sensor_msgs::Image image){
  images[shutterPos] = image;
  imagePos++;
  imagePos %= 2;
}

void shutter2(const shutter::ImagePoseID i){
  shutterCallback();
}

void shutterCallback(){
  puts("shutter");
  if(shutterPos == 0){
    image1 = cv_bridge::toCvCopy(images[0])->image;
  }else{
    image2 = cv_bridge::toCvCopy(images[1])->image;
  }
  mobots_msgs::ImageWithPoseDebug msg;
  msg.image = images[shutterPos];
  publisher.publish(msg);
  shutterPos++;
  shutterPos %= 2;
}

void featuresCallback(const mobots_msgs::FeatureSetWithDeltaPose featuresMsg){
  if(featurePos == 1){
    MessageBridge::copyToCvStruct(featuresMsg, features2);
    featurePos = 0;
      //just ugly copy from testFeatures.cpp
    Delta delta;
    Ptr<FeaturesMatcher> matcher = new CpuFeaturesMatcher(CpuFeaturesMatcher::SURF_DEFAULT);
    bool matchResult = matcher->match(features1, features2, delta);
    if(!matchResult){
      cout << "images do not overlap at all" << endl;
      return;
    }
    cout << "deltaX " << delta.x << endl;
    cout << "deltaY " << delta.y << endl;
    cout << "theta " << delta.theta << " rad = " << toDegree(delta.theta) << "°" << endl;
    Mat aff;
    findRotationMatrix2D(Point2f(0,0), delta.theta, aff);
    aff.at<double>(0,2) = delta.x;
    aff.at<double>(1,2) = delta.y;
    cout << "affen mat: " << endl << aff << endl;
    Mat result;
    Mat result2;
    result2.create(Size(image1.cols+image2.cols, image1.rows+image2.rows), image2.type());
    result.create(Size(image1.cols+image2.cols, image1.rows+image2.rows), image2.type());
    Mat outImg1 = result(Rect(0, 0, image1.cols, image1.rows));
    Mat outImg21 = result2(Rect(0, 0, image1.cols, image1.rows));

    warpAffine(image2, result, aff, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
    image1.copyTo(outImg1);

    image1.copyTo(outImg21);
    warpAffine(image2, result2, aff, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);

    imshow("result", result);
    imshow("result2", result2);
    waitKey(0);
  }else{
    MessageBridge::copyToCvStruct(featuresMsg, features1);
    featurePos++;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "MobotKeyboardController", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  MobotKeyboardController tbk;
  
  boost::thread t = boost::thread(boost::bind(&MobotKeyboardController::keyboardLoop, &tbk));
  ros::NodeHandle nodeHandle;
  publisher = nodeHandle.advertise<mobots_msgs::ImageWithPoseDebug>("ImageWithPose", 2);
  ros::Subscriber sub1 = nodeHandle.subscribe("/FeatureSetWithDeltaPose", 2, featuresCallback);
  ros::Subscriber sub2 = nodeHandle.subscribe("/usb_cam/image_raw", 2, imageCallback);
  ros::Subscriber sub3 = nodeHandle.subscribe("/mobot_pose/ImagePoseID", 2, shutter2);
  posePublisher = nodeHandle.advertise<geometry_msgs::Pose2D>("/mouse/pose", 10);
  pthread_t t2;
  if(simulation)
    pthread_create(&t2, 0, deltaPoseThread, 0);
  ros::spin();
  
  t.interrupt(); //?
  t.join();
  tbk.stopRobot();
  tcsetattr(kfd, TCSANOW, &cooked);
  
  return(0);
}

void MobotKeyboardController::keyboardLoop(){
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
  puts("Press space to shutter");
  
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
	case KEYCODE_SPACE:
	  shutterCallback();
	  speed = 0;
	  turn = 0;
	  break;
    }
    modelState.pose = getStateRequest.response.pose;
    modelState.twist.linear.x = sin(heading)*speed*max_tv;
    modelState.twist.linear.y = cos(heading)*speed*max_tv;
    modelState.twist.angular.z = turn*max_rv;
    setModelStateRequest.request.model_state = modelState;
    setClient.call(setModelStateRequest);
  }
}

