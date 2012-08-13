#include <unistd.h>
#include <ros/ros.h>
#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "geometry_msgs/Pose2D.h"
#include "ImageHandler.h"
#include <../../opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include/ros/init.h>

using namespace std;
using namespace cv;

//#define USE_DUAL

char xAvg;
char yAvg;
float theta;
Ptr<ImageHandler> imageHandler;

static char TAG[] = "[mobox_simul.cpp] ";

static string getPathForMobot(int mobotId, const string& topic){
  stringstream ss;
  ss << "/mobot" << mobotId << "/" << topic;
  return ss.str();
}

void* publisherThread(void* data){
  ros::NodeHandle nodeHandle;
  ros::Rate rate(50);
  ros::Publisher pub = nodeHandle.advertise<geometry_msgs::Pose2D>("/mouse/pose", 2);
  geometry_msgs::Pose2D deltaPose;
  while(ros::ok()){
    deltaPose.x = xAvg;
    deltaPose.y = yAvg;
    deltaPose.theta = theta;
    pub.publish(deltaPose);
    rate.sleep();
  }
}

void* shutterThread(void* data){
  while(ros::ok()){
    cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
    imageHandler->shutterCallback();
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "mouse_node");
  ros::NodeHandle nh("~");
  string mousePath;
  nh.param("mousePath", mousePath, string("/dev/input/mouse2"));
  
  int mouse1 = open(mousePath.c_str(), O_RDONLY);
  if(mouse1 < 0){
    ROS_ERROR("cannot open %s ", mousePath.c_str());
    //exit(1);
  }
#ifdef USE_DUAL
    int mouse2 = open("/dev/input/mouse2", O_RDONLY);
      if(mouse2 < 0){
      cout << "cannot open mouse 2" << endl;
      exit(1);
    }
#endif
  string path = getPathForMobot(0, "ImageWithDeltaPoseAndID");
  ros::Subscriber sub = nh.subscribe("/mobot_pose/ImageWithDeltaPoseAndID", 2, &ImageHandler::shutterCallback2, &(*imageHandler));
  cout << "Press [ENTER] for manual shutter" << endl;
  imageHandler = new ImageHandler;
  pthread_t thread;
  pthread_create(&thread, 0, publisherThread, 0);
  pthread_create(&thread, 0, shutterThread, 0);
  /*int xd=0,yd=0; //x/y movement delta
  int xo=0,yo=0; //x/y overflow (out of range -255 to +255)
  int lb=0,mb=0,rb=0,hs=0,vs=0; //left/middle/right mousebutton*/
  
  char data1[3];
  char data2[3];
  ros::Rate rate(50);
  while(ros::ok()){
    xAvg = 0;
    yAvg = 0;
    theta = 0;
    read(mouse1, &data1, 3);
#ifdef USE_DUAL
      read(mouse2, &data2, 3);
      xAvg = (data1[1]/2) + (data2[1]/2) + ((data1[1]&0x1) & (data2[1]&0x1));
      yAvg = (data1[2]/2) + (data2[2]/2) + ((data1[2]&0x1) & (data2[2]&0x1));
      //das folgende muss noch durch nen induktionsbeweis verifiziert werden :D
      theta = atan2(data2[2]-data1[2], data2[1]-data1[1]);
      cout << "x1=" << int(data1[1]) << " x2=" << int(data2[1]) << " y1=" << int(data1[2]) << " y2=" << int(data2[2]) << " theta=" << int(theta) << endl;
#else
      xAvg = data1[1];
      yAvg = data1[2];
      theta = 0;
#endif*/
    rate.sleep();
    ros::spinOnce();
    /*lb=(b[0]&1)>0;
    rb=(b[0]&2)>0;
    mb=(b[0]&4)>0;
    hs=(b[0]&16)>0;
    vs=(b[0]&32)>0;
    xo=(b[0]&64)>0;
    yo=(b[0]&128)>0;
    xd=b[1];
    yd=b[2];
    printf("xdelta = %d ydelta = %d\n", xd, yd);*/
  }
}
