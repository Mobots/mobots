#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

#include "geometry_msgs/Pose2D.h"

//#define USE_DUAL

using namespace std;

const char TAG[] = "[(simul) hardware_driver] ";

int frequency = 30; //Hz
ros::NodeHandle *nh;
int mouse1, mouse2;

void* singleController(void *data){
  ros::Rate rate(frequency);
  ros::Publisher pub = nh->advertise<geometry_msgs::Pose2D>("mouse/pose", 2);
  geometry_msgs::Pose2D deltaPose;
  char data1[3];
  while(ros::ok()){
    read(mouse1, &data1, 3);
    deltaPose.x = data1[1];
    deltaPose.y = data1[2];
    deltaPose.theta = 0;
    pub.publish(deltaPose);
    rate.sleep();
  }
}

void* dualController(void *data){
  char data1[3];
  char data2[3];
  ros::Rate rate(frequency);
  ros::Publisher pub = nh->advertise<geometry_msgs::Pose2D>("mouse/pose", 2);
  geometry_msgs::Pose2D deltaPose;
  while(ros::ok()){
    char xAvg = 0;
    char yAvg = 0;
    char theta = 0;
    read(mouse1, &data1, 3);
    read(mouse2, &data2, 3);
    xAvg = (data1[1]/2) + (data2[1]/2) + ((data1[1]&0x1) & (data2[1]&0x1));
    yAvg = (data1[2]/2) + (data2[2]/2) + ((data1[2]&0x1) & (data2[2]&0x1));
    //das folgende muss noch durch nen induktionsbeweis verifiziert werden :D
    theta = atan2(data2[2]-data1[2], data2[1]-data1[1]);
    //cout << "x1=" << int(data1[1]) << " x2=" << int(data2[1]) << " y1=" << int(data1[2]) << " y2=" << int(data2[2]) << " theta=" << int(theta) << endl;
    deltaPose.x = xAvg;
    deltaPose.y = yAvg;
    deltaPose.theta = theta;
    pub.publish(deltaPose);
    rate.sleep();
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "hardware_driver");
  nh = new ros::NodeHandle("~");
  string mousePath;
  
  nh->param("path1", mousePath, string("/dev/input/mouse1"));
  mouse1 = open(mousePath.c_str(), O_RDONLY);
  if(mouse1 < 0){
    cerr << TAG << "cannot open " << mousePath << endl;
    exit(1);
  }
  
  int miceCount;
  nh->param("miceCount", miceCount, 1);
  if(miceCount > 1){
    nh->param("path2", mousePath, string("/dev/input/mouse2"));
    int mouse2 = open(mousePath.c_str(), O_RDONLY);
    if(mouse2 < 0){
      cerr << TAG << "cannot open mouse 2" << endl;
      exit(1);
    }
  }
  
  pthread_t thread_t;
  if(miceCount > 1)
    pthread_create(&thread_t, 0, dualController, 0);
  else
    pthread_create(&thread_t, 0, singleController, 0);
    
  ros::spin();
    
}