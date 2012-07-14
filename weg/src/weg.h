#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <mobots_msg/Pose2DPrio.h>
#include <math.h>

typedef struct{
  float x,y,theta;
} pose;

class Weg{
  
  
public:
    weg(int mobotID);
    ~weg();
    ros::Subscriber nextPose_sub;
    
  
  
private:
  std::list<pose> list;
  int argc;
  char argv;
  ros::NodeHandle nh;
  int mobotID;
  pose next,best;
  
  void startWeg();
  void poseCallback(const mobots_msgs::Pose2DPrio &next_pose);
  
  
  
  
}