#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"
#include "mobots_msgs/Pose2DPrio.h"
#include "mobots_msgs/InfraredScan.h"
#include "mobots_msgs/PoseAndID.h"

#include <stdlib.h>
#include <time.h>


ros::Publisher nextPoseRel_1;
ros::Publisher nextPoseRel_2;
ros::Publisher nextPoseRel_3;

struct mobot{
  double x;
  double y;
  double theta;
  int id;
  bool autoExplore;
} mobot_1, mobot_2, mobot_3;


void irCallback(const mobots_msgs::InfraredScan& irScan, int id);
void userCallback(const mobots_msgs::PoseAndID& input);
void wait(int duration); //millisec
void go(int id, int prio, geometry_msgs::Pose2D pose);
void explore(int id, int direction);
void irCallback1(const mobots_msgs::InfraredScan& irScan);
void irCallback2(const mobots_msgs::InfraredScan& irScan);
void irCallback3(const mobots_msgs::InfraredScan& irScan);
void refreshPose(int id, geometry_msgs::Pose2D pose);
void stop(int id);


int main(int argc, char **argv){
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;
  
  ros::Subscriber infraredScan_1 = nh.subscribe("/mobot1/infrared", 5, irCallback1);
  ros::Subscriber infraredScan_2 = nh.subscribe("/mobot2/infrared", 5, irCallback2);
  ros::Subscriber infraredScan_3 = nh.subscribe("/mobot3/infrared", 5, irCallback3);
  ros::Subscriber userInput_1 = nh.subscribe("/mobot1/waypoint_user", 5, userCallback);
  ros::Subscriber userInput_2 = nh.subscribe("/mobot2/waypoint_user", 5, userCallback);
  ros::Subscriber userInput_3 = nh.subscribe("/mobot3/waypoint_user", 5, userCallback);
  
  nextPoseRel_1 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot1/waypoint_rel", 5);
  nextPoseRel_2 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot2/waypoint_rel", 5);
  nextPoseRel_3 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot3/waypoint_rel", 5);
  while(ros::ok())
  explore(1, 1);
  return 1;
}

//TODO
void explore (int id, int direction){
  switch(id){
    case 1:
      mobot_1.autoExplore = true;
      break;
    case 2:
      mobot_2.autoExplore = true;
      break;
    case 3:
      mobot_3.autoExplore = true;
      break;
  }
  int dir;
  if(direction == 0){
    srand(time(NULL));
    dir = rand() % 8 +1;    
  }
  else{
    dir = direction;
  }
  ROS_INFO("Mobot_%i driving in direction %d", id, dir);
  geometry_msgs::Pose2D nextPose;        
    switch(dir){
      case 1:
	nextPose.x = 10.0;
	nextPose.y = 0.0;
	nextPose.theta = 0.0;
	go(id, -2, nextPose);
	wait(500);
	break;
	
      case 2:
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 45.0;
	go(id, -2, nextPose);
	wait(2000);
	break;
	
      case 3:
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = -45.0;
	go(id, -2, nextPose);
	wait(2000);
	break;
	
      case 4:
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 90.0;
	go(id, -2, nextPose);
	wait(4000);
	break;
	
      case 5:
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = -90.0;
	go(id, -2, nextPose);
	wait(4000);
	break;
	
      case 6:
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 135.0;
	go(id, -2, nextPose);
	wait(6000);
	break;
	
      case 7:
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = -135.0;
	go(id, -2, nextPose);
	wait(6000);
	break;
	
     case 8:
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 180.0;
	go(id, -2, nextPose);
	wait(8000);
	break;
    }
  return;
}

void refreshPose(int id, geometry_msgs::Pose2D pose){
  switch(id){
    case 1:
      mobot_1.x += pose.x;
      mobot_1.y += pose.y;
      mobot_1.theta += pose.theta;
      break;
    case 2:
      mobot_2.x += pose.x;
      mobot_2.y += pose.y;
      mobot_2.theta += pose.theta;
      break;
    case 3:
      mobot_3.x += pose.x;
      mobot_3.y += pose.y;
      mobot_3.theta += pose.theta;
      break;
  }
}

void stop(int id){
  geometry_msgs::Pose2D stop;
  stop.x = 0.0;
  stop.y = 0.0;
  stop.theta = 0.0;
  ROS_INFO("Mobot_%i stopping", id);
  go(id, 0, stop); 
}

void go(int id, int prio, geometry_msgs::Pose2D pose){
  mobots_msgs::Pose2DPrio pub;
  refreshPose(id, pose);
  pub.pose = pose;
  pub.prio = prio;
  switch (id){
    case 1:
      nextPoseRel_1.publish(pub);
      break;
    case 2:
      nextPoseRel_2.publish(pub);
      break;
    case 3:
      nextPoseRel_3.publish(pub);
      break;
  }
}

void wait(int duration){
  for(int i = 0 ; i <= (duration/100) ; i++){
    ros::spinOnce();
    ros::Rate r(10);
    r.sleep();
  }
}

void irCallback1(const mobots_msgs::InfraredScan& irScan){
  irCallback(irScan, 1);
}

void irCallback2(const mobots_msgs::InfraredScan& irScan){
  irCallback(irScan, 2);
}

void irCallback3(const mobots_msgs::InfraredScan& irScan){
  irCallback(irScan, 3);
}
//TODO
void irCallback(const mobots_msgs::InfraredScan& irScan, int id){
  for(int i = 0 ; i <= 5 ; i++){
    int scan = irScan.data[i];
    if(i == 0 || i == 1 || i == 5){ // something in front of Mobot
      if(scan == 1){
	stop(id);
      }
    }
  }
}

void userCallback(const mobots_msgs::PoseAndID& input){
  switch(input.id.mobot_id){
    case 1:
      mobot_1.autoExplore = false;
      break;
    case 2:
      mobot_2.autoExplore = false;
      break;
    case 3:
      mobot_3.autoExplore = false;
      break;
  }
  int id = input.id.mobot_id;
  stop(id);  
  wait(2000);
  ROS_INFO("Mobot_%i handling userinput", id);
  go(id, -2, input.pose);
}

