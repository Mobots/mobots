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
  int direction;
  bool autoExplore; // A Mobot in auto Explore Mode is driving straight
} mobot_1, mobot_2, mobot_3;

void moveMobot(int id, int direction);
void refreshPose(int id, geometry_msgs::Pose2D pose);
void stop(int id);
void releasePose(int id, int prio, geometry_msgs::Pose2D pose);
void wait(int duration);

void irCallback(const mobots_msgs::InfraredScan& irScan, int id);
void irCallback1(const mobots_msgs::InfraredScan& irScan);
void irCallback2(const mobots_msgs::InfraredScan& irScan);
void irCallback3(const mobots_msgs::InfraredScan& irScan);
void userCallback(const mobots_msgs::PoseAndID& input);


int main(int argc, char **argv){
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;
  
  {//initialising the subscribers
  ros::Subscriber infraredScan_1 = nh.subscribe("/mobot1/infrared", 20, irCallback1);
  ros::Subscriber infraredScan_2 = nh.subscribe("/mobot2/infrared", 20, irCallback2);
  ros::Subscriber infraredScan_3 = nh.subscribe("/mobot3/infrared", 20, irCallback3);
  ros::Subscriber userInput_1 = nh.subscribe("/mobot1/waypoint_user", 20, userCallback);
  ros::Subscriber userInput_2 = nh.subscribe("/mobot2/waypoint_user", 20, userCallback);
  ros::Subscriber userInput_3 = nh.subscribe("/mobot3/waypoint_user", 20, userCallback);
  
  //initialising the publishers
  nextPoseRel_1 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot1/waypoint_rel", 20);
  nextPoseRel_2 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot2/waypoint_rel", 20);
  nextPoseRel_3 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot3/waypoint_rel", 20);
  
  //initialising the Mobots
  mobot_1.autoExplore = true;
  mobot_1.id = 1;
  mobot_1.theta = 0.0;
  mobot_1.x = 0.0;
  mobot_1.y = 0.0;
  mobot_1.direction = 1;
  
  mobot_2.autoExplore = true;
  mobot_2.id = 2;
  mobot_2.theta = 0.0;
  mobot_2.x = 0.0;
  mobot_2.y = 0.0;
  mobot_2.direction = 1;
  
  mobot_3.autoExplore = true;
  mobot_3.id = 3;
  mobot_3.theta = 0.0;
  mobot_3.x = 0.0;
  mobot_3.y = 0.0;
  mobot_3.direction = 1;}
  
  //entering the event loop TODO
  while(ros::ok()){
    for(int id = 1 ; id <= 3 ; id++){
      switch(id){
	case 1:
	  if(mobot_1.autoExplore){
	    moveMobot(1, 0);
	    moveMobot(1, 1);
	  }
	  break;
	  
	case 2:
	  
	  break;
	  
	case 3:
	  
	  break;
      }
    }
  }
  return 1;
}

/* Giving a Mobot a move command.
 * direction syntax:
 * 0: random movement, turning between -90° and 90°
 * 1: driving straight
 * 2: turning right 45°
 * 3: turning left 45°
 * 4: turning right 90°
 * 5: turning left 90°
 * 6: turning right 135°
 * 7: turning left 135°
 * 8: turning right 180° */
void moveMobot(int id, int direction){
  int dir;
  if(direction == 0 || direction > 8 || direction < 0){
    srand(time(NULL));
    dir = rand() % 4 +2; //number between 2 and 5
  }
  else{
    dir = direction;
  }
  ROS_INFO("Mobot_%i driving in direction %d", id, dir);
  geometry_msgs::Pose2D nextPose;
    switch(dir){
      case 1: //driving straight
	nextPose.x = 1000.0;
	nextPose.y = 0.0;
	nextPose.theta = 0.0;
	releasePose(id, -2, nextPose);
	break;
	
      case 2: //turning right 45°
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 45.0;
	releasePose(id, -2, nextPose);
	break;
	
      case 3: //turning left 45°
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = -45.0;
	releasePose(id, -2, nextPose);
	break;
	
      case 4: //turning right 90°
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 90.0;
	releasePose(id, -2, nextPose);
	break;
	
      case 5: //turning left 90°
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = -90.0;
	releasePose(id, -2, nextPose);
	break;
	
      case 6: //turning right 135°
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 135.0;
	releasePose(id, -2, nextPose);
	break;
	
      case 7: //turning left 135°
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = -135.0;
	releasePose(id, -2, nextPose);
	break;
	
     case 8: //turning 180°
	nextPose.x = 0.0;
	nextPose.y = 0.0;
	nextPose.theta = 180.0;
	releasePose(id, -2, nextPose);
	break;
    }
  return;
}

/* Updating the Mobot Pose in the Database of this Node incremental */
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
    default:
      ROS_INFO("Invalid Mobot ID, refreshing Pose aborted");
  }
}

/*Stopping the Mobot*/
void stop(int id){
  switch(id){
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
  geometry_msgs::Pose2D stop;
  stop.x = 0.0;
  stop.y = 0.0;
  stop.theta = 0.0;
  ROS_INFO("Mobot_%i stopping", id);
  releasePose(id, 0, stop); 
}

/* Publishing the next Pose for the Mobot with a priority */
void releasePose(int id, int prio, geometry_msgs::Pose2D pose){
  if(id == 1 || id == 2  || id == 3){
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
  } else ROS_INFO("invalid Mobot ID, publishing aborted");
}

/* Wait for the given duration in milliseconds and the ROS System
 * is spinning, handling the callback queue ten times a second. */
void wait(int duration){
  ros::Rate r(10);
  for(int i = 0 ; i <= (duration/100) ; i++){
    if(ros::ok()){
      ros::spinOnce();      
      r.sleep();
    }
  }
}

/* Wrapper for the infrared scan callback of Mobot 1.
 * There is no hint of the ID of the Mobot in the IR scan data. */
void irCallback1(const mobots_msgs::InfraredScan& irScan){
  irCallback(irScan, 1);
}

/* Wrapper for the infrared scan callback of Mobot 2 */
void irCallback2(const mobots_msgs::InfraredScan& irScan){
  irCallback(irScan, 2);
}

/* Wrapper for the infrared scan callback of Mobot 3 */
void irCallback3(const mobots_msgs::InfraredScan& irScan){
  irCallback(irScan, 3);
}

/* This deals with the infrared scan data and stops the Mobot if obstacles occur */
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

/* This method deals with user inputs over the gui */
void userCallback(const mobots_msgs::PoseAndID& input){
  int id = input.id.mobot_id;
  stop(id); 
  ROS_INFO("Mobot_%i handling user input", id);
  releasePose(id, -2, input.pose);
}

