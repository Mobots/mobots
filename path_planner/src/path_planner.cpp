#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"
#include "mobots_msgs/Pose2DPrio.h"
#include "mobots_msgs/InfraredScan.h"
#include "mobots_msgs/PoseAndID.h"
#include "path_planner/KeyboardRequest.h"

#include <stdlib.h>
#include <time.h>
#include <cmath>

ros::Publisher nextPoseRel_1;
ros::Publisher nextPoseRel_2;
ros::Publisher nextPoseRel_3;
ros::ServiceServer keyReqServer;

struct mobot{
  double x;
  double y;
  double theta;
  int id;
  bool userControlled;
  bool obstacle;
} mobot_1, mobot_2, mobot_3;

void moveMobot(int id, int direction);
void refreshPose(int id, geometry_msgs::Pose2D pose);
bool nearly_equal(double a, double b, double eps);
void stop(int id);
void releasePose(int id, int prio, geometry_msgs::Pose2D pose);
void wait(int duration);
int handleObstacle(int id, bool scan[6]);


void irCallback(const mobots_msgs::InfraredScan& irScan, int id);
void irCallback1(const mobots_msgs::InfraredScan& irScan);
void irCallback2(const mobots_msgs::InfraredScan& irScan);
void irCallback3(const mobots_msgs::InfraredScan& irScan);
void userCallback(const mobots_msgs::PoseAndID& input);
bool keyReqCallback(path_planner::KeyboardRequest::Request& req, path_planner::KeyboardRequest::Response& res);


int main(int argc, char **argv){
  { //initialising the node
  
    //initialising ROS
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
  
    //initialising the subscribers
    ros::Subscriber infraredScan_1 = nh.subscribe("/mobot1/infrared", 1, irCallback1);
    ros::Subscriber infraredScan_2 = nh.subscribe("/mobot2/infrared", 1, irCallback2);
    ros::Subscriber infraredScan_3 = nh.subscribe("/mobot3/infrared", 1, irCallback3);
    ros::Subscriber userInput_1 = nh.subscribe("/mobot1/waypoint_user", 20, userCallback);
    ros::Subscriber userInput_2 = nh.subscribe("/mobot2/waypoint_user", 20, userCallback);
    ros::Subscriber userInput_3 = nh.subscribe("/mobot3/waypoint_user", 20, userCallback);
    
    //initialising the publishers
    nextPoseRel_1 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot1/waypoint_rel", 20);
    nextPoseRel_2 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot2/waypoint_rel", 20);
    nextPoseRel_3 = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot3/waypoint_rel", 20);
    
    //initialising the keyboad request server
    keyReqServer = nh.advertiseService("/path_planner/keyboard_request", keyReqCallback);
    
    //initialising the Mobots
    mobot_1.id = 1;
    mobot_1.theta = 0.0;
    mobot_1.x = 0.0;
    mobot_1.y = 0.0;
    mobot_1.obstacle = false;
    mobot_1.userControlled = false;

    mobot_2.id = 2;
    mobot_2.theta = 0.0;
    mobot_2.x = 0.0;
    mobot_2.y = 0.0;
    mobot_2.obstacle = false;
    mobot_2.userControlled = false;
    
    mobot_3.id = 3;
    mobot_3.theta = 0.0;
    mobot_3.x = 0.0;
    mobot_3.y = 0.0;
    mobot_3.obstacle = false;
    mobot_3.userControlled = false;
  }
  
  //entering the event loop
  while(ros::ok()){
    for(int id = 1 ; id <= 3 ; id++){
      switch(id){
	case 1:	  
	  if(!mobot_1.userControlled && !mobot_1.obstacle){
	    moveMobot(1, 1);
	  }
	  break;
	  
	case 2:
	  if(!mobot_2.userControlled && !mobot_2.obstacle){
	    moveMobot(2,1);
	  }
	  break;
	  
	case 3:
	  if(!mobot_3.userControlled && !mobot_3.obstacle){
	    moveMobot(3,1);
	  }
	  break;
      }
    }
    wait(10000);
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
  double newTheta;
  switch(id){
    case 1:
      mobot_1.x += pose.x;
      mobot_1.y += pose.y;
      newTheta = mobot_1.theta + pose.theta;
      //if Mobot is turned around 360°, then start with 0°
      if((nearly_equal(newTheta, 360.0, 0.1)) || (nearly_equal(newTheta, -360.0, 0.1))){
	mobot_1.theta = fmod(newTheta, 360.0);
      } else{
	mobot_1.theta += pose.theta;
      }	
      break;
    case 2:
      mobot_2.x += pose.x;
      mobot_2.y += pose.y;
      newTheta = mobot_2.theta + pose.theta;
      if((nearly_equal(newTheta, 360.0, 0.1)) || (nearly_equal(newTheta, -360.0, 0.1))){
	mobot_2.theta = fmod(newTheta, 360.0);
      } else{
      mobot_2.theta += pose.theta;
      }
      break;
    case 3:
      mobot_3.x += pose.x;
      mobot_3.y += pose.y;
      newTheta = mobot_3.theta + pose.theta;
      if((nearly_equal(newTheta, 360.0, 0.1)) || (nearly_equal(newTheta, -360.0, 0.1))){
	mobot_3.theta = fmod(newTheta, 360.0);
      } else{
      mobot_3.theta += pose.theta;
      }
      break;
  }
}

/* Method to check if two doubles are nearly equal */
bool nearly_equal(double a, double b, double eps){
  return std::abs(a-b) <= eps;
}

/*Stopping the Mobot*/
void stop(int id){
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
void irCallback(const mobots_msgs::InfraredScan& irScan, int id){
  bool scanBool[6];
  bool act = false;
  for(int i = 0 ; i <= 5 ; i++){
    scanBool[i] = (1 == irScan.data[i]) ? true : false; //boolean array for further handling of the obstacle 
    if(i == 0 || i == 1 || i == 5){ //something in front of Mobot
      if(scanBool[i]){
	stop(id);
	act = true;
      }
    }
  }
  if(act){
    switch(id){
      case 1:
	mobot_1.obstacle = true;
	break;
      case 2:
	mobot_2.obstacle = true;
	break;
      case 3:
	mobot_3.obstacle = true;
	break;
    }
    int dir = handleObstacle(id, scanBool);
    moveMobot(id, dir);
    switch(id){
      case 1:
	mobot_1.obstacle = false;
	break;
      case 2:
	mobot_2.obstacle = false;
	break;
      case 3:
	mobot_3.obstacle = false;
	break;
    }
  }
}

/* Dealing with the obstacle and giving back a direction for the mobot */
int handleObstacle(int id, bool scan[6]){
  int count = 0; //counter of blocked directions
  int newDir;
  for(int i = 0 ; i <= 5 ; i++){ //count blocked directions
    if(scan[i]) count++;
  }
  
  if(count == 1){ //one direction blocked, turning in opposite direction
    for(int i = 0 ; i <= 5 ; i++){
      if(scan[i]){
	switch(i){
	  case 0:
	    newDir = 8;
	    break;
	  case 1:
	    newDir = 6;
	    break;
	  case 2:
	    newDir = 2;
	    break;
	  case 3:
	    newDir = 1;
	    break;
	  case 4:
	    newDir = 3;
	    break;
	  case 5:
	    newDir = 7;
	    break;
	}
	return newDir;
      }
    }
  }
  
  if(count == 2){ //two directions blocked, check if they are beside one another
    int diff = 0; //difference between two blocked directions
    int dirLeft = 2; //counter for blocked directions to be found
    int first; //first blocked direction
    int second; //second blocked direction
    for(int i = 0 ; i <= 5; i++){
      if(scan[i]){
	diff++;
	if(dirLeft == 2){
	  first = i;
	} else{
	  second = i;
	  break; //both directions found
	}
	dirLeft--;	
      } else if(diff > 0){
	diff++;
      }
    }
    if(diff == 1 || diff == 5){
      switch(first+second){
	case 1: //sensor 0 and 1
	  newDir = 6;
	  break;
	case 3: //sensor 1 and 2
	  newDir = 4;
	  break;
	case 5: //either sensor 2 and 3 or sensor 0 and 5
	  newDir = (first == 2 || second == 2) ? 2 : 7;
	  break;
	case 7: //sensor 3 and 4
	  newDir = 3;
	  break;
	case 9: //sensor 4 and 5
	  newDir = 5;
	  break;
      }
      return newDir;
    }
    if(diff == 2 || diff == 4){
      switch(first+second){
	case 2: //sensor 0 and 2
	  newDir = 4;
	  break;
	case 4: //sensor 1 and 3 or 0 and 4
	  newDir = (first == 4 || second == 4) ? 7 : 2;
	  break;
	case 6: //sensor 2 and 4 or 1 and 5
	  newDir = (first == 2 || second == 2) ? 1 : 8;
	  break;
	case 8: //sensor 3 and 5
	  newDir = 3;
	  break;
      }
      return newDir;
    } else{
      switch(first+second){
	case 3: //sensor 0 and 3
	  newDir = (first == 0) ? 5 : 4;
	  break;
	case 5: //sensor 1 and 4
	  newDir = (first == 1) ? 2 : 7;
	  break;
	case 7: //sensor 2 and 5
	  newDir = (first == 2) ? 3 : 6;
	  break;
      }
    }    
  }
  
  if(count == 3){ //three directions blocked, there are 20 different constellations of blocked directions
    if((!scan[0] && !scan[1] && scan[2] && scan[3] && scan[4] && !scan[5])){
      newDir = 1;
      return newDir;
    }
    
    if((!scan[0] && scan[1] && scan[2] && scan[3] && !scan[4] && !scan[5])
      || (!scan[0] && scan[1] && scan[2] && !scan[3] && scan[4] && !scan[5])
      || (!scan[0] && scan[1] && !scan[2] && scan[3] && scan[4] && !scan[5])){
      newDir = 2;
      return newDir;
    }
    
    if((!scan[0] && !scan[1] && !scan[2] && scan[3] && scan[4] && scan[5])
      || (!scan[0] && !scan[1] && scan[2] && scan[3] && !scan[4] && scan[5])
      || (!scan[0] && !scan[1] && scan[2] && !scan[3] && scan[4] && scan[5])){
      newDir = 3;
      return newDir;
    }
    
    if((scan[0] && !scan[1] && scan[2] && scan[3] && !scan[4] && !scan[5])
      || (!scan[0] && !scan[1] && scan[2] && !scan[3] && scan[4] && scan[5])){
      newDir = 4;
      return newDir;
    }
    
    if((scan[0] && !scan[1] && !scan[2] && scan[3] && scan[4] && !scan[5])
      || (scan[0] && !scan[1] && !scan[2] && scan[3] && !scan[4] && scan[5])){
      newDir = 5;
      return newDir;
    }
    
    if((scan[0] && scan[1] && scan[2] && !scan[3] && !scan[4] && !scan[5])
      || (scan[0] && !scan[1] && scan[2] && !scan[3] && !scan[4] && scan[5])
      || (!scan[0] && scan[1] && scan[2] && !scan[3] && !scan[4] && scan[5])){
      newDir = 6;
      return newDir;
    }
    
    if((scan[0] && !scan[1] && !scan[2] && !scan[3] && scan[4] && scan[5])
      || (!scan[0] && scan[1] && !scan[2] && !scan[3] && scan[4] && scan[5])
      || (scan[0] && scan[1] && !scan[2] && !scan[3] && scan[4] && !scan[5])){
      newDir = 7;
      return newDir;
    }
    
    if((scan[0] && scan[1] && !scan[2] && !scan[3] && !scan[4] && scan[5])){
      newDir = 8;
      return newDir;
    }   
      
    if((!scan[0] && scan[1] && !scan[2] && scan[3] && !scan[4] && scan[5])
      || (!scan[0] && !scan[1] && scan[2] && !scan[3] && scan[4] && !scan[5])){
      count = 6; //stuck go on to count = 6
    }
  }
  
  if(count == 4){ //two directions left, check if they are beside one another or different
    int diff = 0; //difference between two free directions
    int dirLeft = 2; //counter for free directions to be found
    int first; //first direction
    int second; //second direction
    for(int i = 0 ; i <= 5; i++){
      if(!scan[i]){
	diff++;
	if(dirLeft == 2){
	  first = i;
	} else{
	  second = i;
	  break; //both directions found
	}
	dirLeft--; //one direction found -> decrement
      } else if(diff > 0){
	diff++; //increment the difference
      }
    }
    if(diff == 1 || diff == 5){ //free directions are beside one another
      switch(first+second){
	case 1: //sensor 0 and 1
	  newDir = 3;
	  break;
	case 3: //sensor 1 and 2
	  newDir = 5;
	  break;
	case 5: //either sensor 2 and 3 or sensor 0 and 5
	  newDir = (first == 2 || second == 2) ? 7 : 2;
	  break;
	case 7: //sensor 3 and 4
	  newDir = 6;
	  break;
	case 9: //sensor 4 and 5
	  newDir = 4;
	  break;
      }
      return newDir;
    } else{
      //two different directions left, equal to one direction left. Go on to count = 5
      count = 5; 
    }
  }
  
  if(count == 5){ //one direction left, turn Mobot to it
    for(int i = 0 ; i <= 5 ; i++){
      if(!scan[i]){
	switch(i){
	  case 0:
	    newDir = 1; //straight
	    break;
	  case 1:
	    newDir = 3; //left 45°
	    break;
	  case 2:
	    newDir = 7; //left 135°
	    break;
	  case 3:
	    newDir = 8; //180°
	    break;
	  case 4:
	    newDir = 6; //right 135°
	    break;
	  case 5:
	    newDir = 2; // right 45°
	    break;    
	}
	ROS_INFO("Mobot %i turning in last direction possible", id);
	return newDir;
      }
    }
  }
  
  if(count == 6){ //every direction is blocked
    ROS_INFO("Mobot %i is stuck, please help", id);
    ROS_INFO("waiting 30 seconds for Mobot to be set on another place");
    ros::Rate r(1/30);
    r.sleep();
    ROS_INFO("Mobot %i is going on", id);
    return 0;
  }
  return 0;
}

/* This method deals with user inputs over the gui */
void userCallback(const mobots_msgs::PoseAndID& input){
  int id = input.id.mobot_id;
  switch(id){
    case 1:
      mobot_1.userControlled = true;
      break;
    case 2:
      mobot_2.userControlled = true;
      break;
    case 3:
      mobot_3.userControlled = true;
      break;
    default:
      ROS_INFO("invalid Mobot ID, user waypoint not released");
      return;    
  }
  stop(id); 
  ROS_INFO("Mobot_%i handling user input", id);
  releasePose(id, -2, input.pose);
}

/* This method activates the keyboard teleop control for a mobot */
bool keyReqCallback(path_planner::KeyboardRequest::Request& req, path_planner::KeyboardRequest::Response& res){
  bool en = req.enable;
  switch(req.mobot_id){
    case 1:
      mobot_1.userControlled = en;
      break;
    case 2:
      mobot_2.userControlled = en;
      break;
    case 3:
      mobot_3.userControlled = en;
      break;
    default:
      ROS_INFO("invalid Mobot ID, keyboard request not granted");
      res.enabled = false;
      return false;      
  }
  ROS_INFO("keyboard control granted for Mobot %i", req.mobot_id);
  res.enabled = en;
  return true;
}

