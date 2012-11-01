#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"
#include "mobots_msgs/Pose2DPrio.h"
#include "mobots_msgs/InfraredScan.h"
#include "mobots_msgs/PoseAndID.h"
#include "path_planner/KeyboardRequest.h"

#include <stdlib.h>
#include <time.h>
#include <cmath>

/* global attributes */
ros::Publisher nextPoseRel_1;
ros::Publisher nextPoseRel_2;
ros::Publisher nextPoseRel_3;
ros::ServiceServer keyReqServer;
ros::Timer timerMobot_1;
ros::Timer timerMobot_2;
ros::Timer timerMobot_3;

/* Mobot data structure for saving the state of one Mobot. */
struct mobot{
  double x; //relative x position of the mobot
  double y; //relative y position of the mobot
  double theta; //relative turning angle of the mobot
  int id; //mobot id
  bool userControlled; //is the mobot controlled by the user?
  bool obstacle; //is there an obstacle with which the mobot is dealing right now?
  int timer; //timer for reactivating the mobots auto explore mode after an user command
};

struct mobot mobots[3];
/* Declaring the member functions.
 * Member functions for the Mobot control. */
void moveMobot(int id, int direction);
void refreshPose(int id, geometry_msgs::Pose2D pose);
bool nearly_equal(double a, double b, double eps);
void stop(int id);
void releasePose(int id, int prio, geometry_msgs::Pose2D pose);
void wait(int duration);
int handleObstacle(int id, bool scan[6]);

/* Member functions for the communication with the rest of our system. */
void timerCallback1(const ros::TimerEvent& event);
void timerCallback2(const ros::TimerEvent& event);
void timerCallback3(const ros::TimerEvent& event);
void irCallback(const mobots_msgs::InfraredScan& irScan, int id);
void irCallback1(const mobots_msgs::InfraredScan& irScan);
void irCallback2(const mobots_msgs::InfraredScan& irScan);
void irCallback3(const mobots_msgs::InfraredScan& irScan);
void userCallback(const mobots_msgs::PoseAndID& input);
bool keyReqCallback(path_planner::KeyboardRequest::Request& req, path_planner::KeyboardRequest::Response& res);

int main(int argc, char **argv){
  //initialising the node

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
  
  mobots[0].id = 1;
  mobots[0].theta = 0.0;
  mobots[0].x = 0.0;
  mobots[0].y = 0.0;
  mobots[0].obstacle = false;
  mobots[0].userControlled = false;
  mobots[0].timer = 0;

  mobots[1].id = 2;
  mobots[1].theta = 0.0;
  mobots[1].x = 0.0;
  mobots[1].y = 0.0;
  mobots[1].obstacle = false;
  mobots[1].userControlled = false;
  mobots[1].timer = 0;
  
  mobots[2].id = 3;
  mobots[2].theta = 0.0;
  mobots[2].x = 0.0;
  mobots[2].y = 0.0;
  mobots[2].obstacle = false;
  mobots[2].userControlled = false;
  mobots[2].timer = 0;
  
  //initialising the timers for the Mobots
  timerMobot_1 = nh.createTimer(ros::Duration(1), timerCallback1);
  timerMobot_2 = nh.createTimer(ros::Duration(1), timerCallback2);
  timerMobot_3 = nh.createTimer(ros::Duration(1), timerCallback3);

  /* Entering the event loop. If there is no obstacle or the user is not controlling the
   * Mobot, it is driving straight till something occurs. */  
  while(ros::ok()){
    for(int i = 0 ; i <= 2 ; i++){
      if(!mobots[i].userControlled && !mobots[i].obstacle){
	moveMobot(mobots[i].id, 1);
      }   
    }
    wait(3000);
  }
  //stopping the timers
  timerMobot_1.stop();
  timerMobot_2.stop();
  timerMobot_3.stop();
  return 0;
}

/* Giving a Mobot a move command.
 * direction syntax:
 * -1: random movement, turning between -135° and 135° 
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
  if(direction == 0 || direction > 8 || direction < -1){
    srand(time(NULL));
    dir = rand() % 4 +2; //number between 2 and 5
  } else if(direction == -1){
    srand(time(NULL));
    dir = rand() % 3 +6; //number between 6 and 8
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

/* Updating the Mobot Pose in the Database of this Node */
void refreshPose(int id, geometry_msgs::Pose2D pose){
  double newTheta;
  mobots[id-1].x += pose.x;
  mobots[id-1].y += pose.y;
  newTheta = mobots[id-1].theta + pose.theta;
  //if Mobot is turned around 360°, then start with 0°
  if((nearly_equal(newTheta, 360.0, 0.1)) || (nearly_equal(newTheta, -360.0, 0.1))){
    mobots[id-1].theta = fmod(newTheta, 360.0);
  } else{
    mobots[id-1].theta += pose.theta;
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
  return;
}

/* Wait for the given duration in milliseconds and the ROS System
 * is spinning, handling the callback queue ten times a second. */
void wait(int duration){
  ros::Rate r(10);
  for(int i = 0 ; i <= (duration/100) && ros::ok() ; i++){
    ros::spinOnce();      
    r.sleep();
  }
}

/* Timer for the Mobot 1. After 30 seconds of no new user waypoint, the Mobot
 * is reactivating the auto explore mode. */
void timerCallback1(const ros::TimerEvent& event){
  if(mobots[0].userControlled){
    mobots[0].timer++;
    if(mobots[0].timer >= 30){
      ROS_INFO("Mobot 1 going back to autonome work");
      mobots[0].userControlled = false;
    }
  }
}

/* Timer for Mobot 2. */
void timerCallback2(const ros::TimerEvent& event){
  if(mobots[1].userControlled){
    mobots[1].timer++;
    if(mobots[1].timer >= 30){
      ROS_INFO("Mobot 2 going back to autonome work");
      mobots[1].userControlled = false;
    }
  }
}

/* Timer for Mobot 3. */
void timerCallback3(const ros::TimerEvent& event){
  if(mobots[2].userControlled){
    mobots[2].timer++;
    if(mobots[2].timer >= 30){
      ROS_INFO("Mobot 3 going back to autonome work");
      mobots[2].userControlled = false;
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
    mobots[id-1].obstacle = true;
    int dir = handleObstacle(id, scanBool); //generate new direction
    moveMobot(id, dir);	//move Mobot to avoid obstacle
    ros::Duration(10).sleep();
    mobots[id-1].obstacle = false;
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
	  case 0: //only the front is blocked, turn randomly around at 135°, 180° or -135°
	    newDir = -1;
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
    ros::Duration(30).sleep();
    ROS_INFO("Mobot %i is going on", id);
    return 0;
  }
  return 0;
}

/* This method deals with user inputs over the gui. Everytime a new command is recieved
 * the timer of the specific Mobot is set to 0 and the Mobot is stopping and deleting all
 * queued waypoints. */
void userCallback(const mobots_msgs::PoseAndID& input){
  int id = input.id.mobot_id;
  if(id == 1 || id == 2 || id == 3){
    mobots[id-1].userControlled = true;
    mobots[id-1].timer = 0;
    stop(id); 
    ROS_INFO("Mobot_%i handling user input", id);
    releasePose(id, -2, input.pose);
  } else{
    ROS_INFO("&i is an invalid Mobot ID, user waypoint not released", id);
    return;
  }
}

/* This method activates the keyboard teleop control for a mobot */
bool keyReqCallback(path_planner::KeyboardRequest::Request& req, path_planner::KeyboardRequest::Response& res){
  bool en = req.enable;
  int id = req.mobot_id;
  if(id == 1 || id == 2 || id == 3){
    mobots[id-1].userControlled = en;
    ROS_INFO("keyboard control switch granted for Mobot %i", id);
    res.enabled = en;
    return true;
  } else{
    ROS_INFO("%i is an invalid Mobot ID, keyboard request not granted", id);
    res.enabled = false;
    return false;
  }
}