#include "ros/ros.h"
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "mobots_msgs/Pose2DPrio.h"
#include "mobots_msgs/InfraredScan.h"
#include "hardware_driver/ChangeGlobalPose.h"

using namespace std;

//=== constants ===
const char TAG[] = "[hardware_driver] ";

//=== global variables ===
// -- values in Hz --
int mouseFrequency = 10;
int infraredFrequency = 10;

geometry_msgs::Pose2D globalPose, currentTargetPose;
list<geometry_msgs::Pose2D> targetPoses;

ros::NodeHandle *nh;
ros::Subscriber nextPoseSubRel, nextPoseSubAbs;
ros::Publisher mousePosePub, globalPosePub, infraredScanPub;
ros::ServiceClient shutterClient;
ros::ServiceServer setGlobalPoseServer;

//==== method declarations ====

void* singleMouseReader(void*);
void* dualMouseReader(void*);
void* infraredReader(void*);
/**
 * Service to be called by slam
 */
bool changeGlobalPose(hardware_driver::ChangeGlobalPose::Request& req,
							 hardware_driver::ChangeGlobalPose::Response& res);
/**
 * Receives (absolute) waypoints with a priority 
 */
void absPoseCallback(const mobots_msgs::Pose2DPrio&);
/**
 * Receives (relative i.e. delta) waypoints with a priority 
 */
void relPoseCallback(const mobots_msgs::Pose2DPrio&);

//== begin methods ==

int main(int argc, char **argv){
  ros::init(argc, argv, "hardware_driver");
  nh = new ros::NodeHandle;
  
  nextPoseSubRel = nh->subscribe("waypoint_rel", 5, relPoseCallback);
  nextPoseSubAbs = nh->subscribe("waypoint_abs", 5, absPoseCallback);
  
  mousePosePub = nh->advertise<geometry_msgs::Pose2D>("mouse", 2);
  globalPosePub = nh->advertise<geometry_msgs::Pose2D>("pose", 2);
  infraredScanPub = nh->advertise<mobots_msgs::InfraredScan>("infrared", 2);
  
  //shutterClient = nh->serviceClient<shutter::delta>("getDelta");
  setGlobalPoseServer = nh->advertiseService("set_pose", changeGlobalPose);
  
  pthread_t thread_t;
  if(miceCount > 1)
    pthread_create(&thread_t, 0, dualMouseReader, 0);
  else
    pthread_create(&thread_t, 0, singleMouseReader, 0);
    
  pthread_create(&thread_t, 0, infraredReader, 0);
  
  ros::spin();
}

void* singleMouseReader(void* data){
  ros::Rate rate(mouseFrequency);
  ros::Publisher pub = nh->advertise<geometry_msgs::Pose2D>("mouse", 2);  //needs remapping
  geometry_msgs::Pose2D deltaPose;
  char data1[3];
  while(ros::ok()){
    read(mouse1FD, &data1, 3);
    deltaPose.x = data1[1];
    deltaPose.y = data1[2];
    deltaPose.theta = 0;
    pub.publish(deltaPose);
    rate.sleep();
  }
}

void* dualMouseReader(void* data){
  char data1[3];
  char data2[3];
  ros::Rate rate(mouseFrequency);
  ros::Publisher pub = nh->advertise<geometry_msgs::Pose2D>("mouse", 2); //needs remapping
  geometry_msgs::Pose2D deltaPose;
  while(ros::ok()){
    char xAvg = 0;
    char yAvg = 0;
    char theta = 0;
    read(mouse1FD, &data1, 3);
    read(mouse2FD, &data2, 3);
    xAvg = (data1[1]/2) + (data2[1]/2) + ((data1[1]&0x1) & (data2[1]&0x1)); // zweiter Teil um Rundungsfehler zu vermeiden
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

void* infraredReader(void* data){
  /*ros::Rate rate(infraredFrequency);
  ros::Publisher pub = nh->advertise<>("", 2);
  while(ros::ok()){
    read(infraredFD, XX, X);
    pub.publish(XX);
    rate.sleep();
  }*/
}

/**
 *# prio == 0: pose vor allen anderen einfügen, rest verwerfen (default)
  # prio == -1: pose vor allen anderen einfügen
  # prio == -2: pose am ende der liste einfügen
  # prio == sonst: pose an der prio-position einfügen
*/
void absPoseCallback(const mobots_msgs::Pose2DPrio& next_pose){
  switch(next_pose.prio){
	 case -2: 
		targetPoses.push_back(next_pose.pose);
		break;
	 case -1:
		targetPoses.push_front(next_pose.pose);
		currentTargetPose = next_pose.pose;
		break;
	 case 0:
		targetPoses.clear();
		targetPoses.push_back(next_pose.pose);
		currentTargetPose = next_pose.pose;
		break;
	 default:
		list<geometry_msgs::Pose2D>::iterator it = targetPoses.begin();
		int prio = next_pose.prio;
		if(prio > targetPoses.size())
		  prio = targetPoses.size();
		it = it+6;
		targetPoses.insert(it, next_pose.pose);
		break;		
  }
  
}

void relPoseCallback(mobots_msgs::Pose2DPrio& next_pose){
  //make a copy to prevent inconsistencies with multithreading
  geometry_msgs::Pose2D current = globalPose;
  next_pose.pose.x += current.x;
  next_pose.pose.y += current.y;
  next_pose.pose.theta += current.theta;
  absPoseCallback(next_pose);
}
/***********************************************************************************
 * changeGlobalPose kann zwecks aktualisierung der jeweiligen globalen mobot-pose
 * aufgerufen werden. zB durch toro, wenn dieser eine frische fehlerfreiere Position
 * als die des Maussensor-Integrals errechnet hat. Die Methode besorgt die delta-beträge
 * vom shutter, da sich die aktualisierte globalpose auf den letzten "shut" bezieht.
 *
 * Dieser Ablauf ist eventuell systematisch fehlerbehaftet, falls in der Toro-Rechenzeit ein
 * erneuter "shut" auftritt. Dieser Fall sollte dann vorm globalPose-AUfruf durch den Toro-
 * Node überprüft werden, der dann auch das Delta vom shutter besorgen  und bereits
 * aufadiert an diesen Node weitergeben sollte
 *****************************************************************************************/
bool changeGlobalPose(hardware_driver::ChangeGlobalPose::Request& req,
							 hardware_driver::ChangeGlobalPose::Response& res){
  /*shutter::delta srv;
  if (1client.call(srv))
  {						//LOCK ???
   globalPose.x=srv.response.x+req.x;
   globalPose.y=srv.response.y+req.y;
   globalPose.theta=srv.response.theta+req.theta;*/
  if(false){
  } else {
    ROS_ERROR("Failed to call getDelta");
    //change to request-values, as this is ours best bet
    globalPose.x=req.x;
    globalPose.y=req.y;
    globalPose.theta=req.theta;
    return false;   //t
  }
  return true;
}