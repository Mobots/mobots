#include "ros/ros.h"
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "mobots_msgs/Pose2DPrio.h"
#include "hardware_driver/ChangeGlobalPose.h"

using namespace std;

//=== constants ===
const char TAG[] = "[hardware_driver] ";

//=== global variables ===
// -- values in Hz --
int mouseFrequency = 10;
int infraredFrequency = 10;

geometry_msgs::Pose2D globalPose;

ros::NodeHandle *nh;
ros::Subscriber nextPose_sub;
ros::Publisher mousePose_pub, globalPose_pub;
ros::ServiceClient shutterClient;
ros::ServiceServer setGlobalPoseServer;
geometry_msgs::PoseStamped currentGlobalTargetPoses;
int mouse1FD, mouse2FD, infraredFD;

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
 * Receives waypoints with a priority 
 */
void poseCallback(const mobots_msgs::Pose2DPrio&);

//== begin methods ==

int main(int argc, char **argv){
  ros::init(argc, argv, "hardware_driver");
  nh = new ros::NodeHandle;
	nextPose_sub = nh->subscribe("waypoint", 10, poseCallback);
  mousePose_pub = nh->advertise<geometry_msgs::Pose2D>("mouse", 10);
  globalPose_pub = nh->advertise<geometry_msgs::Pose2D>("globalPose", 5);
	shutterClient = nh->serviceClient<shutter::delta>("getDelta");
	setGlobalPoseServer = nh->advertiseService("setGlobalPose", changeGlobalPose);
	
  string mousePath;
  
	ros::param::get("~path1", mousePath);
  nh->param("path1", mousePath, string("/dev/input/mouse1"));
  mouse1FD = open(mousePath.c_str(), O_RDONLY);
  if(mouse1FD < 0){
    cerr << TAG << "cannot open " << mousePath << endl;
	 ROS_ERROR("%s cannot open %s", TAG, mousePath.c_str());
    exit(1);
  }
  
  int miceCount;
  nh->param("miceCount", miceCount, 1);
  if(miceCount > 1){
    nh->param("path2", mousePath, string("/dev/input/mouse2"));
    int mouse2 = open(mousePath.c_str(), O_RDONLY);
    if(mouse2 < 0){
		ROS_ERROR("%s cannot open %s", TAG, mousePath.c_str());
      cerr << TAG << "cannot open " << mousePath << endl;
      exit(1);
    }
  }
  
  /*string infraredPath;
  nh->param("irpath", infraredPath, string("/dev/input/infrared"));
  infraredFD = open(infraredPath.c_str(), O_RDONLY);
  if(infraredFD < 0){
	 cerr << TAG << "cannot open " << infraredPath << endl;
	 ROS_ERROR("%s cannot open %s", TAG, infraredPath.c_str());
    exit(1);
  }*/
  
  string mouseFrequencyKey("/hardware_driver/mouseFrequency");
  ros::param::get(mouseFrequencyKey, mouseFrequency);
  
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

void poseCallback(const mobots_msgs::Pose2DPrio &next_pose){
    double x=next_pose.pose.x;
    double y=next_pose.pose.y;
    double theta=next_pose.pose.theta;
}

void poseStampedCallback(const geometry_msgs::PoseStamped& next_pose){
    double x = next_pose.pose.position.x;
    double y = next_pose.pose.position.y;
    double theta = 2*acos(next_pose.pose.orientation.w);
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
  shutter::delta srv;
  if (client.call(srv))
  {						//LOCK ???
   globalPose.x=srv.response.x+req.x;
   globalPose.y=srv.response.y+req.y;
   globalPose.theta=srv.response.theta+req.theta;
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