#include <ros/ros.h>
#include "signal.h"
#include <iostream>
#include "geometry_msgs/Pose2D.h"

/**
 * This test program is used to send mock positions
 */

using namespace std;

void sigHandler(int signum){
  cout << endl;
  exit(0);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mockPosition");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>("/mobot0/pose", 1);
  double x, y, theta;
  geometry_msgs::Pose2D pose;
  signal(SIGINT, sigHandler);
  while(true){
	 cout << "x: " << flush;
	 cin >> x;
	 cout << "y: " << flush;
	 cin >> y;
	 cout << "theta: " << flush;
	 cin >> theta;
	 cout << "new position: " << x << ", " <<
		y << ", " << theta << endl << endl;
	 pose.x = x;
	 pose.y = y;
	 pose.theta = theta;
	 pub.publish(pose);
  }
}