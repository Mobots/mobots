#include <ros/ros.h>
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <boost/lexical_cast.hpp>
#include "mobots_msgs/Pose2DPrio.h"

using namespace std;


int main(int argc, char *argv[]) {

	ros::init(argc, argv, "sdfsdfsdf");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<mobots_msgs::Pose2DPrio>("/mobot0/waypoint_rel", 5);

	if(argc < 4){
		cout << "not enough arguments" << endl;
		exit(1);
		}

	mobots_msgs::Pose2DPrio msg;
	msg.prio = 0;
	msg.pose.x = boost::lexical_cast<float>(argv[1]);
	msg.pose.y = boost::lexical_cast<float>(argv[2]);
	msg.pose.theta = boost::lexical_cast<float>(argv[3]);
	
	std::cout << "sending: " << msg.pose.x << ',' << msg.pose.y << ',' << msg.pose.theta << ")" << std::endl;
	ros::Rate rate(5);
	while(pub.getNumSubscribers() == 0){
		cout << ".";
		rate.sleep();
	}
	cout << endl;
	pub.publish(msg);

	//return EXIT_SUCCESS;

	return EXIT_SUCCESS;
}

