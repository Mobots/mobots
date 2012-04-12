#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * The imageHandler method.
 */
void imageHandler(const boost::shared_ptr<Message const>& msg){
//	ROS_INFO("Got msg: [%s]", msg->);
}

/**
 * The main method.
 */
int main(int argc, char **argv){
	//Initialization
	ros::init(argc, argv, "toro_client");
	ros::NodeHandle handle;
	ros::Subscriber sub = handle.subscribe("robot_image_pos", 1000, imageHandler);
	ros::spin();
	return 0;
}