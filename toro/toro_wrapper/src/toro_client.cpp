#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mobots_msgs/MobotImagePose.h"

/**
 * The imageHandler method.
 */
void imageHandler(const mobots_msgs::MobotImagePose::ConstPtr& msg)
{
	ROS_INFO("MobotID: %i", msg->mobotID);
}

/**
 * The main method.
 */
int main(int argc, char **argv)
{
	// The node is called toro_client
	ros::init(argc, argv, "toro_client");
	ros::NodeHandle handle;
	// The topic name is mobot_image_pose
	ros::Subscriber sub = handle.subscribe("mobot_image_pose", 1000, imageHandler);
	ROS_INFO("Check: Spin-pre");
	ros::spin();
	return 0;
}