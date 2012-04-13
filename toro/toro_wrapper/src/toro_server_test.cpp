#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This server is used to check if the toro_client.cpp works.
 * The MobotImagePose msg is tested.
 */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "toro_server_test");
	ros::NodeHandle handle;
	ros::Publisher pub = handle.advertise<MobotImagePose>("mobot_image_pose", 1000);
	ros::Rate loop_rate(1);
	
	int count = 0;
	while (ros::ok())
	{
		MobotImagePose msg;
		std::stringstream ss;
		ss << "jpeg";
		msg.format = ss.str();
		msg.mobotID = count;
		
		ROS_INFO("MobotID: %s / format: %s", msg.mobotID, msg.format.c_str());
		
		pub.publish(msg);
		
		ros::spinOnce();
		
		loop_rate.sleep();
		++count;
	}
	
	return 0;
}
	
