#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mobots_msgs/ImageWithDeltaPoseAndID.h"
#include "image_transport/image_transport.h"
#include "opencv/cvwimage.h"
#include "opencv/highgui.h"
#include "iostream"
#include "fstream"
#include "vector"

/**
 * This server is used to check if the toro_client.cpp works.
 * The ImageWithDeltaPoseAndID msg is tested.
 */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "toro_server_test");
	ros::NodeHandle handle;
	ros::Publisher pub = handle.advertise<mobots_msgs::ImageWithDeltaPoseAndID>("mobot_image_pose", 10);
	
	mobots_msgs::ImageWithDeltaPoseAndID msg;
	msg.pose.x = 0;
	msg.pose.y = 42;
	msg.image.encoding = "jpg";
	std::ifstream imageFile(argv[1], std::ios::binary);
	imageFile.seekg(0, std::ios::end);
	int length = imageFile.tellg();
	ROS_INFO("length: %i", length);
	char buffer[length];
	imageFile.seekg(0, std::ios::beg);
	
	imageFile.read(buffer, length);
	ROS_INFO("readcheck. [0]=%i, [1]=%i",buffer[0], buffer[1]);
	msg.image.data.assign(buffer, buffer + sizeof(buffer)/sizeof(char));
	ROS_INFO("data.size: %i", msg.image.data.size());
	
	ros::Rate loop_rate(1);
	
	int count = 0;
	while (handle.ok())
	{
		ROS_INFO("Sent msg no: %i", count);
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		
		msg.pose.x++;
		msg.pose.y++;
	}
	return 0;
}
