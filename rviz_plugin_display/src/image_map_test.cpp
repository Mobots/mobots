#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "map_visualization/GetImageWithPose.h"
#include "image_transport/image_transport.h"
#include "opencv/cvwimage.h"
#include "opencv/highgui.h"
#include "iostream"
#include <string>
#include "fstream"
#include "vector"

/**
 * "image_store_rel_pose" (ImageWithPoseAndID) test publisher
 */
void relPublisher(std::string filePath, ros::NodeHandle* handle){
	ros::Publisher pub = handle->advertise<mobots_msgs::ImageWithPoseAndID>("image_store_save", 10);
	mobots_msgs::ImageWithPoseAndID msg;
	// Set image info data
	msg.pose.x = 40.23;
	msg.pose.y = 42.23;
	msg.pose.theta = 30.00;
	msg.image.encoding = "jpg";
	msg.id.session_id = 1;
	msg.id.mobot_id = 3;
	msg.id.image_id = 4;
	// Load image data
	std::ifstream imageFile(filePath.c_str(), std::ios::binary);
	imageFile.seekg(0, std::ios::end);
	int length = imageFile.tellg();
	ROS_INFO("length: %i", length);
	char buffer[length];
	imageFile.seekg(0, std::ios::beg);
	imageFile.read(buffer, length);
	msg.image.data.assign(buffer, buffer + sizeof(buffer)/sizeof(char));
	ROS_INFO("data.size: %i", msg.image.data.size());
	
	ros::Rate loop_rate(0.3);
	
	int count = 0;
	while (handle->ok())
	{
		ROS_INFO("Sent msg no: %i", count);
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		msg.pose.x++;
		msg.pose.y++;
		msg.id.image_id++;
		count++;
	}
	return;
}

void absPublisher(int sessionID, int mobotID, int imageID, ros::NodeHandle* handle){
	ros::Publisher pub = handle->advertise<mobots_msgs::ImageWithPoseAndID>("image_store_abs_pose", 10);
	mobots_msgs::ImageWithPoseAndID msg;
	// Set image info data
	msg.pose.x = 0;
	msg.pose.y = 0;
	msg.pose.theta = 0;
	msg.image.encoding = "jpg";
	msg.id.session_id = sessionID;
	msg.id.mobot_id = mobotID;
	msg.id.image_id = imageID;
	// Load image data
	
	ros::Rate loop_rate(0.5);
	
	int count = 0;
	while (handle->ok())
	{
		ROS_INFO("Sent msg no: %i", count);
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		msg.pose.x--;
		msg.pose.y++;
		msg.pose.theta++;
		msg.id.image_id++;
		count++;
	}
	return;
}

/**
 * This server checks if image_map_display.cpp works.
 */
int main(int argc, char **argv)
{
	if(argc < 2){
		ROS_INFO("Need more arguments");
		return 0;
	}
	std::string argument1(argv[1]);
	ros::init(argc, argv, "image_store_test");
	ros::NodeHandle handle;
	
	if(argument1 == "rel" && argc == 3){
		relPublisher(std::string(argv[2]), &handle);
	} else if(argument1 == "abs" && argc == 5){
		absPublisher(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), &handle);
	} else {
		ROS_INFO("Usage: image_store_test [rel /path/to/file.jpg] [abs]");
	}
	return 0;
}
