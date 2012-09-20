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
 * This is an example of how to interface with the image_store server.
 */
void saveRequest(std::string filePath, ros::NodeHandle* handle){
	ros::Publisher pub = handle->advertise<mobots_msgs::ImageWithPoseAndID>("shutter_image_delta_pose", 10);
	mobots_msgs::ImageWithPoseAndID msg;
	// Set image info data
	msg.pose.x = 40.23;
	msg.pose.y = 42.23;
	msg.pose.theta = 30.00;
	msg.image.encoding = "jpg";
	msg.id.mobot_id = 3;
	msg.id.image_id = 0;
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
	
	ros::Rate loop_rate(1);
	
	while (handle->ok())
	{
		ROS_INFO("Sent msg no: %i", msg.id.image_id++);
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		// Change image info data (pose)
		msg.pose.x++;
		msg.pose.y++;
	}
	return;
}

void getRequest(int sessionID, int mobotID, int imageID, ros::NodeHandle* handle){
	ros::ServiceClient client = handle->serviceClient<map_visualization::GetImageWithPose>("image_store_get");
	map_visualization::GetImageWithPose srv;
	srv.request.id.session_id = sessionID;
	srv.request.id.mobot_id = mobotID;
	srv.request.id.image_id = imageID;
	
	if(client.call(srv)){
		if(srv.response.error == 0){
			ROS_INFO("Returned image");
		} else {
			ROS_INFO("Error Message: %i", srv.response.error);
		}
	} else {
		ROS_INFO("Errror Message: Server Error");
	}
	return;
}

/**
 * This server is used to check if the toro_client.cpp works.
 * The ImagePoseID msg is tested.
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
	
	if(argument1 == "save" && argc == 3){
		saveRequest(std::string(argv[2]), &handle);
	} else if(argument1 == "get" && argc == 5){
		getRequest(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), &handle);
	} else {
		ROS_INFO("Usage: image_store_test [save /path/to/file.jpg] [get sessionID mobotID imageID]");
	}
	return 0;
}
