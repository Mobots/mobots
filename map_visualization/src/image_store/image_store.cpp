#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "image_info.cpp"
#include "map_visualization/GetImage.h"


int mobotNumber = 3; // Number of Mobots
char savePath[] = "/home/moritz/"; // used to change the default save path
char naming[] = "%ssession-%i/mobotID%i-%i.%s"; // /home/john/session-1/mobotID2-12.jpg

int *imageCounter;

/**
 * The imageHandler method. 
 * Image naming convention: mobotID-imageNo.jpeg
 * TODO add session IDs and folders
 * TODO check if a session is already has images
 */
void imageHandlerIn(const mobots_msgs::ImageWithDeltaPoseAndID::ConstPtr& msg){
	ImageInfo info(0, msg->mobot_id, imageCounter[msg->mobot_id], msg->pose.x,  msg->pose.y, msg->pose.theta, msg->image.encoding.c_str(), msg->image.data);
	ROS_INFO("image_store: image saved: %i", imageCounter[msg->mobot_id]);
	imageCounter[msg->mobot_id]++;
}

/**
 * 
 */
bool imageHandlerOut(map_visualization::GetImage::Request &req, map_visualization::GetImage::Response &res){
	ImageInfo info(req.sessionID, req.mobotID, req.imageID);
	
	// Copy image data from HDD to server response
	std::ifstream::pos_type size;
	std::ifstream imageFile(info.getImagePath(), std::ios::binary);
	if(imageFile.is_open()){
		imageFile.seekg (0, std::ios::end);
		size = imageFile.tellg();
		char* fileData = new char[size];
		imageFile.seekg (0, std::ios::beg);
		imageFile.read (fileData, size); // Image Data
		imageFile.close();
		
  		const std::vector<unsigned char> fifth (fileData, fileData + sizeof(fileData) / sizeof(int) );
		res.image.image.data = fifth;
		delete fileData;
	} else {
		res.error = "Image file not found";
		return true;
	}
	return true;
}

/**
 * The main method.
 */
int main(int argc, char **argv){
	// The node is called image_in
	ros::init(argc, argv, "image_store_server");
	ros::NodeHandle n;
	// The topic name is mobot_image_pose
	ros::Subscriber sub = n.subscribe("image_store_save", 10, imageHandlerIn);
	ros::ServiceServer service = n.advertiseService("image_store_get", imageHandlerOut);
	
	imageCounter = new int[mobotNumber];
	for(int i = 0; i < mobotNumber; i++){
		imageCounter[i] = 0;
	}
	
	ROS_INFO("Image_store: Ready");
	ros::spin();
	return 0;
}
