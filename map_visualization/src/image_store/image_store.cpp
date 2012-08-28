/**
 * For further information: http://pc3.bime.de/dokuwiki/doku.php?id=mobots:software:gui
 * Writen by Moritz Ulmer, Uni Bremen
 */

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
 * This Method saves incoming messages. The logic is found in
 * "image_info". The imageCounter is incremented.
 * TODO check if a session is already has images
 */
void imageHandlerIn(const mobots_msgs::ImageWithDeltaPoseAndID::ConstPtr& msg){
	ImageInfo info(0, msg->mobot_id, imageCounter[msg->mobot_id], msg->pose.x,  msg->pose.y, msg->pose.theta, msg->image.encoding.c_str(), msg->image.data);
	ROS_INFO("image_store: image saved: %i", imageCounter[msg->mobot_id]);
	imageCounter[msg->mobot_id]++;
}

/**
 * This Method returns an image and its info upon a valid request.
 * TODO Move logic to image_info
 */
bool imageHandlerOut(map_visualization::GetImage::Request &req, map_visualization::GetImage::Response &res){
	ImageInfo info(req.sessionID, req.mobotID, req.imageID);
	if(info.getErrorStatus() != 0){
		res.error = info.getErrorStatus();
		return true;
	}
	res.image.image.data = info.getImageData();
	return true;
}

/**
 * This node saves images and thier data for later use. The topic to
 * save images is "image_store_save" and to get them and thier data is
 * "image_store_get". "mobots_msgs::ImageWithDeltaPoseAndID" is used to
 * save and "map_visualization::GetImage".
 */
int main(int argc, char **argv){
	// The node is called image_store_server
	ros::init(argc, argv, "image_store_server");
	ros::NodeHandle n;
	// To save images: image_store_save
	// To get images: image_store_get
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
