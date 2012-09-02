/**
 * For further information: http://pc3.bime.de/dokuwiki/doku.php?id=mobots:software:gui
 * Writen by Moritz Ulmer, Uni Bremen
 */

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include "image_info.cpp"
#include "map_visualization/GetImageWithPose.h"
#include "mobots_msgs/ImageWithPoseAndID.h"

/**
 * This Method saves incoming messages. The logic is found in
 * "image_info". The imageCounter is incremented.
 * TODO check if a session is already has images
 * TODO forward images to "image_map_display"
 */
void imageHandlerIn(const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg){
	image_info_data infoData{
		{msg->id.session_id, msg->id.mobot_id, msg->id.image_id},
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1},
		{0,0,0,0},
		msg->image.encoding
	};
	ImageInfo info(&infoData, msg->image.data);
	ROS_INFO("image_store: image saved: %i", msg->id.image_id);
}

/**
 * This Method returns an image and its info upon a valid request.
 */
bool imageHandlerOut(map_visualization::GetImageWithPose::Request &req, map_visualization::GetImageWithPose::Response &res){
	image_id_t id{req.id.session_id, req.id.mobot_id, req.id.image_id};
	ImageInfo info(&id);
	if(info.getErrorStatus() != 0){
		res.error = info.getErrorStatus();
		return true;
	}
	res.image.data = info.getImageData();
	return true;
}

/**
 * This node saves images and thier data for later use. The topic to
 * save images is "image_store_save" and to get them and thier data is
 * "image_store_get". "mobots_msgs::ImageWithPoseAndID" is used to
 * save and "map_visualization::GetImageWithPose" is used to get.
 * TODO Write method to delete/modify images/info. Forward changes to
 * "image_map_display".
 */
int main(int argc, char **argv){
	// The node is called image_store_server
	ros::init(argc, argv, "image_store_server");
	ros::NodeHandle n;
	// To save images: image_store_save
	// To get images: image_store_get
	ros::Subscriber sub = n.subscribe("image_store_save", 10, imageHandlerIn);
	ros::ServiceServer service = n.advertiseService("image_store_get", imageHandlerOut);
	
	ROS_INFO("Image_store: Ready");
	ros::spin();
	return 0;
}