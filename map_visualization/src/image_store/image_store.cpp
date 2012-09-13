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
#include "mobots_msgs/PoseAndID.h"

// Saves the last pose of each mobot
std::vector<poseT> deltaPoseBuffer;
int currentSessionID = 0;
ros::Publisher* relativePub;
ros::Publisher* absolutePub;

/**
 * When a new session is activated, the buffer for the delta pose to relative
 * pose conversion has to be updated.
 */
void refreshDeltaPoseBuffer(){
	ImageInfo imageInfo;
	IDT id;
	deltaPoseBuffer.clear();
	id.sessionID = currentSessionID;
	for(id.mobotID = 0; imageInfo.getErrorStatus() == 0; id.mobotID++){
		imageInfo.loadLast(&id);
		deltaPoseBuffer.push_back(imageInfo.getRelPose());
	}
}

/**
 * This Method saves incoming messages.
 * TODO check if a session is already has images
 * TODO add support for non-square images
 */
void imageDeltaPoseHandler(const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg){
	imageInfoData infoData{
		{msg->id.session_id, msg->id.mobot_id, msg->id.image_id},
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1}, // delta
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1}, // relative
		{0,0,0,0}, // absolute
		{msg->image.width, msg->image.height, msg->image.encoding}
	};
	// Check if the correct session is used
	if(currentSessionID != infoData.id.sessionID){
		currentSessionID = infoData.id.sessionID;
		refreshDeltaPoseBuffer();
	}
	// All delta poses exept the first one need to be added to the last one.
	if(infoData.id.imageID != 0){
		infoData.relPose= infoData.relPose + deltaPoseBuffer[infoData.id.mobotID];
	}
	// If the delta pose buffer vector is too small
	if(infoData.id.mobotID > deltaPoseBuffer.size()){
		deltaPoseBuffer.resize(infoData.id.mobotID);
	}
	deltaPoseBuffer[infoData.id.mobotID] = infoData.relPose;
	ImageInfo imageInfo(&infoData, msg->image.data);
	// Relay the image and updated pose to Rviz
	mobots_msgs::ImageWithPoseAndID relayMsg;
	relayMsg.pose.x = infoData.relPose.x;
	relayMsg.pose.y = infoData.relPose.y;
	relayMsg.pose.theta = infoData.relPose.theta;
	relayMsg.id.session_id = infoData.id.sessionID;
	relayMsg.id.mobot_id = infoData.id.mobotID;
	relayMsg.id.image_id = infoData.id.imageID;
	relayMsg.image.width = infoData.image.width;
	relayMsg.image.height = infoData.image.height;
	relayMsg.image.encoding = infoData.image.encoding;
	relayMsg.image.data = msg->image.data;
	relativePub->publish(relayMsg);
	ros::spinOnce();
	
	ROS_INFO("image_store: image saved: %i", msg->id.image_id);
}

void absolutePoseHandler(const mobots_msgs::PoseAndID::ConstPtr& msg){
	imageInfoData infoData{
		{msg->id.session_id, msg->id.mobot_id, msg->id.image_id},
		{0,0,0,0},
		{0,0,0,0},
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1},
		{0,0,0}
	};
	ImageInfo info(&infoData);
	absolutePub->publish(*msg);
	ros::spinOnce();
	ROS_INFO("image_store: image saved: %i", msg->id.image_id);
}

/**
 * This Method returns an image and its info upon a valid request.
 * TODO return relPose and absPose
 */
bool imageHandlerOut(map_visualization::GetImageWithPose::Request &req, map_visualization::GetImageWithPose::Response &res){
	IDT id{req.id.session_id, req.id.mobot_id, req.id.image_id};
	ImageInfo info(&id);
	if(info.getErrorStatus() != 0){
		res.error = info.getErrorStatus();
		return true;
	}
	// 0 - Both, 1 - Image, 2 - Pose
	if(req.type != 2){
		res.image.data = info.getImageData();
	}
	if(req.type != 1){
		poseT delPose = info.getDelPose();
		poseT relPose = info.getRelPose();
		poseT absPose = info.getAbsPose();
		res.del_pose.x = delPose.x;
		res.del_pose.y = delPose.y;
		res.del_pose.theta = delPose.theta;
		res.rel_pose.x = relPose.x;
		res.rel_pose.y = relPose.y;
		res.rel_pose.theta = relPose.theta;
		res.abs_pose.x = absPose.x;
		res.abs_pose.y = absPose.y;
		res.abs_pose.theta = absPose.theta;
	}
	return true;
}

/**
 * This node saves images and thier data for later use. The topic to
 * save images is "image_store_save" and to get them and thier data is
 * "image_store_get". "mobots_msgs::ImageWithPoseAndID" is used to
 * save and "map_visualization::GetImageWithPose" is used to get.
 * TODO Write method to delete images/info. Forward changes to
 * "image_map_display".
 */
int main(int argc, char **argv){
	// The node is called image_store_server
	ros::init(argc, argv, "image_store_server");
	ros::NodeHandle n;
	// To save images: image_store_save
	// To get images: image_store_get
	ros::Subscriber deltaSub = n.subscribe
		("shutter_image_delta_pose", 10, imageDeltaPoseHandler);
	ros::Subscriber absoluteSub = n.subscribe
		("slam_absolute_pose", 10, absolutePoseHandler);
	ros::Publisher relPub = n.advertise<mobots_msgs::ImageWithPoseAndID>("image_store_rel_pose", 10);
	relativePub = &relPub;
	ros::Publisher absPub = n.advertise<mobots_msgs::PoseAndID> ("image_store_abs_pose", 10);
	absolutePub = &absPub;
	ros::ServiceServer service = n.advertiseService
		("image_store_get", imageHandlerOut);
	
	ROS_INFO("Image_store: Ready");
	ros::spin();
	return 0;
}