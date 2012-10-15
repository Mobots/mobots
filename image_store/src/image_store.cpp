/**
 * For further information: http://pc3.bime.de/dokuwiki/doku.php?id=mobots:software:gui
 * Writen by Moritz Ulmer, Hauke  Uni Bremen
 */

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "image_pose_data_types.h"
#include "image_pose.h"
#include "feature.h"

#include "map_visualization/GetImageWithPose.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/PoseAndID.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"

// The NULL pose
poseT zeroPose{0,0,0,1};
// Saves the last pose of each mobot
std::map<int, poseT> deltaPoseBuffer;
int currentSessionID = 0;
ros::Publisher* relativePub;
ros::Publisher* absolutePub;

/**
 * When a new session is activated, the buffer for the delta pose to relative
 * pose conversion has to be updated.
 */
void refreshDeltaPoseBuffer(){
	ROS_INFO("[refreshDeltaPoseBuffer]");
	ImagePose imagePose;
	IDT id;
	deltaPoseBuffer.clear();
	id.sessionID = currentSessionID;
	for(id.mobotID = 0; imagePose.getErrorStatus() == 0; id.mobotID++){
		imagePose.loadLast(&id);
		deltaPoseBuffer[id.mobotID] = imagePose.getRelPose();
		ROS_INFO("mobotID: %i", id.mobotID);
	}
}

/**
 * Callback to save the image and its poses, calculate the relative pose,
 * and relay the image with the relative pose. 
 * TODO check if a session already has images
 */
void imageDeltaPoseHandler(const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg){
	ROS_INFO("deltaPose1");
	imagePoseData infoData{
		{msg->id.session_id, msg->id.mobot_id, msg->id.image_id},
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1}, // delPose (relative)
		{0,0,0,0}, // relPose (relative)
		{0,0,0,0}, // absPose (absolute)
		{msg->image.width, msg->image.height, msg->image.encoding}
	};
	ROS_INFO("deltaPose2");
	// Check if the correct session is used
	if(currentSessionID != infoData.id.sessionID){
		currentSessionID = infoData.id.sessionID;
		refreshDeltaPoseBuffer();
	}
	ROS_INFO("deltaPose3");
	// All delta poses exept the first one need to be added to the last one.
	if(deltaPoseBuffer.count(infoData.id.mobotID) != 0){
		infoData.relPose = infoData.delPose + deltaPoseBuffer[infoData.id.mobotID];
	}
	// If the delta pose buffer vector is too small
	ROS_INFO("deltaPose5: image: %i", msg->image.data.size());
	ROS_INFO("deltaPose5: buffer: %i", deltaPoseBuffer.size());
	deltaPoseBuffer[infoData.id.mobotID] = infoData.relPose;
	ImagePose imagePose(&infoData, msg->image.data);
	ROS_INFO("deltaPose6");
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
	// Send message
	ros::spinOnce();
	
	ROS_INFO("image_store: image saved: %i", msg->id.image_id);
}

/**
 * Callback to save the absolute the pose, and relay it to all Rviz instances
 */
void absolutePoseHandler(const mobots_msgs::PoseAndID::ConstPtr& msg){
	imagePoseData infoData{
		{msg->id.session_id, msg->id.mobot_id, msg->id.image_id},
		{0,0,0,0},
		{0,0,0,0},
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1},
		{0,0,0}
	};
	ImagePose imagePose(&infoData);
	absolutePub->publish(*msg);
	ros::spinOnce();
	ROS_INFO("image_store: image saved: %i", msg->id.image_id);
}

/**
 * Callback to send an image and/or its poses
 */
bool imageHandlerOut(map_visualization::GetImageWithPose::Request &req, map_visualization::GetImageWithPose::Response &res){
	IDT id{req.id.session_id, req.id.mobot_id, req.id.image_id};
	ImagePose imagePose(&id);
	if(imagePose.getErrorStatus() != 0){
		res.error = imagePose.getErrorStatus();
		return true;
	}
	// 0 - Both, 1 - Image, 2 - Pose
	if(req.type != 2){
		res.image.data = imagePose.getImageData();
		res.image.encoding = imagePose.getEncoding();
		res.image.width = imagePose.getWidth();
		res.image.height = imagePose.getHeight();
	}
	if(req.type != 1){
		poseT delPose = imagePose.getDelPose();
		poseT relPose = imagePose.getRelPose();
		poseT absPose = imagePose.getAbsPose();
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

void featureSetHandler(const mobots_msgs::FeatureSetWithPoseAndID& msg){
  if(!FeatureStore::saveFeatureSet(msg))
	 ROS_ERROR("%s: Error writing feature set to disk: session_id: %i, mobot_id: %i, image_id: %i", __PRETTY_FUNCTION__, 
				  msg.id.session_id, msg.id.mobot_id, msg.id.image_id);
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
  const int mobotCount = 3;
	// The node is called image_store_server
	ros::init(argc, argv, "image_store");
	if(!ros::param::get("/sessionID", currentSessionID))
		ROS_ERROR("%s /sessionID is not set, sessionID set to 0", __FILE__);
	std::stringstream stream;
	stream << "mkdir ~/session-" << currentSessionID;
	system(stream.str().c_str());
	ros::NodeHandle n;
	// To save images: image_store_save
	// To get images: image_store_get
	ros::Subscriber deltaSub = n.subscribe
		("image_pose_id", 10, imageDeltaPoseHandler);
	ros::Subscriber absoluteSub = n.subscribe
		("slam/abs_pose", 10, absolutePoseHandler);
	 ros::Subscriber* featuresetSubs = new ros::Subscriber[mobotCount];
	 for(int i = 0; i < mobotCount; i++){
		std::stringstream ss;
		ss << "mobot" << i << "/featureset_pose_id";
		featuresetSubs[i] = n.subscribe(ss.str(), 10, featureSetHandler);
	 }
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
