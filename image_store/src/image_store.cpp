/**
 * For further information: http://pc3.bime.de/dokuwiki/doku.php?id=mobots:software:gui
 * Writen by Moritz Ulmer, Hauke  Uni Bremen
 */

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.h>
//#include <opencv2/core/core.hpp>

#include "image_pose_data_types.h"
#include "image_pose.h"
#include "feature.h"

#include "map_visualization/GetImageWithPose.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/PoseAndID.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include <mobots_common/utils.h>
#include <mobots_common/constants.h>

// The NULL pose
poseT zeroPose{0,0,0,1};
// Saves the last pose of each mobot
std::map<int, poseT> deltaPoseBuffer;
int currentSessionID;
ros::Publisher* relativePub;
ros::Publisher* absolutePub;
ros::Publisher* deltaPubs;
image_transport::Publisher* imagePubs;

/**
 * When a new session is activated, the buffer for the delta pose to relative
 * pose conversion has to be updated.
 * UNTESTED
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
 * Handler to save the image and its poses, calculate the relative pose,
 * and relay the image with the relative pose. 
 * TODO check if a session already has images
 */
void imageDeltaPoseHandler(const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg){
	imagePoseData infoData{
		{msg->id.session_id, msg->id.mobot_id, msg->id.image_id},
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1}, // delPose (relative)
		{0,0,0,0}, // relPose (relative)
		{0,0,0,0}, // absPose (absolute)
		{msg->image.width, msg->image.height, msg->image.encoding}
	};
	// Check if the correct session is used
	if(currentSessionID != infoData.id.sessionID){
		currentSessionID = infoData.id.sessionID;
		refreshDeltaPoseBuffer();
	}
	// All delta poses exept the first one need to be added to the last one.
	if(deltaPoseBuffer.count(infoData.id.mobotID) != 0){
		infoData.relPose = infoData.delPose + deltaPoseBuffer[infoData.id.mobotID];
	}
	// If the delta pose buffer vector is too small
	deltaPoseBuffer[infoData.id.mobotID] = infoData.relPose;
	ImagePose imagePose(&infoData, msg->image.data);
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
    // Relay images through image_transport
    if(imagePubs[msg->id.mobot_id].getNumSubscribers() != 0){
        cv::Mat mat;
        if(msg->image.encoding == "jpg" || msg->image.encoding == "png"){
            mat = cv::imdecode(msg->image.data, 1);
        } else {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, "bgr8");
            mat = cv_ptr->image;
        }
        IplImage mat_ipl = mat;
        sensor_msgs::ImagePtr imageMsg = sensor_msgs::CvBridge::cvToImgMsg(&mat_ipl, "bgr8");
        imagePubs[msg->id.mobot_id].publish(imageMsg);
    }
    if(deltaPubs[msg->id.mobot_id].getNumSubscribers() != 0){
        deltaPubs[msg->id.mobot_id].publish(*msg);
    }
	// Send message
	ros::spinOnce();
}

/**
 * Handler to save the absolute the pose, and relay it to all Rviz instances
 */
void absolutePoseHandler(const mobots_msgs::PoseAndID::ConstPtr& msg){
	imagePoseData infoData{
		{msg->id.session_id, msg->id.mobot_id, msg->id.image_id},
		{0,0,0,0},
		{0,0,0,0},
		{msg->pose.x, msg->pose.y, msg->pose.theta, 1},
		//{0,0,""}
	};
    infoData.image.width = 0;
    infoData.image.height = 0;
    infoData.image.encoding = "";
	ImagePose imagePose(&infoData);
	//absolutePub->publish(*msg);
	//ros::spinOnce();
}

/**
 * Handler to send an image and/or its poses.
 * Type: 0 - Image + Pose, 1 - Pose
 */
bool imageHandlerOut(map_visualization::GetImageWithPose::Request &req, map_visualization::GetImageWithPose::Response &res){
	IDT id{req.id.session_id, req.id.mobot_id, req.id.image_id};
	ImagePose imagePose(&id);
	if(imagePose.getErrorStatus() != 0){
		res.error = imagePose.getErrorStatus();
		return true;
	}
	// 0 - Both, 1 - Only Pose
	if(req.type == 0){
		res.image.data = imagePose.getImageData();
		res.image.encoding = imagePose.getEncoding();
		res.image.width = imagePose.getWidth();
		res.image.height = imagePose.getHeight();
	}
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
	
	return true;
}

/**
 * Handler for the incoming featuresets
 * Currently just saves them on hdd
 */
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
    const int mobotCount = mobots_common::constants::mobot_count;
    // The node is called image_store_server
	ros::init(argc, argv, "image_store");
	currentSessionID = 0;
	if(!ros::param::get("/sessionID", currentSessionID)){
		currentSessionID = 0;
		ROS_WARN("%s: /sessionID is not set, sessionID set to 0", __FILE__);
	}
	std::string savePathRoot;
	if(ros::param::get("/image_store/path", savePathRoot))
		mobots_common::store::setBasePath(savePathRoot);
	if(!mobots_common::store::createDirs(currentSessionID)){
		ROS_ERROR("%s in %s cannot create mobot data dirs for session %d", __PRETTY_FUNCTION__, __FILE__, currentSessionID);
		exit(1);
	}
	
	ros::NodeHandle n;
	// To save images: image_store_save
	// To get images: image_store_get
	ros::Subscriber absoluteSub = n.subscribe("/slam/abs_pose", 10, absolutePoseHandler);
	ros::Subscriber* deltaSubs = new ros::Subscriber[mobotCount];
	ros::Subscriber* featuresetSubs = new ros::Subscriber[mobotCount];
    imagePubs = new image_transport::Publisher[mobotCount];
    deltaPubs = new ros::Publisher[mobotCount];
    image_transport::ImageTransport it(n);
	for(int i = 0; i < mobotCount; i++){
		std::stringstream ss;
		ss << "/mobot" << i << "/featureset_pose_id";
		featuresetSubs[i] = n.subscribe(ss.str(), 10, featureSetHandler);
		std::stringstream ss2;
		ss2 << "/mobot" << i << "/image_pose_id";
		deltaSubs[i] = n.subscribe(ss2.str(), 10, imageDeltaPoseHandler);
        std::stringstream ss3;
        ss3 << "/mobot" << i << "/image";
        image_transport::Publisher imagePub = it.advertise(ss3.str(), 1);
        imagePubs[i] = imagePub;
        std::stringstream ss4;
        ss4 << "/mobot" << i << "/image_pose_id_relay";
        ros::Publisher deltaPub = n.advertise<mobots_msgs::ImageWithPoseAndID>(ss4.str(), 5);
        deltaPubs[i] = deltaPub;
	}
	ros::Publisher relPub = n.advertise<mobots_msgs::ImageWithPoseAndID>("/image_store/rel_pose", 10);
	relativePub = &relPub;
    
	ros::Publisher absPub = n.advertise<mobots_msgs::PoseAndID> ("/image_store/abs_pose", 10);
	absolutePub = &absPub;
	ros::ServiceServer service = n.advertiseService("/image_store/get", imageHandlerOut);
	
	ros::spin();
	return 0;
}
