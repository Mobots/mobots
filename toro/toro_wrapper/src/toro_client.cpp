#include "ros/ros.h"
#include "mobots_msgs/ImageWithPose.h"
#include "mobots_msgs/Config.h"
#include "iostream"
#include "fstream"

int mobotNumber = 3; //Anzahl der Mobots
std::string savePath = ""; //used to change the default save path
std::string naming = "%s%i/id%i-%i.%s"; // /home/john/1/id1-12.jpeg

int *imageCounter;
int session = 0;
std::ofstream toroFile;

/**
 * The imageHandler method. 
 * Image naming convention: mobotID-imageNo.jpeg
 * TODO add session IDs and folders
 * TODO add jpeg checker. Save as "*.jpg"
 * TODO check if a session is already has images
 */
void imageHandler(const mobots_msgs::ImageWithPose::ConstPtr& msg){
	ROS_INFO("MobotID: %i / Pose: %f,%f,%f", msg->mobotID, msg->pose.pose.pose.position.x, msg->pose.pose.pose.position.y, msg->pose.pose.pose.position.z);
	char imagePath[1000];
	sprintf(imagePath, naming.c_str(), savePath.c_str(), msg->mobotID, imageCounter[msg->mobotID], msg->image.format.c_str());
	ROS_INFO("%s", imagePath);
	
	std::ofstream imageFile(imagePath, std::ios::binary);
	ROS_INFO("data.size: %i", msg->image.data.size());
	imageFile.write((const char*) &(msg->image.data[0]), (std::streamsize) msg->image.data.size());
	imageFile.close();
	
	imageCounter[msg->mobotID]++;
}

/**
 * The configHandler method. An API to set variables.
 * TODO getter interface
 * TODO setter for session and imageCounter
 */
void configHandler(const mobots_msgs::Config::ConstPtr& msg){
	std::string key;
	if(msg->node.compare("toro_client")){
		if(msg->key.compare("savePath")){
			ROS_INFO("Change option: savePath from %s to %s", savePath.c_str(), msg->value.c_str());
			savePath = msg->value;
		} else if(msg->key.compare("mobotNumber")){
			ROS_INFO("Change option: mobotNumber from %i to %s", mobotNumber, msg->value.c_str());
			mobotNumber = atoi(msg->value.c_str());
		} else if(msg->key.compare("naming")){
			ROS_INFO("Change option: naming from %s to %s", naming.c_str(), msg->value.c_str());
			naming = msg->value;
		} else {
			ROS_INFO("No matching options");
		}
	}
}

/**
 * The main method.
 */
int main(int argc, char **argv){
	// The node is called toro_client
	ros::init(argc, argv, "toro_client");
	ros::NodeHandle handle;
	// The topic name is mobot_image_pose
	ros::Subscriber sub = handle.subscribe("mobot_image_pose", 10, imageHandler);
	ros::Subscriber subConfig = handle.subscribe("mobot_config", 1000, configHandler);
	
	imageCounter = new int[mobotNumber];
	for(int i = 0; i < mobotNumber; i++){
		imageCounter[i] = 0;
	}
	
	ROS_INFO("Check: Spin-pre");
	ros::spin();
	return 0;
}