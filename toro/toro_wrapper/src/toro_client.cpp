#include "ros/ros.h"
#include "mobots_msgs/ImageWithPose.h"
#include "iostream"
#include "fstream"

int mobotNumber = 3; //Anzahl der Mobots
int *imageCounter;
std::ofstream imageFile;
std::ofstream toroFile;
std::string savePath = ""; //used to change the default save path
const std::string naming = "%sid%i-%i.%s"; // /home/john/id1-12.jpeg

/**
 * The imageHandler method. 
 * Image naming convention: mobotID-imageNo.jpeg
 * TODO add session IDs and folders
 * TODO add jpeg checker. Save as "*.jpg"
 */
void imageHandler(const mobots_msgs::ImageWithPose::ConstPtr& msg)
{
	ROS_INFO("MobotID: %i / Pose: %f,%f,%f", msg->mobotID, msg->pose.pose.pose.position.x, msg->pose.pose.pose.position.y, msg->pose.pose.pose.position.z);
	char imagePath[1000];
	sprintf(imagePath, naming.c_str(), savePath.c_str(), msg->mobotID, imageCounter[msg->mobotID], msg->image.format.c_str());
	ROS_INFO("%s", imagePath);
	
	imageFile.open(imagePath, std::ios::binary);
	ROS_INFO("data.size: %i", msg->image.data.size());
	imageFile.write((const char*) &(msg->image.data[0]), (std::streamsize) msg->image.data.size());
	imageFile.close();
	
	imageCounter[msg->mobotID]++;
}

/**
 * The main method.
 */
int main(int argc, char **argv)
{
	// The node is called toro_client
	ros::init(argc, argv, "toro_client");
	ros::NodeHandle handle;
	// The topic name is mobot_image_pose
	ros::Subscriber sub = handle.subscribe("mobot_image_pose", 10, imageHandler);
	
	imageCounter = new int[mobotNumber];
	for(int i = 0; i < mobotNumber; i++)
	{
		imageCounter[i] = 0;
	}
	
	ROS_INFO("Check: Spin-pre");
	ros::spin();
	return 0;
}