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
<<<<<<< HEAD
 * This Method saves incoming messages. The logic is found in
 * "image_info". The imageCounter is incremented.
=======
 * Create a directory.
 * @Param path Takes the full filepath. Removes the filename and recursively
 * creates the directory tree. 
 */
void mkdir(char* path){
	size_t pos = 0;
	int pathLength = 0;
	while(path[pathLength] != '\0'){
		if(path[pathLength] == '/'){
			pos = pathLength;
		}
		pathLength++;
	}
	char subPath[pos + 2];
	
	for(uint i = 0; i < pos + 1; i++){
		subPath[i] = path[i];
	}
	subPath[pos + 1] = '\0';
	std::string s = subPath;
	boost::filesystem::create_directories(s);
}

/**
 * The imageHandler method. 
 * Image naming convention: mobotID-imageNo.jpeg
 * TODO add session IDs and folders
>>>>>>> 7600c0cfd7825244935da494abc68b77bcb95055
 * TODO check if a session is already has images
 */
void imageHandlerIn(const mobots_msgs::ImageWithDeltaPoseAndID::ConstPtr& msg){
	float a = 0;
	ImageInfo info(0, msg->mobot_id, imageCounter[msg->mobot_id], msg->pose.x,  msg->pose.y, msg->pose.theta, "jpg", msg->image.data);
	//ImageInfo info(0, msg->mobot_id, imageCounter[msg->mobot_id], msg->pose.x, msg->pose.y, msg->pose.theta, "jpg", msg->image.data);	
	
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
	/*std::ifstream::pos_type size;
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
	// Error 1 = Image not found
		res.error = 1;
		return true;
	}*/
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
<<<<<<< HEAD
	// To save images: image_store_save
	// To get images: image_store_get
	ros::Subscriber sub = n.subscribe("image_store_save", 10, imageHandlerIn);
	ros::ServiceServer service = n.advertiseService("image_store_get", imageHandlerOut);
=======
	// The topic name is mobot_image_pose
	ros::Subscriber sub = n.subscribe("image_in", 10, imageHandlerIn);
	ros::ServiceServer service = n.advertiseService("image_out", imageHandlerOut);
>>>>>>> 7600c0cfd7825244935da494abc68b77bcb95055
	
	imageCounter = new int[mobotNumber];
	for(int i = 0; i < mobotNumber; i++){
		imageCounter[i] = 0;
	}
	
	ROS_INFO("Image_store: Ready");
	ros::spin();
	return 0;
}
