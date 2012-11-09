/**
 * For further information: http://pc3.bime.de/dokuwiki/doku.php?id=mobots:software:gui
 * Writen by Moritz Ulmer, Uni Bremen
 */
#include <iostream>
#include <fstream>
#include <limits>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <ros/console.h>
#include <mobots_common/utils.h>

#include "image_pose_data_types.h"
#include "image_pose.h"


ImagePose::ImagePose(){
	errorStatus = 0;
}

/**
 * Constructor to access a saved image, its properties, and poses. The image
 * is loaded with the get request.
 */
ImagePose::ImagePose(const IDT* id_){
	errorStatus = 0;
	infoData.id = *id_;
	infoPath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ".info");
	try{
		infoData.load(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
		errorStatus = 102;
	}
	imagePath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, '.' + infoData.image.encoding);
}

/**
 * Constructor to update the infoData. If a pose is not 'enabled', it will not
 * be overwritten.
 */
ImagePose::ImagePose(const imagePoseData* infoData_){
	errorStatus = 0;
	infoData.id = infoData_->id;
	infoPath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ".info");
	try{
		infoData.load(infoPath);
		if(infoData_->delPose.enable == 1){
			infoData.delPose = infoData_->delPose;
		}
		if(infoData_->relPose.enable == 1){
			infoData.relPose = infoData_->relPose;
		}
		if(infoData_->absPose.enable == 1){
			infoData.absPose = infoData_->absPose;
		}
		infoData.save(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
		errorStatus = 102;
	}
}

/**
 * Constructor to store an image, its properties, and poses.
 */
ImagePose::ImagePose(const imagePoseData* infoData_, const std::vector<unsigned char> imageData_){
	errorStatus = 0;
	infoData = *infoData_;
	imageData = imageData_;
	imagePath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, '.' + infoData.image.encoding);
	infoPath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ".info");
	initWrite();
}

/**
 * Loads the image data.
 * @return if fail, return 1. Else return 0.
 */
int ImagePose::initReadImage(){
	std::ifstream imageFile(imagePath.c_str(), std::ios::binary);
	if(imageFile.is_open()){
		// Get filesize, create buffer, reset file pointer
		imageFile.seekg(0, std::ios::end);
		int size = imageFile.tellg();
		char buffer[size];
		imageFile.seekg(0, std::ios::beg);
		// Load image
		imageFile.read(buffer, size);
		imageData.assign(buffer, buffer + sizeof(buffer)/sizeof(char));
	} else {
		ROS_INFO("initImageRead: infoFile open error");
		errorStatus = 103;
		return 1;
	}
	imageFile.close();
	return 0;
}

/**
 * Save an image and its info file to disk.
 */
int ImagePose::initWrite(){
#if 0
  // Create the directory
	char* infoFolderPath = concPath();
	boost::filesystem::create_directories(infoFolderPath);
	delete[] infoFolderPath;
#endif
  
	// Save info to disk
	try{
		infoData.save(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
		errorStatus = 102;
		return 1;
	}
	// Save image to disk
	std::ofstream imageFile(imagePath.c_str(), std::ios::binary);
	if(imageFile.is_open()){
		imageFile.write((const char*) &imageData[0], imageData.size() * sizeof(unsigned char));
		imageFile.close();
	} else {
		ROS_INFO("ImagePose:%s: Can't save image", imagePath.c_str());
		return 1;
	}
	return 0;
}

/**
 * Loads the most recent info file of a mobot
 */
void ImagePose::loadLast(const IDT* id_){
	infoData.id = *id_;
	infoData.id.imageID = 0;
	infoPath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ".info");
	if(!boost::filesystem::exists(infoPath)){
		errorStatus = 102;
		return;
	}
	infoData.id.imageID++;
	infoPath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ".info");
	while(boost::filesystem::exists(infoPath)){
		infoData.id.imageID++;
		infoPath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ".info");
	}
	infoData.id.imageID--;
	infoPath = mobots_common::store::getPathForID(infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ".info");
	try{
		infoData.load(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
		errorStatus = 102;
	}
}


/**
 * If the image is not loaded, it will be loaded from the disk.
 */
std::vector<unsigned char> ImagePose::getImageData(){
	if(imageData.size() == 0){
		if(initReadImage() == 0){
			ROS_INFO("initImageRead: Success:%s", imagePath.c_str());
		} else {
			ROS_INFO("initImageRead: Failure:%s", imagePath.c_str());
		}
	}
	return imageData;
}
