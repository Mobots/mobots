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

#include "image_pose_data_types.cpp"

// Filename ending for the files
std::string savePathRoot("~/"); // save path root
std::string fileConvention("mobotID%i-%i.%s"); // mobotID2-12.jpg
std::string folderConvention("%ssession-%i/"); // /home/john/session-1/
std::string infoEnding("info");

/**
 * Saves an image and its infos in a file or loads the image and infos.
 * TODO calculate imagePath size
 */
class ImagePose{
public:
	ImagePose();
	ImagePose(const IDT* id);
	ImagePose(const imagePoseData* infoData_);
	ImagePose(const imagePoseData* infoData_, const std::vector<unsigned char>);

	int getErrorStatus(){return errorStatus;}
	
	poseT getDelPose(){return infoData.delPose;}
	void setDelPose(const poseT* pose){infoData.delPose = *pose;}
	
	poseT getRelPose(){return infoData.relPose;}
	void setRelPose(const poseT* pose){infoData.relPose = *pose;}
	
	poseT getAbsPose(){return infoData.absPose;}
	void setAbsPose(const poseT* pose){infoData.absPose = *pose;}
	
	IDT getID(){return infoData.id;}
	void setID(const IDT* id){infoData.id = *id;}
	
	std::string getEncoding(){return infoData.image.encoding;}
	void setEncoding(const std::string* encoding)
		{infoData.image.encoding = *encoding;}
	int getWidth(){return infoData.image.width;}
	void setWidth(int width){infoData.image.width = width;}
	int getHeight(){return infoData.image.height;}
	void setHeight(int height){infoData.image.height = height;}
	
	std::vector<unsigned char> getImageData();
	void loadLast(const IDT* id_);
private:
	std::string savePath;
	int errorStatus;
	
	std::string infoPath;
	imagePoseData infoData;
	
	std::string imagePath;
	std::vector<unsigned char> imageData;
	
	char* concPath(const char*);
	char* concPath();
	int initReadImage();
	int initWrite();
};

ImagePose::ImagePose(){
	savePath = savePathRoot;
	errorStatus = 0;
}

/**
 * Constructor to access a saved image, its properties, and poses. The image
 * is loaded with the get request.
 */
ImagePose::ImagePose(const IDT* id_){
	errorStatus = 0;
	savePath = savePathRoot;
	infoData.id = *id_;
	infoPath = concPath(infoEnding.c_str());
	try{
		infoData.load(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
		errorStatus = 102;
	}
	imagePath = concPath(infoData.image.encoding.c_str());
}

/**
 * Constructor to update the infoData. If a pose is not 'enabled', it will not
 * be overwritten.
 */
ImagePose::ImagePose(const imagePoseData* infoData_){
	errorStatus = 0;
	infoData.id = infoData_->id;
	infoPath = concPath(infoEnding.c_str());
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
	savePath = savePathRoot;
	infoData = *infoData_;
	imageData = imageData_;
	imagePath = concPath(infoData.image.encoding.c_str());
	infoPath = concPath(infoEnding.c_str());
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
	// Create the directory
	char* infoFolderPath = concPath();
	boost::filesystem::create_directories(infoFolderPath);
	delete infoFolderPath;
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
	infoPath = concPath(infoEnding.c_str());
	if(!boost::filesystem::exists(infoPath)){
		errorStatus = 102;
		return;
	}
	infoData.id.imageID++;
	infoPath = concPath(infoEnding.c_str());
	while(boost::filesystem::exists(infoPath)){
		infoData.id.imageID++;
		infoPath = concPath(infoEnding.c_str());
	}
	infoData.id.imageID--;
	infoPath = concPath(infoEnding.c_str());
	try{
		infoData.load(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
		errorStatus = 102;
	}
}

/**
 * Takes the ID's needed to identify an image and then concatinates them
 * into a filepath.
 * TODO calculate pathlength
 */
char* ImagePose::concPath(const char* ending){
	char* path = new char[1000];
	std::string pathConvention = folderConvention + fileConvention;
	sprintf(path, pathConvention.c_str(), savePath.c_str(), infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ending);
	return path;
}

/**
 * Takes the sessionID and concatinates the root path plus the session folder
 * into a filepath.
 * TODO calculate pathlength
 */
char* ImagePose::concPath(){
	char* path = new char[1000];
	sprintf(path, folderConvention.c_str(), savePath.c_str(), infoData.id.sessionID);
	return path;
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
