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

#include "image_info_data.cpp"

// Filename ending for the files
std::string savePathRoot("/home/moritz/"); // save path root
std::string fileConvention("mobotID%i-%i.%s"); // mobotID2-12.jpg
std::string folderConvention("%ssession-%i/"); // /home/john/session-1/
std::string infoEnding("info");

/**
 * Saves an image and its infos in a file or loads the image and infos.
 * TODO calculate imagePath size
 */
class ImageInfo{
public:
	ImageInfo();
	ImageInfo(const image_id_t* id);
	ImageInfo(const image_info_data* infoData_);
	ImageInfo(const image_info_data* infoData_, const std::vector<unsigned char>);

	pose_t getRelPose();
	void setRelPose(const pose_t* pose);
	pose_t getAbsPose();
	void setAbsPose(const pose_t* pose);
	image_id_t getID();
	void setID(const image_id_t* id);
	std::string getEncoding();
	void setEncoding(const std::string* encoding);
	
	int getErrorStatus();
	std::vector<unsigned char> getImageData();
	void loadLast(const image_id_t* id_);
private:
	std::string savePath;
	int errorStatus;
	
	std::string infoPath;
	image_info_data infoData;
	
	std::string imagePath;
	std::vector<unsigned char> imageData;
	
	char* concPath(const char*);
	char* concPath();
	int initReadImage();
	int initWrite();
};

ImageInfo::ImageInfo(){
	savePath = savePathRoot;
	errorStatus = 0;
}

/**
 * Constructor if the image exists. The image is loaded only when it is needed.
 */
ImageInfo::ImageInfo(const image_id_t* id_){
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
	imagePath = concPath(infoData.encoding.c_str());
}

/**
 * Constructor to update the infoData. If a pose is not 'enabled', it will not
 * be overwritten.
 */
ImageInfo::ImageInfo(const image_info_data* infoData_){
	errorStatus = 0;
	infoData.id = infoData_->id;
	infoPath = concPath(infoEnding.c_str());
	try{
		infoData.load(infoPath);
		if(infoData_->rel_pose.enable != 0){
			infoData.rel_pose = infoData_->rel_pose;
		}
		if(infoData_->abs_pose.enable != 0){
			infoData.abs_pose = infoData_->abs_pose;
		}
		infoData.save(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
		errorStatus = 102;
	}
}

/**
 * Constructor if the image does not exist. This is the case when a new image
 * is recieved to be stored.
 */
ImageInfo::ImageInfo(const image_info_data* infoData_, const std::vector<unsigned char> imageData_){
	errorStatus = 0;
	savePath = savePathRoot;
	infoData = *infoData_;
	imageData = imageData_;
	imagePath = concPath(infoData.encoding.c_str());
	infoPath = concPath(infoEnding.c_str());
	initWrite();
}

/**
 * Loads the image data.
 * @return if fail, return 1. Else return 0.
 */
int ImageInfo::initReadImage(){
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
 * Save an image and its info file.
 */
int ImageInfo::initWrite(){
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
		ROS_INFO("ImageInfo:%s: Can't save image", imagePath.c_str());
		return 1;
	}
	return 0;
}

void ImageInfo::loadLast(const image_id_t* id_){
	infoData.id = *id_;
	infoData.id.imageID = 0;
	infoPath = concPath(infoEnding.c_str());
	if(!boost::filesystem::exists(infoPath)){
		errorStatus = 102;
		return;
	}
	while(boost::filesystem::exists(infoPath)){
		infoPath = concPath(infoEnding.c_str());
		infoData.id.imageID++;
	}
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
 */
char* ImageInfo::concPath(const char* ending){
	char* path = new char[1000];
	std::string pathConvention = folderConvention + fileConvention;
	sprintf(path, pathConvention.c_str(), savePath.c_str(), infoData.id.sessionID, infoData.id.mobotID, infoData.id.imageID, ending);
	return path;
}

/**
 * Takes the sessionID and concatinates the root path plus the session folder
 * into a filepath.
 */
char* ImageInfo::concPath(){
	char* path = new char[1000];
	sprintf(path, folderConvention.c_str(), savePath.c_str(), infoData.id.sessionID);
	return path;
}

int ImageInfo::getErrorStatus(){return errorStatus;}

pose_t ImageInfo::getRelPose(){return infoData.rel_pose;}
void ImageInfo::setRelPose(const pose_t* pose){infoData.rel_pose = *pose;}
pose_t ImageInfo::getAbsPose(){return infoData.abs_pose;}
void ImageInfo::setAbsPose(const pose_t* pose){infoData.abs_pose = *pose;}
image_id_t ImageInfo::getID(){return infoData.id;}
void ImageInfo::setID(const image_id_t* id){infoData.id = *id;}
std::string ImageInfo::getEncoding(){return infoData.encoding;}
void ImageInfo::setEncoding(const std::string* encoding)
	{infoData.encoding = *encoding;}

std::vector<unsigned char> ImageInfo::getImageData(){
	if(imageData.size() == 0){
		if(initReadImage() == 0){
			ROS_INFO("initImageRead: Success:%s", imagePath.c_str());
		} else {
			ROS_INFO("initImageRead: Failure:%s", imagePath.c_str());
		}
	}
	return imageData;
}