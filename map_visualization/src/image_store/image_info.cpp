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
	ImageInfo(const image_id_t* id);
	ImageInfo(const image_info_data* infoData_);
	ImageInfo(const image_info_data* infoData_, const std::vector<unsigned char>);
	
	const char* getEncoding();
	void setEncoding(char*);
	const char* getImagePath();
	int getErrorStatus();
	std::vector<unsigned char> getImageData();
private:
	std::string savePath;
	std::string infoPath;
	
	image_info_data infoData;
	
	std::string imagePath;
	std::vector<unsigned char> imageData;
	size_t imageSize;
	std::string encoding;
	
	char* concPath(const char*);
	char* concPath();
	int initReadImage();
	int initWrite();
	int errorStatus;
};

/**
 * Constructor if the image exists
 */
ImageInfo::ImageInfo(const image_id_t* id){
	savePath = savePathRoot;
	infoData.id = *id;
	infoPath = concPath(infoEnding.c_str());
	errorStatus = 0;
	try{
		infoData.load(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
	}
	// Only now do we have the encoding type(= image ending)
	imagePath = concPath(encoding.c_str());
	if(initReadImage() == 0){
		ROS_INFO("initImageRead: Success:%s", imagePath.c_str());
	} else {
		ROS_INFO("initImageRead: Failure:%s", imagePath.c_str());
		return;
	}
}

/**
 * Constructor to update the infoData. If a pose is not 'enabled', it will not
 * be overwritten.
 */
ImageInfo::ImageInfo(const image_info_data* infoData_){
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
	}
}

/**
 * Constructor if the image does not exist. This is the case when a new image
 * is recieved to be stored.
 */
ImageInfo::ImageInfo(const image_info_data* infoData_, const std::vector<unsigned char> imageData_){
	savePath = savePathRoot;
	infoData = *infoData_;
	
	imageData = imageData_;
	imageSize = imageData.size();
	imagePath = concPath(infoData.encoding.c_str());
	infoPath = concPath(infoEnding.c_str());
	errorStatus = 0;
	try{
		infoData.save(infoPath);
	}catch (std::exception &e){
		ROS_INFO("Error: %s", e.what());
	}
	if(initWrite() == 0){
		ROS_INFO("ImageInfo: Init Write Success:%s", imagePath.c_str());
	} else {
		ROS_INFO("ImageInfo: Init Write Failure:%s", imagePath.c_str());
 	}
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
	//Create the directory
	char* infoFolderPath = concPath();
	boost::filesystem::create_directories(infoFolderPath);
	delete infoFolderPath;
	
	//Save image to disk
	std::ofstream imageFile(imagePath.c_str(), std::ios::binary);
	if(imageFile.is_open()){
		//imageFile.write((const char*) imageData, imageSize);
		imageFile.write((const char*) &imageData[0], imageSize * sizeof(unsigned char));
		imageFile.close();
	} else {
		ROS_INFO("ImageInfo:%s: Can't save image", imagePath.c_str());
		return 1;
	}
	return 0;
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

const char* ImageInfo::getEncoding(){
	return encoding.c_str();
}
void ImageInfo::setEncoding(char* encoding_){
	encoding = encoding_;
	return;
}
const char* ImageInfo::getImagePath(){
	return imagePath.c_str();
}
int ImageInfo::getErrorStatus(){
	return errorStatus;
}
std::vector<unsigned char> ImageInfo::getImageData(){
	return imageData;
}