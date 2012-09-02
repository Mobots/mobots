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

/**
  * Image info file convention. dataConvention[] is used.
  * TODO move to XML
  * # This expresses a pose(x, y, theta), sessionID, mobotID, encoding
  * 
  * uint8 sessionID
  * uint8 mobotID
  * uint8 imageID
  * 
  * float64 delta_x
  * float64 delta_y
  * float64 delta_theta
  * 
  * float64 absolute_x
  * float64 absolute_y
  * float64 absolute_theta
  * 
  * string encoding
  */

uint sessionIDLine = 3;
uint mobotIDLine = 4;
uint imageIDLine = 5;

uint poseXLine = 7;
uint poseYLine = 8;
uint poseThetaLine = 9;

/*uint absPoseXLine = 10;
uint absPoseYLine = 11;
uint absPoseThetaLine = 12;*/

uint encodingLine = 11;

// Filename ending for the files
std::string savePathRoot("/home/moritz/"); // save path root
std::string fileConvention("mobotID%i-%i.%s"); // mobotID2-12.jpg
std::string folderConvention("%ssession-%i/"); // /home/john/session-1/
char dataConvention[] = "%s\n\n%i\n%i\n%i\n\n%f\n%f\n%f\n\n%s";
std::string infoEnding("info");
char infoHeader[] = "# This expresses a pose(x, y, theta), sessionID, mobotID, encoding";

/**
 * Saves an image and its infos in a file or loads the image and infos.
 * TODO save the objects in binary/xml form?
 * TODO calculate imagePath size
 */
class ImageInfo{
public:
	// Load an image/infos: sessionID, mobotID, imageID
	ImageInfo(const image_id_t* id);
	// Save an image/infos: sessionID, mobotID, imageID, poseX,
	// poseY, poseTheta, encoding, imageData
	ImageInfo(const image_info_data* infoData_, const std::vector<unsigned char>);
	
	std::string savePath;
	uint sessionID;
	uint mobotID;
	uint imageID;
	
	std::string infoPath;
	float poseX;
	float poseY;
	float poseTheta;
	
	std::string imagePath;
	std::vector<unsigned char> imageData;
	size_t imageSize;
	std::string encoding;
	
	const char* getEncoding();
	void setEncoding(char*);
	const char* getImagePath();
	int getErrorStatus();
	std::vector<unsigned char> getImageData();
private:
	int goToLine(std::ifstream& file, uint num);
	char* concPath(const char*);
	char* concPath();
	//int initReadInfo();
	int initReadImage();
	int initWrite();
	void printVars();
	int errorStatus;

	image_info_data infoData;
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
	/*if(initReadInfo() == 0){
		ROS_INFO("initInfoRead: Success:%s", infoPath.c_str());
	} else {
		ROS_INFO("initInfoRead: Failure:%s", infoPath.c_str());
		return;
	}*/
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
 * Loads the meta data of an image.
 * @return if fail, return 1. Else return 0.
 */
/*int ImageInfo::initReadInfo(){
	std::ifstream infoFile(infoPath.c_str());
	std::string value;
	if(infoFile.is_open()){
		try {
			goToLine(infoFile, poseXLine);
			std::getline(infoFile, value);
			poseX = boost::lexical_cast<float>(value);
			ROS_INFO("intiInfoRead: poseX: %f", poseX);
			
			goToLine(infoFile, poseYLine);
			std::getline(infoFile, value);
			poseY = boost::lexical_cast<float>(value);
			ROS_INFO("intiInfoRead: poseY: %f", poseY);
			
			goToLine(infoFile, poseThetaLine);
			std::getline(infoFile, value);
			poseTheta = boost::lexical_cast<float>(value);
			ROS_INFO("intiInfoRead: poseTheta: %f", poseTheta);
			
			goToLine(infoFile, encodingLine);
			std::getline(infoFile, value);
			encoding = value;
			ROS_INFO("intiInfoRead: encoding: %s", encoding.c_str());
		} catch(boost::bad_lexical_cast &) {
			ROS_INFO("initRead: Lexical Cast");
			infoFile.close();
			errorStatus = 102;
			return 1;
		}
	} else {
		ROS_INFO("initInfoRead: file does not exist");
		errorStatus = 101;
		return 1;
	}
	infoFile.close();
	return 0;
}*/

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
	
	//Save info file to disk
	/*char* infoContents = new char[1000];
	sprintf(infoContents, dataConvention, infoHeader, sessionID, mobotID, imageID, poseX, poseY, poseTheta, encoding.c_str());
	std::ofstream infoFile(infoPath.c_str());
	if(infoFile.is_open()){
		infoFile << infoContents;
		infoFile.close();
	} else {
		ROS_INFO("ImageInfo:%s: Can't save info", infoPath.c_str());
		delete infoContents;
		return 1;
	}
	delete infoContents;*/
	return 0;
}

/**
 * Places the file pointer at the begining of the num'th line. This is used for
 * loading data from the info file.
 */
int ImageInfo::goToLine(std::ifstream& file, uint num){
	file.seekg(std::ios::beg);
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try {
		for(uint i = 0; i < num - 1; i++){
			file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		}
	} catch(std::ifstream::failure e) {
		ROS_INFO("ImageInfo/goToLine: Exception setting file pointer");
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