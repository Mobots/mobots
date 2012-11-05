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
	int errorStatus;
	
	std::string infoPath;
	imagePoseData infoData;
	
	std::string imagePath;
	std::vector<unsigned char> imageData;
	
	int initReadImage();
	int initWrite();
};