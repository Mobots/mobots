#pragma once

struct IDT{
	int sessionID;
	int mobotID;
	int imageID;
};

struct poseT{
	float x;
	float y;
	float theta;
	bool enable;
	//das muss hier direkt in die klasse, ansonsten kriegt gcc das nicht geschissen
	poseT operator+(const poseT& other){
	 float resultX = x + other.x;
	 float resultY = y + other.y;
	 float resultTheta = theta + other.theta;
	 return poseT{resultX, resultY, resultTheta, 1};
  }
};

struct imageT{
	int width;
	int height;
	
	std::string encoding;
};

struct imagePoseData{
	IDT id;
	poseT delPose; // delta_pose
	poseT relPose; // relative_pose
	poseT absPose; // absolute_pose
	imageT image;
	void load(const std::string &filename);
	void save(const std::string &filename);
};