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
	poseT operator+(const poseT&);
};

struct imageT{
	int width;
	int height;
	
	std::string encoding;
};

poseT poseT::operator+(const poseT& other)
{
	float resultX = x + other.x;
	float resultY = y + other.y;
	float resultTheta = theta + other.theta;
	return poseT{resultX, resultY, resultTheta, 1};
}

struct imageInfoData{
	IDT id;
	poseT delPose; // delta_pose
	poseT relPose; // relative_pose
	poseT absPose; // absolute_pose
	imageT image;
	void load(const std::string &filename);
	void save(const std::string &filename);
};