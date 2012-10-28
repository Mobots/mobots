#ifndef IMAGE_MAP_VISUAL_H
#define IMAGE_MAP_VISUAL_H

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreDataStream.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreAxisAlignedBox.h>
#include <OGRE/OgreEntity.h>

//#include <OGRE/OgreMaterial.h>

#include <iostream>
#include <fstream>
#include <list>

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include <ros/ros.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class Rectangle2D;
}

namespace map_visualization{
	
class ImageMapVisual{
public:
	// Creates the root node. Images are placed into a tree hierarchy.
	// root -> sessions -> mobots -> images
	ImageMapVisual(Ogre::SceneManager* scene_manager);
	virtual ~ImageMapVisual();

    int insertImage(float poseX, float poseY, float poseTheta,
        int sessionID, int mobotID, int imageID, cv::Mat mat);
	int showImage(int sessionID, int mobotID, int imageID);
	int hideImage(int sessionID, int mobotID, int imageID);
	int deleteImage(const std::string* nodeName);
	
	int showMobot(int sessionID, int mobotID);
	int hideMobot(int sessionID, int mobotID);
	int deleteMobot(const std::string* nodeName);
	
	int showSession(int sessionID);
	int hideSession(int sessionID);
    int deleteSession(const std::string* nodeName);
	
	int deleteAllImages();

	
    int setImagePose(float poseX, float poseY, float poseTheta, int sessionID, int mobotID, int imageID);

    void deleteMobotModel(const std::string* nodeName);
    void deleteAllMobotModels();
    int setMobotModel(int mobotID, float poseX, float poseY, float poseTheta);

  // If the node is not found, a the node and its path is created.
	Ogre::SceneNode* getNode(int sessionID, int mobotID, int imageID);
  // If the node is not found, a NULL pointer is returned.
	Ogre::SceneNode* findNode(int sessionID, int mobotID, int imageID);
private:
	Ogre::MaterialPtr material_;
	Ogre::TexturePtr texture_;
	Ogre::TextureUnitState* tex_unit_;
	Ogre::ManualObject* manual_object_;
    Ogre::Rectangle2D* rectangle_;

	int width_;
	int height_;

    Ogre::SceneNode* rootImageNode;
    Ogre::SceneNode* rootMobotModelNode;
	Ogre::SceneManager* sceneManager;
	std::list<Ogre::ManualObject*> manualObjects;
    void createColourCube(int mobotID);
};

}
#endif //IMAGE_MAP_VISUAL_H
