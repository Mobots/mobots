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

#include <boost/lexical_cast.hpp>

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
class ImageMapDisplay;

struct poseT{
    float x;
    float y;
    float theta;
};

class ImageMapVisual{
public:
	// Creates the root node. Images are placed into a tree hierarchy.
	// root -> sessions -> mobots -> images
    ImageMapVisual(Ogre::SceneManager* sceneManager_, ImageMapDisplay *display_);
	virtual ~ImageMapVisual();

    int insertImage(int sessionID, int mobotID, int imageID,
        float poseX, float poseY, float poseTheta, cv::Mat mat);
	int showImage(int sessionID, int mobotID, int imageID);
	int hideImage(int sessionID, int mobotID, int imageID);
	int deleteImage(const std::string* nodeName);
    int deleteImage(int sessionID, int mobotID, int imageID);
	
    int showMobotImages(int sessionID, int mobotID);
    int hideMobotImages(int sessionID, int mobotID);
    int deleteMobotImages(const std::string* nodeName);
    int deleteMobotImages(int sessionID, int mobotID);
	
    int showSessionImages(int sessionID);
    int hideSessionImages(int sessionID);
    int deleteSessionImages(const std::string* nodeName);
    int deleteSessionImages(int sessionID);
	
	int deleteAllImages();

    int setImagePose(int sessionID, int mobotID, int imageID, float poseX, float poseY, float poseTheta, int poseType);

    int imageToAbsPose(int sessionID, int mobotID, int imageID);
    int imageToRelPose(int sessionID, int mobotID, int imageID);

    int mobotToAbsPose(int sessionID, int mobotID);
    int mobotToRelPose(int sessionID, int mobotID);

    int sessionToAbsPose(int sessionID);
    int sessionToRelPose(int sessionID);

    static const int RELATIVE_POSE_NODE = 0;
    static const int ABSOLUTE_POSE_NODE = 1;

    void deleteMobotModel(int mobotID);
    void deleteAllMobotModels();
    int setMobotModel(int mobotID, float poseX, float poseY, float poseTheta);

private:
    // Used for publishing state changes to other ROS nodes(image_map_info)
    ImageMapDisplay* display;

    // Ogre scene objects
    Ogre::MaterialPtr material;
    Ogre::TexturePtr texture;
    Ogre::TextureUnitState* texUnit;
    Ogre::ManualObject* manualObject;
    Ogre::Rectangle2D* rectangle;

    // Ogre scene nodes
    Ogre::SceneNode* rootImageNode; // root for all images
    Ogre::SceneNode* rootMobotModelNode; // root for all mobot models
    Ogre::SceneManager* sceneManager;
    poseT pose;
    std::map<std::string, poseT> poseMap;

    int imageToPose(Ogre::SceneNode* imageNode, int poseType);
    int mobotToPose(Ogre::SceneNode* mobotNode, int poseType);
    int sessionToPose(Ogre::SceneNode* sessionNode, int poseType);

    void deleteMobotModel(const std::string* nodeName);

    // If the node is not found, a the node and its path is created.
    Ogre::SceneNode* getNode(int sessionID, int mobotID, int imageID);
    // If the node is not found, a NULL pointer is returned.
    Ogre::SceneNode* findNode(int sessionID, int mobotID, int imageID);

    void createColourCube(int mobotID);
};

}
#endif //IMAGE_MAP_VISUAL_H
