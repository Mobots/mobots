#include <ros/ros.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class Plane;
}

namespace map_visualization{
	
class ImageMapVisual{
public:
	// Creates the root node. Images are placed into a tree hierarchy.
	// root -> sessions -> mobots -> images
	ImageMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
	virtual ~ImageMapVisual();

	// Add an image to the scene. The path is created and its loaded status is
	// the same as the previous image.
	int insertImage(
		float poseX, float poseY, float poseTheta,
		int sessionID, int mobotID, int imageID,
		const std::vector<unsigned char>* imageData,
		const std::string* encoding, int width, int height
	);
	// Make an image visible
	int loadImage(int sessionID, int mobotID, int imageID);
	// Make an image invisible
	int unloadImage(int sessionID, int mobotID, int imageID);
	// Delete an image
	int deleteImage(int sessionID, int mobotID, int imageID);
	// Make all images belonging to a mobot visible
	int loadMobot(int sessionID, int mobotID);
	// Make all images belonging to a mobot invisible
	int unloadMobot(int sessionID, int mobotID);
	// Delete all images belonging to a mobot
	int deleteMobot(int sessionID, int mobotID);
	// Make all images belonging to a session visible
	int loadSession(int sessionID);
	// Make all images belonging to a session invisible
	int unloadSession(int sessionID);
	// Delete all images belonging to a session
	int deleteSession(int sessionID);
	
	void setPose(float poseX, float poseY, float poseTheta, int sessionID, int mobotID, int imageID);
	// Updates the orientation of all images belonging to a mobot
	//void setOrientation(const linkedlist<Ogre::Quaternion>* orientationList, int sessionID, int mobotID);
private:
	Ogre::MaterialPtr material_;
	Ogre::TexturePtr texture_;
	Ogre::TextureUnitState* tex_unit_;
	Ogre::ManualObject* manual_object_;

	int width_;
	int height_;

	Ogre::SceneNode* rootNode;
	Ogre::SceneManager* sceneManager;
	std::list<Ogre::ManualObject*> manualObjects;

	// Get the node in which the image entity is to be placed
	Ogre::SceneNode* getImageNode(int sessionID, int mobotID, int imageID);
};

}