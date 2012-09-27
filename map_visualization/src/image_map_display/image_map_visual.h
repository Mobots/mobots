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
	int showImage(int sessionID, int mobotID, int imageID);
	int hideImage(int sessionID, int mobotID, int imageID);
	int deleteImage(const std::string* nodeName);
	int showMobot(int sessionID, int mobotID);
	int hideMobot(int sessionID, int mobotID);
	int deleteMobot(std::string nodeName);
	int showSession(int sessionID);
	int hideSession(int sessionID);
	int deleteSession(std::string& nodeName);
	
	void setPose(float poseX, float poseY, float poseTheta, int sessionID, int mobotID, int imageID);
	// Updates the orientation of all images belonging to a mobot
	//void setOrientation(const linkedlist<Ogre::Quaternion>* orientationList, int sessionID, int mobotID);
	
	// If the node is not found, a NULL pointer is returned.
	Ogre::SceneNode* getNode(int sessionID, int mobotID, int imageID);
	// If the node is not found, a the node and its path is created.
	Ogre::SceneNode* findNode(int sessionID, int mobotID, int imageID);
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

	// returns the name of the specified map tile (manual object)
	std::string getMapObjectName(int sessionID, int mobotID, int imageID);
};

}
