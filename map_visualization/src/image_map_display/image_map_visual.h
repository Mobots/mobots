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
	ImageMapVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
	virtual ~ImageMapVisual();
	
	int insertImage(
		const Ogre::Quaternion& orientation,
		const Ogre::Vector3& position,
		int sessionID, int mobotID, int imageID,
		const std::vector<unsigned char>* imageData,
		const std::string* encoding
	);
	int loadImage(int sessionID, int mobotID, int imageID);
	int unloadImage(int sessionID, int mobotID, int imageID);
	int deleteImage(int sessionID, int mobotID, int imageID);
	int loadMobot(int sessionID, int mobotID);
	int unloadMobot(int sessionID, int mobotID);
	int deleteMobot(int sessionID, int mobotID);
	int loadSession(int sessionID);
	int unloadSession(int sessionID);
	int deleteSession(int sessinoID);
	
	void setFrameOrientation(const Ogre::Quaternion& orientation, int sessionID, int mobotID, int imageID);
	void setFramePosition(const Ogre::Vector3& position, int sessionID, int mobotID, int imageID);
private:
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  Ogre::TextureUnitState* tex_unit_;
  Ogre::ManualObject* manual_object_;
  
  int width_;
  int height_;
  
  Ogre::SceneNode* frame_node_;
  
  Ogre::SceneManager* scene_manager_;
};

}