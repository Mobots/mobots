#include <sensor_msgs/Imu.h>

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
	void setFrameOrientation(const Ogre::Quaternion& orientation);
	void setFramePosition(const Ogre::Vector3& position);

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