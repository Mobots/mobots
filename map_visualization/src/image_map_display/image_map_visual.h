#ifndef IMU_VISUAL_H
#define IMU_VISUAL_H

#include <sensor_msgs/Imu.h>

#include <ros/ros.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class Plane;
}

namespace rviz
{
class Arrow;
}

namespace map_visualization
{

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of ImuVisual represents the visualization of a single
// sensor_msgs::Imu message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class ImageMapVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  ImageMapVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~ImageMapVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const sensor_msgs::Imu::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way ImuVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Imu message.
  void setColor( float r, float g, float b, float a );

private:
  // The object implementing the actual arrow shape
  rviz::Arrow* acceleration_arrow_;
  Ogre::Plane* plane_;
  Ogre::Entity* entity_ground_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  Ogre::TextureUnitState* tex_unit_;
  Ogre::ManualObject* manual_object_;
  Ogre::Image* image_;
  
  int width_;
  int height_;
  std::string filename_;
  // A SceneNode whose pose is set to match the coordinate frame of
  // the Imu message header.
  Ogre::SceneNode* frame_node_;
  Ogre::SceneNode* frame_node2_;
  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
  
  //bool LoadImage(const Ogre::String& texture_name, const Ogre::String& texture_path);
  //bool LoadImage(const Ogre::String& texture_name, const Ogre::String& texture_path);
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // IMU_VISUAL_H
