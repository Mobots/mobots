#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreDataStream.h>

#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include <rviz/ogre_helpers/arrow.h>

#include "image_map_visual.h"

namespace map_visualization
{

// BEGIN_TUTORIAL
ImageMapVisual::ImageMapVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();
  frame_node2_ = parent_node->createChildSceneNode();

  acceleration_arrow_ = new rviz::Arrow( scene_manager_, frame_node_ );
   
  ROS_INFO("Check1");
  static int count = 0;
  std::stringstream ss;
  ss << "MapObjectMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias( -16.0f, 0.0f );
  material_->setCullingMode( Ogre::CULL_NONE );
  material_->setDepthWriteEnabled(false);
  
  ROS_INFO("Check2");
  static int tex_count = 0;
  std::stringstream ss2;
  ss2 << "MapTexture" << tex_count++;
  size_t pos = filename_.find_last_of('.');
  Ogre::String ext = filename_.substr(pos+1);
  
  ROS_INFO("Check3.0| %s", filename_.c_str());
  IplImage* img = cvLoadImage( "/home/moritz/TillEvil.jpg", 1);
  ROS_INFO("Check3.1| %i,%i;%i;%i-%i", img->width, img->height, img->imageSize, sizeof(img->imageData), img->depth);  
  
  ROS_INFO("Check4");
  texture_ = Ogre::TextureManager::getSingleton().createManual(ss2.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, img->width, img->height, 1, 1, Ogre::PF_A8R8G8B8);
  // Material + Texture = TextureUnit
  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit_ = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit_ = pass->createTextureUnitState();
  }
  tex_unit_->setTextureName(texture_->getName());
  tex_unit_->setTextureFiltering( Ogre::TFO_NONE );
  
  ROS_INFO("Check5");
  static int map_count = 0;
  std::stringstream ss3;
  ss3 << "MapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject( ss3.str() );
  frame_node2_->attachObject( manual_object_ );
  
  width_ = 5;
  height_ = 5;
  
  ROS_INFO("Check6");
  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
      
      // Top right
      manual_object_->position( width_, height_, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
      
      // Top left
      manual_object_->position( 0.0f, height_, 0.0f );
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
    
    // Second triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
      
      // Bottom right
      manual_object_->position( width_, 0.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
      
      // Top right
      manual_object_->position( width_, height_, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  manual_object_->end();
  ROS_INFO("Check7");
}

ImageMapVisual::~ImageMapVisual()
{
  // Delete the arrow to make it disappear.
  delete acceleration_arrow_;

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
  scene_manager_->destroySceneNode( frame_node2_ );
}

void ImageMapVisual::setMessage( const sensor_msgs::Imu::ConstPtr& msg )
{
  const geometry_msgs::Vector3& a = msg->linear_acceleration;

  // Convert the geometry_msgs::Vector3 to an Ogre::Vector3.
  Ogre::Vector3 acc( a.x, a.y, a.z );

  // Find the magnitude of the acceleration vector.
  float length = acc.length();

  // Scale the arrow's thickness in each dimension along with its length.
  Ogre::Vector3 scale( length, length, length );
  acceleration_arrow_->setScale( scale );

  // Set the orientation of the arrow to match the direction of the
  // acceleration vector.
  acceleration_arrow_->setDirection( acc );
  
  // Color in the plane with the image from the msg
  entity_ground_->setMaterialName("Examples/Rockwall");
  entity_ground_->setCastShadows(false);
}

// Position and orientation are passed through to the SceneNode.
void ImageMapVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void ImageMapVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void ImageMapVisual::setColor( float r, float g, float b, float a )
{
  acceleration_arrow_->setColor( r, g, b, a );
}

}

