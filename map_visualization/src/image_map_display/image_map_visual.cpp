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

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
  frame_node2_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
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
  filename_ = "TillEvil.jpg";
  static int tex_count = 0;
  std::stringstream ss2;
  ss2 << "MapTexture" << tex_count++;
  size_t pos = filename_.find_last_of('.');
  Ogre::String ext = filename_.substr(pos+1);
  
  ROS_INFO("Check3.0| %s", filename_.c_str());
  cv::Mat img = cv::imread( "TillEvil.jpg", 0 );
  cv::Size size = img.size();
  ROS_INFO("Check3.0| %i", sizeof(img.data));
  Ogre::DataStreamPtr imgStrm(new Ogre::MemoryDataStream(img.data, img.elemSize()));
  ROS_INFO("Check3.2");
  
  /*ROS_INFO("Check3");
  std::ifstream i;
  ROS_INFO("Check3.1");
  //Ogre::FileStreamDataStream* pFS = 0;
  ROS_INFO("Check3.2");
  i.open(filename_.c_str(), std::ios::binary | std::ios::in);
  if(i.is_open())
    ROS_INFO("Check3.2.1");
  ROS_INFO("Check3.3");
  Ogre::DataStreamPtr strm(new Ogre::FileStreamDataStream(filename_, &i, false));
  //pFS = new Ogre::FileStreamDataStream(&i, false);
  ROS_INFO("Check3.4");
  //Ogre::DataStreamPtr strm(pFS);*/
  ROS_INFO("Check3.5");
  image_->loadRawData(imgStrm, size.width, size.height, Ogre::PF_BYTE_RGB);
  ROS_INFO("Check3.6");
  //i.close();
  
  ROS_INFO("Check4");
  texture_ = Ogre::TextureManager::getSingleton().loadImage(ss2.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, *image_, Ogre::TEX_TYPE_2D, Ogre::MIP_DEFAULT, 1.0f, false, Ogre::PF_UNKNOWN, false);
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
  // Create the plane on which a texture is applied
  /*ROS_INFO("Check plane_");
  plane_ = new Ogre::Plane(Ogre::Vector3::UNIT_Y, 0);
  
  Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      *plane_, 5, 5, 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Z);
  
  entity_ground_ = scene_manager_->createEntity("GroundEntity", "ground");
  frame_node2_->attachObject(entity_ground_);*/
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

/*bool LoadImage(const Ogre::String& texture_name, const Ogre::String& texture_path){
  bool image_loaded = false;
  std::ifstream ifs(texture_path.c_str(), std::ios::binary|std::ios::in);
  if (ifs.is_open()){
    Ogre::String tex_ext;
    Ogre::String::size_type index_of_extension = texture_path.find_last_of('.');
    if (index_of_extension != Ogre::String::npos)
    {
      tex_ext = texture_path.substr(index_of_extension+1);
      Ogre::DataStreamPtr data_stream(new Ogre::FileStreamDataStream(texture_path, &ifs, false));
      Ogre::Image img;
      img.load(data_stream, tex_ext);
      Ogre::TextureManager::getSingleton().loadImage(texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, img, Ogre::TEX_TYPE_2D, 0, 1.0f);
      image_loaded = true;
    }
    ifs.close();
  }
  return image_loaded;
}*/
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

