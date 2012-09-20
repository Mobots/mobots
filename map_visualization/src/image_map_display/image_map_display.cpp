#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include "mobots_msgs/ImageWithPoseAndID.h"

#include "image_map_visual.h"
#include "image_map_display.h"

#include <boost/filesystem.hpp>
namespace map_visualization{
	
ImageMapDisplay::ImageMapDisplay()
  : Display()
  , scene_node_( NULL )
{
}

// After the parent rviz::Display::initialize() does its own setup, it
// calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.
void ImageMapDisplay::onInitialize()
{
  // Make an Ogre::SceneNode to contain all our visuals.
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), scene_node_);
  setStatus(rviz::status_levels::Warn, "Topic", "Finished testing");
}

ImageMapDisplay::~ImageMapDisplay()
{
	ROS_INFO("delete");
	unsubscribe();
	clear();
	ROS_INFO("delete");
}

// Clear the visuals by deleting their objects.
void ImageMapDisplay::clear(){
	ROS_INFO("clear");
	if(visual_ != NULL){
		delete visual_;
		visual_ = NULL;
	}
	ROS_INFO("clear");
}

void ImageMapDisplay::setTopic(const std::string& topic){
	ROS_INFO("setTopic");
	unsubscribe();
	clear();
	topic_ = topic;
	subscribe();

	// Broadcast the fact that the variable has changed.
	propertyChanged( topic_property_ );

	// Make sure rviz renders the next time it gets a chance.
	causeRender();
	ROS_INFO("setTopic");
}

void ImageMapDisplay::subscribe()
{
	ROS_INFO("subscribe");
}

void ImageMapDisplay::unsubscribe()
{
	ROS_INFO("unsubscribe");
}

void ImageMapDisplay::onEnable()
{
	ROS_INFO("onenable");
	subscribe();
	visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), scene_node_);
	testVisual(visual_, "/home/moritz/TillEvil.jpg");
	ROS_INFO("onenable");
}

void ImageMapDisplay::onDisable()
{
	ROS_INFO("ondisable");
	unsubscribe();
	clear();
	ROS_INFO("ondisable");
}

// Override rviz::Display's reset() function to add a call to clear().
void ImageMapDisplay::reset()
{
	ROS_INFO("reset");
  Display::reset();
  clear();
  ROS_INFO("reset");
}

// Override createProperties() to build and configure a Property
// object for each user-editable property.  ``property_manager_``,
// ``property_prefix_``, and ``parent_category_`` are all initialized before
// this is called.
void ImageMapDisplay::createProperties(){
	ROS_INFO("properties");
	topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>(
		"Topic", property_prefix_,
		boost::bind( &ImageMapDisplay::getTopic, this ),
		boost::bind( &ImageMapDisplay::setTopic, this, _1 ),
		parent_category_, this );
	setPropertyHelpText(topic_property_, "mobots_msgs::ImageMapDisplay topic to subscribe to." );
	rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
	topic_prop->setMessageType( ros::message_traits::datatype<mobots_msgs::ImageWithPoseAndID>() );
	ROS_INFO("properties");
}

void ImageMapDisplay::testVisual(ImageMapVisual* visual_, std::string filePath){
	ROS_INFO("testVisual");
	std::ifstream imageFile(filePath.c_str(), std::ios::binary);
	if(!boost::filesystem::exists(filePath.c_str())){
		ROS_INFO("File not exists");
	}
	imageFile.seekg(0, std::ios::end);
	int length = imageFile.tellg();
	char buffer[length];
	imageFile.seekg(0, std::ios::beg);
	imageFile.read(buffer, length);
	std::vector<unsigned char>imageData;
	imageData.assign(buffer, buffer + sizeof(buffer) / sizeof(char));
	float a = 0.0;
	std::string enc = "jpg";
	visual_->insertImage(a,a,a, 0,0,0, &imageData, &enc, 4,4);
	ROS_INFO("testVisual");
	visual_->setPose(1,1,0, 0,0,0);
	visual_->insertImage(a,a,a, 0,0,1, &imageData, &enc, 4,4);
	visual_->hideImage(0,0,0);
	visual_->hideImage(0,0,1);
	visual_->showImage(0,0,1);
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( map_visualization, ImageMap, map_visualization::ImageMapDisplay, rviz::Display )
