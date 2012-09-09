#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreLight.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include "image_map_visual.h"

#include "image_map_display.h"

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
}

ImageMapDisplay::~ImageMapDisplay()
{
  unsubscribe();
  clear();
}

// Clear the visuals by deleting their objects.
void ImageMapDisplay::clear()
{
  setStatus( rviz::status_levels::Warn, "Topic", "No messages received" );
}

void ImageMapDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  clear();
  topic_ = topic;
  subscribe();
  
  // Make sure rviz renders the next time it gets a chance.
  causeRender();
}

void ImageMapDisplay::subscribe()
{
}

void ImageMapDisplay::unsubscribe()
{
}

void ImageMapDisplay::onEnable()
{
  subscribe();
}

void ImageMapDisplay::onDisable()
{
  unsubscribe();
  clear();
}

// Override rviz::Display's reset() function to add a call to clear().
void ImageMapDisplay::reset()
{
  Display::reset();
  clear();
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( map_visualization, ImageMap, map_visualization::ImageMapDisplay, rviz::Display )
