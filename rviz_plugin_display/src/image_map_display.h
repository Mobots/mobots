/**
 * For further information: http://pc3.bime.de/dokuwiki/doku.php?id=mobots:software:gui
 * Writen by Moritz Ulmer, Hauke Hansen, Uni Bremen
 */

#include <boost/filesystem.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>

#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/PoseAndID.h"

#include "image_map_visual.h"

namespace Ogre
{
class SceneNode;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace rviz_plugin_display
{
class ImageMapVisual;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// The ImageMapDisplay class itself just implements the buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, ImageMapVisual.
class ImageMapDisplay: public rviz::Display
{
public:
	// Constructor.
    ImageMapDisplay();
	virtual ~ImageMapDisplay();

	// Overrides of public virtual functions from the Display class.
	virtual void onInitialize();
	virtual void reset();
	virtual void createProperties();

	// Setter and getter functions for user-editable properties.
	void setRelPoseTopic(const std::string& relPoseTopic);
	const std::string& getRelPoseTopic(){return relPoseTopic;}
	void setAbsPoseTopic(const std::string& absPoseTopic);
	const std::string& getAbsPoseTopic(){return absPoseTopic;}

	// Overrides of protected virtual functions from Display.  As much
	// as possible, when Displays are not enabled, they should not be
	// subscribed to incoming data and should not show anything in the
	// 3D view.  These functions are where these connections are made
	// and broken.
protected:
	virtual void onEnable();
	virtual void onDisable();
private:
	// Internal helpers which do the work of subscribing and
	// unsubscribing from the ROS topic.
	void subscribe();
	void unsubscribe();

	// A helper to clear this display back to the initial state.
	void clear();
	
	// Subscriber Handlers
	void relPoseCallback(const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg);
	void absPoseCallback(const mobots_msgs::PoseAndID::ConstPtr& msg);

	// Test
	void testVisual(ImageMapVisual* visual_, std::string fileName);
	
	// A node in the Ogre scene tree to be the parent of all our visuals.
	Ogre::SceneNode* scene_node_;
	ImageMapVisual* visual_;

	// Data Input
	ros::Subscriber relPoseSub;
	ros::Subscriber absPoseSub;

	// User-editable property variables.
	std::string relPoseTopic;
	std::string absPoseTopic;
	rviz::ROSTopicStringPropertyWPtr relPoseTopicProperty;
	rviz::ROSTopicStringPropertyWPtr absPoseTopicProperty;
	
};

}
