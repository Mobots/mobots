#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <sensor_msgs/Imu.h>
#include <rviz/display.h>

#include <OGRE/OgreLight.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreMaterialManager.h>

namespace Ogre
{
class SceneNode;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace map_visualization
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
	virtual void fixedFrameChanged();
	virtual void reset();
	virtual void createProperties();

	// Setter and getter functions for user-editable properties.
	void setTopic(const std::string& topic);
	const std::string& getTopic() { return topic_; }

	// Overrides of protected virtual functions from Display.  As much
	// as possible, when Displays are not enabled, they should not be
	// subscribed to incoming data and should not show anything in the
	// 3D view.  These functions are where these connections are made
	// and broken.
protected:
	virtual void onEnable();
	virtual void onDisable();

	Ogre::MovablePlane* mPlane;
	Ogre::Entity*       mPlaneEnt;
	Ogre::SceneNode*    mPlaneNode;

	// Function to handle an incoming ROS message.
private:
	// Internal helpers which do the work of subscribing and
	// unsubscribing from the ROS topic.
	void subscribe();
	void unsubscribe();

	// A helper to clear this display back to the initial state.
	void clear();

	// A node in the Ogre scene tree to be the parent of all our visuals.
	Ogre::SceneNode* scene_node_;

	// User-editable property variables.
	std::string topic_;
};

}