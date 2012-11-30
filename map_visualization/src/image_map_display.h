/**
 * For further information: http://pc3.bime.de/dokuwiki/doku.php?id=mobots:software:gui
 * Writen by Moritz Ulmer, Hauke Hansen, Uni Bremen
 */
#ifndef IMAGE_MAP_DISPLAY_H
#define IMAGE_MAP_DISPLAY_H

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <cv_bridge/cv_bridge.h>

#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>

#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/PoseAndID.h"
#include "mobots_msgs/IDKeyValue.h"

#include "map_visualization/GetImageWithPose.h"
#include "map_visualization/RemoteProcedureCall.h"
#include "image_map_visual.h"
#include "../include/map_visualization/definitions.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class PanelDockWidget;
}

namespace map_visualization{
class ImageMapVisual;

/**
 * Plugin main class. Wraps the 3D scene (ImageMapVisual). The ImageMapDisplay
 * class itself implements the ROS communication, editable parameters, and the
 * Display subclass machinery.
 */
class ImageMapDisplay : public rviz::Display
{
public:
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
    void setImageStoreTopic(const std::string& imageStoreTopic);
    const std::string& getImageStoreTopic(){return imageStoreTopic;}
    void setMobotPoseCount(const std::string& topic);
    const std::string& getMobotPoseCount();

    void sendInfoUpdate(int sessionID, int mobotID, int key, int value);

protected:
    virtual void onEnable();
    virtual void onDisable();

private:
    // Internal helpers which do the work of subscribing and
    // unsubscribing from the ROS topic.
    void subscribe();
    void unsubscribe();

    // Get all images in the series upto the specified one
    void retrieveImageSeries(int sessionID, int mobotID);

    // A helper to clear this display back to the initial state.
    void clear();

    // ROS Subscriber/Publisher/Service Callbacks
    void relPoseCallback(const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg);
    void absPoseCallback(const mobots_msgs::PoseAndID::ConstPtr& msg);
    void mobotPoseCallback(int mobotID, const geometry_msgs::Pose2D::ConstPtr &msg);
    void retrieveImages(int sessionID, int mobotID);
    bool updateRvizCallback(RemoteProcedureCall::Request &req,
                            RemoteProcedureCall::Response &res);

    // ROS data Input/Output
    ros::Subscriber relPoseSub;
    ros::Subscriber absPoseSub;
    ros::ServiceClient imageStoreClient;
    std::vector<ros::Subscriber> mobotPoseSub;
    ros::Publisher infoPub;
    ros::ServiceServer updateRvizServer;

    // Test
    void testVisual(ImageMapVisual* visual_, std::string fileName);

    // 3D Scene
    Ogre::SceneNode* scene_node_;
    ImageMapVisual* visual_;

    // User-editable property variables.
    std::string relPoseTopic;
    std::string absPoseTopic;
    std::string imageStoreTopic;
    int mobotPoseCount;
    std::string mobotPoseCountStr;
    rviz::ROSTopicStringPropertyWPtr relPoseTopicProperty;
    rviz::ROSTopicStringPropertyWPtr absPoseTopicProperty;
    rviz::ROSTopicStringPropertyWPtr imageStoreTopicProperty;
    rviz::ROSTopicStringPropertyWPtr mobotPoseCountProperty;
};

}
#endif //IMAGE_MAP_DISPLAY_H
