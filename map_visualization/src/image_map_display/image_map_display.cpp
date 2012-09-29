#include <boost/filesystem.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>

#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/PoseAndID.h"

#include "image_map_visual.h"
#include "image_map_display.h"
#include "image_map_info.h"

namespace map_visualization{
	
ImageMapDisplay::ImageMapDisplay()
  : Display()
  , scene_node_(NULL)
  , visual_(NULL)
  , info(NULL)
{
    startQT();
}

ImageMapDisplay::~ImageMapDisplay(){
	ROS_INFO("delete");
	unsubscribe();
	ROS_INFO("delete");
}

// Clear the map by deleting image_map_visual object
void ImageMapDisplay::clear(){
	ROS_INFO("clear");
	if(visual_ != NULL){
		delete visual_;
	}
    if(info != NULL){ // reset mobot_info
        delete info;
    }
    if(info_thread != NULL){ // quit the Qt Application if it is started
        qtApp.quit();
    }
	visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), scene_node_);
    startQT();
}

// start the Qt Application and launch the qtThread
void startQT(){
    qtApp = new QApplication(0, NULL);
    info = new Mobots_Info(0);
    info.show();
    info_thread = new boost::thread(qtThread());
}

// this Thread will terminatet when the slot quit() is called
void qtThread(){
    return qtApp->exec();
}

// After the parent rviz::Display::initialize() does its own setup, it
// calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.
// TODO implement service
void ImageMapDisplay::onInitialize(){
  	ROS_INFO("[onInitialize]");
	//absPoseSub_ = NULL;
	setStatus(rviz::status_levels::Warn, "Topic", "Finished Initializing");
}

void ImageMapDisplay::onEnable(){
	ROS_INFO("onenable");
	subscribe();
	visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), scene_node_);
	testVisual(visual_, "/home/moritz/TillEvil.jpg");
	ROS_INFO("onenable");
}

void ImageMapDisplay::onDisable(){
	ROS_INFO("ondisable");
	unsubscribe();
	clear();
	ROS_INFO("ondisable");
}

void ImageMapDisplay::subscribe(){
	ROS_INFO("subscribe");
	if(!isEnabled()){
		return;
	}
	if(!relPoseTopic.empty()){
		try{
			ROS_INFO("Subscribing");
			relPoseSub = update_nh_.subscribe
				(relPoseTopic, 1, &ImageMapDisplay::relPoseCallback, this);
			setStatus(rviz::status_levels::Ok, "Topic", "OK");
		}
		catch(ros::Exception& e){
			setStatus(rviz::status_levels::Error, "Topic", std::string
				("Error subscribing Relative: ") + e.what());
		}
	}
	/*if(!absPoseTopic.empty()){
		try{
			ros::Subscriber absPoseSub = update_nh_.subscribe(absPoseTopic, 10,
				&ImageMapDisplay::absPoseHandler, this);
			absPoseSub_ = &absPoseSub;
			setStatus(rviz::status_levels::Ok, "Topic", "OK");
		}
		catch(ros::Exception& e){
		setStatus(rviz::status_levels::Error, "Topic", std::string
			("Error subscribing: ") + e.what());
		}
	}*/
	ROS_INFO("subscribe");
}

// Find the equivalent to unsubscribe
void ImageMapDisplay::unsubscribe(){
	ROS_INFO("unsubscribe");
	relPoseSub.shutdown();
	/*if(absPoseSub_ != NULL){
		absPoseSub_->shutdown();
		absPoseSub_ = NULL;	
	}*/
}



void ImageMapDisplay::setRelPoseTopic(const std::string& topic){
	ROS_INFO("setRelPoseTopic: %s", relPoseTopic.c_str());
	unsubscribe();
	relPoseTopic = topic;
	subscribe();
	clear();
	// Broadcast the fact that the variable has changed.
	propertyChanged(relPoseTopicProperty);
	
	// Make sure rviz renders the next time it gets a chance.
	causeRender();
	ROS_INFO("setRelPoseTopic: %s", relPoseTopic.c_str());
}

void ImageMapDisplay::setAbsPoseTopic(const std::string& topic){
	ROS_INFO("setAbsPoseTopic: %s", topic.c_str());
	unsubscribe();
	clear();
	absPoseTopic = topic;
	subscribe();

	// Broadcast the fact that the variable has changed.
	propertyChanged(absPoseTopicProperty);

	// Make sure rviz renders the next time it gets a chance.
	causeRender();
	ROS_INFO("setAbsPoseTopic");
}



// TODO pass poses to image_map_info
void ImageMapDisplay::relPoseCallback(
	const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg){
	ROS_INFO("[imageRelPoseHandler]");
	visual_->insertImage(msg->pose.x, msg->pose.y, msg->pose.theta,
		msg->id.session_id, msg->id.mobot_id, msg->id.image_id,
		&msg->image.data, &msg->image.encoding,
		msg->image.width, msg->image.height);

    info->addPicture(msg->id.mobot_id); // add picture to the picture counter
}

// TODO pass poses to image_map_info
// TODO pass the information about the absolute pose to Mobot_Info
void ImageMapDisplay::absPoseHandler(
	const mobots_msgs::PoseAndID::ConstPtr& msg){
	ROS_INFO("[imageAbsPoseHandler]");
	visual_->setPose(msg->id.session_id, msg->id.mobot_id,	msg->id.image_id,
		msg->pose.x, msg->pose.y, msg->pose.theta);
}

// Override rviz::Display's reset() function to add a call to clear().
void ImageMapDisplay::reset(){
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
	relPoseTopicProperty = property_manager_->createProperty<rviz::ROSTopicStringProperty>(
		"RelativePoseTopic", property_prefix_,
		boost::bind(&ImageMapDisplay::getRelPoseTopic, this),
		boost::bind(&ImageMapDisplay::setRelPoseTopic, this, _1),
		parent_category_, this );
	absPoseTopicProperty = property_manager_->createProperty<rviz::ROSTopicStringProperty>(
		"AbsolutePoseTopic", property_prefix_,
		boost::bind(&ImageMapDisplay::getAbsPoseTopic, this),
		boost::bind(&ImageMapDisplay::setAbsPoseTopic, this, _1),
		parent_category_, this );
		
	setPropertyHelpText(relPoseTopicProperty, "Relative pose topic to subscribe to.");
	setPropertyHelpText(absPoseTopicProperty, "Absolute pose topic to subscribe to.");
	rviz::ROSTopicStringPropertyPtr relPoseTopicProp = relPoseTopicProperty.lock();
	rviz::ROSTopicStringPropertyPtr absPoseTopicProp = absPoseTopicProperty.lock();
	relPoseTopicProp->setMessageType
		(ros::message_traits::datatype<mobots_msgs::ImageWithPoseAndID>());
	absPoseTopicProp->setMessageType
		(ros::message_traits::datatype<mobots_msgs::PoseAndID>());
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
}

} // end namespace map_visualization

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(map_visualization, ImageMap,
	map_visualization::ImageMapDisplay, rviz::Display)
