#include "image_map_display.h"

namespace map_visualization{
	
ImageMapDisplay::ImageMapDisplay()
  : Display()
  , scene_node_(NULL)
  , visual_(NULL)
{
}

ImageMapDisplay::~ImageMapDisplay(){
	ROS_INFO("delete");
	unsubscribe();
	delete visual_;
	ROS_INFO("delete");
}

// Clear the map by deleting image_map_visual object
void ImageMapDisplay::clear(){
	ROS_INFO("clear");
	delete visual_;
	visual_ = new ImageMapVisual(vis_manager_->getSceneManager());
}

// After the parent rviz::Display::initialize() does its own setup, it
// calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.
// TODO implement service
void ImageMapDisplay::onInitialize(){
	ROS_INFO("[onInitialize]");
	setStatus(rviz::status_levels::Warn, "Topic", "Finished Initializing");
}

void ImageMapDisplay::onEnable(){
	ROS_INFO("onenable");
	subscribe();
  visual_ = new ImageMapVisual(vis_manager_->getSceneManager());
  testVisual(visual_, "/home/moritz/TillEvil.jpg");
  qtEnable();
	ROS_INFO("onenable");
}

void ImageMapDisplay::onDisable(){
	ROS_INFO("[onDisable]");
	unsubscribe();
	delete visual_;
  visual_ = NULL;
  qtEnable();
	ROS_INFO("[onDisable]");
}

void ImageMapDisplay::qtEnable(){
  ROS_INFO("[qtEnable]");
  widget_ = new DriveWidget(0);
  rviz::WindowManagerInterface* wm = vis_manager_->getWindowManager();
  if(wm){
    panel_container_ = wm->addPane(name_, widget_);
  }
  ROS_INFO("[qtEnable]");
}

void ImageMapDisplay::qtDisable(){
  ROS_INFO("[qtDisable]");
  delete panel_container_;
  delete widget_;
  ROS_INFO("[qtDisable]");
}

void ImageMapDisplay::subscribe(){
	ROS_INFO("[subscribe]");
	if(!isEnabled()){
		return;
	}
	if(!relPoseTopic.empty()){
		try{
			ROS_INFO("Subscribing");
			relPoseSub = update_nh_.subscribe(relPoseTopic, 3,
				&ImageMapDisplay::relPoseCallback, this);
			setStatus(rviz::status_levels::Ok, "Topic", "OK");
		}
		catch(ros::Exception& e){
			setStatus(rviz::status_levels::Error, "Topic", std::string
				("Error subscribing Relative: ") + e.what());
		}
	}
	if(!absPoseTopic.empty()){
		try{
			ROS_INFO("Subscribing");
			absPoseSub = update_nh_.subscribe(absPoseTopic, 10,
				&ImageMapDisplay::absPoseCallback, this);
			setStatus(rviz::status_levels::Ok, "Topic", "OK");
		}
		catch(ros::Exception& e){
			setStatus(rviz::status_levels::Error, "Topic", std::string
				("Error subscribing Absolute: ") + e.what());
		}
	}
	ROS_INFO("[subscribe]");
}

void ImageMapDisplay::unsubscribe(){
	ROS_INFO("[unsubscribe]");
	relPoseSub.shutdown();
	absPoseSub.shutdown();
	ROS_INFO("[unsubscribe]");
}

void ImageMapDisplay::setRelPoseTopic(const std::string& topic){
	ROS_INFO("setRelPoseTopic: %s", relPoseTopic.c_str());
	unsubscribe();
	clear();
	relPoseTopic = topic;
	subscribe();
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
	ROS_INFO("[imageRelPoseCallback]");
	visual_->insertImage(msg->pose.x, msg->pose.y, msg->pose.theta,
		msg->id.session_id, msg->id.mobot_id, msg->id.image_id,
		&msg->image.data, &msg->image.encoding,
		msg->image.width, msg->image.height);
}

// TODO pass the information about the absolute pose to Mobot_Info
void ImageMapDisplay::absPoseCallback(
	const mobots_msgs::PoseAndID::ConstPtr& msg){
	ROS_INFO("[imageAbsPoseCallback]");
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

} // end namespace rviz_plugin_display

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(map_visualization, ImageMapDisplay,
  map_visualization::ImageMapDisplay, rviz::Display)