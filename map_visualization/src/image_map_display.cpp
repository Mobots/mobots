#include "image_map_display.h"

namespace map_visualization{
	
ImageMapDisplay::ImageMapDisplay()
  : Display()
  , scene_node_(NULL)
  , visual_(NULL)
{
}

ImageMapDisplay::~ImageMapDisplay(){
    //ROS_INFO("delete");
	unsubscribe();
	delete visual_;
    //ROS_INFO("delete");
}

// Clear the map by deleting image_map_visual object
void ImageMapDisplay::clear(){
    //ROS_INFO("clear");
	delete visual_;
	visual_ = new ImageMapVisual(vis_manager_->getSceneManager());
}

// After the parent rviz::Display::initialize() does its own setup, it
// calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.
// TODO implement service
void ImageMapDisplay::onInitialize(){
    //ROS_INFO("[onInitialize]");
	setStatus(rviz::status_levels::Warn, "Topic", "Finished Initializing");
}

void ImageMapDisplay::onEnable(){
    //ROS_INFO("onenable");
	subscribe();
    visual_ = new ImageMapVisual(vis_manager_->getSceneManager());
    testVisual(visual_, "/home/moritz/TillEvil.jpg");
    //ROS_INFO("onenable");
}

void ImageMapDisplay::onDisable(){
    //ROS_INFO("[onDisable]");
	unsubscribe();
	delete visual_;
    visual_ = NULL;
    //ROS_INFO("[onDisable]");
}

void ImageMapDisplay::reset(){
    //ROS_INFO("reset");
    Display::reset();
    clear();
    //ROS_INFO("reset");
}

void ImageMapDisplay::subscribe(){
    //ROS_INFO("[subscribe]");
	if(!isEnabled()){
		return;
	}
    setStatus(rviz::status_levels::Ok, "Topic", "OK");
	if(!relPoseTopic.empty()){
		try{
            //ROS_INFO("Subscribing");
			relPoseSub = update_nh_.subscribe(relPoseTopic, 3,
				&ImageMapDisplay::relPoseCallback, this);
		}
		catch(ros::Exception& e){
			setStatus(rviz::status_levels::Error, "Topic", std::string
                ("Error subscribing relative: ") + e.what());
		}
	}
	if(!absPoseTopic.empty()){
		try{
            //ROS_INFO("Subscribing");
            absPoseSub = update_nh_.subscribe(absPoseTopic, 1000,
				&ImageMapDisplay::absPoseCallback, this);
		}
		catch(ros::Exception& e){
			setStatus(rviz::status_levels::Error, "Topic", std::string
                ("Error subscribing absolute: ") + e.what());
        }
	}
    if(!imageStoreTopic.empty()){
        try{
            ros::ServiceClient client = update_nh_.serviceClient<GetImageWithPose>(imageStoreTopic);
        }
        catch(ros::Exception& e){
            setStatus(rviz::status_levels::Error, "Topic", std::string
                ("Error connecting to image store: ") + e.what());
        }
    }
    return;
    //ROS_INFO("[subscribe]");
}

void ImageMapDisplay::unsubscribe(){
    //ROS_INFO("[unsubscribe]");
	relPoseSub.shutdown();
	absPoseSub.shutdown();
    imageStoreClient.shutdown();
    //ROS_INFO("[unsubscribe]");
}

void ImageMapDisplay::setRelPoseTopic(const std::string& topic){
    //ROS_INFO("setRelPoseTopic: %s", relPoseTopic.c_str());
	unsubscribe();
	clear();
	relPoseTopic = topic;
	subscribe();
	// Broadcast the fact that the variable has changed.
	propertyChanged(relPoseTopicProperty);
	// Make sure rviz renders the next time it gets a chance.
	causeRender();
    //ROS_INFO("setRelPoseTopic: %s", relPoseTopic.c_str());
}

void ImageMapDisplay::setAbsPoseTopic(const std::string& topic){
    //ROS_INFO("setAbsPoseTopic: %s", topic.c_str());
	unsubscribe();
	clear();
	absPoseTopic = topic;
	subscribe();
	propertyChanged(absPoseTopicProperty);
	causeRender();
    //ROS_INFO("setAbsPoseTopic");
}

void ImageMapDisplay::setImageStoreTopic(const std::string& topic){
    //ROS_INFO("setAbsPoseTopic: %s", topic.c_str());
    unsubscribe();
    clear();
    imageStoreTopic = topic;
    subscribe();
    propertyChanged(imageStoreTopicProperty);
    causeRender();
    //ROS_INFO("setAbsPoseTopic");
}

// TODO pass information to image_map_info
// TODO retrieveImageSeries
void ImageMapDisplay::relPoseCallback(
	const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg){
    //ROS_INFO("[imageRelPoseCallback]");
	visual_->insertImage(msg->pose.x, msg->pose.y, msg->pose.theta,
		msg->id.session_id, msg->id.mobot_id, msg->id.image_id,
		&msg->image.data, &msg->image.encoding,
		msg->image.width, msg->image.height);
    // If the first image is missing(ID=0), get all until the recieved image.
    if(visual_->findNode(msg->id.session_id, msg->id.mobot_id, 0) == NULL){
        ROS_INFO("[ImageMapDisplay] Attemting to complete missing image series");
        //retrieveImageSeries(msg->id.session_id, msg->id.mobot_id);
    }
}

// TODO pass information to image_map_info
void ImageMapDisplay::absPoseCallback(
	const mobots_msgs::PoseAndID::ConstPtr& msg){
    //ROS_INFO("[imageAbsPoseCallback]");
    if(visual_->setPose(msg->id.session_id, msg->id.mobot_id, msg->id.image_id,
        msg->pose.x, msg->pose.y, msg->pose.theta) != 0){
        ROS_INFO("[imageMapDisplay] No image to assign abs pose to");
    }
}

// TODO implemented dual pose(rel + abs) storage
// Long method. Through in thread? -> locking of visual...
void ImageMapDisplay::retrieveImages(int sessionID, int mobotID){
    GetImageWithPose srv;
    srv.request.id.session_id = sessionID;
    srv.request.id.mobot_id = mobotID;
    srv.request.id.image_id = 0;
    srv.request.type = 0;
    while(imageStoreClient.call(srv)){
        ROS_INFO("[retrieveImages] calling image store:s%im%ii%i",
                 srv.request.id.session_id, srv.request.id.mobot_id,
                 srv.request.id.image_id);
        // End of images or error. Errors in image_store pkg
        if(srv.response.error){
            ROS_INFO("[retrieveImages] error: %i", srv.response.error);
            return;
        }
        visual_->insertImage(srv.response.rel_pose.x, srv.response.rel_pose.y,
                srv.response.rel_pose.theta, srv.request.id.session_id,
                srv.request.id.mobot_id, srv.request.id.image_id,
                &srv.response.image.data, &srv.response.image.encoding,
                srv.response.image.width, srv.response.image.height);
        srv.request.id.image_id++;
    }
    return;
}

// Override createProperties() to build and configure a Property
// object for each user-editable property.  ``property_manager_``,
// ``property_prefix_``, and ``parent_category_`` are all initialized before
// this is called.
void ImageMapDisplay::createProperties(){
    //ROS_INFO("properties");
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
    imageStoreTopicProperty = property_manager_->createProperty<rviz::ROSTopicStringProperty>(
        "ImageStoreTopic", property_prefix_,
        boost::bind(&ImageMapDisplay::getImageStoreTopic, this),
        boost::bind(&ImageMapDisplay::setImageStoreTopic, this, _1),
        parent_category_, this );
		
    setPropertyHelpText(relPoseTopicProperty, "Relative pose topic to subscribe to.");
	setPropertyHelpText(absPoseTopicProperty, "Absolute pose topic to subscribe to.");
    //setPropertyHelpText(imageStoreTopicProperty, "Image Store topic to connect to.");
	rviz::ROSTopicStringPropertyPtr relPoseTopicProp = relPoseTopicProperty.lock();
	rviz::ROSTopicStringPropertyPtr absPoseTopicProp = absPoseTopicProperty.lock();
    //rviz::ROSTopicStringPropertyPtr imageStoreTopicProp = imageStoreTopicProperty.lock();
	relPoseTopicProp->setMessageType
		(ros::message_traits::datatype<mobots_msgs::ImageWithPoseAndID>());
	absPoseTopicProp->setMessageType
        (ros::message_traits::datatype<mobots_msgs::PoseAndID>());
//    imageStoreTopicProp->setMessageType
    //    (ros::message_traits::datatype<map_visualization::GetImageWithPose>());
    //ROS_INFO("properties");
}

void ImageMapDisplay::testVisual(ImageMapVisual* visual_, std::string filePath){
    //ROS_INFO("testVisual");
	std::ifstream imageFile(filePath.c_str(), std::ios::binary);
	if(!boost::filesystem::exists(filePath.c_str())){
        return;
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
    //ROS_INFO("testVisual");
	visual_->setPose(1,1,0, 0,0,0);
}

} // end namespace rviz_plugin_display

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(map_visualization, ImageMapDisplay,
  map_visualization::ImageMapDisplay, rviz::Display)
