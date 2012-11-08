#include "image_map_display.h"

namespace map_visualization{
	
ImageMapDisplay::ImageMapDisplay()
  : Display()
  , scene_node_(NULL)
  , visual_(NULL)
  , mobotPoseCount(0)
{
}

ImageMapDisplay::~ImageMapDisplay(){
	unsubscribe();
	delete visual_;
}

// Clear the map by deleting image_map_visual object
void ImageMapDisplay::clear(){
	delete visual_;
    visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), this);
}

// After the parent rviz::Display::initialize() does its own setup, it
// calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.
// TODO implement service
void ImageMapDisplay::onInitialize(){
	setStatus(rviz::status_levels::Warn, "Topic", "Finished Initializing");
}

void ImageMapDisplay::onEnable(){
	subscribe();
    visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), this);
    //testVisual(visual_, "/home/moritz/TillEvil.jpg");
}

void ImageMapDisplay::onDisable(){
	unsubscribe();
	delete visual_;
    visual_ = NULL;
}

void ImageMapDisplay::reset(){
    Display::reset();
    clear();
}

/**
 * Subscription to 3 static topics:
 *  - relative pose with image
 *  - absolute pose
 *  - image store get image and pose service
 * Subscription to each Mobot's topic:
 *  - pose
 */
void ImageMapDisplay::subscribe(){
	if(!isEnabled()){
		return;
	}
    setStatus(rviz::status_levels::Ok, "Topic", "OK");
	if(!relPoseTopic.empty()){
		try{
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
            imageStoreClient = update_nh_.serviceClient<GetImageWithPose>(imageStoreTopic);
        }
        catch(ros::Exception& e){
            setStatus(rviz::status_levels::Error, "Topic", std::string
                ("Error connecting to image store: ") + e.what());
        }
    }
    // Subscribe to the pose and image topics of each mobot
    if(mobotPoseCount > 0){
        try{
            ros::Subscriber sub;
            std::string topic;
            while(mobotPoseSub.size() < mobotPoseCount){
                topic = "/mobot" + boost::lexical_cast<std::string>(mobotPoseSub.size());
                topic += "/pose";

                boost::function<void(const geometry_msgs::Pose2D::ConstPtr&)> callback =
                        boost::bind(&ImageMapDisplay::mobotPoseCallback, this,
                        mobotPoseSub.size() + 1, _1);
                sub = update_nh_.subscribe<geometry_msgs::Pose2D>(topic, 10, callback);
                mobotPoseSub.push_back(sub);
            }
            ROS_INFO("mobotPoseSub size: %i", mobotPoseSub.size());
        }
        catch(ros::Exception& e){
            setStatus(rviz::status_levels::Error, "Topic", std::string
                ("Error connecting to mobot: ") + e.what());
        }
    }
    if(true){
        try{
            infoPub = update_nh_.advertise<mobots_msgs::IDKeyValue>("/image_map/update_push", 10);
        }
        catch(ros::Exception& e){
            ROS_ERROR("[Rviz] failed to create publisher");
            setStatus(rviz::status_levels::Error, "Topic", std::string
                ("Error advertising publisher infoPub : ") + e.what());
        }
    }
    if(true){
        updateRvizServer = update_nh_.advertiseService("/image_map/rpc", &ImageMapDisplay::updateRvizCallback, this);
    }
    return;
}

void ImageMapDisplay::unsubscribe(){
	relPoseSub.shutdown();
	absPoseSub.shutdown();
    imageStoreClient.shutdown();
    for(int i = 0; i < mobotPoseSub.size(); i++){
        mobotPoseSub[i].shutdown();
    }
    mobotPoseSub.clear();
}

void ImageMapDisplay::setRelPoseTopic(const std::string& topic){
    unsubscribe();
	clear();
	relPoseTopic = topic;
	subscribe();
	// Broadcast the fact that the variable has changed.
	propertyChanged(relPoseTopicProperty);
	// Make sure rviz renders the next time it gets a chance.
	causeRender();
}

void ImageMapDisplay::setAbsPoseTopic(const std::string& topic){
	unsubscribe();
	clear();
	absPoseTopic = topic;
	subscribe();
	propertyChanged(absPoseTopicProperty);
	causeRender();
}

void ImageMapDisplay::setImageStoreTopic(const std::string& topic){
    unsubscribe();
    clear();
    imageStoreTopic = topic;
    subscribe();
    propertyChanged(imageStoreTopicProperty);
    causeRender();
}

void ImageMapDisplay::setMobotPoseCount(const std::string& topic){
    unsubscribe();
    clear();
    try {
        mobotPoseCount = boost::lexical_cast<int>(topic);
    } catch( boost::bad_lexical_cast const& ) {
        mobotPoseCount = 0;
        ROS_ERROR("Error: mobot pose count was not valid %s", topic.c_str());
    }
    subscribe();
    propertyChanged(mobotPoseCountProperty);
    causeRender();
}

const std::string& ImageMapDisplay::getMobotPoseCount(){
    mobotPoseCountStr = boost::lexical_cast<std::string>(mobotPoseCount);
    return mobotPoseCountStr;
}

// TODO pass information to image_map_info
// TODO retrieveImageSeries
void ImageMapDisplay::relPoseCallback(
	const mobots_msgs::ImageWithPoseAndID::ConstPtr& msg){
    cv::Mat mat;
    if(msg->image.encoding == "jpg" || msg->image.encoding == "png"){
        mat = cv::imdecode(msg->image.data, 1);
    } else {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, "rgb8");
        mat = cv_ptr->image;
    }
    visual_->insertImage(msg->id.session_id, msg->id.mobot_id, msg->id.image_id,
                         msg->pose.x, msg->pose.y, msg->pose.theta,
                         mat);
    // If the first image is missing(ID=0), get all until the recieved image.
    /*if(visual_->findNode(msg->id.session_id, msg->id.mobot_id, 0) == NULL){
        ROS_INFO("[ImageMapDisplay] Attemting to retrieve missing images");
        retrieveImageSeries(msg->id.session_id, msg->id.mobot_id);
    }*/
}

// TODO pass information to image_map_info
void ImageMapDisplay::absPoseCallback(
	const mobots_msgs::PoseAndID::ConstPtr& msg){
    ROS_INFO("[absPoseCallback] pose(%f,%f,%f)", msg->pose.x, msg->pose.y, msg->pose.theta);
    if(visual_->setImagePose(msg->id.session_id, msg->id.mobot_id, msg->id.image_id,
                        msg->pose.x, msg->pose.y, msg->pose.theta, ImageMapVisual::ABSOLUTE_POSE_NODE) != 0){
        ROS_ERROR("[imageMapDisplay] No image to assign abs pose to");
    }
}

void ImageMapDisplay::mobotPoseCallback(int mobotID,
        const geometry_msgs::Pose2D::ConstPtr& msg){
    visual_->setMobotModel(mobotID, msg->x, msg->y, msg->theta);
}

// TODO implemented dual pose(rel + abs) storage
// Long method. Through in thread? -> locking of visual...
void ImageMapDisplay::retrieveImages(int sessionID, int mobotID){
    GetImageWithPose srv;
    srv.request.id.session_id = sessionID;
    srv.request.id.mobot_id = mobotID;
    srv.request.id.image_id = 0;
    srv.request.type = 0;
    cv::Mat mat;
    while(imageStoreClient.call(srv)){
        ROS_INFO("[retrieveImages] calling image store:s%im%ii%i",
                srv.request.id.session_id, srv.request.id.mobot_id,
                srv.request.id.image_id);
        // End of images or error. Errors in image_store pkg
        if(srv.response.error){
            ROS_ERROR("[retrieveImages] error: %i", srv.response.error);
            return;
        }
        if(srv.response.image.encoding == "jpg" || srv.response.image.encoding == "png"){
            mat = cv::imdecode(srv.response.image.data, 1);
        } else {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.image, "rgb8");
            mat = cv_ptr->image;
        }
        visual_->insertImage(srv.request.id.session_id, srv.request.id.mobot_id,
                srv.request.id.image_id, srv.response.rel_pose.x, srv.response.rel_pose.y,
                srv.response.rel_pose.theta, mat);
        srv.request.id.image_id++;
    }
    return;
}

void ImageMapDisplay::sendInfoUpdate(int sessionID, int mobotID, int key, int value){
    mobots_msgs::IDKeyValue msg;
    msg.id.session_id = sessionID;
    msg.id.mobot_id = mobotID;
    msg.key = key;
    msg.value = value;
    infoPub.publish(msg);
}

bool ImageMapDisplay::updateRvizCallback(map_visualization::RemoteProcedureCall::Request &req,
                                         map_visualization::RemoteProcedureCall::Response &res){
    int function = req.function;
    switch(function){
    case INSERT_IMAGE:
        ROS_INFO("[Rviz] Unsupported RPC: insert image");
        //visual_->insertImage(req.id.session_id, req.id.mobot_id, req.id.image_id,0,0,0,0);
        break;
    case SHOW_IMAGE:
        visual_->showImage(req.id.session_id, req.id.mobot_id, req.id.image_id);
        break;
    case HIDE_IMAGE:
        visual_->hideImage(req.id.session_id, req.id.mobot_id, req.id.image_id);
        break;
    case DELETE_IMAGE:
        ROS_INFO("[Rviz] Unsupported RPC: delete image");
        //visual_->deleteImage(std::string);
        break;

    case SHOW_MOBOT:
        visual_->showMobot(req.id.session_id, req.id.mobot_id);
        break;
    case HIDE_MOBOT:
        visual_->hideMobot(req.id.session_id, req.id.mobot_id);
        break;
    case DELETE_MOBOT:
        ROS_INFO("[Rviz] Unsupported RPC: delete mobot");
        //visual_->deleteMobot(std::string);
        break;

    case SHOW_SESSION:
        visual_->showSession(req.id.session_id);
        break;
    case HIDE_SESSION:
        visual_->hideSession(req.id.session_id);
        break;
    case DELETE_SESSION:
        ROS_INFO("[Rviz] Unsupported RPC: delete session");
        //visual_->deleteSession(std::string);
        break;

    case DELETE_ALL_IMAGES:
        visual_->deleteAllImages();
        break;

    case SET_IMAGE_POSE:
        ROS_INFO("[Rviz] Unsupported RPC: set image pose");
        //visual_->setImagePose(req.id.session_id, req.id.mobot_id, req.id.image_id, 0,0,0,0);
        break;

    case DELETE_MOBOT_MODEL:
        ROS_INFO("[Rviz] Unsupported RPC: delete mobot model");
        //visual_->deleteMobotModel(std::string);
        break;
    case DELETE_ALL_MOBOT_MODELS:
        visual_->deleteAllMobotModels();
        break;
    case SET_MOBOT_MODEL:
        ROS_INFO("[Rviz] Unsupported RPC: set mobot model");
        //visual_->setMobotModel(0,0,0,0);
        break;
    }
    res.result = 0;
    return true;
}

// Override createProperties() to build and configure a Property
// object for each user-editable property.  ``property_manager_``,
// ``property_prefix_``, and ``parent_category_`` are all initialized before
// this is called.
void ImageMapDisplay::createProperties(){
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
    mobotPoseCountProperty = property_manager_->createProperty<rviz::ROSTopicStringProperty>(
        "MobotPoseCount", property_prefix_,
        boost::bind(&ImageMapDisplay::getMobotPoseCount, this),
        boost::bind(&ImageMapDisplay::setMobotPoseCount, this, _1),
        parent_category_, this );
		
    setPropertyHelpText(relPoseTopicProperty, "Relative pose topic to subscribe to.");
	setPropertyHelpText(absPoseTopicProperty, "Absolute pose topic to subscribe to.");
    setPropertyHelpText(imageStoreTopicProperty, "Image Store topic to connect to.");
    setPropertyHelpText(mobotPoseCountProperty, "Number of mobots to connect to and display thier poses.");
    rviz::ROSTopicStringPropertyPtr relPoseTopicProp = relPoseTopicProperty.lock();
	rviz::ROSTopicStringPropertyPtr absPoseTopicProp = absPoseTopicProperty.lock();
    //rviz::ROSTopicStringPropertyPtr imageStoreTopicProp = imageStoreTopicProperty.lock();
    relPoseTopicProp->setMessageType
        (ros::message_traits::datatype<mobots_msgs::ImageWithPoseAndID>());
	absPoseTopicProp->setMessageType
        (ros::message_traits::datatype<mobots_msgs::PoseAndID>());
//    imageStoreTopicProp->setMessageType
    //    (ros::message_traits::datatype<map_visualization::GetImageWithPose>());
}

void ImageMapDisplay::testVisual(ImageMapVisual* visual_, std::string filePath){
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
    cv::Mat mat = cv::imdecode(imageData, 1);
    visual_->insertImage(0,0,0, 0,0,3.1415927, mat);
    visual_->insertImage(0,1,1, 1,1,1.5707963, mat);
    visual_->insertImage(0,2,2, 2,2,3.1415927, mat);
    visual_->insertImage(0,3,3, 3,3,4.712389, mat);
    visual_->insertImage(0,4,4, 4,4,6.2831853, mat);
    visual_->setMobotModel(1,0.5,0.6,0);
}

} // end namespace rviz_plugin_display

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(map_visualization, ImageMapDisplay,
  map_visualization::ImageMapDisplay, rviz::Display)
