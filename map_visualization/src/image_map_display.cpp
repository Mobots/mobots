#include "image_map_display.h"

//! Contains classes used to visualize a map
/*!
  These classes visualize a map of image tiles. They are are placed together to
  create a large map of the ground. The images are classified with thier ID so
  that they can later be manipulated (hidden, poses changed, deleted).
  */
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


//! Clear the map
/*!
  Clear the map by deleting and creating a new image_map_visual object
 */
void ImageMapDisplay::clear(){
	delete visual_;
    visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), this);
}

//! Instatiate the class
/*!
  After the parent (Rviz) instatiates itself, the plugin's onInitialize function
  is called and initializes its components.
 */
void ImageMapDisplay::onInitialize(){
	setStatus(rviz::status_levels::Warn, "Topic", "Finished Initializing");
}

//! Called on enabling the class
/*!
  The ROS and Ogre interfaces are started.
  */
void ImageMapDisplay::onEnable(){
	subscribe();
    if(visual_ == NULL){
        visual_ = new ImageMapVisual(vis_manager_->getSceneManager(), this);
    }
    testVisual(visual_, "/home/moritz/TillEvil.jpg");
}

//! Called on disabling the class
/*!
  The ROS interfaces are shutdown and Ogre cleared.
  */
void ImageMapDisplay::onDisable(){
	unsubscribe();
	delete visual_;
    visual_ = NULL;
}

//! Called on reseting the class
/*!
  The plugin parent is reset and the plugin's state is reset.
  \sa clear()
  */
void ImageMapDisplay::reset(){
    Display::reset();
    clear();
}

//! Start ROS interfaces: subscribers, publishers, service server and client
/*!
 Subscription to 3 static topics:
  - relative pose with image from the shutter through image store
  - absolute pose from toro
  - get image and pose service from image store
  - remote procedure calls from image map info
 Subscription to each Mobot's topic:
  - pose

  Topics can be changed through the Rviz interface. On change all interfaces
  are shutdown and restarted.
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
                        mobotPoseSub.size(), _1);
                sub = update_nh_.subscribe<geometry_msgs::Pose2D>(topic, 10, callback);
                mobotPoseSub.push_back(sub);
            }
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

//! Shutdown all ROS interfaces
/*!
  All ROS interfaces are shutdown and the pose subscribers are deleted, since
  thier count is variable.
  */
void ImageMapDisplay::unsubscribe(){
	relPoseSub.shutdown();
	absPoseSub.shutdown();
    imageStoreClient.shutdown();
    for(int i = 0; i < mobotPoseSub.size(); i++){
        mobotPoseSub[i].shutdown();
    }
    mobotPoseSub.clear();
    updateRvizServer.shutdown();
}

//! Set the topic name for the image with relative pose and ID subscriber
/*!
  All ROS interfaces are shutdown and then restarted after the topic name has
  been changed. By shutting down all interfaces, it simplifies the process
  */
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

//! Set the topic name for the absolute pose and ID subscriber
/*!
  All ROS interfaces are shutdown and then restarted after the topic name has
  been changed. By shutting down all interfaces, it simplifies the process
  */
void ImageMapDisplay::setAbsPoseTopic(const std::string& topic){
	unsubscribe();
	clear();
	absPoseTopic = topic;
	subscribe();
	propertyChanged(absPoseTopicProperty);
	causeRender();
}

//! Set the topic name for the image store service client
/*!
  All ROS interfaces are shutdown and then restarted after the topic name has
  been changed. By shutting down all interfaces, it simplifies the process
  */
void ImageMapDisplay::setImageStoreTopic(const std::string& topic){
    unsubscribe();
    clear();
    imageStoreTopic = topic;
    subscribe();
    propertyChanged(imageStoreTopicProperty);
    causeRender();
}

//! Set the number of mobots for which pose changes are subscribed to
/*!
  All ROS interfaces are shutdown and then restarted after the topic name has
  been changed. By shutting down all interfaces, it simplifies the process
  */
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

//! Getter function for the mobotPoseCount variable
/*!
  All ROS interfaces are shutdown and then restarted after the topic name has
  been changed. By shutting down all interfaces, it simplifies the process
  */
const std::string& ImageMapDisplay::getMobotPoseCount(){
    mobotPoseCountStr = boost::lexical_cast<std::string>(mobotPoseCount);
    return mobotPoseCountStr;
}

//! The callback for image with relative pose and ID subscriber
/*!
  An image is inserted into the 3D scene with the ID and pose provided.
  */
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
    // TODO retrieveImageSeries
    // If the first image is missing(ID=0), get all until the recieved image.
    /*if(visual_->findNode(msg->id.session_id, msg->id.mobot_id, 0) == NULL){
        ROS_INFO("[ImageMapDisplay] Attemting to retrieve missing images");
        retrieveImageSeries(msg->id.session_id, msg->id.mobot_id);
    }*/
}

//! The callback for the absolute pose and ID subscriber
/*!
  A new pose is added and set for the image identified with the ID. If the
  image does not exist the pose is not added.
  */
void ImageMapDisplay::absPoseCallback(
	const mobots_msgs::PoseAndID::ConstPtr& msg){
    ROS_INFO("[absPoseCallback] pose(%f,%f,%f)", msg->pose.x, msg->pose.y, msg->pose.theta);
    if(visual_->setImagePose(msg->id.session_id, msg->id.mobot_id, msg->id.image_id,
                        msg->pose.x, msg->pose.y, msg->pose.theta, ImageMapVisual::ABSOLUTE_POSE_NODE) != 0){
        ROS_ERROR("[imageMapDisplay] No image to assign abs pose to");
    }
}

//! The callback for setting the pose of the mobot model
void ImageMapDisplay::mobotPoseCallback(int mobotID,
        const geometry_msgs::Pose2D::ConstPtr& msg){
    visual_->setMobotModel(mobotID, msg->x, msg->y, msg->theta);
}

//! Retrieves the images sent before the plugin initialized.
/*!
  Implementation not complete nor tested.
  */
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

//! Output ROS interface to image_map_info
/*!
  Changes in the 3D scene are sent to image_map_info nodes. They include
  updates of the image count, changing the pose types, and deletion of images
  */
void ImageMapDisplay::sendInfoUpdate(int sessionID, int mobotID, int key, int value){
    mobots_msgs::IDKeyValue msg;
    msg.id.session_id = sessionID;
    msg.id.mobot_id = mobotID;
    msg.key = key;
    msg.value = value;
    infoPub.publish(msg);
}

//! Input ROS interface to image_map_info
/*!
  Requests to change the 3D scene recieved from image_map_info nodes. They include
  showing and hiding images, changing the pose types, and deletion of images
  */
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
        visual_->deleteImage(req.id.session_id, req.id.mobot_id, req.id.image_id);
        break;

    case SHOW_MOBOT_IMAGES:
        visual_->showMobotImages(req.id.session_id, req.id.mobot_id);
        break;
    case HIDE_MOBOT_IMAGES:
        visual_->hideMobotImages(req.id.session_id, req.id.mobot_id);
        break;
    case DELETE_MOBOT_IMAGES:
        visual_->deleteMobotImages(req.id.session_id, req.id.mobot_id);
        break;

    case SHOW_SESSION_IMAGES:
        visual_->showSessionImages(req.id.session_id);
        break;
    case HIDE_SESSION_IMAGES:
        visual_->hideSessionImages(req.id.session_id);
        break;
    case DELETE_SESSION_IMAGES:
        visual_->deleteSessionImages(req.id.session_id);
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
    case SHOW_RELATIVE_MOBOT:
        visual_->mobotToRelPose(req.id.session_id, req.id.mobot_id);
        break;
    case SHOW_RELATIVE_SESSION:
        visual_->sessionToRelPose(req.id.session_id);
        break;
    case DELETE_RELATIVE_MOBOT:
        ROS_INFO("[Rviz] Unsupported RPC: delete relative mobot pose");
        break;
    case DELETE_RELATIVE_SESSION:
        ROS_INFO("[Rviz] Unsupported RPC: delete relative session pose");
        break;
    case SHOW_ABSOLUTE_MOBOT:
        visual_->mobotToAbsPose(req.id.session_id, req.id.mobot_id);
        break;
    case SHOW_ABSOLUTE_SESSION:
        visual_->sessionToAbsPose(req.id.session_id);
        break;
    case DELETE_ABSOLUTE_MOBOT:
        ROS_INFO("[Rviz] Unsupported RPC: delete absolute mobot pose");
        break;
    case DELETE_ABSOLUTE_SESSION:
        ROS_INFO("[Rviz] Unsupported RPC: delete absolute session pose");
        break;
    default:
        ROS_INFO("[Rviz] Unsupported RPC: unknown call");
        res.result = -1;
        return true;
    }
    res.result = 0;
    return true;
}

//! Initialize Rviz properties
/*!
  Override createProperties() to build and configure a Property object for each
  user-editable property.  ``property_manager_``, ``property_prefix_``, and
  ``parent_category_`` are all initialized before this is called.
  */
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

//! Test class
/*!
  Sample images, poses, and mobot models are loaded into the scene
  */
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
    visual_->insertImage(0,1,0, 1,1,1.5707963, mat);
    visual_->insertImage(0,2,0, 2,2,3.1415927, mat);
    visual_->insertImage(0,3,0, 3,3,4.712389, mat);
    visual_->insertImage(0,4,0, 4,4,6.2831853, mat);
    visual_->setImagePose(0,0,0,0,0.5,0,ImageMapVisual::ABSOLUTE_POSE_NODE);
    visual_->setImagePose(0,1,0,1,1.5,0,ImageMapVisual::ABSOLUTE_POSE_NODE);
    visual_->setImagePose(0,2,0,2,2.5,0,ImageMapVisual::ABSOLUTE_POSE_NODE);
    visual_->setImagePose(0,3,0,3,3.5,0,ImageMapVisual::ABSOLUTE_POSE_NODE);
    visual_->setMobotModel(0,0.5,0.6,0);
    visual_->setMobotModel(1,-0.5,0.6,0);
    visual_->setMobotModel(2,-1,0.6,0);
}

}

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(map_visualization, ImageMapDisplay,
  map_visualization::ImageMapDisplay, rviz::Display)
