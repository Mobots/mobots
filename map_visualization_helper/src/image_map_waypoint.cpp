#include "image_map_waypoint.h"

namespace map_visualization{

ImageMapWaypoint::ImageMapWaypoint(int argc, char** argv):
    init_argc(argc) ,
    init_argv(argv) ,
    activeMobotID(0) ,
    activeSessionID(0)
{
}

ImageMapWaypoint::~ImageMapWaypoint(){
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

void ImageMapWaypoint::run(){
    ros::init(init_argc, init_argv, "image_map_info");
    if ( ! ros::master::check() ) {
        return;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh_;
    nh = &nh_;
    // Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber poseRelaySub_ = nh->subscribe("/image_map/pose", 10,
                                          &ImageMapWaypoint::poseRelayHandler, this);
    poseRelaySub = &poseRelaySub_;
    ros::Publisher poseRelayPub_ = nh->advertise<mobots_msgs::PoseAndID>("/waypoint_user", 10);
    poseRelayPub = &poseRelayPub_;
    ros::Subscriber updateInfoSub_ = nh->subscribe("/image_map/update_push", 10,
                                                   &ImageMapWaypoint::updateInfoHandler, this);
    updateInfoSub = &updateInfoSub_;
    ros::spin();
}

// TODO resubscribe poseRelayPub
void ImageMapWaypoint::setActiveMobot(int mobotID){
    activeMobotID = mobotID;
    ROS_INFO("activeMobotID: %i", activeMobotID);
    return;
}

void ImageMapWaypoint::poseRelayHandler(const geometry_msgs::PoseStamped::ConstPtr& msgIn){
    ROS_INFO("[poseRouterCallback]");
    mobots_msgs::PoseAndID msgOut;

    msgOut.pose.x = msgIn->pose.position.x;
    msgOut.pose.y = msgIn->pose.position.y;
    // Extract rotation out of quaternion
    float theta = msgIn->pose.orientation.w;
    if(msgIn->pose.orientation.z < 0){
        theta *= -1;
    }
    msgOut.pose.theta = 2 * acos(theta);
    msgOut.id.mobot_id = activeMobotID;
    ROS_INFO("Position(%f, %f, %f), Orientation(%f, %f, %f, %f) = Angle: %f",
             msgIn->pose.position.x, msgIn->pose.position.y, msgIn->pose.position.z,
             msgIn->pose.orientation.x, msgIn->pose.orientation.y, msgIn->pose.orientation.z,
             msgIn->pose.orientation.w, msgOut.pose.theta * 180 / PI);
    poseRelayPub->publish(msgOut);
    return;
}

void ImageMapWaypoint::updateInfoHandler(const mobots_msgs::IDKeyValue::ConstPtr& msg){
    ROS_INFO("[updateInfoHandler] s%im%ii%i: %s-%s", msg->id.session_id, msg->id.mobot_id,
             msg->id.image_id, msg->key.c_str(), msg->value.c_str());
    return;
}

}
