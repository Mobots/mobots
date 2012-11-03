#include "image_map_waypoint.h"

namespace map_visualization{

ImageMapWaypoint::ImageMapWaypoint(int argc, char** argv):
    init_argc(argc) ,
    init_argv(argv) ,
    activeMobotID(0) ,
    activeSessionID(0)
{
}

/**********************************************************
  * Initialization
  *********************************************************/

ImageMapWaypoint::~ImageMapWaypoint(){
    if(ros::isStarted()) {
        unsubscribe();
    }
}

void ImageMapWaypoint::process(){
    ros::init(init_argc, init_argv, "image_map_info");
    if ( ! ros::master::check() ) {
        return;
    }
    ROS_INFO("[Map_Visualization_Helper] running");
    // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh_;
    nh = &nh_;
    subscribe();
    ros::spin();
    Q_EMIT finished();
}

void ImageMapWaypoint::subscribe(){
    ROS_INFO("[Map_Visualization_Helper] subscribing");
    // Input: User Waypoints from Rviz
    ros::Subscriber poseRelaySub_ = nh->subscribe("/image_map/pose", 10,
            &ImageMapWaypoint::poseRelayHandler, this);
    poseRelaySub = &poseRelaySub_;

    // Output: User Waypoints to path_planner
    ros::Publisher poseRelayPub_ = nh->advertise
            <mobots_msgs::PoseAndID>("/path_planner/waypoint_user", 10);
    poseRelayPub = &poseRelayPub_;

    // Input: Update data in the table
    ros::Subscriber updateInfoSub_ = nh->subscribe("/image_map/update_push", 10,
            &ImageMapWaypoint::updateInfoHandler, this);
    updateInfoSub = &updateInfoSub_;

    // Input/Output: Update state in the Rviz 3D scene
    ros::ServiceClient updateRvizClient_ = nh->serviceClient
            <map_visualization::RemoteProcedureCall>("/image_map/rpc");
    updateRvizClient = &updateRvizClient_;
}

void ImageMapWaypoint::unsubscribe(){
    // Shutdown every handle created through this NodeHandle.
    nh->shutdown();
}

/**********************************************************
  * ROS Callbacks / Publishers / Service Clients
  *********************************************************/

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
    ros::spinOnce();
    return;
}

void ImageMapWaypoint::updateInfoHandler(const mobots_msgs::IDKeyValue::ConstPtr& msg){
    ROS_INFO("[updateInfoHandler] s%im%ii%i: %i-%i", msg->id.session_id, msg->id.mobot_id,
             msg->id.image_id, msg->key, msg->value);
    Q_EMIT dataChanged(msg->id.session_id, msg->id.mobot_id, msg->key, msg->value);
    return;
}

int ImageMapWaypoint::updateRviz(int function, std::string operands){
    ROS_INFO("[updateRviz]");
    map_visualization::RemoteProcedureCall srv;
    srv.request.function = function;
    srv.request.operands = operands;

    if (updateRvizClient->call(srv)){
        ROS_INFO("Called Rviz: %i(%s) = %i", function, operands.c_str(),
                srv.response.result);
    } else {
        ROS_ERROR("Call Rviz failed: %i(%s) = %i", function,
                operands.c_str(), srv.response.result);
        return -1;
    }
    return srv.response.result;
}

/**********************************************************
  * Qt Slots
  *********************************************************/

// Slot to change the ID of the waypoints relayed through
// TODO resubscribe poseRelayPub
// TODO connect from QComboBox
void ImageMapWaypoint::setActiveMobot(QString mobotID){
    activeMobotID = mobotID.toInt();
    ROS_INFO("activeMobotID: %i", activeMobotID);
    return;
}

}
