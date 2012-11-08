#include <map_visualization/definitions.h>
#include <QCoreApplication>

#include <ros/duration.h>

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
        nh->shutdown();
    }
}

/**********************************************************
  * Main Thread
  *********************************************************/

void ImageMapWaypoint::process(){
    ros::init(init_argc, init_argv, "image_map_info");
    if ( ! ros::master::check() ) {
        return;
    }
    // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh_;
    nh = &nh_;

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

    while(ros::ok()){
        QCoreApplication::processEvents();
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    Q_EMIT finished();
}

/**********************************************************
  * ROS Callbacks / Publishers / Service Clients
  *********************************************************/

// Handles the incoming user waypoint (pose) from rviz. Adds the mobotID
void ImageMapWaypoint::poseRelayHandler(const geometry_msgs::PoseStamped::ConstPtr& msgIn){
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
    poseRelayPub->publish(msgOut);
    ros::spinOnce();
    return;
}

// Handles the incoming updates about the 3D scene in Rviz
void ImageMapWaypoint::updateInfoHandler(const mobots_msgs::IDKeyValue::ConstPtr& msg){

    Q_EMIT rvizChanged(msg->id.session_id, msg->id.mobot_id, msg->key, msg->value);
    return;}

// Sends commands (remote procedure calls) to the 3D scene in Rviz
int ImageMapWaypoint::updateRviz(int sessionID, int mobotID, int key, int value){
    ROS_INFO("[updateRviz]");
    map_visualization::RemoteProcedureCall srv;
    srv.request.id.session_id = sessionID;
    srv.request.id.mobot_id = mobotID;
    switch(key){
    case ENABLED:
        switch(value){
        case -1:
            srv.request.function = DELETE_MOBOT;
            break;
        case 0:
            srv.request.function = HIDE_MOBOT;
            break;
        case 1:
            srv.request.function = SHOW_MOBOT;
            break;
        }
        break;
    case RELATIVE:
        switch(value){
        case -1:
            srv.request.function = DELETE_RELATIVE_MOBOT;
            break;
        case 0:
            srv.request.function = SHOW_ABSOLUTE_MOBOT;
            break;
        case 1:
            srv.request.function = SHOW_RELATIVE_MOBOT;
            break;
        }
        break;
    case ABSOLUTE:
        switch(value){
        case -1:
            srv.request.function = DELETE_ABSOLUTE_MOBOT;
            break;
        case 0:
            srv.request.function = SHOW_RELATIVE_MOBOT;
            break;
        case 1:
            srv.request.function = SHOW_ABSOLUTE_MOBOT;
            break;
        }
        break;
    }

    if (updateRvizClient->call(srv)){
        ROS_INFO("Called Rviz: %i() = %i", srv.request.function, srv.response.result);
    } else {
        ROS_ERROR("Call Rviz failed: %i() = %i", srv.request.function, srv.response.result);
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
    //ROS_INFO("activeMobotID: %i", activeMobotID);
    return;
}

}
