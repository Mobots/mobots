#include <map_visualization/definitions.h>
#include <QCoreApplication>

#include <ros/duration.h>

#include "image_map_waypoint.h"

namespace map_visualization{

ImageMapWaypoint::ImageMapWaypoint(int argc, char** argv):
    init_argc(argc) ,
    init_argv(argv) ,
    activeMobotID(0) ,
    activeSessionID(0) ,
    mobotPoseCount(3)
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

    // Subscribe to the pose topic of each mobot
    if(mobotPoseCount > 0){
        try{
            ros::Subscriber sub;
            std::string topic;
            while(mobotPoseSub.size() < mobotPoseCount){
                topic = "/mobot" + boost::lexical_cast<std::string>(mobotPoseSub.size());
                topic += "/pose";

                boost::function<void(const geometry_msgs::Pose2D::ConstPtr&)> handler =
                        boost::bind(&ImageMapWaypoint::mobotPoseHandler, this,
                        mobotPoseSub.size(), _1);
                sub = nh->subscribe<geometry_msgs::Pose2D>(topic, 1, handler);
                mobotPoseSub.push_back(sub);
            }
        }
        catch(ros::Exception& e){
            std::string error = e.what();
            ROS_ERROR("Error connecting to mobot: %s", error.c_str());
        }
    }

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
    if(mobotPoseBuffer.size() <= activeMobotID){
        poseT zeroPose = {0,0,0};
        mobotPoseBuffer.resize(activeMobotID + 1, zeroPose);
    }
    mobots_msgs::PoseAndID msgOut;

    msgOut.pose.x = msgIn->pose.position.x - mobotPoseBuffer[activeMobotID].x;
    msgOut.pose.y = msgIn->pose.position.y - mobotPoseBuffer[activeMobotID].y;;
    // Extract rotation out of quaternion. Undefined behavior of pose rotation.
    // Check out 2d Nav Goal behavior of Rviz (fuerte)
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

void ImageMapWaypoint::mobotPoseHandler(int mobotID, const geometry_msgs::Pose2D::ConstPtr &msg){
    if(mobotPoseBuffer.size() <= mobotID){
        poseT zeroPose = {0,0,0};
        mobotPoseBuffer.resize(mobotID + 1, zeroPose);
    }
    poseT pose = {msg->x, msg->y, msg->theta};
    mobotPoseBuffer[mobotID] = pose;
}

/**********************************************************
  * Qt Slots
  *********************************************************/

// Slot to change the ID of the waypoints relayed through
// TODO resubscribe poseRelayPub
// TODO connect from QComboBox
void ImageMapWaypoint::setActiveMobot(QString mobotID){
    activeMobotID = mobotID.toInt();
    return;
}

}
