#include "image_map_waypoint.h"

namespace map_visualization{

ImageMapWaypoint::ImageMapWaypoint(int argc, char** argv):
    init_argc(argc) ,
    init_argv(argv)
{}

ImageMapWaypoint::~ImageMapWaypoint(){
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool ImageMapWaypoint::init(){
    ros::init(init_argc,init_argv,"image_map_info");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber poseRouterSub = n.subscribe("/image_map/pose", 10,
            &ImageMapWaypoint::poseRelayHandler, this);
    ros::Publisher poseRouterPub = n.advertise<geometry_msgs::Pose2D>("/mobot1/waypoint_user", 10);


    start();
    return true;
}

void ImageMapWaypoint::run(){
    qDebug() << "hello from worker thread " << thread()->currentThreadId();
    ros::init(init_argc, init_argv, "image_map_info");
    ROS_INFO("I'm alive");
    ros::spin();
}

void ImageMapWaypoint::setActiveMobot(int mobotID){
    activeMobotID = mobotID;
    ROS_INFO("activeMobotID: %i", activeMobotID);
    return;
}

void ImageMapWaypoint::poseRelayHandler(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("[poseRouterCallback]");
    geometry_msgs::Pose2D msgOut;
    msgOut.x = msg->pose.position.x;
    msgOut.y = msg->pose.position.y;
    msgOut.theta = msg->pose.orientation.w;
    ROS_INFO("[poseRouterCallback] (x,y,theta) - (%f,%f,%f)", msgOut.x, msgOut.y, msgOut.theta);

    return;
}

}
