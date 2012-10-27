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

/*bool ImageMapWaypoint::init(){
    ros::init(init_argc, init_argv, "image_map_info");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh_;
    nh = &nh_;
    // Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber poseRelaySub_ = nh->subscribe("/image_map/pose", 10,
                                          &ImageMapWaypoint::poseRelayHandler, this);
    poseRelaySub = &poseRelaySub_;
    ros::Publisher poseRelayPub_ = nh->advertise<geometry_msgs::Pose2D>("/mobot1/waypoint_user", 10);
    poseRelayPub = &poseRelayPub_;
    start();
    return true;
}*/

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
    ros::Publisher poseRelayPub_ = nh->advertise<geometry_msgs::Pose2D>("/mobot1/waypoint_user", 10);
    poseRelayPub = &poseRelayPub_;
    ros::Subscriber updateInfoSub_ = nh->subscribe("/image_map/update_push", 10,
                                                   &ImageMapWaypoint::updateInfoHandler, this);
    updateInfoSub = &updateInfoSub_;
    qDebug() << "hello from worker thread " << thread()->currentThreadId();
    ROS_INFO("I'm alive");
    ros::spin();
}

void ImageMapWaypoint::setActiveMobot(int mobotID){
    activeMobotID = mobotID;
    ROS_INFO("activeMobotID: %i", activeMobotID);
    return;
}

void ImageMapWaypoint::poseRelayHandler(const geometry_msgs::PoseStamped::ConstPtr& msgIn){
    ROS_INFO("[poseRouterCallback]");
    geometry_msgs::Pose2D msgOut;

    msgOut.x = msgIn->pose.position.x;
    msgOut.y = msgIn->pose.position.y;
    // Extract rotation out of quaternion
    float theta = msgIn->pose.orientation.w;
    if(msgIn->pose.orientation.z < 0){
        theta *= -1;
    }
    msgOut.theta = 2 * acos(theta);
    ROS_INFO("Position(%f, %f, %f), Orientation(%f, %f, %f, %f) = Angle: %f", msgIn->pose.position.x,
             msgIn->pose.position.y, msgIn->pose.position.z, msgIn->pose.orientation.x,
             msgIn->pose.orientation.y, msgIn->pose.orientation.z, msgIn->pose.orientation.w,
             msgOut.theta * 180 / PI);
    poseRelayPub->publish(msgOut);
    return;
}

}
