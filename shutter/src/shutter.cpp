#include "shutter.h"

int main(int argc, char** argv){
ros::init(argc, argv, "shutter");
Shutter shutter(0,1.06805,0.80104); //l/b für Simulator: 1.06805,0.80104
}
Shutter::Shutter(int mobot_ID,double l, double b):id(mobot_ID),g(l,b) //Instanzierung von Geometry
{
    argc = 0;
    std::stringstream s;
    s << "shutter_" << mobot_ID;
    ros::init(argc, (char**)argv, s.str());
    ros::NodeHandle nh;
    Shutter::startShutter(); 
}

Shutter::~Shutter() {
}

void Shutter::startShutter()
{
    ROS_INFO("Shutterfunktion gestartet.");
    poseImage_pub = nh.advertise<mobots_msgs::ImagePoseID>("/mobot_pose/ImagePoseID", 2);

    image_sub = nh.subscribe("/usb_cam/image_raw", 5, &Shutter::imageCallback, this);
    pose_sub = nh.subscribe("/mouse/pose", 100, &Shutter::mouseCallback, this);
    
    overlap = 0.2;
    dX = 0;
    dY = 0;
    dTheta = 0;
    globalTheta = 0;
    globalX = 0;
    globalY = 0;

    ros::spin();

}

void Shutter::publishMessage(double &x, double &y, double &theta, const sensor_msgs::Image &image) {
    ipid.Mobot_ID = id;
    ipid.image = image;

    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    ipid.pose = pose;
    poseImage_pub.publish(ipid);
    
    std::cout << "publish" << std::endl;

}



void Shutter::imageCallback(const sensor_msgs::Image &mobot_image) {
    std::cout << g.checkPicture(dX, dY, dTheta) << " - " << overlap << std::endl;
    if (g.checkPicture(dX, dY, dTheta) < overlap) {
        publishMessage(dX, dY, dTheta, mobot_image);
        dX = 0;
        dY = 0;
        dTheta = 0; // evtl. lock machen
    } 
}

void Shutter::mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
    dX += mouse_data.x;
    globalX += mouse_data.x;
    dY += mouse_data.y;
    globalY += mouse_data.y;
    dTheta += mouse_data.theta;
    globalTheta += mouse_data.theta;


}
