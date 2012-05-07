#include "shutter.h"

Shutter::Shutter(int mobot_ID):id(mobot_ID)
{
    ros::init(argv, argc, "shutter_" << mobot_ID);
    ros::NodeHandle nh;
    Shutter::startShutter();
}

Shutter::~Shutter() {
}

void Shutter::startShutter()
{
    poseImage_pub = nh.advertise("/mobot_pose/ImagePoseID", 2);

    image_sub = nh.subscribe("/usb_cam/image_raw", 5, &Shutter::imageCallback, this);
    pose_sub = nh.subscribe("/mouse/pose", 100, &Shutter::mouseCallback, this);

    dX = 0;
    dY = 0;
    dTheta = 0;
    globalTheta = 0;
    globalX = 0;
    globalY = 0;

    ros::spin();

}

void Shutter::publishMessage(double &x, double &y, double &theta, sensor_msgs::Image &image) {
    ipid.Mobot_ID = id;
    ipid.image = image;

    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    ipid.pose = pose;
    poseImage_pub.publish(ipid);

}

double Shutter::checkPicture(double x, double y, double theta) {
    double r = sqrt((x*x/4) + (y*y/4));
    return 1 - (r/y);
}


void Shutter::imageCallback(const sensor_msgs::Image &mobot_image) {
    if (checkPicture(dX, dY, dTheta) < overlap) {
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
