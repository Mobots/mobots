#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include "shutter/delta.h"
#include <mobots_msgs/ImageWithPoseAndID.h>
#include <math.h>
#include "geometry.h"



class Shutter {


public:
    Shutter(double l, double b);
    ~Shutter();
    void imageCallback(const sensor_msgs::Image &mobot_image);
    void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
    void startShutter();
    void publishMessage(double& x, double& y, double& theta, const sensor_msgs::Image &image);
    //getDelta-Service:
    bool getDelta(shutter::delta::Request &req, shutter::delta::Response &res);
    double checkPicture(double x, double y, double theta);


private:
	 const char* TAG = "[shutter] ";
    ros::Subscriber pose_sub;
    ros::Subscriber image_sub;
    ros::Publisher poseImage_pub;
    mobots_msgs::ImageWithPoseAndID ipid;
    
	 int sessionID;
	 int imageID;
	 int mobotID;

    Geometry g;
    double overlap, dX, dY, dTheta;
    int argc;
    char argv;	
    ros::NodeHandle nh;

};
