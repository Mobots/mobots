#include <cstdio>
#include <iostream>
#include <ros/ros.h>
//#include "shutter/delta.h"
#include <mobots_msgs/ImageWithDeltaPoseAndID.h>
#include <math.h>
#include "geometry.h"



class Shutter {


public:
    Shutter(int mobot_ID, double l,double b);
    ~Shutter();
    void imageCallback(const sensor_msgs::Image &mobot_image);
    void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
    void startShutter();
    void publishMessage(double& x, double& y, double& theta, const sensor_msgs::Image &image);
    //getDelta-Service:
    //bool getDelta(shutter::delta::Request &req, shutter::delta::Response &res); 
    double checkPicture(double x, double y, double theta);


    ros::Subscriber pose_sub;
    ros::Subscriber image_sub;
    ros::Publisher poseImage_pub;
    mobots_msgs::ImageWithDeltaPoseAndID ipid;
    
    
private:
    int id;
    Geometry g;
    double overlap, dX, dY, dTheta;
    int argc;
    char argv;	
    ros::NodeHandle nh;

};
