#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include "shutter/delta.h"
#include <mobots_msgs/ImageWithPoseAndID.h>
#include <math.h>
#include "geometry.h"

#pragma once

class Shutter {

public:
    Shutter(int mobotID, double l, double b);
    virtual ~Shutter();
    virtual void startShutter();

protected:
    ros::Subscriber pose_sub;
    ros::Subscriber image_sub;
    ros::Publisher poseImage_pub;
    mobots_msgs::ImageWithPoseAndID ipid;
    
		int sessionID;
		int imageID;
		int mobotID;

		Geometry g;
		double overlap, dX, dY, dTheta;
		ros::NodeHandle nh;
		
		void publishMessage(double& x, double& y, double& theta, const sensor_msgs::Image &image);

private:
		void imageCallback(const sensor_msgs::Image &mobot_image);
    virtual void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
		double checkPicture(double x, double y, double theta);
		
		//getDelta-Service:
		bool getDelta(shutter::delta::Request &req, shutter::delta::Response &res);
};

/**
 * This shutter version polls the pictures itself from the usb cam
 * instead of receiving it over a tcp socket from usb_cam node
 */
class Shutter2 : public Shutter{
public:
    Shutter2(int mobotID, double l, double b);
		virtual ~Shutter2();
    virtual void startShutter();
		
private:
		virtual void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
};