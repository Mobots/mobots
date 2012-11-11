#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include "shutter/delta.h"
#include <mobots_msgs/ImageWithPoseAndID.h>
#include <math.h>
#include "geometry.h"
#include "libusb_cam/usb_cam.h"

#pragma once

//license notice missing
//pi fuckup behauptete sonst die funktion wär schonmal woanders definiert
static inline bool fillImage(sensor_msgs::Image& image,
						const std::string& encoding_arg,
						uint32_t rows_arg,
						uint32_t cols_arg,
						uint32_t step_arg,
						const void* data_arg){
	image.encoding = encoding_arg;
	image.height   = rows_arg;
	image.width    = cols_arg;
	image.step     = step_arg;
	size_t st0 = (step_arg * rows_arg);
	image.data.resize(st0);
	memcpy(&image.data[0], data_arg, st0);

	image.is_bigendian = 0;
	return true;
}

class Shutter {
	
public:
	
    Shutter(int mobotID, double l, double b);
    virtual ~Shutter();
    virtual void startShutter();

				/**
	 * treshold - run the geometry overlap check after every 'treshold' times the 
	 * mouse callback has been called
	 */
		static const int MOUSE_THRESHOLD = 0;
		
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
				void deflate();
		//getDelta-Service:
		virtual bool getDelta(shutter::delta::Request &req, shutter::delta::Response &res);

private:

		void imageCallback(const sensor_msgs::Image &mobot_image);
    virtual void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
		double checkPicture(double x, double y, double theta);
		void publishMessage(double x, double y, double theta, const sensor_msgs::Image& image);
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
		/**
		 * attempt at least CAM_QUERY_THRESHOLD-times to query an image from webcam to get 
		 * a valid pictures (bug in v4l2?)
		 */
		static const int CAM_QUERY_THRESHOLD;
		usb_cam_camera_image_t* camera_image_;
		int callbackCount;
		inline void grabImageMsg(mobots_msgs::ImageWithPoseAndID& msg);
		inline void publishMessage(double x, double y, double theta);
		virtual void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
				//getDelta-Service:
		virtual bool getDelta(shutter::delta::Request &req, shutter::delta::Response &res);
};

/**
 * ftw
 */
class Shutter3 : public Shutter , public UsbCamErrorHandler{
public:	
    Shutter3(int mobotID, double l, double b);
		void handleError(const char* error);
		void initCamera();
		virtual ~Shutter3();
    virtual void startShutter();
		
private:
		int imageWidth, imageHeight;
		int callbackCount;
		inline void publishMessage(double x, double y, double theta);
		virtual void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
};