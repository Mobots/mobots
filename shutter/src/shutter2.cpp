#include "usb_cam/usb_cam.h"
#include "shutter.h"
#include <sensor_msgs/fill_image.h>

const int Shutter2::MOUSE_THRESHOLD = 5;
const int Shutter2::CAM_QUERY_THRESHOLD = 6;

Shutter2::Shutter2(int mobotID, double l, double b): Shutter(mobotID, l, b){
	camera_image_ = usb_cam_camera_start("/dev/video1",
			IO_METHOD_MMAP,
			PIXEL_FORMAT_YUYV,
			640,
			480);
}

Shutter2::~Shutter2(){
}

void Shutter2::startShutter(){
	ROS_INFO("Shutter: using poll method");
	Shutter::startShutter();
}

/*void Shutter2::imageCallback(const sensor_msgs::Image &mobot_image) {
    double currentOverlap = g.checkPicture(dX, dY, dTheta); //entspricht der derzeitigen überlappung
    frame++;
    if(frame == 25){
      std::cout << "Überlappung: " << currentOverlap << "of " << overlap << "  => " << currentOverlap/overlap*100 << "%" << std::endl;
      frame = 0;
    }
    if (currentOverlap < overlap) {
    std::cout << "shuttered" << std::endl;
        publishMessage(dX, dY, dTheta, mobot_image);
        dX = 0;
        dY = 0;
        dTheta = 0;
    }
}*/

inline void Shutter2::publishMessage(double x, double y, double theta) {
	geometry_msgs::Pose2D pose;
	pose.x = x;
	pose.y = y;
	pose.theta = theta;

	ipid.pose = pose;
	ipid.id.image_id = imageID;
	poseImage_pub.publish(ipid);
	imageID++;
}

inline void Shutter2::grabImageMsg(mobots_msgs::ImageWithPoseAndID& msg){
	for(int i = 0; i < CAM_QUERY_THRESHOLD; i++)
		usb_cam_camera_grab_image(camera_image_);
	fillImage(msg.image, "rgb8", camera_image_->height, camera_image_->width, 3 * camera_image_->width, camera_image_->image);
	
}

void Shutter2::mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
    dX += mouse_data.x;
    dY += mouse_data.y;
    dTheta += mouse_data.theta;
		
		callbackCount++;
		if(callbackCount >= Shutter2::MOUSE_THRESHOLD){
			callbackCount = 0;
			double currentOverlap = g.checkPicture(dX, dY, dTheta); //entspricht der derzeitigen überlappung
			if (currentOverlap < overlap) {
				std::cout << __FILE__ << "shuttering" << std::endl;
				grabImageMsg(ipid);
				publishMessage(dX, dY, dTheta);
				dX = 0;
				dY = 0;
				dTheta = 0;
			}
		}
}