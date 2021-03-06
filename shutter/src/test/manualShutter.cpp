#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <usb_cam/usb_cam.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "mobots_msgs/ImageWithPoseAndID.h"

using namespace std;

const char TAG[] = "[manualShutter] ";
sensor_msgs::Image currentImage;
ros::Publisher pub;
usb_cam_camera_image_t* camera_image_;

void imageCallback(const sensor_msgs::Image &image){
  currentImage = image;
}

void* shutterThread(void* data){
  mobots_msgs::ImageWithPoseAndID msg;
  int imageID = 0;
  int mobotID = 0;
  int sessionID = 0;
  while(ros::ok()){
		cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
		camera_image_->is_new = 0;
		for(int i = 0; i < 6; i++){
			usb_cam_camera_grab_image(camera_image_);
			cout << "is new " << camera_image_->is_new << endl;
		}
		fillImage(msg.image, "rgb8", camera_image_->height, camera_image_->width, 3 * camera_image_->width, camera_image_->image);
		//msg.image = currentImage;
		pub.publish(msg);
		cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(msg.image);
		stringstream ss;
		ss << msg.id.image_id << ".png" << endl;
		//vector<uchar> buf;
		//cv::imencode(".png", imagePtr->image, buf);
		cv::imwrite(ss.str(), imagePtr->image);
		msg.id.image_id++;
		cout << TAG << "shuttered" << endl;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "manualShutter");
  ros::NodeHandle nh;
  pub = nh.advertise<mobots_msgs::ImageWithPoseAndID>("/mobot1/ImageWithPoseAndID", 2);
  //ros::Subscriber image_sub = nh.subscribe("usb_cam/image_raw", 1, imageCallback);
  pthread_t thread_t;
  pthread_create(&thread_t, 0, shutterThread, 0);
  cout << TAG << "Press [enter] for shutter" << endl;
  camera_image_ = usb_cam_camera_start("/dev/video1",
        IO_METHOD_MMAP,
        PIXEL_FORMAT_YUYV,
        640,
        480);
  ros::spin();
  usb_cam_camera_shutdown();
}