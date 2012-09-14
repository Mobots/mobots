#include <unistd.h>
#include <iostream>
#include <ros/ros.h>

#include "mobots_msgs/ImageWithPoseAndID.h"

using namespace std;

const char TAG[] = "[manualShutter] ";
sensor_msgs::Image currentImage;
ros::Publisher pub;

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
    msg.image = currentImage;
    pub.publish(msg);
	msg.id.image_id++;
    cout << TAG << "shuttered" << endl;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "manualShutter");
  ros::NodeHandle nh;
  pub = nh.advertise<mobots_msgs::ImageWithPoseAndID>("shutter_image_delta_pose", 2);
  ros::Subscriber image_sub = nh.subscribe("usb_cam/image_raw", 1, imageCallback);
  pthread_t thread_t;
  pthread_create(&thread_t, 0, shutterThread, 0);
  cout << TAG << "Press [enter] for shutter" << endl;
  ros::spin();
}