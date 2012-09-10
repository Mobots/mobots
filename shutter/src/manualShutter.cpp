#include <unistd.h>
#include <iostream>
#include <ros/ros.h>

#include "mobots_msgs/ImageWithDeltaPoseAndID.h"

using namespace std;

const char TAG[] = "[manualShutter] ";
sensor_msgs::Image currentImage;
ros::Publisher pub;

void imageCallback(const sensor_msgs::Image &image){
  currentImage = image;
}

void* shutterThread(void* data){
  mobots_msgs::ImageWithDeltaPoseAndID msg;
  while(ros::ok()){
    cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
    msg.image = currentImage;
    pub.publish(msg);
    cout << TAG << "shuttered" << endl;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "manualShutter");
  ros::NodeHandle nh;
  pub = nh.advertise<mobots_msgs::ImageWithDeltaPoseAndID>("ImageWithDeltaPoseAndID", 2);
  ros::Subscriber image_sub = nh.subscribe("usb_cam/image_raw", 1, imageCallback);
  pthread_t thread_t;
  pthread_create(&thread_t, 0, shutterThread, 0);
  cout << TAG << "Press [enter] for shutter" << endl;
  ros::spin();
}