#include "ros/ros.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "cv_bridge/cv_bridge.h"

using namespace std;


void imageCallback(const mobots_msgs::ImageWithPoseAndID& msg){
  cout << "got pic" << endl;
  cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg.image, "bgr8");
  cv::imshow("pic", ptr->image);
  cv::waitKey(0);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "adsfasdfadf");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/mobot0/image_pose_id", 5, imageCallback);
  ros::spin();  
}