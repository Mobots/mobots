#include "ros/ros.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "mobots_common/constants.h"

using namespace std;


void imageCallback(const mobots_msgs::ImageWithPoseAndID& msg){
  cout << "got pic" << endl;
  cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg.image, "bgr8");
	stringstream ss;
	ss << "mobot " << msg.id.mobot_id;
  cv::imshow(ss.str(), ptr->image);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "adsfasdfadf");
  ros::NodeHandle nh;
	ros::Subscriber subs[mobots_common::constants::mobot_count];
	for(int i = 0; i < mobots_common::constants::mobot_count; i++){
		stringstream ss;
		ss << "/mobot" << i << "/image_pose_id";
		subs[i] = nh.subscribe(ss.str(), 5, imageCallback);
	}
  ros::spin();  
}