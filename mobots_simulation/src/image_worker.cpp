#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr&);

image_transport::Subscriber image_sub; 
image_transport::Publisher image_pub;
cv::Mat oldDescriptors;

int main(int argc, char **argv){
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_sub = it.subscribe("/my_cam/image", 1, imageCallback);
	image_pub = it.advertise("/my_cam/featured", 1);
	ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr p2;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
		p2 = cv_bridge::toCvCopy(msg);
	}catch(cv_bridge::Exception &e){
		ROS_ERROR("cv_bridge exception: %s", e.what());	
	}
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

	cv::SIFT detector;
	detector(cv_ptr->image, cv::Mat(), keypoints, descriptors);
	//cv::FAST(cv_ptr->image, keypoints, 1);
 	//BriefDescriptorExtractor descriptorExtractor; 	
	//descriptorExtractor.compute(cv_ptr->image, keypoints, descriptors);
	cv::BruteForceMatcher<cv::Hamming> matcher;
	vector<cv::DMatch> matches;
	//if(oldDescriptors){	
	 // matcher.match(descriptors, oldDescriptors, matches);
	  //for(int i = matches.size() -1; i >= 0; i--){
	   // cout << "Distance: " << matches[i].distance << endl;
	  //}
	  
	//}
	cv::drawKeypoints(p2->image, keypoints, p2->image);
	image_pub.publish(p2->toImageMsg());
	oldDescriptors = descriptors;
}
