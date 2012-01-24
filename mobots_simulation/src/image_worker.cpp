#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr&);

image_transport::Subscriber image_sub; 
image_transport::Publisher image_pub;

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
	double time = (double)cv::getTickCount();
	//cv::FAST detector;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat mask;
	cv::FAST(cv_ptr->image, keypoints, 1);
	time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
	cv::drawKeypoints(p2->image, keypoints, p2->image);
	image_pub.publish(p2->toImageMsg());
}
