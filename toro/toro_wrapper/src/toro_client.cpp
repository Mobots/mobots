#include "ros/ros.h"
#include "mobots_msgs/MobotImagePose.h"
#include "image_transport/image_transport.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include "iostream"
#include "fstream"

/**
 * The imageHandler method.
 */
void imageHandler(const mobots_msgs::ImageWithPose::ConstPtr& msg)
{
	ROS_INFO("MobotID: %i / Header: %i - %s / image: %s", msg->mobotID, msg->image.header.seq, msg->image.header.frame_id.c_str(), msg->image.encoding.c_str());
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("Got a callback!");
  sensor_msgs::CvBridge bridge;
  try
  {
    cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/**
 * The main method.
 */
int main(int argc, char **argv)
{
	// The node is called toro_client
	ros::init(argc, argv, "toro_client");
	ros::NodeHandle handle;
	// The topic name is mobot_image_pose
	/*ros::Subscriber sub = handle.subscribe("mobot_image_pose", 1000, imageHandler);
	ROS_INFO("Check: Spin-pre");
	ros::spin();
	return 0;*/
	
	cvNamedWindow("view");
	cvStartWindowThread();
	image_transport::ImageTransport it(handle);
	image_transport::Subscriber sub = it.subscribe("mobot_image_pose", 1, imageCallback);
	ros::spin();
	cvDestroyWindow("view");
	return 0;
}