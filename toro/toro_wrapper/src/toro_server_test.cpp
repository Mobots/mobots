#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mobots_msgs/MobotImagePose.h"
#include "image_transport/image_transport.h"
#include "opencv/cvwimage.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include "iostream"
#include "fstream"

/**
 * This server is used to check if the toro_client.cpp works.
 * The MobotImagePose msg is tested.
 */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "toro_server_test");
	ros::NodeHandle handle;
	image_transport::ImageTransport it(handle);
	image_transport::Publisher pub = it.advertise("mobot_image_pose", 1);
	cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
	sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
	
	ros::Rate loop_rate(1);
	
	int count = 0;
	while (handle.ok())
	{
		ROS_INFO("Sent msg no: %i", count);
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
	
	return 0;
}
	
