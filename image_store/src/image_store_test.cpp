#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "map_visualization/GetImageWithPose.h"
#include "image_transport/image_transport.h"
#include "opencv/cvwimage.h"
#include "opencv/highgui.h"
#include "iostream"
#include <string>
#include "fstream"
#include "vector"

#include "image_pose_data_types.h"
#include "feature.cpp"
#include <assert.h>

/**
 * This is an example of how to interface with the image_store server.
 */
void saveRequest(std::string filePath, ros::NodeHandle* handle){
	ros::Publisher pub = handle->advertise<mobots_msgs::ImageWithPoseAndID>("shutter_image_delta_pose", 10);
	mobots_msgs::ImageWithPoseAndID msg;
	// Set image info data
	msg.pose.x = 1.23;
	msg.pose.y = 2.23;
	msg.pose.theta = 0.00;
	msg.image.encoding = "jpg";
	msg.id.session_id = 0;
	msg.id.mobot_id = 0;
	msg.id.image_id = 0;
	msg.image.width = 100;
	msg.image.height = 100;
	// Load image data
	std::ifstream imageFile(filePath.c_str(), std::ios::binary);
	imageFile.seekg(0, std::ios::end);
	int length = imageFile.tellg();
	ROS_INFO("length: %i", length);
	char buffer[length];
	imageFile.seekg(0, std::ios::beg);
	imageFile.read(buffer, length);
	msg.image.data.assign(buffer, buffer + sizeof(buffer)/sizeof(char));
	ROS_INFO("data.size: %i", msg.image.data.size());
	
	ros::Rate loop_rate(0.5);
	
	while (handle->ok())
	{
		ROS_INFO("Sent msg no: %i", msg.id.image_id);
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		// Change image info data (pose)
		msg.pose.x++;
		msg.pose.y++;
		msg.id.image_id++;
	}
	return;
}

void getRequest(int sessionID, int mobotID, int imageID, ros::NodeHandle* handle){
	ros::ServiceClient client = handle->serviceClient<map_visualization::GetImageWithPose>("image_store_get");
	map_visualization::GetImageWithPose srv;
	srv.request.id.session_id = sessionID;
	srv.request.id.mobot_id = mobotID;
	srv.request.id.image_id = imageID;
	
	if(client.call(srv)){
		if(srv.response.error == 0){
			ROS_INFO("Returned image");
		} else {
			ROS_INFO("Error Message: %i", srv.response.error);
		}
	} else {
		ROS_INFO("Errror Message: Server Error");
	}
	return;
}

/**
 * Here the actual feature_store test is performed
 */
void featuresReceived(const mobots_msgs::FeatureSetWithPoseAndID& msg){
  std::cout << "feature_store test received features" << std::endl;
  IDT id;
  id.imageID = msg.id.image_id;
  id.mobotID = msg.id.mobot_id;
  id.sessionID = msg.id.session_id;
  FeatureStore::saveFeatureSet(msg);
  std::cout << "feature_store test saved features" << std::endl;
  mobots_msgs::FeatureSetWithPoseAndID msg2;
  FeatureStore::loadFeatureSet(id, msg2);
  std::cout << "feature_store test loaded features" << std::endl;
  assert(msg.id.image_id == msg2.id.image_id);
  assert(msg.id.mobot_id == msg2.id.mobot_id);
  assert(msg.id.session_id == msg2.id.session_id);
  assert(msg.pose.x == msg2.pose.x);
  assert(msg.pose.y == msg2.pose.y);
  assert(msg.pose.theta == msg2.pose.theta);
  assert(msg.features.keyPoints.size() == msg2.features.keyPoints.size());
  for(int i = 0; i < msg.features.keyPoints.size(); i++){
	 assert(msg.features.keyPoints[i].angle == msg2.features.keyPoints[i].angle);
	 assert(msg.features.keyPoints[i].class_id == msg2.features.keyPoints[i].class_id);
	 assert(msg.features.keyPoints[i].octave == msg2.features.keyPoints[i].octave);
	 assert(msg.features.keyPoints[i].pt.x == msg2.features.keyPoints[i].pt.x);
	 assert(msg.features.keyPoints[i].size == msg2.features.keyPoints[i].size);	 
  }
	assert(msg.features.descriptors.data == msg2.features.descriptors.data);
  int somethingWentWrong;
	assert(somethingWentWrong);
}

//for feature_store test
ros::Subscriber featureSub;
ros::Publisher publisher;
ros::NodeHandle* nh;

/**
 * Helper method for feature_store test
 */
void copyMatToImageMSg(const cv::Mat& in, mobots_msgs::ImageWithPoseAndID& out2){
  sensor_msgs::Image* out = &out2.image;
  out->height = in.rows;
  out->width = in.cols;
  out->encoding = "mono8";
  out->is_bigendian = 0;
  out->step = in.cols * in.elemSize();
  out->data.resize(in.rows * out->step);
  if(in.isContinuous()){
    memcpy(&out->data[0], in.data, in.rows * out->step);
  }else{
    // Copy row by row
    uchar* ros_data_ptr = (uchar*)(&out->data[0]);
    uchar* cv_data_ptr = in.data;
    for (int i = 0; i < in.rows; i++){
      memcpy(ros_data_ptr, cv_data_ptr, out->step);
      ros_data_ptr += out->step;
      cv_data_ptr += in.step;
    }
  }
}

/**
 * tests the feature store
 */
void testFeatureStore(){  
  //obtain featureset
  //first read image and send it to feature_detector
  mobots_msgs::ImageWithPoseAndID i;
  i.id.image_id = 0xDEADBEEF;
  i.id.mobot_id = -1;
  i.pose.theta = 5;
  i.pose.x = 1;
  FILE* fp;
  char result [1000];
  fp = popen("rospack find slam","r");
  fread(result, 1, sizeof(result), fp);
  pclose(fp);
  std::stringstream ss;
  result[strlen(result)-1] = '\0';
  ss << "/home/jonas/mobots/slam" << "/pics/" << 1 << ".png";
  cv::Mat img = cv::imread(ss.str(), 1);
  std::cout << "using " << ss.str() << std::endl;
  nh = new ros::NodeHandle;
  copyMatToImageMSg(img, i);
  featureSub = nh->subscribe("featureset_pose_id", 2, featuresReceived);
  publisher = nh->advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id", 2);
  sleep(1);
  publisher.publish(i);
}

/**
 * This server is used to check if the toro_client.cpp works.
 * The ImagePoseID msg is tested.
 */
int main(int argc, char **argv)
{
	//==== feature_store test ==== 
  ros::init(argc, argv, "image_store_test");
  testFeatureStore();
  ros::spin();
	//==== end feature store test
	if(argc < 2){
		ROS_INFO("Need more arguments");
		return 0;
	}
	std::string argument1(argv[1]);
	ros::init(argc, argv, "image_store_test");
	ros::NodeHandle handle;
	
	if(argument1 == "save" && argc == 3){
		saveRequest(std::string(argv[2]), &handle);
	} else if(argument1 == "get" && argc == 5){
		getRequest(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), &handle);
	} else {
		ROS_INFO("Usage: image_store_test [save /path/to/file.jpg] [get sessionID mobotID imageID]");
	}
	return 0;
}
