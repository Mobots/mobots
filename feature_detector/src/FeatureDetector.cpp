#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <iostream>

#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesFinder.h"
#include "profile.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

cv::Ptr<FeaturesFinder> detector;
ros::Publisher publisher;

char* TAG;


void processImage(const mobots_msgs::ImageWithPoseAndID& msg){
#if DEBUG
	ROS_INFO("%s processImage", TAG);
#endif
	FeatureSet features;
	if(msg.image.encoding == string("png")){
		cv::Mat img = cv::imdecode(msg.image.data, 0);
		detector->computeFeatureSet(img, features);
	}else{
		cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(msg.image, "mono8");
		detector->computeFeatureSet(imagePtr->image, features);
	}
  mobots_msgs::FeatureSetWithPoseAndID result;
  MessageBridge::copyToRosMessage(features, result.features);
  result.pose = msg.pose;
  result.id = msg.id;
  publisher.publish(result);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "feature_detector");
  ros::NodeHandle nodeHandle;
  stringstream ss;
  ss << "[" << nodeHandle.getNamespace() << "/feature_detector]";
  TAG = new char[ss.str().size()+1];
  strcpy(TAG, ss.str().c_str());
  ros::Subscriber subscriber = nodeHandle.subscribe("image_pose_id", 10, processImage);
  publisher = nodeHandle.advertise<mobots_msgs::FeatureSetWithPoseAndID>("featureset_pose_id", 10);
	int sliceCount = 5;
	int maxFeaturesPerslice = 500;
	ros::param::get("/feature_detector/slice_count", sliceCount);
	ros::param::get("/feature_detector/features_per_slice", maxFeaturesPerslice);
	cout << "slicecnt " << sliceCount << "  maxfeatures " << maxFeaturesPerslice << endl;
  detector = new OrbFeaturesFinder(maxFeaturesPerslice, sliceCount);
  ROS_INFO("%s now spinning", TAG);
  ros::spin();
  return 0;
}
