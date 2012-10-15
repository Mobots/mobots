#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <fstream>

#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesFinder.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "mobots_msgs/ImageWithPoseAndID.h"

using namespace std;

cv::Ptr<FeaturesFinder> detector;
ros::Publisher publisher;

char* TAG;


void processImage(const mobots_msgs::ImageWithPoseAndID& image){
  ROS_INFO("%s processImage", TAG);
  FeatureSet features;
  cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image.image, "mono8");
  detector->computeFeatureSet(imagePtr->image, features);
  mobots_msgs::FeatureSetWithPoseAndID result;
  MessageBridge::copyToRosMessage(features, result.features);
  result.pose = image.pose;
  result.id = image.id;
  publisher.publish(result);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "feature_detector");
  ros::NodeHandle nodeHandle;
  stringstream ss;
  ss << "[" << nodeHandle.getNamespace() << "/feature_detector]";
  TAG = new char[ss.str().size()+1];
  strcpy(TAG, ss.str().c_str());
  ros::Subscriber subscriber = nodeHandle.subscribe("image_pose_id", 100, processImage);
  publisher = nodeHandle.advertise<mobots_msgs::FeatureSetWithPoseAndID>("featureset_pose_id", 10); 
  detector = FeaturesFinder::getDefault();
  ROS_INFO("%s now spinning", TAG);
  ros::spin();
  return 0;
}
