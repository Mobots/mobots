#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <iostream>

#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesFinder.h"
#include "mobots_msgs/FeatureSetWithDeltaPoseAndID.h"
#include "mobots_msgs/ImageWithDeltaPoseAndID.h"

using namespace std;

cv::Ptr<FeaturesFinder> detector;
ros::Publisher publisher;

char* TAG;


void processImage(const mobots_msgs::ImageWithDeltaPoseAndID& image){
  ROS_INFO("%s processImage", TAG);
  FeatureSet features;
  cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image.image);
  detector->computeFeatureSet(imagePtr->image, features);
  mobots_msgs::FeatureSetWithDeltaPoseAndID result;
  MessageBridge::copyToRosMessage(features, result);
  publisher.publish(result);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "feature_detector");
  ros::NodeHandle nodeHandle;
  stringstream ss;
  ss << "[" << nodeHandle.getNamespace() << "/feature_detector]";
  TAG = new char[ss.str().size()+1];
  strcpy(TAG, ss.str().c_str());
  ros::Subscriber subscriber = nodeHandle.subscribe("ImageWithDeltaPoseAndID", 100, processImage);
  publisher = nodeHandle.advertise<mobots_msgs::FeatureSetWithDeltaPoseAndID>("FeatureSetWithDeltaPoseAndID", 10); 
  //detector = new SurfFeaturesFinder(400, 3, 4, 4, 2, false);
  detector = FeaturesFinder::getDefault();
  //detector = new FastFeaturesFinder;
  ROS_INFO("%s now spinning", TAG);
  ros::spin();
  return 0;
}
