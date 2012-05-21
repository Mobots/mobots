#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <iostream>

#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesFinder.h"
#include "mobots_msgs/FeatureSetWithDeltaPose.h"
#include "mobots_msgs/ImageWithPoseDebug.h"

using namespace std;

cv::Ptr<FeaturesFinder> detector;
ros::Publisher publisher;


void processImage(const mobots_msgs::ImageWithPoseDebug& image){
  cout << "processImage" << endl;
  ImageFeatures features;
  cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image.image);
  detector->findFeatures(imagePtr->image, features);
  mobots_msgs::FeatureSetWithDeltaPose result;
  MessageBridge::copyToRosMessage(features, result);
  publisher.publish(result);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "FeatureDetector");
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber = nodeHandle.subscribe("ImageWithPose", 100, processImage);
  publisher = nodeHandle.advertise<mobots_msgs::FeatureSetWithDeltaPose>("FeatureSetWithDeltaPose", 10); 
  //detector = new SurfFeaturesFinder(400, 3, 4, 4, 2, false);
  //detector = new OrbFeaturesFinder(1400);
  detector = new FastFeaturesFinder;
  cout << "now spinning" << endl;
  ros::spin();
  return 0;
}
