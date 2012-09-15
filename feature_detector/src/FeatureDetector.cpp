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


void processImage(const mobots_msgs::ImageWithDeltaPoseAndID& image){
  cout << "processImage" << endl;
  FeatureSet features;
  cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image.image);
  detector->computeFeatureSet(imagePtr->image, features);
  mobots_msgs::FeatureSetWithDeltaPoseAndID result;
  MessageBridge::copyToRosMessage(features, result);
  publisher.publish(result);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "FeatureDetector");
  ros::NodeHandle nodeHandle;
  nodeHandle.getNamespace();
  ros::Subscriber subscriber = nodeHandle.subscribe("ImageWithDeltaPoseAndID", 100, processImage);
  publisher = nodeHandle.advertise<mobots_msgs::FeatureSetWithDeltaPoseAndID>("FeatureSetWithDeltaPoseAndID", 10); 
  //detector = new SurfFeaturesFinder(400, 3, 4, 4, 2, false);
  detector = FeaturesFinder::getDefault();
  //detector = new FastFeaturesFinder;
  cout << "now spinning" << endl;
  ros::spin();
  return 0;
}
