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

static char TAG[] = "[FeatureDetector] ";

string getPathForMobot(int mobotId, const string& topic){
  stringstream ss;
  ss << "/mobot" << mobotId << "/" << topic;
  return ss.str();
}


void processImage(const mobots_msgs::ImageWithPoseDebug& image){
  ROS_INFO("%s processImage", TAG);
  ImageFeatures features;
  cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image.image);
  detector->findFeatures(imagePtr->image, features);
  mobots_msgs::FeatureSetWithDeltaPose result;
  MessageBridge::copyToRosMessage(features, result);
  publisher.publish(result);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "FeatureDetector");
  ros::NodeHandle nodeHandle("~");
  int mobotId;
  nodeHandle.param("mobotID", mobotId, 0);
  if(mobotId == -1){
    ROS_ERROR("%s no mobotID specified, exiting.", TAG);
    exit(1);
  }
  string imagesPath = getPathForMobot(mobotId, "ImageWithDeltaPoseAndId");
  ros::Subscriber subscriber = nodeHandle.subscribe(imagesPath, 100, processImage);
  string featuresPath = getPathForMobot(mobotId, "FeatureSetWithDeltaPose");
  publisher = nodeHandle.advertise<mobots_msgs::FeatureSetWithDeltaPose>(featuresPath, 10); 
  
  //detector = new SurfFeaturesFinder(400, 3, 4, 4, 2, false);
  detector = new OrbFeaturesFinder(1000);
  cout << TAG << "now spinning on id:" << mobotId << endl;
  ros::spin();
  return 0;
}
