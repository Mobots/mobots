#pragma once

#include <ros/ros.h>
#include "feature_detector/FeaturesMatcher.h" //debug and shit
#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/MessageBridge.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


class ImageHandler{
private:
  char shutterPos;
  char featurePos;
  ros::Subscriber featureSetSubscriber;
  ros::Subscriber imageSubscriber;


  FeatureSet features1;
  FeatureSet features2;
  
public:
  ImageHandler();
  
  ~ImageHandler(){}

  void shutterCallback(const mobots_msgs::ImageWithPoseAndID &imageWithPoseAndId);
  
private:  
  void featuresCallback(const mobots_msgs::FeatureSetWithPoseAndID &featuresMsg);
};