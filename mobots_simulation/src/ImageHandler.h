#pragma once

#include <ros/ros.h>
#include "../../feature_detector/src/FeaturesMatcher.h" //debug and shit
#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/MessageBridge.h"
#include "mobots_msgs/FeatureSetWithDeltaPose.h"
#include "mobots_msgs/ImageWithPoseDebug.h"
#include "mobots_msgs/ImagePoseID.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


class ImageHandler{
private:
  char imagePos;
  char shutterPos;
  char featurePos;
  ros::Publisher publisher;
  ros::Subscriber featureSetSubscriber;
  ros::Subscriber imageSubscriber;

  sensor_msgs::Image images[2];
  ImageFeatures features1;
  ImageFeatures features2;
  
public:
  ImageHandler();
  
  ~ImageHandler(){}
  
  void shutterCallback();

  void shutterCallback2(const mobots_msgs::ImagePoseID imageWithPoseAndId);
  
private:
  void findRotationMatrix2D(cv::Point2f center, double angle, cv::Mat& rotMat);

  inline double toDegree(double rad);

  void imageCallback(const sensor_msgs::Image image);

  void featuresCallback(const mobots_msgs::FeatureSetWithDeltaPose featuresMsg);
};