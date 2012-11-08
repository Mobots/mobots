#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <geometry_msgs/Pose2D.h>

#include "feature_detector/FeaturesFinder.h"

#pragma once

typedef struct{
	geometry_msgs::Pose2D delta;
	double varX; //units in pixel²
	double varY;
	double varTheta; //rad²
}MatchResult;

class FeaturesMatcher{
public:
  FeaturesMatcher(){}
  virtual ~FeaturesMatcher(){}
  virtual bool match(const FeatureSet& img1, const FeatureSet& img2, MatchResult& result) const = 0;
  /**
  * Use this method to retrieve the currently best matcher
  */
  static cv::Ptr<FeaturesMatcher> getDefault();
};

class CpuFeaturesMatcher : public FeaturesMatcher{
private:
  cv::Ptr<cv::DescriptorMatcher> matcher;
public:
  virtual ~CpuFeaturesMatcher(){}
  CpuFeaturesMatcher(const std::string& type);
  virtual bool match(const FeatureSet& img1, const FeatureSet& img2, MatchResult& result) const;
  
  static const char SURF_DEFAULT[];
  static const char ORB_DEFAULT[];
};

class GpuFeaturesMatcher : public FeaturesMatcher{
public:
  virtual ~GpuFeaturesMatcher(){}
  GpuFeaturesMatcher(){}
  virtual bool match(const FeatureSet& img1, const FeatureSet& img2, MatchResult& result) const{
    return false;
  }
};