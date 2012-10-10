#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "feature_detector/FeaturesFinder.h"

#pragma once

typedef struct{
  double x;
  double y;
  double theta;
} Delta;

class FeaturesMatcher{
public:
  FeaturesMatcher(){}
  virtual ~FeaturesMatcher(){}
  virtual bool match(const FeatureSet& img1, const FeatureSet& img2, Delta& delta) const = 0;
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
  virtual bool match(const FeatureSet& img1, const FeatureSet& img2, Delta& delta) const;
  
  static const char SURF_DEFAULT[];
  static const char ORB_DEFAULT[];
};

class GpuFeaturesMatcher : public FeaturesMatcher{
public:
  virtual ~GpuFeaturesMatcher(){}
  GpuFeaturesMatcher(){}
  virtual bool match(const FeatureSet& img1, const FeatureSet& img2, Delta& delta) const{
    return false;
  }
};