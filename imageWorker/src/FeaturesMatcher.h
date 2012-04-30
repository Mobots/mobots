#include "FeaturesFinder.h"

#pragma once

using namespace std;

typedef struct{
  double x;
  double y;
  double theta;
} Delta;

class FeaturesMatcher{
protected:
  float ratioThreshold;
public:
  FeaturesMatcher(){
    this->ratioThreshold = 0.6; //0.6 seems to be good..
  }
  void setRatioThreshold(float ratioThreshold){
    this->ratioThreshold = ratioThreshold;
  }
  virtual ~FeaturesMatcher(){}
  virtual bool match(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const = 0;
};

class CpuFeaturesMatcher : public FeaturesMatcher{
private:
  Ptr<DescriptorMatcher> matcher;
public:
  virtual ~CpuFeaturesMatcher(){}
  CpuFeaturesMatcher(const string& type);
  virtual bool match(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const;
  
  static const char SURF_DEFAULT[];
  static const char ORB_DEFAULT[];
};

class GpuFeaturesMatcher : public FeaturesMatcher{
public:
  virtual ~GpuFeaturesMatcher(){}
  GpuFeaturesMatcher(){}
  virtual bool match(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const{
    return false;
  }
};