#include "FeaturesFinder.h"

#pragma once

using namespace std;

typedef struct{
  double x;
  double y;
  double theta;
} Delta;

class FeaturesMatcher{
public:
  virtual ~FeaturesMatcher(){}
  virtual void findDelta(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const = 0;
};

class CpuFeaturesMatcher : public FeaturesMatcher{
private:
  Ptr<DescriptorMatcher> matcher;
public:
  virtual ~CpuFeaturesMatcher(){}
  CpuFeaturesMatcher(const string& type);
  virtual void findDelta(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const;
  
  static const char SURF_DEFAULT[];
  static const char ORB_DEFAULT[];
};

class GpuFeaturesMatcher : public FeaturesMatcher{
public:
  virtual ~GpuFeaturesMatcher(){}
  GpuFeaturesMatcher(){}
  virtual void findDelta(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const;
};