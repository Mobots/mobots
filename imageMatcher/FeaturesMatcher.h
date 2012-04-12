#include "FeaturesFinder.h"

#pragma once

using namespace std;

typedef struct{
  double x;
  double y;
  double theta;
} Delta;

typedef struct{
  Mat H; //homegraphy matrix, where Image1 * H = Image2
  Delta delta;
} ImageMatchResult;

class FeaturesMatcher{
public:
  virtual ~FeaturesMatcher(){}
  virtual void match(const ImageFeatures& img1, const ImageFeatures& img2, ImageMatchResult& result) const = 0;
};

class CpuFeaturesMatcher : public FeaturesMatcher{
private:
  Ptr<DescriptorMatcher> matcher;
public:
  virtual ~CpuFeaturesMatcher(){}
  CpuFeaturesMatcher(const string& type);
  virtual void match(const ImageFeatures& img1, const ImageFeatures& img2, ImageMatchResult& result) const;
  
  static const char SURF_DEFAULT[];
  static const char ORB_DEFAULT[];
};

class GpuFeaturesMatcher : public FeaturesMatcher{
public:
  virtual ~GpuFeaturesMatcher(){}
  GpuFeaturesMatcher(){}
  virtual void match(const ImageFeatures& img1, const ImageFeatures& img2, ImageMatchResult& result) const{}
};